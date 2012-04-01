#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // I2C (aka 2-wire) interface for the real-time clock that has its own battery.
#include <Time.h>
#include <DS1307RTC.h>

#define USE_SD
//#define LOG_ONLY
#define NO_OUTDOOR

#ifdef USE_SD
#include <SD.h>
#endif

// SD card + ethernet will use pins 4 and 10-13.
#ifdef USE_SD
const int SD_pin = 4;
File dataFile;
#endif

// Temp sensor variables
#define ONE_WIRE_BUS 8
#define TEMPERATURE_PRECISION 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//OneWire  ds(ONE_WIRE_BUS);
byte num_loops_since_temp_col_header = 0; 
byte sensor_idx;
byte read_attempts;

#define NUM_ROOM_SENSORS 6
#define ADDR_LEN 8
/* Sensor addresses, ordered by their height from the floor (highest to lowest) */
const byte room_temp_sensor_addr[NUM_ROOM_SENSORS][ADDR_LEN] = {
/* Bamboo string #1. Top to bottom: */
  { 0x28, 0x71, 0xAE, 0xAA, 0x03, 0x00, 0x00, 0x76 },
  { 0x28, 0xAE, 0x95, 0xAA, 0x03, 0x00, 0x00, 0x62 }, 
  { 0x28, 0xE5, 0xA8, 0xAA, 0x03, 0x00, 0x00, 0x87 }, 
  { 0x28, 0xB9, 0x40, 0xBB, 0x03, 0x00, 0x00, 0x87 }, 
  { 0x28, 0xC8, 0x28, 0xBB, 0x03, 0x00, 0x00, 0x07 }, 
  { 0x28, 0x9B, 0x70, 0xAA, 0x03, 0x00, 0x00, 0x6C } 
  
/* hanging string
  { 0x28, 0x68, 0x94, 0xAA, 0x03, 0x00, 0x00, 0x82}, 
  { 0x28, 0xD4, 0xA6, 0xAA, 0x03, 0x00, 0x00, 0xFF}, 
  { 0x28, 0x5C, 0x8A, 0xAA, 0x03, 0x00, 0x00, 0x6D}, 
  { 0x28, 0xE9, 0x6B, 0xAA, 0x03, 0x00, 0x00, 0x96}, 
  { 0x28, 0x2F, 0x87, 0xAA, 0x03, 0x00, 0x00, 0x40}, 
  { 0x28, 0x1F, 0x9F, 0xAA, 0x03, 0x00, 0x00, 0xEF}
*/

/* Copper rod line  {0x28, 0xB9, 0x40, 0xBB, 0x03, 0x00, 0x00, 0x87},
  {0x28, 0x94, 0x54, 0xBB, 0x03, 0x00, 0x00, 0x18},
  {0x28, 0xC8, 0x28, 0xBB, 0x03, 0x00, 0x00, 0x07},
  {0x28, 0x4E, 0x29, 0xBB, 0x03, 0x00, 0x00, 0x92},
  {0x28, 0xD7, 0x1B, 0xBB, 0x03, 0x00, 0x00, 0xB2},
  {0x28, 0x65, 0x58, 0xBB, 0x03, 0x00, 0x00, 0x7C} */
  
};
const byte outdoor_temp_sensor_addr[ADDR_LEN] = { 0x28, 0x4D, 0x4F, 0xBB, 0x03, 0x00, 0x00, 0x46 };

DeviceAddress addr;
float temps[NUM_ROOM_SENSORS];
float outdoor_temp;

/* Sensor heights in inches, ordered by their height from the floor (highest to lowest) */
/* copper pipe heights: const float heights[NUM_ROOM_SENSORS] = { 94, 79.25, 69.5, 45, 22.5, 3.5 }; */
const float heights[NUM_ROOM_SENSORS] = { 96, 72, 60, 48, 24, 0 };


// Fan control variables
const int fan_control_pin = 3;
unsigned long fan_on_time;
unsigned long fan_on_duration;
const unsigned long fan_on_response = 10 * 1000; // fans kick on for this many milliseconds at a time.
byte fan_speed;
byte serial_fan_override;
const float delta_temp_threshold = 11.0;
const float delta_temp_maxout = 40.0;
// Old min & max: 60 & 111
// 3/26/2012 Note: 111 to 180 feels chilly when the heater has been off for a while.
const byte min_fan_speed = 105;
const byte max_fan_speed = 140;



float pow_e( float x ) {
  return pow(x,2.718281828);
}


void setup() {
  // Speed up the PWM frequency for the fan control pin.  PWM frequency tuning Instructions: http://www.arduino.cc/playground/Main/TimerPWMCheatsheet  */
  switch( fan_control_pin ) {
    case 5: case 6:
      //  Messing with TCCR0B changes the timer used by delay() and millis(). Multiply or divide values appropriately to use those functions. e.g.  time_multiplier = 64;
      TCCR0B = TCCR0B & 0b11111000 | 0x01; // pins 5 & 6. 0x01 means 62500 Hz
      break;
    case 9: case 10:
      TCCR1B = TCCR1B & 0b11111000 | 0x01; // pins 9 & 10. 0x01 means 31250 Hz
      break;
    case 3: case 11:
      TCCR2B = TCCR2B & 0b11111000 | 0x01; // pins 11 & 3. 0x01 means 31250 Hz
      break;
  }
  
  // Initialize fan state
  fan_on_duration = 0;
  fan_on_time = 0;
  fan_speed = 0;
  serial_fan_override = 0;
  
  // Initialize the fan control pin
  pinMode(fan_control_pin, OUTPUT);
  analogWrite(fan_control_pin, fan_speed);

  // initialize serial communication:
  Serial.begin(9600);
  
  // locate devices on the bus
  Serial.print("Locating devices... Found ");
  sensors.begin();
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // Set resolution
  for( sensor_idx = 0; sensor_idx < NUM_ROOM_SENSORS; sensor_idx++ ) {
    memcpy(addr, room_temp_sensor_addr[sensor_idx], ADDR_LEN );
    sensors.setResolution(addr, TEMPERATURE_PRECISION);
//    sensors.setResolution((DeviceAddress)(room_temp_sensor_addr[sensor_idx]), TEMPERATURE_PRECISION);

  }
  // Initalize temp value
  outdoor_temp = DEVICE_DISCONNECTED;

  // Time section
  setSyncProvider(RTC.get);   // the                                                                                                       function to get the time from the RTC
  if(timeStatus()!= timeSet) 
     Serial.println("Unable to sync with the RTC");
  else
     Serial.println("RTC has set the system time");      

#ifdef USE_SD
  // Initialize the SD card
  pinMode(10, OUTPUT);
  if (!SD.begin(SD_pin)) {
    Serial.println("SD Card failed, or not present");
  }
  Serial.println("SD card initialized.");
#endif

}


void loop() {
  int incomingByte;
  char command;
  int num_valid_readings;
  /* Sensor variables */
  float temp_min;
  float temp_max;

  // Print Column headers if we haven't printed them for a while
  if( num_loops_since_temp_col_header >= 30 ) { num_loops_since_temp_col_header = 0; }
  if( num_loops_since_temp_col_header == 0 ) {
    Serial.println("");
    Serial.println("Sensor temps (in F)...");
    Serial.print(  "Timestamp, FanSpeed, OutdoorTemp");
    for( sensor_idx = 0; sensor_idx < NUM_ROOM_SENSORS; sensor_idx++ ) {
      Serial.print(",  Temp@");
      Serial.print(heights[sensor_idx]);
      Serial.print("in");
    }
    Serial.println("");
  }
  
  // Read sensors and record min & max
  num_valid_readings = 0;
  sensors.requestTemperatures();
  for( sensor_idx = 0; sensor_idx < NUM_ROOM_SENSORS; sensor_idx++ ) {
    memcpy(addr, room_temp_sensor_addr[sensor_idx], ADDR_LEN );
    temps[sensor_idx] = sensors.getTempC(addr);
    read_attempts = 1;
    if(temps[sensor_idx] == DEVICE_DISCONNECTED) { do {
      sensors.requestTemperatures();
      temps[sensor_idx] = sensors.getTempC(addr);
      read_attempts++;
    } while (temps[sensor_idx] == DEVICE_DISCONNECTED  && read_attempts < 10); }
    if( temps[sensor_idx] == DEVICE_DISCONNECTED ) { 
      Serial.println("");
      Serial.print(read_attempts);
      Serial.print(" read failures on sensor ");
      Serial.println(sensor_idx);
    } else {
      // Don't include the floor temperature in the delta T calculation. 
      if( sensor_idx != NUM_ROOM_SENSORS - 1 ) {
        if( num_valid_readings == 0 ) {
            temp_min = temps[sensor_idx];
            temp_max = temps[sensor_idx];
        } else {
          if( temp_min > temps[sensor_idx] ) temp_min = temps[sensor_idx];
          if( temp_max < temps[sensor_idx] ) temp_max = temps[sensor_idx];
        }
      }
      num_valid_readings++;
    }
  }
  // Read outdoor temperature
#ifndef NO_OUTDOOR
  memcpy(addr, outdoor_temp_sensor_addr, ADDR_LEN );
  outdoor_temp = sensors.getTempC(addr);
  read_attempts = 1;
  if(outdoor_temp == DEVICE_DISCONNECTED) { do {
    sensors.requestTemperatures();
    outdoor_temp = sensors.getTempC(addr);
    read_attempts++;
  } while (outdoor_temp == DEVICE_DISCONNECTED  && read_attempts < 10); }
  if( read_attempts >= 10 ) { 
    Serial.println("");
    Serial.print(read_attempts);
    Serial.println(" read failures on outdoor sensor.");
  }
#endif

  num_loops_since_temp_col_header++;

  // If the temperature difference is more than the threshold, update fan control settings for so many seconds. Also do a reality check on the reading.
#ifndef LOG_ONLY
  if( ( 9.0/5.0*(temp_max - temp_min) > delta_temp_threshold )  && ( serial_fan_override == 0 ) ) {
    fan_on_duration = fan_on_response;
    fan_on_time = millis();
    if( 9.0/5.0*(temp_max - temp_min) > delta_temp_maxout ) {
      fan_speed = max_fan_speed;
    } else {
// Linear
//      fan_speed = (byte)( ((float)min_fan_speed) + ((float)(max_fan_speed - min_fan_speed)) * (9.0/5.0*(temp_max - temp_min) - delta_temp_threshold) /  (delta_temp_maxout - delta_temp_threshold) );
// power, ratio^e
      fan_speed = (byte)( ((float)min_fan_speed) + ((float)(max_fan_speed - min_fan_speed)) * pow_e((9.0/5.0*(temp_max - temp_min) - delta_temp_threshold) /  (delta_temp_maxout - delta_temp_threshold) ));
    }
    serial_fan_override = 0;
  }
  // Check if its time to turn the fan OFF
  if( ( fan_on_duration > 0 ) && ( fan_on_duration + fan_on_time <= millis() ) ) {
    fan_on_duration = 0;
    fan_speed = 0;
    serial_fan_override = 0;
  }
  analogWrite(fan_control_pin, fan_speed);
#endif

  // Print out system state to the serial port & log file
  Serial.print('"');
  Serial.print(month());
  Serial.print("/");
  Serial.print(day());
  Serial.print("/");
  Serial.print(year());
  Serial.print(' ');
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.print('"');
  Serial.print(",  ");
  Serial.print(fan_speed);
  Serial.print(",  ");
  if( outdoor_temp == DEVICE_DISCONNECTED ) {
    Serial.print( "ERR" );
  } else {
    Serial.print( sensors.toFahrenheit(outdoor_temp) );
//    Serial.print( outdoor_temp );
  }
  for( sensor_idx = 0; sensor_idx < NUM_ROOM_SENSORS; sensor_idx++ ) {
    Serial.print(",  ");
    if( temps[sensor_idx] == DEVICE_DISCONNECTED ) {
      Serial.print( "ERR" );
    } else {
      Serial.print( sensors.toFahrenheit(temps[sensor_idx]) );
//      Serial.print( temps[sensor_idx] );
    }
  }
  Serial.println("");


#ifdef USE_SD
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print('"');
    dataFile.print(month());
    dataFile.print("/");
    dataFile.print(day());
    dataFile.print("/");
    dataFile.print(year());
    dataFile.print(' ');
    dataFile.print(hour());
    dataFile.print(":");
    dataFile.print(minute());
    dataFile.print(":");
    dataFile.print(second());
    dataFile.print('"');
    dataFile.print(",");
    dataFile.print(fan_speed);
    dataFile.print(",");
    if( outdoor_temp == DEVICE_DISCONNECTED ) {
      dataFile.print( "ERR" );
    } else {
      dataFile.print(sensors.toFahrenheit(outdoor_temp));
//      dataFile.print(outdoor_temp);
    }
    for( sensor_idx = 0; sensor_idx < NUM_ROOM_SENSORS; sensor_idx++ ) {
      dataFile.print(",");
      if( temps[sensor_idx] == DEVICE_DISCONNECTED ) {
        dataFile.print( "ERR" );
      } else {
        dataFile.print( sensors.toFahrenheit(temps[sensor_idx]) );
//        dataFile.print( temps[sensor_idx] );
      }
    }
    dataFile.println("");
    dataFile.close();
  } else {
    Serial.println("Error opening datalog.txt");
  } 
#endif

  
  /* Fan controlled via incoming serial data 
       Look for incoming fan speeds. When 3-digit numbers come in, update fan speeds in 30-second bursts.
  */
  if (Serial.available() >= 1) {

    Serial.println("");
    Serial.println("Reading serial data");
    incomingByte = Serial.read();
    command = incomingByte;
    switch (command) {
      case 'D':
      case 'd':
#ifdef USE_SD
        dataFile = SD.open("datalog.txt", FILE_READ);
        if( dataFile ) {
          incomingByte = dataFile.read();
          while ( incomingByte > 0 ) {
            Serial.write(incomingByte);
            incomingByte = dataFile.read();
          }
        }
        dataFile.close();
#endif
        break;
#ifndef LOG_ONLY
      case 'F':
      case 'f':
        /* 3-digit read  */
        fan_speed = 0;
        incomingByte = Serial.read();
        fan_speed = fan_speed + 100 * (incomingByte - '0');
        incomingByte = Serial.read();
        fan_speed = fan_speed + 10 * (incomingByte - '0');
        incomingByte = Serial.read();
        fan_speed = fan_speed + 1 * (incomingByte - '0');
        fan_on_duration = (unsigned long)30*1000;
        fan_on_time = millis();
        /* Analog controlled PWM with frequency defined in setup(). */
        analogWrite(fan_control_pin, fan_speed);
        serial_fan_override = 1;
        // Report status
        Serial.print("Fan speed set to ");
        Serial.println(fan_speed);
        break;
#endif
    }      
  } 
  delay(700);
}

