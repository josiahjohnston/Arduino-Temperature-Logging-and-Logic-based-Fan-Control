#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // I2C (aka 2-wire) interface for the real-time clock that has its own battery.
#include <Time.h>
#include <DS1307RTC.h>

#define USE_SD
#define LOG_ONLY

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
  {0x28, 0xB9, 0x40, 0xBB, 0x03, 0x00, 0x00, 0x87},
  {0x28, 0x94, 0x54, 0xBB, 0x03, 0x00, 0x00, 0x18},
  {0x28, 0xC8, 0x28, 0xBB, 0x03, 0x00, 0x00, 0x07},
  {0x28, 0x4E, 0x29, 0xBB, 0x03, 0x00, 0x00, 0x92},
  {0x28, 0xD7, 0x1B, 0xBB, 0x03, 0x00, 0x00, 0xB2},
  {0x28, 0x65, 0x58, 0xBB, 0x03, 0x00, 0x00, 0x7C}
};
const byte outdoor_temp_sensor_addr[ADDR_LEN] = { 0x28, 0x4D, 0x4F, 0xBB, 0x03, 0x00, 0x00, 0x46 };

DeviceAddress addr;
float temps[NUM_ROOM_SENSORS];
float outdoor_temp;

/* Sensor heights in inches, ordered by their height from the floor (highest to lowest) */
const float heights[NUM_ROOM_SENSORS] = { 94, 79.25, 69.5, 45, 22.5, 3.5 };


// Fan control variables
const int fan_control_pin = 3;
unsigned long fan_on_time;
unsigned long fan_on_duration;
const unsigned long fan_on_response = 10 * 1000; // fans kick on for this many milliseconds at a time.
byte fan_speed;
const float delta_temp_threshold = 11.0;
const float delta_temp_maxout = 25.0;
const byte min_fan_speed = 60;
const byte max_fan_speed = 111;






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

  // Time section
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
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
  /* Sensor variables */
  float temp;
  float temp_min;
  float temp_max;

  // Print Column headers if we haven't printed them for a while
  if( num_loops_since_temp_col_header >= 30 ) { num_loops_since_temp_col_header = 0; }
  if( num_loops_since_temp_col_header == 0 ) {
    Serial.println("");
    Serial.print("Date: ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(", ");
    Serial.println(year());
    Serial.println("                                ... Sensor heights (in inches)");
    Serial.print(  "Timestamp, FanSpeed, OutdoorTemp");
    for( sensor_idx = 0; sensor_idx < NUM_ROOM_SENSORS; sensor_idx++ ) {
      Serial.print(",  ");
      Serial.print(heights[sensor_idx]);
    }
    Serial.println("");
    Serial.println("Sensor temps (in F)...");
  }
  
  // Read sensors and record min & max
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
    if( read_attempts >= 10 ) { 
      Serial.println("");
      Serial.print(read_attempts);
      Serial.print(" read failures on sensor ");
      Serial.println(sensor_idx);
    }
    temps[sensor_idx] = sensors.toFahrenheit(temps[sensor_idx]);
    if( sensor_idx == 0 ) {
        temp_min = temps[sensor_idx];
        temp_max = temps[sensor_idx];
    }
    if( temp_min > temps[sensor_idx] ) temp_min = temps[sensor_idx];
    if( temp_max < temps[sensor_idx] ) temp_max = temps[sensor_idx];
  }
  // Read outdoor temperature
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
    Serial.print(" read failures on outdoor sensor.");
  }
  outdoor_temp = sensors.toFahrenheit(outdoor_temp);
  num_loops_since_temp_col_header++;

  // If the temperature difference is more than the threshold, update fan control settings for so many seconds. Also do a reality check on the reading.
#ifndef LOG_ONLY
  if( (temp_max - temp_min) > delta_temp_threshold ) {
    fan_on_duration = fan_on_response;
    fan_on_time = millis();
    if( (temp_max - temp_min) > delta_temp_maxout ) {
      fan_speed = max_fan_speed;
    } else {
      fan_speed = (byte)( ((float)min_fan_speed) + ((float)(max_fan_speed - min_fan_speed)) * ((float)(temp_max - temp_min - delta_temp_threshold)) /  ((float) delta_temp_maxout - delta_temp_threshold) );
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
  Serial.print(outdoor_temp);
  for( sensor_idx = 0; sensor_idx < NUM_ROOM_SENSORS; sensor_idx++ ) {
    Serial.print(",  ");
    Serial.print( temps[sensor_idx] );
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
    dataFile.print(outdoor_temp);
    for( sensor_idx = 0; sensor_idx < NUM_ROOM_SENSORS; sensor_idx++ ) {
      dataFile.print(",");
      dataFile.print( temps[sensor_idx] );
    }
    dataFile.println("");
    dataFile.close();
  } else {
    Serial.println("Error opening datalog.txt");
  } 
#endif

  // Check if its time to turn the fan OFF
#ifndef LOG_ONLY
  if( ( fan_on_duration > 0 ) && ( fan_on_duration + fan_on_time <= millis() ) ) {
    fan_on_duration = 0;
    fan_speed = 0;
    analogWrite(fan_control_pin, fan_speed);
    return;
  }
  analogWrite(fan_control_pin, fan_speed);
#endif
  delay(700);
  
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
        // Report status
        Serial.print("Fan speed set to ");
        Serial.println(fan_speed);
        break;
#endif
    }      
  } 
}

