const int fan_control_pin = 5;
int incomingByte;      // a variable to read incoming serial data into
int fan_speed;      // a variable to read incoming serial data into
const int max_fan_speed = 255;
char buf[100];


void setup() {
/*  PWM frequency tuning Instructions: http://www.arduino.cc/playground/Main/TimerPWMCheatsheet  */
// Can hear Fan noise at 30 Hz (0x07), 122 Hz (0x06), 244 Hz (0x05), 488 Hz (default setting 0x04), 977 Hz (0x03), 3907 Hz (0x02), 7812 Hz (pins 5/6 0x02), 
// Fan works well without noise at 31250 Hz (0x01 on most pins) and 62500 Hz (0x01 on pins 5 & 6)

  // Speed up the PWM frequency for pins 5 and 6 from 976.6 Hz to 62500 Hz
  TCCR0B = TCCR0B & 0b11111000 | 0x01;
  // Speed up the PWM frequency for pins 9 and 10 from 488.3 Hz to 30.52 Hz
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  // Speed up the PWM frequency for pins 11 and 3 from 488.3 Hz to 31250 Hz
  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  // initialize the I/O pins
  pinMode(fan_control_pin, OUTPUT);
  digitalWrite(fan_control_pin, LOW);
  // initialize serial communication:
  Serial.begin(9600);
}

void loop() {
  // see if there's incoming serial data:
  if (Serial.available() >= 3) {

    /* 3-digit read  */
    fan_speed = 0;
    incomingByte = Serial.read();
    fan_speed = fan_speed + 100 * (incomingByte - '0');
    incomingByte = Serial.read();
    fan_speed = fan_speed + 10 * (incomingByte - '0');
    incomingByte = Serial.read();
    fan_speed = fan_speed + 1 * (incomingByte - '0');

    if( fan_speed > max_fan_speed ) {
        fan_speed = max_fan_speed;
    }
    if( fan_speed < 0 ) {
        fan_speed = 0;
    }


    /* Analog controlled PWM with frequency defined in setup(). */
    analogWrite(fan_control_pin, fan_speed);

    /* Binary input of fan
    
    // if it's a '1', turn on the relay:
    incomingByte = Serial.read();
    if (incomingByte == '1') {
      digitalWrite(fan_control_pin, HIGH);
      fan_speed=1;
    } 
    // if it's a '0', turn off the relay:
    if (incomingByte == '0') {
      digitalWrite(fan_control_pin, LOW);
      fan_speed=0;
    } 
*/    
    sprintf(buf, "Fan speed is %d of %d on pin %d.\n", fan_speed, max_fan_speed, fan_control_pin);
    
    // Report status
    Serial.print(buf);
  }

/* Software controlled PWM with periods of max_fan_speed ms. With max_fan_speed of 30 ms or 100 ms, the PWM frequency is 33.3 Hz or 10 Hz. 
  digitalWrite(fan_control_pin, HIGH);
  delay(fan_speed);
  digitalWrite(fan_control_pin, LOW);
  delay(max_fan_speed - fan_speed);
*/

}

