#include <Arduino.h>
#include <PWMServo.h>

PWMServo myservo1;  // create servo object to control a servo
PWMServo myservo2;
PWMServo myservo3;
PWMServo myservo4;

int pos = 0;    // variable to store the servo position

void setup() {
  myservo1.attach(5);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(6);
  myservo3.attach(7);
  myservo4.attach(8);
  //myservo.attach(SERVO_PIN_A, 1000, 2000); // some motors need min/max setting
}


void loop() {
  for(pos = 0; pos < 180; pos += 1) { // goes from 0 degrees to 180 degrees, 1 degree steps
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);
    myservo3.write(pos);
    myservo4.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for(pos = 180; pos>=1; pos-=1) {   // goes from 180 degrees to 0 degrees
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);
    myservo3.write(pos);
    myservo4.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}