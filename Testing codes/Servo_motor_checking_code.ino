#include <Servo.h>

Servo myServo;  // Create servo object to control a servo

const int servoPin = 9; // Pin connected to the servo signal wire
int angle = 0;          // Variable to store the servo position

void setup() {
  myServo.attach(servoPin); // Attaches the servo on pin 9
  
  // 1. Get the servo motor to the initial position of 0 degrees
  myServo.write(0);
  
  // Wait 1 second to give the motor time to physically reach 0
  delay(1000); 
}

void loop() {
  // 2. Gradually go to 180 degrees
  for (angle = 0; angle <= 180; angle += 1) { 
    myServo.write(angle);              
    delay(15); // Wait 15 milliseconds between each degree
  }
  
  // Optional: Wait at 180 degrees for 2 seconds
  delay(2000); 
  
  // Optional: Snap back to 0 degrees to repeat the cycle
  myServo.write(0);
  delay(1000);
  
  // NOTE: If you only want it to move exactly ONCE and then stop forever, 
  // remove the two lines above and uncomment the line below:
  // while(true); 
}