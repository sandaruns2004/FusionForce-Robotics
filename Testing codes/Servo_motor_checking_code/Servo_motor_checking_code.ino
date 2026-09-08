#include <Servo.h>

Servo myServo;  // Create servo object to control a servo
Servo myServo2; // ADDED: Create a second servo object
Servo myServo3;
Servo myServo4;
Servo myServo5;
Servo myServo6;
Servo myServo7;
Servo myServo8;
Servo myServo9;
Servo myServo10;
Servo myServo11;
Servo myServo12;

const int servoPin = 9;  // Pin connected to the servo signal wire
const int servoPin2 = 10; // ADDED: Pin connected to the second servo signal wire
const int servoPin3 = 11;
const int servoPin4 = 12;
const int servoPin5 = 8;
const int servoPin6 = 7;
const int servoPin7 = 6;
const int servoPin8 = 5;
const int servoPin9 = 4;
const int servoPin10 = 3;
const int servoPin11 = 2;
const int servoPin12 = 13;

int angle = 0;          // Variable to store the servo position

void setup() {
  myServo.attach(servoPin); 
  myServo2.attach(servoPin2); 
  myServo3.attach(servoPin3);
  myServo4.attach(servoPin4);
  myServo5.attach(servoPin5);
  myServo6.attach(servoPin6);
  myServo7.attach(servoPin7);
  myServo8.attach(servoPin8);
  myServo9.attach(servoPin9);
  myServo10.attach(servoPin10);
  myServo10.attach(servoPin11);
  myServo10.attach(servoPin12);

  // 1. Get the servo motor to the initial position of 0 degrees
  myServo.write(45); //FL 001
  myServo2.write(180); //FL 003 
  myServo3.write(100); //FL 002 
  myServo4.write(135); //FR 001
  myServo5.write(80); //FR 002
  myServo6.write(0); //FR 003 
  myServo7.write(135); //BL 001
  myServo8.write(80); //BL 002 
  myServo9.write(0); //BL 003 
  myServo10.write(0); //BR 001
  myServo11.write(0); //BR 002
  myServo12.write(0); //BR 003


  // Wait 1 second to give the motor time to physically reach 0
  delay(1000); 
}

void loop() {
  // 2. Gradually go to 180 degrees
  
  for (angle = 0; angle <= 90; angle += 1) { 
    myServo11.write(angle);              
    delay(15); // Wait 15 milliseconds between each degree
  }
  delay(100);
    for (angle = 90; angle >= 0; angle -= 1) { 
    myServo11.write(angle);              
    delay(15); // Wait 15 milliseconds between each degree
  }
  // delay(100);
  //   for (angle = 180; angle >= 0; angle -= 1) {            
  //   myServo7.write(angle); // ADDED: Move second servo along with the first
  //   delay(15); // Wait 15 milliseconds between each degree
  // }
  // delay(100);
  //   for (angle = 0; angle <= 180; angle += 1) { 
  //   myServo7.write(angle); // ADDED: Move second servo along with the first             
  //   delay(15); // Wait 15 milliseconds between each degree
  // }
  //   for (angle = 100; angle >= 50; angle -= 1) { 
    
  //   // 1. Write the current angle to the first servo
  //   myServo3.write(angle);              
    
  //   // 2. Calculate and write the scaled angle for the third servo
  //   int angle3 = map(angle, 100, 50, 180, 150);
  //   myServo2.write(angle3);
    
  //   delay(15); // Wait 15 milliseconds between each step
  // }
  // delay(100);
  // for (angle = 80; angle <= 120; angle += 1) { 
    
  //   // 1. Write the current angle to the first servo
  //   //myServo3.write(angle);              
  //   myServo5.write(angle);
    
  //   // 2. Calculate and write the scaled angle for the third servo
  //   int angle3 = map(angle, 80, 120, 130, 150);
  //   //myServo2.write(angle3);
  //   myServo6.write(angle3);
    
  //   delay(15); // Wait 15 milliseconds between each step
  // }

// ONE master loop controlling all four motors at the same time
//     for (angle = 100; angle >= 50; angle -= 1) { 
    
//     // 1. Write the master angle directly to myServo3
//     myServo3.write(angle);              
    
//     // 2. Map all the other servos based on myServo3's current position
//     int angle2 = map(angle, 100, 50, 180, 150); // myServo2 goes 180 to 150
//     int angle5 = map(angle, 100, 50, 70, 120);  // myServo5 goes 70 to 120
//     int angle6 = map(angle, 100, 50, 0, 30);    // myServo6 goes 0 to 30
//     int angle8 = map(angle, 100, 50, 80, 130);  // myServo8 goes 80 to 130
//     int angle9 = map(angle, 100, 50, 0, 30);    // myServo9 goes 0 to 30
    
//     // 3. Command the other five servos
//     myServo2.write(angle2);
//     myServo5.write(angle5);
//     myServo6.write(angle6);
//     myServo8.write(angle8); // Fixed: was writing to myServo6
//     myServo9.write(angle9); // Fixed: was writing to myServo6
    
//     delay(15); // Wait 15 milliseconds between each step
// }
//   // Optional: Wait at 180 degrees for 2 seconds
//   delay(2000); 
//     for (angle = 50; angle <= 100; angle += 1) { 
    
//     // 1. Write the master angle directly to myServo3
//     myServo3.write(angle);              
    
//     // 2. Map all the other servos based on myServo3's current position
//     // (Note: The comments have been updated to match the actual mapped values)
//     int angle2 = map(angle, 50, 100, 150, 180); // myServo2 goes 150 to 180
//     int angle5 = map(angle, 50, 100, 120, 70);  // myServo5 goes 120 to 70
//     int angle6 = map(angle, 50, 100, 30, 0);    // myServo6 goes 30 to 0
//     int angle8 = map(angle, 50, 100, 130, 80);    // myServo6 goes 30 to 0
//     int angle9 = map(angle, 50, 100, 30, 0);    // myServo6 goes 30 to 0
    
//     // 3. Command the other three servos
//     myServo2.write(angle2);
//     myServo5.write(angle5);
//     myServo6.write(angle6);
//     myServo8.write(angle8);
//     myServo9.write(angle9);
    
//     delay(15); // Wait 15 milliseconds between each step
//   }
  // Optional: Snap back to 0 degrees to repeat the cycle
  myServo.write(45);
  myServo2.write(180); // ADDED: Get second servo to initial position
  myServo3.write(100);
  myServo4.write(135);
  myServo5.write(80);
  myServo6.write(0); // ADDED: Snap back second servo
  myServo7.write(135); //BL 001 
  myServo8.write(80);
  myServo9.write(0);
  myServo10.write(0);
  myServo11.write(0);
  myServo12.write(0);

  delay(1000);
  
  // NOTE: If you only want it to move exactly ONCE and then stop forever, 
  // remove the two lines above and uncomment the line below:
  // while(true); 
}