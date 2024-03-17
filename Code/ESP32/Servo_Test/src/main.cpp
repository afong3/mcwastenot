#include <ESP32Servo.h>

// Define Servo objects
Servo myservo1; Servo myservo2; Servo myservo3; Servo myservo4;
Servo myservo5; Servo myservo6; Servo myservo7; Servo myservo8;

// Define servo pins
int servoPin1 = 3; int servoPin2 = 34; int servoPin3 = 9; int servoPin4 = 37;
int servoPin5 = 36; int servoPin6 = 7;
int angle = 85;

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo1.setPeriodHertz(50);
  
  // Attach servos to their pins
  myservo1.attach(servoPin1, 500, 2500);
  myservo2.attach(servoPin2, 500, 2500);
  myservo3.attach(servoPin3, 500, 2500);
  myservo4.attach(servoPin4, 500, 2500);
  myservo5.attach(servoPin5, 500, 2500);
  myservo6.attach(servoPin6, 500, 2500);

  myservo1.write(angle);
  myservo2.write(angle);
  myservo3.write(angle);
  myservo4.write(angle);
  myservo5.write(angle);
  myservo6.write(angle);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // Initialize the LED state
}

void loop() {
  digitalWrite(13, HIGH); // LED on
  delay(500); // Delay for half a second
  digitalWrite(13, LOW); // LED off
  delay(500); // Delay for half a second

  if (angle < 180) {
    for (angle = 0; angle <= 180; angle += 1) {
      myservo1.write(angle);
      myservo2.write(angle);
      myservo3.write(angle);
      myservo4.write(angle);
      myservo5.write(angle);
      myservo6.write(angle);
      delay(30);
    }
  }

}
