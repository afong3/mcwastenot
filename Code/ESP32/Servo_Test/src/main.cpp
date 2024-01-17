#include <ESP32Servo.h>

Servo myservo;  // Create a servo object

int servoPin = 4;  // GPIO pin where the servo is connected
int angle = 0;      // Servo position in degrees

void setup() {
    // Calibrate the pulse width range to 600-2400 microseconds
    myservo.attach(servoPin, 700, 2300);  
    myservo.write(90); // Set to middle position
}

void loop() {
    // Sweep from 0 to 180 degrees
    for (angle = 0; angle <= 180; angle += 1) {
        myservo.write(angle);  // Tell servo to go to position in variable 'angle'
        delay(15);             // Wait 15ms for the servo to reach the position
    }

    // Sweep back from 180 to 0 degrees
    for (angle = 180; angle >= 0; angle -= 1) {
        myservo.write(angle);  
        delay(15);             
    }
}
