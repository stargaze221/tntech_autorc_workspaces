#include <Servo.h>

Servo steeringServo;  // Create a Servo object

void setup() {
  Serial.begin(9600);
  steeringServo.attach(9);  // Connect servo to pin D9
  Serial.println("Starting servo steering test...");
}

void loop() {
  // Sweep from left to right
  for (int angle = 0; angle <= 180; angle += 10) {
    steeringServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(500);
  }

  // Sweep back from right to left
  for (int angle = 180; angle >= 0; angle -= 10) {
    steeringServo.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(500);
  }

  delay(1000);  // Short pause before repeating
}
