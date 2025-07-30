#include <Servo.h>

// === PIN DEFINITIONS ===
// You can change these to any valid digital pin numbers as needed
const int ch1_pin = 3;     // CH1: Steering input from RC receiver (PWM signal)
const int ch2_pin = 4;     // CH2: Throttle input from RC receiver (PWM signal)
const int ch3_pin = 5;     // CH3: Mode switch or auxiliary input (PWM signal)
const int servo_pin = 2;   // Output PWM pin to control steering servo (can change to any PWM-capable pin: 3, 5, 6, 9, 10, 11 on UNO)

// === GLOBAL VARIABLES ===
Servo steering_servo;      // Servo object to control the steering
String input_line = "";    // Buffer for incoming serial data
int servo_angle = 90;      // Initial steering angle (neutral position)

void setup() {
  // Initialize serial communication (must match ROS 2 serial node baud rate)
  Serial.begin(57600);

  // Set RC input pins as INPUT
  pinMode(ch1_pin, INPUT);
  pinMode(ch2_pin, INPUT);
  pinMode(ch3_pin, INPUT);

  // Attach servo to the defined PWM pin and set initial position
  steering_servo.attach(servo_pin);
  steering_servo.write(servo_angle);
}

void loop() {
  // === READ RC INPUT FROM RECEIVER ===
  // Use pulseIn() to measure high pulse width (in microseconds)
  uint32_t rc1 = pulseIn(ch1_pin, HIGH, 25000);  // Read CH1 (Throttle)
  uint32_t rc2 = pulseIn(ch2_pin, HIGH, 25000);  // Read CH2 (Steering)
  uint32_t rc3 = pulseIn(ch3_pin, HIGH, 25000);  // Read CH3 (Mode)

  // If all pulse widths are valid, send them over serial
  if (rc1 > 900 && rc1 < 2100 &&
      rc2 > 900 && rc2 < 2100 &&
      rc3 > 900 && rc3 < 2100) {

    // Format: CH1:xxxx CH2:xxxx CH3:xxxx
    Serial.print("CH1:");
    Serial.print(rc1);
    Serial.print(" CH2:");
    Serial.print(rc2);
    Serial.print(" CH3:");
    Serial.println(rc3);
  }

  // === READ SERIAL COMMAND (e.g., "ANGLE:120\n") ===
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(input_line);
      input_line = "";  // Reset buffer
    } else {
      input_line += c;  // Accumulate input characters
    }
  }

  delay(20);  // Maintain ~50 Hz loop rate
}

// === PARSE AND EXECUTE COMMANDS RECEIVED VIA SERIAL ===
void processCommand(const String& line) {
  if (line.startsWith("ANGLE:")) {
    int angle = line.substring(6).toInt();  // Extract integer after "ANGLE:"
    if (angle >= 0 && angle <= 180) {
      servo_angle = angle;                  // Update global angle
      steering_servo.write(servo_angle);    // Command servo to new angle
    }
  }
}
