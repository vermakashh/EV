// Arduino code for throttle control with Serial Monitor feedback
const int throttlePin = 9; // PWM pin connected to the throttle
int throttleValue = 0; // Initial throttle value (0 = stop)
const int maxThrottle = 255; // Maximum throttle value (8-bit PWM)

void setup() {
  Serial.begin(9600); // Start serial communication with Jetson Nano
  pinMode(throttlePin, OUTPUT); // Set the throttle pin as output
  analogWrite(throttlePin, throttleValue); // Start with throttle at 0 (stop)
  Serial.println("System initialized. Waiting for commands...");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read command from Jetson Nano
    Serial.print("Command received: ");
    Serial.println(command); // Print the received command

    if (command == 'W') {
      increaseThrottle(); // Increase throttle if 'W' command is received
    } else if (command == 'S') {
      stopThrottle(); // Stop throttle if 'S' command is received
    }

    // Update PWM output
    analogWrite(throttlePin, throttleValue);

    // Display the current throttle level after action
    Serial.print("Throttle value: ");
    Serial.println(throttleValue);
  }
}

void increaseThrottle() {
  throttleValue += 10; // Increase throttle by 10 units
  if (throttleValue > maxThrottle) {
    throttleValue = maxThrottle; // Cap the throttle at the maximum
  }
  Serial.println("Increasing throttle...");
}

void stopThrottle() {
  throttleValue = 0; // Set throttle to 0 (stop the vehicle)
  Serial.println("Throttle stopped.");
}
