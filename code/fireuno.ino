#include <Servo.h> // Include the Servo library

// Create two Servo objects
Servo servoX; // For the X-axis (pan) servo
Servo servoY; // For the Y-axis (tilt) servo

// Define the digital pins where your servos are connected
int servoXPin = 9;  // Connect X-axis servo signal wire to Digital Pin 9
int servoYPin = 10; // Connect Y-axis servo signal wire to Digital Pin 10

// Define the digital pin for the pump relay
// IMPORTANT: Make sure this matches the pin you connected to your relay module's IN pin.
int pumpPin = 7; // Connect Relay Module's IN pin to Digital Pin 7

// Define the state for the relay to turn the pump ON and OFF
// Most common 5V relay modules are active-low (LOW turns ON, HIGH turns OFF)
// If your pump stays ON constantly, or never turns ON, try flipping these two values.
const int PUMP_ON_STATE = LOW;   // Send LOW to IN pin to turn pump ON (for active-low relay)
const int PUMP_OFF_STATE = HIGH; // Send HIGH to IN pin to turn pump OFF (for active-low relay)

// Variables to store incoming angle commands and pump state
int incomingAngleX = 90; // Default center angle for X-axis
int incomingAngleY = 90; // Default center angle for Y-axis
int incomingPumpState = 0; // 0 for OFF, 1 for ON (as sent from Python)

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino Pan-Tilt & Pump Controller Ready!");

  // Attach the servo objects to their respective pins
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);

  // Set the pump control pin as an OUTPUT
  pinMode(pumpPin, OUTPUT);

  // Set initial position for both servos
  servoX.write(incomingAngleX);
  servoY.write(incomingAngleY);
  Serial.print("Initial servo positions: X=");
  Serial.print(incomingAngleX);
  Serial.print(", Y=");
  Serial.println(incomingAngleY);

  // Ensure pump is initially off
  digitalWrite(pumpPin, PUMP_OFF_STATE);
  Serial.println("Pump initialized to OFF.");
}

void loop() {
  // Check if data is available from the serial port
  if (Serial.available() > 0) {
    // Read the incoming string until a newline character ('\n') is received
    // Python sends angles as "X_angle,Y_angle,Pump_State\n"
    String data = Serial.readStringUntil('\n');

    // Find the position of the first comma (for X angle)
    int firstCommaIndex = data.indexOf(',');
    // Find the position of the second comma (for Y angle and Pump State)
    int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);

    // If both commas are found, parse the angles and pump state
    if (firstCommaIndex != -1 && secondCommaIndex != -1) {
      // Extract the substring for X-axis angle
      String angleXString = data.substring(0, firstCommaIndex);
      // Extract the substring for Y-axis angle
      String angleYString = data.substring(firstCommaIndex + 1, secondCommaIndex);
      // Extract the substring for Pump State
      String pumpStateString = data.substring(secondCommaIndex + 1);

      // Convert the extracted strings to integers
      incomingAngleX = angleXString.toInt();
      incomingAngleY = angleYString.toInt();
      incomingPumpState = pumpStateString.toInt(); // 0 or 1

      // Constrain angles to the valid servo range (0-180 degrees)
      incomingAngleX = constrain(incomingAngleX, 0, 180);
      incomingAngleY = constrain(incomingAngleY, 0, 180);

      // Write the new angles to the servos
      servoX.write(incomingAngleX);
      servoY.write(incomingAngleY);

      // Control the pump based on the incoming state
      if (incomingPumpState == 1) { // If Python sent 1 (ON)
        digitalWrite(pumpPin, PUMP_ON_STATE);
        // Serial.println("Pump ON"); // Uncomment for verbose pump logging on Arduino
      } else { // If Python sent 0 (OFF)
        digitalWrite(pumpPin, PUMP_OFF_STATE);
        // Serial.println("Pump OFF"); // Uncomment for verbose pump logging on Arduino
      }

      // Optional: Print the set values for debugging (useful to see if Arduino is receiving correctly)
      // Serial.print("Set X=");
      // Serial.print(incomingAngleX);
      // Serial.print(", Y=");
      // Serial.print(incomingAngleY);
      // Serial.print(", Pump State=");
      // Serial.println(incomingPumpState);
    } else {
      // Handle malformed data (e.g., if Python sends garbage)
      Serial.print("Invalid data format received: ");
      Serial.println(data);
    }
  }
}