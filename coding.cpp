#include <SoftwareSerial.h>  
// Include library to enable communication with the Bluetooth module.

// Solenoid pins
#define solenoid_in 11
#define solenoid_out 12
// Define solenoid control pins for input and output.

// YF-S201 Flow Sensors
#define FLOW_SENSOR_PIN 2 //out
#define FLOW_SENSOR_2_PIN 3
// Define pins connected to the two flow sensors.

// Variables to hold the flow pulse count, updated by interrupt.

float flowRate = 0;
float flowRate2 = 0;
// Variables to store flow rate values for the two sensors.

int leakage = 0;
// Variable to track if a leak is detected.

#define switch_1 13
// Define a pin for a switch and initialize the switch state.

#define SensorPin A0
float calibration_value = 22.09;
// Define pH sensor pin and its calibration value.

unsigned long int avgValue;
// Variable to store average reading from the pH sensor.

int buf[10], temp;
// Buffer and temporary variable to store pH sensor readings for averaging.

const int trigPin = 5; //Vcc - brown , trig - blue , echo - white , gnd - green
const int echoPin = 6;
// Define the trigger and echo pins for the ultrasonic sensor.

long duration;
int distance;
// Variables to store the duration of the ultrasonic pulse and the calculated distance.

SoftwareSerial Bluetooth(8, 9);//rx,tx
// Initialize software serial communication for HC-05 Bluetooth module on pins 8 (RX) and 9 (TX).

// Flow sensor interrupt handlers
void countFlow() {
  flowCount++;
}
// Interrupt service routine (ISR) for the first flow sensor. Each pulse increases flowCount.

void countFlow2() {
  flowCount2++;
}
// ISR for the second flow sensor.

float measureDistance() {
  // Measure the distance using the HC-SR04 ultrasonic sensor.

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Clear the trigger pin (set LOW for 2 microseconds).

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Send a 10 microsecond pulse on the trigger pin.

  long duration = pulseIn(echoPin, HIGH);
  // Measure the duration of the echo pulse.

  float distance = duration * 0.0343 / 2;
  // Convert the duration into distance (using the speed of sound = 343 m/s).

  return distance;
  // Return the calculated distance.
}

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);
  // Start serial communication at 9600 baud for debugging and Bluetooth.

  pinMode(SensorPin, INPUT);
  // Set the pH sensor pin as input.

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // Set ultrasonic sensor pins as output (trigger) and input (echo).

  pinMode(FLOW_SENSOR_PIN, INPUT);
  pinMode(FLOW_SENSOR_2_PIN, INPUT);
  // Set flow sensor pins as input.

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), countFlow, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_2_PIN), countFlow2, RISING);
  // Attach interrupts to both flow sensors to count pulses on rising edge.

  pinMode(solenoid_in, OUTPUT);
  pinMode(solenoid_out, OUTPUT);
  // Set solenoid control pins as output.

  pinMode(switch_1, INPUT_PULLUP);
  // Set the switch pin as input with an internal pull-up resistor.
}

void loop() {
  // Reading and averaging pH sensor values.
  for (int i = 0; i < 10; i++) {
    buf[i] = analogRead(SensorPin);
    delay(10);
  }
  // Read pH sensor values into the buffer array 10 times.

  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  // Sort the buffer array to smooth the readings.

  avgValue = 0;
  for (int i = 2; i < 8; i++) avgValue += buf[i];
  // Calculate the average value of the center readings (excluding extremes).

  float phValue = (float)avgValue * 5.0 / 1024 / 6;
  phValue = -5.70 * phValue + calibration_value;
  // Convert the average reading into a pH value using calibration.

  Serial.print("pH Value: ");
  Serial.println(phValue);
  Bluetooth.print("pH Value: ");
  Bluetooth.println(phValue);
  // Print the pH value to both serial monitor and Bluetooth.

  if (leakage == 0) {
    // If no leakage is detected, check water conditions and the ultrasonic sensor.

    if (phValue >= 6 && phValue < 8) {
      Serial.println("Water conditions OK");
      Bluetooth.println("Water conditions OK");
      // If pH value is within the acceptable range, print message.

      float distance1 = measureDistance();
      Serial.print("Distance: ");
      Serial.println(distance1);
      Bluetooth.print("Distance: ");
      Bluetooth.println(distance1);
      // Measure and print the distance using the ultrasonic sensor.

      flowRate = (flowCount / 7.5);
      Serial.print("Flow Rate: ");
      Serial.println(flowRate);
      Bluetooth.print("Flow Rate: ");
      Bluetooth.println(flowRate);
      // Calculate and print the flow rate based on pulse count.

      if (distance1 < 9) {
        Serial.println("Tank is full");
        Bluetooth.println("Tank is full");
        digitalWrite(solenoid_in, HIGH);
        // If the tank is full (distance < 9cm), 

        delay(2000);
        float distance2 = measureDistance();
        float distanceDiff = distance1 - distance2;
        // Measure again after 2 seconds to check for leaks based on the distance difference.

        if (flowRate == 0) {
          if (distanceDiff > 1) {
            Serial.println("Leak detected");
            Bluetooth.println("Leak detected");
            leakage = 1;
            digitalWrite(solenoid_out, LOW);
            digitalWrite(solenoid_in, HIGH);
            // If the flow is zero but the water level drops, a leak is detected. Activate solenoid_out.

          } else {
            Serial.println("No leak detected");
            Bluetooth.println("No leak detected");
            // No leak detected based on the distance difference.
          }
        }
      }
      //when tank is not full
       else {
        digitalWrite(solenoid_in, LOW);
        digitalWrite(solenoid_out, HIGH);
        // If the tank is not full, open the solenoid to let water in.

        flowRate2 = (flowCount2 / 7.5); // 7.5 pulsr per second
        Serial.println(flowRate2);
        if (flowRate2 == 0) {
          Serial.println("Water is not entering");
          Bluetooth.println("Water is not entering");
          // If flow rate from the second sensor is zero, water is not coming in.
        }

        float flowDiff = flowRate - flowRate2;
        if (flowDiff > 0.5) { //minutes per litre
          Serial.println("Warning leak!");
          Bluetooth.println("Warning leak!");
          // If there is a significant difference between the flow rates, issue a leak warning.

        } else {
          Serial.println("No leak");
          Bluetooth.println("No leak");
          // No significant difference in flow rates, no leak detected.
        }
      }
    }//if PH value not between 6-8
     else {
      if (digitalRead(switch_1) == LOW) {
        while (true) {
          float distance1 = measureDistance();
          if (distance1 < 9) {
            break;
          }
          digitalWrite(solenoid_in, LOW);
          digitalWrite(solenoid_out, HIGH);
          // If water is unusable (pH not within range), wait until the tank is empty.
        }
      }

      Serial.println("Unusable water");
      Bluetooth.println("Unusable water");
      digitalWrite(solenoid_in, HIGH);
      digitalWrite(solenoid_out, HIGH);
      // If the pH value is out of range, indicate that water is unusable and close both solenoids.
    }
  }//when there is a leakage
  else {
    digitalWrite(solenoid_in, HIGH);
    digitalWrite(solenoid_out, LOW);
    Serial.println("Danger leak!");
    Bluetooth.println("Danger leak!");
    // If a leak is detected, keep solenoid_in active and solenoid_out inactive.

    if (digitalRead(switch_1) == LOW) leakage = 0;
    // Reset leakage state if the switch is pressed.
  }

  flowCount = 0;
  flowCount2 = 0;
  // Reset flow counts for the next iteration.

  delay(1500);
  // Delay 1.5 seconds before the next loop iteration.
}