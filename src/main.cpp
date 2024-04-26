#include <Arduino.h>
#include "EEPROMHandler.hpp"
#include "fastprotoc.hpp"

// Define whether the triangular signal is enabled
#define TRIANGLE_ENABLE

// Pin Definitions
#define HALL_SENSOR_PIN A0                                  // Analog pin for Hall sensor
#define COIL_OUTPUT_PIN 9                                   // Digital pin for coil output

// Serial Communication Standards
#define BAUD_RATE 115200
#define CONNECTION_UPDATE 0
#define SETPOINT_UPDATE 1
#define KP_UPDATE 2
#define KI_UPDATE 3
#define KD_UPDATE 4
#define HALL_UPDATE 5
#define PWM_UPDATE 6

// PID defaults
#define DEFAULT_SETPOINT 140.0f
#define DEFAULT_KP 130.0f
#define DEFAULT_KI 500.0f
#define DEFAULT_KD 100.0f

// Constants
const int eeSetpointAddress = 0;                            // EEPROM address of stored setpoint
const int eeKpAddress = sizeof(float);                      // EEPROM address of stored proportional gain
const int eeKiAddress = 2 * sizeof(float);                  // EEPROM address of stored integral gain
const int eeKdAddress = 3 * sizeof(float);                  // EEPROM address of stored derivative gain
const unsigned long PACKET_SEND_INTERVAL = 20;              // Interval for sending packets (in milliseconds)
const byte LED_PINS[] = {2, 3, 4, 5, 6, 7, 11, 12, 9, 8};   // Pins connected to LEDs
const int SETPOINT_LOW = 140;                               // Lower limit of the setpoint
const int SETPOINT_HIGH = 210;                              // Upper limit of the setpoint
const int INTEGRAL_SATURATION = 2000;                       // Integral saturation value
const unsigned int PID_LOOP_DELAY = 10;                     // Microseconds delay for PID loop
const float DT = 0.1;                                       // Time interval for derivative calculation
const int NUM_PERIODS = 50;                                 // Number of previous oscillation periods to consider
const unsigned long ADJUSTMENT_INTERVAL = 100;              // Time interval for adjusting PID parameters (in milliseconds)
const int HALL_VALUE_THRESHOLD = 340;                       // Threshold value for the Hall sensor indicating no object
const float TOLERANCE = 10.0;                               // Tolerance for considering the object to be stationary

// PID Values
float setpoint;                                             // Desired setpoint
float Kp;                                                   // Proportional gain
float Ki;                                                   // Integral gain
float Kd;                                                   // Derivative gain

float prevSetpoint = 0.0;                                   // Previous setpoint
float prevKp = 0.0;                                         // Previous proportional gain
float prevKi = 0.0;                                         // Previous integral gain
float prevKd = 0.0;                                         // Previous derivative gain

// EEPROM Addresses
EEPROMHandler eepromHandler;


// Variables
int hallValue = 0;                                          // Current Hall sensor reading
float output = 0.0;                                         // PID output value
float integral = 0.0;                                       // Integral term of PID controller
float derivative = 0.0;                                     // Derivative term of PID controller
float error = 0.0;                                          // Error between setpoint and measured value
float previousError = 0.0;                                  // Previous error for derivative calculation
float triangularSignalStep = 20.0;                          // Step value for triangular signal
unsigned long triangularSignalTime = 0;                     // Time for triangular signal update
unsigned long previousSignalTime = 0;                       // Time for previous signal update

// Function prototypes
void handleSerialInput();
void updatePID();

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(COIL_OUTPUT_PIN, OUTPUT);

  // Initialize LEDs
  for (byte i = 0; i < sizeof(LED_PINS); i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], HIGH);
    delay(40);
  }

  setpoint = eepromHandler.get(eeSetpointAddress, DEFAULT_SETPOINT);
  Kp = eepromHandler.get(eeKpAddress, DEFAULT_KP);
  Ki = eepromHandler.get(eeKiAddress, DEFAULT_KI);
  Kd = eepromHandler.get(eeKdAddress, DEFAULT_KD);

  // Turn off LEDs after initialization
  for (byte i = 0; i < sizeof(LED_PINS); i++) {
    digitalWrite(LED_PINS[i], LOW);
    delay(40);
  }
}

void loop() {
  handleSerialInput();                  // Check for serial input
  updatePID();                          // Update PID controller
  delayMicroseconds(PID_LOOP_DELAY);    // Introduce delay for PID loop
}

void handleSerialInput() {
  Packet packet;
  if (receivePacket(&packet)) {
    float data = packet.data;

    switch (packet.identifier) {
      case SETPOINT_UPDATE:
        setpoint = data;
        eepromHandler.set(eeSetpointAddress, setpoint);
        break;
      case KP_UPDATE:
        Kp = data;
        eepromHandler.set(eeKpAddress, Kp);
        break;
      case KI_UPDATE:
        Ki = data;
        eepromHandler.set(eeKiAddress, Ki);
        break;
      case KD_UPDATE:
        Kd = data;
        eepromHandler.set(eeKdAddress, Kd);
        break;
      default:
        break;
    }
  }
}

void updatePID() {
  unsigned long currentTime = millis();

#ifdef TRIANGLE_ENABLE
  // Update setpoint periodically if enabled
  if (currentTime - triangularSignalTime >= PACKET_SEND_INTERVAL) {
    triangularSignalTime = currentTime;
    setpoint += triangularSignalStep;

    if (setpoint >= SETPOINT_HIGH || setpoint <= SETPOINT_LOW)
      triangularSignalStep *= -1;

    sendPacket(SETPOINT_UPDATE, setpoint, updatePID);
  }
#endif

  // Read Hall sensor
  hallValue = analogRead(HALL_SENSOR_PIN);

  // Update the PID variables
  error = (setpoint - hallValue);
  integral = constrain(integral + error * DT, -INTEGRAL_SATURATION, INTEGRAL_SATURATION);
  derivative = (error - previousError) / DT;

  // Calculate the PWM value
  output = 0 - Kp * error - Ki * integral - Kd * derivative;
  previousError = error;

  // Truncate and limit
  int electromagnetSetting = constrain(output, 0, 255);

  // Turn off electromagnet if Hall sensor reading indicates no object
  if (hallValue > HALL_VALUE_THRESHOLD) {
    electromagnetSetting = 0;
  }

  // Apply the electromagnet setting
  analogWrite(COIL_OUTPUT_PIN, electromagnetSetting);

  // Send sensor and PWM updates periodically
  currentTime = millis();
  if (currentTime - previousSignalTime >= PACKET_SEND_INTERVAL) {
    sendPacket(HALL_UPDATE, hallValue, updatePID);
    sendPacket(PWM_UPDATE, electromagnetSetting, updatePID);
    previousSignalTime = currentTime;
  }

  // Send parameter updates if changed
  if (setpoint != prevSetpoint) sendPacket(SETPOINT_UPDATE, setpoint, updatePID);
  if (Kp != prevKp) sendPacket(KP_UPDATE, Kp, updatePID);
  if (Ki != prevKi) sendPacket(KI_UPDATE, Ki, updatePID);
  if (Kd != prevKd) sendPacket(KD_UPDATE, Kd, updatePID);

  // Update previous values
  prevSetpoint = setpoint;
  prevKp = Kp;
  prevKi = Ki;
  prevKd = Kd;
}
