#include <Arduino.h>
#include "fastprotoc.hpp"

// Pin Definitions
#define HallsensorPin A0                                  // Analog pin for Hall sensor
#define CoilOutputPin 9                                   // Digital pin for coil output

// Serial Communication Standarts
#define BAUD_RATE 115200
#define CONNECTION_UPDATE = 0
#define SETPOINT_UPDATE 1
#define KP_UPDATE 2
#define KI_UPDATE 3
#define KD_UPDATE 4
#define HALL_UPDATE 5
#define PWM_UPDATE 6

// Constants
const byte ledPins[] = {2, 3, 4, 5, 6, 7, 11, 12, 9, 8};  // Pins connected to LEDs
const int setpointLow = 140;                              // Lower limit of the setpoint
const int setpointHigh = 210;                             // Upper limit of the setpoint
const int integralSaturation = 2000;                      // Integral saturation value
const unsigned int pidLoopDelay = 10;                     // Microseconds delay for PID loop
const float dt = 0.1;                                     // Time interval for derivative calculation
const int numPeriods = 50;                                // Number of previous oscillation periods to consider
const unsigned long adjustmentInterval = 100;             // Time interval for adjusting PID parameters (in milliseconds)
const int hallValueThreshold = 340;                       // Threshold value for the Hall sensor indicating no object
const float tolerance = 10.0;                             // Tolerance for considering the object to be stationary

// PID Values
float setpoint = 140.0;                                   // Desired setpoint
float Kp = 100.0;                                         // Proportional gain
float Ki = 500.0;                                         // Integral gain
float Kd = 130.0;                                         // Derivative gain

float prevSetpoint = 0.0;
float prevKp = 0.0;
float prevKi = 0.0;
float prevKd = 0.0;

// Variables
int hallValue = 0;                                        // Current Hall sensor reading
float output = 0.0;                                       // PID output value
float integral = 0.0;                                     // Integral term of PID controller
float derivative = 0.0;                                   // Derivative term of PID controller
float error = 0.0;                                        // Error between setpoint and measured value
float previousError = 0.0;                                // Previous error for derivative calculation
unsigned long previousSignalTime = 0;

// Function prototypes
void handleSerialInput();
void updatePID();

void setup() {
  Serial.begin(BAUD_RATE);

  // Initialize coil
  pinMode(CoilOutputPin, OUTPUT);

  // Initialize LED pins
  for (byte i = 0; i < sizeof(ledPins); i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], HIGH);
    delay(40);
  }

  for (byte i = 0; i < sizeof(ledPins); i++) {
    digitalWrite(ledPins[i], LOW);
    delay(40);
  }
}

void loop() {
  handleSerialInput();              // Check for serial input
  updatePID();                      // Update PID controller
  delayMicroseconds(pidLoopDelay);  // Introduce delay for PID loop
}

// Handle serial input
void handleSerialInput() {
  Packet packet;
  if (receivePacket(&packet)) {
    float data = packet.data;

    switch (packet.identifier) {
      case SETPOINT_UPDATE:
        setpoint = data;
        break;
      case KP_UPDATE:
        Kp = data;
        break;
      case KI_UPDATE:
        Ki = data;
        break;
      case KD_UPDATE:
        Kd = data;
        break;
      default:
        break;
    }
  }
}

// Update the PID controller
void updatePID() {
#ifdef TriangleEnable
  // Update setpoint every 20ms if enabled
  unsigned long currentTime = millis();
  if (currentTime - triangularSignalTime >= 20) {
    triangularSignalTime = currentTime;
    setpoint += triangularSignalStep;

    if (setpoint >= setpointHigh || setpoint <= setpointLow)
      triangularSignalStep *= -1;

    //Serial.print("Setpoint: ");
    //Srial.println(setpoint);
  }
#endif

  // Read Hall sensor
  hallValue = analogRead(HallsensorPin);

  // Update the PID variables
  error = (setpoint - hallValue);
  integral = constrain(integral + error * dt, (integralSaturation * -1), integralSaturation);
  derivative = (error - previousError) / dt;

  // Calculate the PWM value
  output = 0 - Kp * error - Ki * integral - Kd * derivative;
  previousError = error;

  // Truncate and limit
  int electromagnetSetting = constrain(output, 0, 255);

  // Turn off electromagnet if Hall sensor reading indicates no object
  if (hallValue > 340) {
    electromagnetSetting = 0;
  }

  // Apply the electromagnet setting
  analogWrite(CoilOutputPin, electromagnetSetting);

  unsigned long currentTime = millis();
  if (currentTime - previousSignalTime >= 20) {
    sendPacket(HALL_UPDATE, hallValue, updatePID);
    sendPacket(PWM_UPDATE, electromagnetSetting, updatePID);
    previousSignalTime = currentTime;
  }

  if (setpoint != prevSetpoint) sendPacket(SETPOINT_UPDATE, setpoint, updatePID);
  if (Kp != prevKp) sendPacket(KP_UPDATE, Kp, updatePID);
  if (Ki != prevKi) sendPacket(KI_UPDATE, Ki, updatePID);
  if (Kd != prevKd) sendPacket(KD_UPDATE, Kd, updatePID);

  prevSetpoint = setpoint;
  prevKp = Kp;
  prevKi = Ki;
  prevKd = Kd;
}