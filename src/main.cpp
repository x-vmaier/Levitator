/**
 * @file main.cpp
 * @brief Main Arduino sketch file for controlling an electromagnet system.
 * 
 * This sketch controls an electromagnet system using a PID controller.
 * It receives input via serial communication and updates the setpoint,
 * proportional, integral, and derivative gains accordingly.
 */

#include <Arduino.h>
#include "EEPROMHandler.hpp"
#include "fastprotoc.hpp"

//#define TRIANGLE_ENABLE                                   // EXPERIMENTAL: Enable triangle waveform generation
#define HALL_SENSOR_PIN A0                                  // Analog pin for Hall sensor
#define COIL_OUTPUT_PIN 9                                   // Digital pin for coil output

#define BAUD_RATE 115200                                    // Monitor speed
#define SETPOINT_UPDATE 0                                   // Setpoint update packet id
#define KP_UPDATE 1                                         // Proportional gain update packet id
#define KI_UPDATE 2                                         // Integral gain update packet id
#define KD_UPDATE 3                                         // Derivative gain update packet id
#define HALL_UPDATE 4                                       // Hall update packet id
#define PWM_UPDATE 5                                        // PWM update packet id

#define DEFAULT_SETPOINT 140.0f                             // Default setpoint value
#define DEFAULT_KP 130.0f                                   // Default proportional gain
#define DEFAULT_KI 500.0f                                   // Default integral gain
#define DEFAULT_KD 100.0f                                   // Default derivative gain

const int eeSetpointAddress = 0;                            // EEPROM address of stored setpoint
const int eeKpAddress = sizeof(float);                      // EEPROM address of stored proportional gain
const int eeKiAddress = 2 * sizeof(float);                  // EEPROM address of stored integral gain
const int eeKdAddress = 3 * sizeof(float);                  // EEPROM address of stored derivative gain
const byte LED_PINS[] = {2, 3, 4, 5, 6, 7, 11, 12, 9, 8};   // Pins connected to LEDs
const int SETPOINT_LOW = 140;                               // Lower limit of the setpoint
const int SETPOINT_HIGH = 210;                              // Upper limit of the setpoint
const int INTEGRAL_SATURATION = 2000;                       // Integral saturation value
const float DT = 0.1;                                       // Time interval for derivative calculation
const unsigned long PACKET_SEND_INTERVAL = 20;              // Interval for sending packets (in milliseconds)
const unsigned long EEPROM_WRITE_INTERVAL = 500;            // Interval for writing to EEPROM (in milliseconds)
const int HALL_VALUE_THRESHOLD = 340;                       // Threshold value for the Hall sensor indicating no object

float setpoint;                                             // Desired setpoint
float Kp;                                                   // Proportional gain
float Ki;                                                   // Integral gain
float Kd;                                                   // Derivative gain

float prevSetpoint = 0.0;                                   // Previous setpoint
float prevKp = 0.0;                                         // Previous proportional gain
float prevKi = 0.0;                                         // Previous integral gain
float prevKd = 0.0;                                         // Previous derivative gain

int hallValue = 0;                                          // Current Hall sensor reading
float output = 0.0;                                         // PID output value
float integral = 0.0;                                       // Integral term of PID controller
float derivative = 0.0;                                     // Derivative term of PID controller
float error = 0.0;                                          // Error between setpoint and measured value
float previousError = 0.0;                                  // Previous error for derivative calculation
float triangularSignalStep = 20.0;                          // Step value for triangular signal
unsigned long triangularSignalTime = 0;                     // Time for triangular signal update
unsigned long previousSignalTime = 0;                       // Time for previous signal update
unsigned long previousEEPROMTime = 0;                       // Time for previous EEPROM write

EEPROMHandler eepromHandler;

void handleSerialInput();
void updatePID();

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(COIL_OUTPUT_PIN, OUTPUT);

  for (byte i = 0; i < sizeof(LED_PINS); i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], HIGH);
    delay(40);
  }

  setpoint = eepromHandler.get(eeSetpointAddress, DEFAULT_SETPOINT);
  Kp = eepromHandler.get(eeKpAddress, DEFAULT_KP);
  Ki = eepromHandler.get(eeKiAddress, DEFAULT_KI);
  Kd = eepromHandler.get(eeKdAddress, DEFAULT_KD);

  for (byte i = 0; i < sizeof(LED_PINS); i++) {
    digitalWrite(LED_PINS[i], LOW);
    delay(40);
  }
}

void loop() {
  handleSerialInput();
  updatePID();
}

/**
 * @brief Handles incoming serial input.
 * 
 * This function reads incoming serial data, updates system parameters
 * accordingly based on the received packet identifier, and stores them
 * to the EEPROM at a certain interval.
 */
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
        eepromHandler.set(eeKiAddress, Ki);
        break;
      case KD_UPDATE:
        Kd = data;
        eepromHandler.set(eeKdAddress, Kd);
        break;
      default:
        break;
    }

    unsigned long currentTime = millis();
    if (currentTime - previousEEPROMTime < EEPROM_WRITE_INTERVAL) return;
    previousEEPROMTime = currentTime;

    if (setpoint != prevSetpoint) eepromHandler.set(eeSetpointAddress, setpoint);
    if (Kp != prevKp) eepromHandler.set(eeKpAddress, Kp);
    if (Ki != prevKi) eepromHandler.set(eeKiAddress, Ki);
    if (Kd != prevKd) eepromHandler.set(eeKdAddress, Kd);
  }
}

/**
 * @brief Updates the PID controller.
 * 
 * This function calculates the output of the PID controller based on
 * the current setpoint, proportional, integral, and derivative gains.
 * It also updates the electromagnet setting and sends update packets
 * periodically over serial communication.
 */
void updatePID() {
  unsigned long currentTime = millis();

#ifdef TRIANGLE_ENABLE
  if (currentTime - triangularSignalTime >= PACKET_SEND_INTERVAL) {
    triangularSignalTime = currentTime;
    setpoint += triangularSignalStep;

    if (setpoint >= SETPOINT_HIGH || setpoint <= SETPOINT_LOW)
      triangularSignalStep *= -1;

    sendPacket(SETPOINT_UPDATE, setpoint);
  }
#endif

  hallValue = analogRead(HALL_SENSOR_PIN);

  error = (setpoint - hallValue);
  integral = constrain(integral + error * DT, -INTEGRAL_SATURATION, INTEGRAL_SATURATION);
  derivative = (error - previousError) / DT;

  output = 0 - Kp * error - Ki * integral - Kd * derivative;
  int electromagnetSetting = constrain(output, 0, 255);
  if (hallValue > HALL_VALUE_THRESHOLD)
    electromagnetSetting = 0;

  analogWrite(COIL_OUTPUT_PIN, electromagnetSetting);

  currentTime = millis();
  if (currentTime - previousSignalTime >= PACKET_SEND_INTERVAL) {
    sendPacket(HALL_UPDATE, hallValue);
    sendPacket(PWM_UPDATE, electromagnetSetting);
    previousSignalTime = currentTime;
  }

  if (setpoint != prevSetpoint) sendPacket(SETPOINT_UPDATE, setpoint);
  if (Kp != prevKp) sendPacket(KP_UPDATE, Kp);
  if (Ki != prevKi) sendPacket(KI_UPDATE, Ki);
  if (Kd != prevKd) sendPacket(KD_UPDATE, Kd);

  previousError = error;
  prevSetpoint = setpoint;
  prevKp = Kp;
  prevKi = Ki;
  prevKd = Kd;
}
