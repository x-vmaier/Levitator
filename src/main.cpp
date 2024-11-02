#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "USART.hpp"
#include "EEPROMHandler.hpp"

// #define SERIAL_COM // Activate serial communication

// Configuration Macros
#define HALL_SENSOR_PIN PC0       // Analog pin for Hall sensor
#define COIL_OUTPUT_PIN PB1       // Digital pin for coil output
#define BAUD_RATE 115200          // UART speed
#define PID_INTERVAL 0.0005       // PID update interval
#define PWM_LIMIT 255             // PWM maximum limit
#define SETPOINT_LOW 100          // Minimum setpoint value
#define SETPOINT_HIGH 300         // Maximum setpoint value
#define HALL_THRESHOLD 340        // Hall sensor threshold
#define INTEGRAL_LIMIT 2000       // Integral windup limit
#define PACKET_INTERVAL 20        // Serial send interval in ms
#define EEPROM_WRITE_INTERVAL 500 // EEPROM update interval in ms

#define ADC_CHANNEL_MASK 0x0F
#define ADC_CHANNEL_SHIFT 4

// Packet Macros
#define SETPOINT_UPDATE 0 // Setpoint update packet id
#define KP_UPDATE 1       // Proportional gain update packet id
#define KI_UPDATE 2       // Integral gain update packet id
#define KD_UPDATE 3       // Derivative gain update packet id
#define HALL_UPDATE 4     // Hall update packet id
#define PWM_UPDATE 5      // PWM update packet id

// Configuration Variables
PIDConfig pidConfig; // PID configuration
const uint8_t LED_PINS[] = {2, 3, 4, 5, 6, 7, 11, 12, 9, 8};
const uint8_t LED_COUNT = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

// State Variables
float prevError = 0.0f, integral = 0.0f;
unsigned long lastPacketTime = 0, lastEEPROMTime = 0;
volatile uint8_t pwmOutput = 0;
volatile uint16_t hallSensorValue = 0;
volatile uint16_t latestADCValue = 0;
volatile unsigned long millisCounter = 0;

// Function Prototypes
void ADC_init();
void PWM_init();
void PID_init();
uint16_t readADC(uint8_t channel);
void setPWMDutyCycle(uint8_t dutyCycle);
void runningLight(uint8_t state);
void handleSerialInput();
void sendData();
void storeToEEPROM();
void loadFromEEPROM();

// Constrain function to limit values
inline float constrain(float value, float min, float max)
{
  return value < min ? min : (value > max ? max : value);
}

int main()
{
  USART_init(BAUD_RATE);
  ADC_init();
  PWM_init();
  PID_init();
  sei();

  runningLight(1);
  loadFromEEPROM(); // Load all PID config from EEPROM
  runningLight(0);

  while (1)
  {
#ifdef SERIAL_COM
    handleSerialInput();
    sendData();
#endif
  }
  return 0;
}

// PID Update Function
ISR(TIMER2_COMPA_vect)
{
  millisCounter++;

  // PID calculations
  hallSensorValue = readADC(HALL_SENSOR_PIN);
  float error = pidConfig.setpoint - hallSensorValue;
  integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative = error - prevError;
  prevError = error;

  float output = -(pidConfig.Kp * error + pidConfig.Ki * integral + pidConfig.Kd * derivative);
  pwmOutput = (hallSensorValue > HALL_THRESHOLD) ? 0 : constrain(output, 0, PWM_LIMIT);
  setPWMDutyCycle(pwmOutput);
}

// Initialization Functions
void ADC_init()
{
  ADMUX = (1 << REFS0) | (HALL_SENSOR_PIN & ADC_CHANNEL_MASK); // Set reference to AVcc and select channel
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1);                       // Set prescaler to 64 (250 kHz ADC clock)
  ADCSRA |= (1 << ADEN);                                       // Enable ADC
  ADCSRA |= (1 << ADIE);                                       // Enable ADC interrupt
}

void PWM_init()
{
  DDRB |= (1 << COIL_OUTPUT_PIN);                     // Set pin as output
  TCCR1A = (1 << COM1A1) | (1 << WGM11);              // Clear OC1A on compare match, set at BOTTOM, WGM mode 14
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10); // WGM mode 14, no prescaling
  ICR1 = 399;                                         // Set TOP value for ~40 kHz frequency (16 MHz / (399 + 1))
  OCR1A = 0;                                          // Start with 0% duty cycle
}

void PID_init()
{
  TCCR2A = (1 << WGM21);              // CTC mode
  TCCR2B = (1 << CS21) | (1 << CS20); // Prescaler of 64
  OCR2A = 249;                        // Set compare match value for 1ms (assuming 16MHz clock)
  TIMSK2 = (1 << OCIE2A);             // Enable Timer 2 compare interrupt
}

ISR(ADC_vect)
{
  latestADCValue = ADC;  // Read ADC value and store
  ADCSRA |= (1 << ADSC); // Start next conversion
}

uint16_t readADC(uint8_t channel)
{
  ADMUX = (ADMUX & 0xF0) | (channel & ADC_CHANNEL_MASK); // Select channel
  ADCSRA |= (1 << ADSC);                                 // Start conversion
  return latestADCValue;                                 // Return the latest available value
}

void setPWMDutyCycle(uint8_t dutyCycle)
{
  OCR1A = dutyCycle;
}

void runningLight(uint8_t state)
{
  for (uint8_t i = 0; i < LED_COUNT; i++)
  {
    uint8_t pin = LED_PINS[i];
    if (pin < 8)
    {
      DDRD |= (1 << pin);
      PORTD = state ? (PORTD | (1 << pin)) : (PORTD & ~(1 << pin));
    }
    else
    {
      DDRB |= (1 << (pin - 8));
      PORTB = state ? (PORTB | (1 << (pin - 8))) : (PORTB & ~(1 << (pin - 8)));
    }
    _delay_ms(40);
  }
}

void handleSerialInput()
{
  Packet packet;
  if (receivePacket(&packet))
  {
    float data = packet.data;
    switch (packet.identifier)
    {
    case SETPOINT_UPDATE:
      pidConfig.setpoint = constrain(data, SETPOINT_LOW, SETPOINT_HIGH);
      storeToEEPROM();
      break;
    case KP_UPDATE:
      pidConfig.Kp = data;
      storeToEEPROM();
      break;
    case KI_UPDATE:
      pidConfig.Ki = data;
      storeToEEPROM();
      break;
    case KD_UPDATE:
      pidConfig.Kd = data;
      storeToEEPROM();
      break;
    }
  }
}

void storeToEEPROM()
{
  unsigned long currentTime = millisCounter;
  if (currentTime - lastEEPROMTime >= EEPROM_WRITE_INTERVAL)
  {
    lastEEPROMTime = currentTime;
    EEPROMHandler::setPIDConfig(pidConfig); // Store the entire PID config
  }
}

void loadFromEEPROM()
{
  EEPROMHandler::getPIDConfig(pidConfig); // Load the entire PID config
}

void sendData()
{
  unsigned long currentTime = millisCounter;
  if (currentTime - lastPacketTime >= PACKET_INTERVAL)
  {
    sendPacket(HALL_UPDATE, hallSensorValue);
    sendPacket(PWM_UPDATE, pwmOutput);
    lastPacketTime = currentTime;
  }

  // Removed previous setpoint and gains tracking
  static float prevSetpoint = -1;
  static float prevKp = -1;
  static float prevKi = -1;
  static float prevKd = -1;

  if (pidConfig.setpoint != prevSetpoint)
    sendPacket(SETPOINT_UPDATE, pidConfig.setpoint);
  if (pidConfig.Kp != prevKp)
    sendPacket(KP_UPDATE, pidConfig.Kp);
  if (pidConfig.Ki != prevKi)
    sendPacket(KI_UPDATE, pidConfig.Ki);
  if (pidConfig.Kd != prevKd)
    sendPacket(KD_UPDATE, pidConfig.Kd);

  prevSetpoint = pidConfig.setpoint;
  prevKp = pidConfig.Kp;
  prevKi = pidConfig.Ki;
  prevKd = pidConfig.Kd;
}
