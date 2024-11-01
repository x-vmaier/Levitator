#ifndef EEPROMHANDLER_H
#define EEPROMHANDLER_H

#define EEPROM_SIZE 1024

// PID Gains and Setpoint
struct PIDConfig
{
  float setpoint = 140.0f;
  float Kp = 130.0f;
  float Ki = 500.0f;
  float Kd = 100.0f;
};

class EEPROMHandler
{
public:
    // Store the entire PIDConfig struct to EEPROM
    static void setPIDConfig(const PIDConfig &config);

    // Load the entire PIDConfig struct from EEPROM
    static void getPIDConfig(PIDConfig &config);

    // Clear the entire EEPROM
    static void clear();
};

#endif // EEPROMHANDLER_H
