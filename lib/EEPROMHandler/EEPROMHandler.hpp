#ifndef EEPROMHANDLER_H
#define EEPROMHANDLER_H

#define EEPROM_SIZE 1024
#define CONFIG_SIGNATURE 0xDEADBEEF

// PID Gains and Setpoint
struct PIDConfig
{
  uint32_t signature; // Magic number for validation
  float setpoint;
  float Kp;
  float Ki;
  float Kd;

  void initDefaults()
  {
    signature = CONFIG_SIGNATURE;
    setpoint = 140.0f;
    Kp = 130.0f;
    Ki = 500.0f;
    Kd = 100.0f;
  }
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
