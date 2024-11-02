#include <avr/io.h>
#include <avr/eeprom.h>
#include <stdint.h>
#include <string.h>

#include "EEPROMHandler.hpp"

// EEPROM Addresses for PIDConfig
const uint8_t eePIDConfigAddress = 0;

// Function to store the entire PIDConfig struct to EEPROM
void EEPROMHandler::setPIDConfig(const PIDConfig &config)
{
    eeprom_update_block((const void *)&config, (void *)eePIDConfigAddress, sizeof(PIDConfig));
}

// Function to load the entire PIDConfig struct from EEPROM
void EEPROMHandler::getPIDConfig(PIDConfig &config)
{
    eeprom_read_block((void *)&config, (const void *)eePIDConfigAddress, sizeof(PIDConfig));

    // Check for the validity of the signature
    if (config.signature != CONFIG_SIGNATURE)
    {
        // EEPROM is not initialized or contains invalid data, set to defaults
        config.initDefaults();
        setPIDConfig(config); // Save defaults back to EEPROM
    }
}

// Function to clear the entire EEPROM
void EEPROMHandler::clear()
{
    uint8_t zero[EEPROM_SIZE] = {0};                   // Create a zero-filled array
    eeprom_update_block(zero, (void *)0, EEPROM_SIZE); // Clear EEPROM in one go
}
