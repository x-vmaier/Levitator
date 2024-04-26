#pragma once

#include <Arduino.h>
#include <EEPROM.h>

/**
 * @class EEPROMHandler
 * @brief Class for handling EEPROM operations.
 * 
 * This class provides methods for initializing, reading, writing,
 * and clearing data in EEPROM.
 */
class EEPROMHandler {
public:
    /**
     * @brief Constructor for EEPROMHandler class.
     */
    EEPROMHandler() {}

    /**
     * @brief Writes a variable to EEPROM at a specified address.
     * 
     * This method writes a variable to EEPROM at the specified address.
     * It only writes to EEPROM if the new value is different from the
     * existing value at the address.
     * 
     * @tparam T The data type of the variable.
     * @param address The EEPROM address to write the variable.
     * @param variable The variable to write.
     */
    template<typename T>
    void set(int address, const T& variable);

    /**
     * @brief Reads a variable from EEPROM at a specified address.
     * 
     * This method reads a variable from EEPROM at the specified address
     * and returns its value. If the EEPROM address has not been written
     * before, it returns the defaultValue.
     * 
     * @tparam T The data type of the variable.
     * @param address The EEPROM address to read the variable.
     * @param defaultValue The default value to return if the EEPROM address has not been written before.
     * @return The value read from EEPROM or defaultValue if the address has not been written before.
     */
    template<typename T>
    T get(int address, const T& defaultValue);

    /**
     * @brief Clears the entire EEPROM.
     * 
     * This method clears all data stored in EEPROM by writing zeros
     * to all EEPROM addresses.
     */
    void clear();
};
