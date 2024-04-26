#include "EEPROMHandler.hpp"

void EEPROMHandler::clear() {
    for (int i = 0; i < EEPROM.length(); ++i) {
        EEPROM.update(i, 0);
    }
}

template<typename T>
void EEPROMHandler::set(int address, const T& variable) {
    T buffer;
    EEPROM.get(address, buffer);
    if (variable != buffer) {
        EEPROM.put(address, variable);
    }
}

template<typename T>
T EEPROMHandler::get(int address, const T& defaultValue) {
    T buffer;
    EEPROM.get(address, buffer);
    if (isnan(buffer)) {
        set(address, defaultValue);
        return defaultValue;
    } else {
        return buffer;
    }
}
