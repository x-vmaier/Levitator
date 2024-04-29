/**
 * @file fastprotoc.cpp
 * @brief Source file for fastprotoc library.
 * 
 * This file defines functions for serializing, deserializing, sending, and receiving packets
 * over serial communication.
 */

#include "fastprotoc.hpp"

const uint8_t START_DELIMITER = 0x7B;   // '{'
const uint8_t SEPARATOR = 0x3A;         // ':'
const uint8_t END_DELIMITER = 0x7D;     // '}'

void serialize(Packet *packet, int identifier, float data) {
    packet->start_delimiter = START_DELIMITER;
    packet->identifier = identifier;
    packet->separator = SEPARATOR;
    packet->data = data;
    packet->end_delimiter = END_DELIMITER;
}

void deserialize(Packet *packet, int *buffer) {
    packet->start_delimiter = buffer[0];
    packet->identifier = buffer[1];
    packet->separator = buffer[2];
    packet->data = *(float*)&buffer[3];
    packet->end_delimiter = buffer[7];
}

void sendPacket(int identifier, float data) {
    if (Serial.available() > 0)
        while (Serial.available() > 0) continue;
        
    Packet packet;
    serialize(&packet, identifier, data);
    Serial.write((char*)&packet, sizeof(Packet));
}

bool receivePacket(Packet *packet) {
    if (Serial.available() >= static_cast<int>(sizeof(Packet))) {
        Serial.readBytes((char*)packet, sizeof(Packet));
        if (packet->start_delimiter == START_DELIMITER && packet->separator == SEPARATOR && packet->end_delimiter == END_DELIMITER) {
            return true;
        }
    }
    return false;
}
