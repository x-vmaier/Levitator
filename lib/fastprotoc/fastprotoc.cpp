#include "fastprotoc.hpp"

const char START_DELIMITER = '[';
const char SEPARATOR = ':';
const char END_DELIMITER = ']';

void serialize(Packet *packet, int identifier, int data) {
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
    packet->data = buffer[3];
    packet->end_delimiter = buffer[4];
}

void sendPacket(int identifier, int data, VoidFunctionPtr func) {
    if (Serial.available() > 0)
        // Call func to avoid blocking the main code
        while (Serial.available() > 0) func();
        
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
