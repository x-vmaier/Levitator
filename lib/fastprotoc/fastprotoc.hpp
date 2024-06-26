/**
 * @file fastprotoc.hpp
 * @brief Header file for fastprotoc library.
 * 
 * This file declares functions and structures for serializing,
 * deserializing, sending, and receiving packets over serial communication.
 */

#pragma once

#include "Arduino.h"

/**
 * @brief Represents a packet structure for communication.
 * 
 * This struct defines the structure of a packet used for communication.
 * It includes start and end delimiters, identifier, separator, and data.
 */
typedef struct {
    uint8_t start_delimiter;    // Start of packet delimiter
    uint8_t identifier;         // Identifier of the packet
    uint8_t separator;          // Separator between identifier and data
    float data;                 // Data payload of the packet
    uint8_t end_delimiter;      // End of packet delimiter
} Packet;

/**
 * @brief Serializes data into a packet.
 * 
 * @param packet Pointer to the packet structure to be populated.
 * @param identifier Identifier of the packet.
 * @param data Data to be included in the packet.
 */
void serialize(Packet *packet, int identifier, float data);

/**
 * @brief Deserializes a packet from a buffer.
 * 
 * @param packet Pointer to the packet structure to be populated.
 * @param buffer Buffer containing the serialized packet data.
 */
void deserialize(Packet *packet, int *buffer);

/**
 * @brief Sends a packet over serial communication.
 * 
 * This function sends a packet over the serial communication channel.
 * It first checks if there is any incoming serial data, then serializes
 * the packet and sends it over serial.
 * 
 * @param identifier Identifier of the packet.
 * @param data Data to be included in the packet.
 */
void sendPacket(int identifier, float data);

/**
 * @brief Receives a packet from serial communication.
 * 
 * This function receives a packet from the serial communication channel.
 * It reads incoming serial data, deserializes the packet, and validates it.
 * 
 * @param packet Pointer to the packet structure to store the received packet.
 * @return true if a valid packet is received, false otherwise.
 */
bool receivePacket(Packet *packet);
