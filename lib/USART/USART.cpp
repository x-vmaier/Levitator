#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "USART.hpp"

const uint8_t START_DELIMITER = 0x7B; // '{'
const uint8_t SEPARATOR = 0x3A;       // ':'
const uint8_t END_DELIMITER = 0x7D;   // '}'

const uint8_t PACKET_SIZE = sizeof(Packet); // Define expected packet size
static volatile uint8_t receivedBytes = 0;  // Number of bytes received

// USART initialization
void USART_init(unsigned long baud)
{
    uint16_t ubrr = (F_CPU + (baud * 8)) / (baud * 16) - 1; // Calculate UBRR value for the desired baud rate
    UBRR0H = (ubrr >> 8);                                   // Set the high byte of UBRR
    UBRR0L = ubrr;                                          // Set the low byte of UBRR
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);   // Enable RX, TX, and RX interrupt
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                 // 8 data bits, no parity, 1 stop bit
}

// USART transmit function
void USART_transmit(uint8_t data)
{
    while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
    UDR0 = data;                      // Send data
}

// USART receive function
uint8_t USART_receive()
{
    while (!(UCSR0A & (1 << RXC0))); // Wait for data to be received
    return UDR0;                     // Get and return received data from the buffer
}

// Interrupt Service Routine for USART RX
ISR(USART_RX_vect)
{
    // When a byte is received, increment the count
    if (receivedBytes < PACKET_SIZE)
    {
        USART_receive(); // Read the byte to clear the interrupt flag
        receivedBytes++; // Increment the received bytes counter
    }
}

// Return the number of available bytes
uint8_t USART_available()
{
    return receivedBytes; // Return the number of received bytes
}

// Reset the received bytes counter
void USART_clear()
{
    receivedBytes = 0; // Reset the count to zero
}

// Packet serialization
void serialize(Packet *packet, int identifier, float data)
{
    packet->start_delimiter = START_DELIMITER;
    packet->identifier = identifier;
    packet->separator = SEPARATOR;
    packet->data = data;
    packet->end_delimiter = END_DELIMITER;
}

// Packet deserialization
void deserialize(Packet *packet, uint8_t *buffer)
{
    packet->start_delimiter = buffer[0];
    packet->identifier = buffer[1];
    packet->separator = buffer[2];
    packet->data = *((float *)&buffer[3]);
    packet->end_delimiter = buffer[7];
}

// Send a packet
void sendPacket(int identifier, float data)
{
    Packet packet;
    serialize(&packet, identifier, data);

    for (uint8_t i = 0; i < PACKET_SIZE; i++)
    {
        USART_transmit(((uint8_t *)&packet)[i]);
    }
}

// Receive a packet
int receivePacket(Packet *packet)
{
    // Check if enough data is available
    if (USART_available() >= PACKET_SIZE)
    {
        uint8_t buffer[PACKET_SIZE];

        // Read the expected number of bytes
        for (uint8_t i = 0; i < PACKET_SIZE; i++)
        {
            buffer[i] = USART_receive();
        }

        // Deserialize the packet
        deserialize(packet, buffer);

        // Check packet integrity
        if (packet->start_delimiter == START_DELIMITER &&
            packet->end_delimiter == END_DELIMITER &&
            packet->separator == SEPARATOR)
        {
            USART_clear(); // Reset the count after successfully receiving the packet
            return true;   // Valid packet received
        }
    }
    return false; // No valid packet received
}
