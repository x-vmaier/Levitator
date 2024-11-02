#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>

#include "USART.hpp"

const uint8_t START_DELIMITER = 0x7B; // '{'
const uint8_t SEPARATOR = 0x3A;       // ':'
const uint8_t END_DELIMITER = 0x7D;   // '}'
const uint8_t PACKET_SIZE = sizeof(Packet);

#define SERIAL_TX_BUFFER_SIZE 128
#define SERIAL_RX_BUFFER_SIZE 128

typedef uint8_t tx_buffer_index_t;
typedef uint8_t rx_buffer_index_t;

static volatile uint8_t tx_buffer[SERIAL_TX_BUFFER_SIZE];
static volatile tx_buffer_index_t tx_buffer_head = 0;
static volatile tx_buffer_index_t tx_buffer_tail = 0;

static volatile uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];
static volatile rx_buffer_index_t rx_buffer_head = 0;
static volatile rx_buffer_index_t rx_buffer_tail = 0;

static volatile bool data_available = false;

// USART initialization
void USART_init(uint32_t baud)
{
    uint16_t ubrr = (F_CPU / 16 / baud) - 1;                              // Calculate UBRR value for the desired baud rate
    UBRR0H = (ubrr >> 8);                                                 // Set the high byte of UBRR
    UBRR0L = ubrr;                                                        // Set the low byte of UBRR
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0); // Enable RX, TX, and RX interrupt
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                               // 8 data bits, no parity, 1 stop bit
}

// Check for available data in the RX buffer
uint8_t USART_available()
{
    return (rx_buffer_head - rx_buffer_tail + SERIAL_RX_BUFFER_SIZE) % SERIAL_RX_BUFFER_SIZE;
}

// Transmit a byte
void USART_transmit(uint8_t data)
{
    // Wait for empty transmit buffer
    while (((tx_buffer_head + 1) % SERIAL_TX_BUFFER_SIZE) == tx_buffer_tail); // Wait until there is space in the buffer

    // Add data to the transmit buffer
    tx_buffer[tx_buffer_head] = data;
    tx_buffer_head = (tx_buffer_head + 1) % SERIAL_TX_BUFFER_SIZE;

    // Enable the transmit interrupt
    UCSR0B |= (1 << UDRIE0);
}

// Flush the transmit buffer
void USART_flush(void)
{
    while (tx_buffer_head != tx_buffer_tail); // Wait until all data is sent
}

// Receive a byte
uint8_t USART_receive(void)
{
    if (rx_buffer_head == rx_buffer_tail)
    {
        return 0; // No data available
    }
    else
    {
        uint8_t data = rx_buffer[rx_buffer_tail];
        rx_buffer_tail = (rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
        return data;
    }
}

void USART_clear()
{
    tx_buffer_head = 0;
    tx_buffer_tail = 0;
    rx_buffer_head = 0;
    rx_buffer_tail = 0;
}

// Interrupt Service Routine for USART RX
ISR(USART_RX_vect)
{
    if (((rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE) != rx_buffer_tail)
    {
        uint8_t data = UDR0; // Read the received data
        rx_buffer[rx_buffer_head] = data;
        rx_buffer_head = (rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
        data_available = true; // Set flag indicating data is available
    }
}

// Interrupt Service Routine for USART TX
ISR(USART_UDRE_vect)
{
    if (tx_buffer_head != tx_buffer_tail)
    {
        // Send the next byte from the transmit buffer
        UDR0 = tx_buffer[tx_buffer_tail];
        tx_buffer_tail = (tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
    }
    else
    {
        // Disable the transmit interrupt if there is no data to send
        UCSR0B &= ~(1 << UDRIE0);
    }
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
