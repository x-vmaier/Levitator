#ifndef USART_H
#define USART_H

#include <stdint.h>

typedef struct
{
    uint8_t start_delimiter;
    int identifier;
    uint8_t separator;
    float data;
    uint8_t end_delimiter;
} Packet;

void USART_init(unsigned long baud);
void USART_transmit(uint8_t data);
uint8_t USART_receive();
uint8_t USART_available();
void USART_clear();

void serialize(Packet *packet, int identifier, float data);
void deserialize(Packet *packet, uint8_t *buffer);
void sendPacket(int identifier, float data);
int receivePacket(Packet *packet);

#endif // USART_H
