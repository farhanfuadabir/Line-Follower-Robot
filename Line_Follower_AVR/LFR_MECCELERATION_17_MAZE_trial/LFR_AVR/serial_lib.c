#include "serial_lib.h"

void UART_init(void)
{
	UBRR0H = 0;
	UBRR0L = 129;
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	UCSR0C |= (1 << USBS0) | (3 << UCSZ00);
}

// void serial.sendInt(unsigned int data)
// {
// 	while (!(UCSRA & (1 << UDRE)));
// 	
// 	UCSRB &= ~(1 << TXB8);
// 	
// 	if (data & 0x0100) UCSRB |= (1 << TXB8);
// 	
// 	UDR = data;
// }

void Serial_sendInt(unsigned int data, int intBase)
{
	while (!(UCSR0A & (1 << UDRE0)));
	char intBuffer[20];
	itoa(data, intBuffer, intBase);
	Serial_sendString(intBuffer);
}

void Serial_sendIntLN(unsigned int data, int intBase)
{
	while (!(UCSR0A & (1 << UDRE0)));
	char intBuffer[20];
	itoa(data, intBuffer, intBase);
	Serial_sendString(intBuffer);
	Serial_sendString("\n");
}

void Serial_sendIntTAB(unsigned int data, int intBase)
{
	while (!(UCSR0A & (1 << UDRE0)));
	char intBuffer[20];
	itoa(data, intBuffer, intBase);
	Serial_sendString(intBuffer);
	Serial_sendString("\t");
}

void Serial_sendChar(char c)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void Serial_sendString(char *s)
{
	for(int i = 0; s[i] != '\0'; i++)
	{
		Serial_sendChar(s[i]);
	}
}
void Serial_sendStringLN(char *s)
{
	for(int i = 0; s[i] != '\0'; i++)
	{
		Serial_sendChar(s[i]);
	}
	Serial_sendString("\n");
}


unsigned char Serial_receive(void)
{
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}