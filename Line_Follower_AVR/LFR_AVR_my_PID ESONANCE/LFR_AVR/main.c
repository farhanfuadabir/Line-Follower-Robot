/*
 * LFR_AVR.c
 *
 * ATmega32A, 3 Sonar, 8 IR sensor array
 * Fuse bits: Low Fuse: 3F, High Fuse: C9
 * Clock Frequency: 16 MHz
 *
 * Created: 6/20/2016 9:54:18 AM
 * Author : FUAD
 */ 

#define F_CPU 16000000UL

#define trig1_DDR				DDRD
#define trig1_PORT				PORTD
#define trig1_PIN				4
#define trig2_DDR				DDRD
#define trig2_PORT				PORTD
#define trig2_PIN				5
#define trig3_DDR				DDRB
#define trig3_PORT				PORTB
#define trig3_PIN				1
#define echo1_DDR				DDRD
#define echo1_PORT				PORTD
#define echo1_PIN				3
#define echo2_DDR				DDRD
#define echo2_PORT				PORTD
#define echo2_PIN				2
#define echo3_DDR				DDRB
#define echo3_PORT				PORTB
#define echo3_PIN				2
#define motor1a_DDR				DDRC
#define motor1a_PORT			PORTC
#define motor1a_PIN				7
#define motor1b_DDR				DDRC
#define motor1b_PORT			PORTC
#define motor1b_PIN				6
#define motor2a_DDR				DDRC
#define motor2a_PORT			PORTC
#define motor2a_PIN				5	
#define motor2b_DDR				DDRC
#define motor2b_PORT			PORTC
#define motor2b_PIN				4
#define enable1_DDR				DDRD
#define enable1_PORT			PORTD
#define enable1_PIN				7
#define enable2_DDR				DDRB
#define enable2_PORT			PORTB
#define enable2_PIN				3

#define maxSpeed				180 // was 210
#define targetValue				0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>
#include "serial_lib.h"
#include "ADC_routine.h"
#include "PWM_routine.h"

#define enable1					PWM0
#define enable2					PWM2

// const uint16_t compareValue[8] = {700, 700, 700, 700, 750, 620, 850, 580z}; 
const uint16_t compareValue[8] = {600, 600, 600, 600, 600, 600, 600, 450};
volatile int sensorValue = 0;
volatile signed int mappedValue = 0;
volatile signed int prev_mappedValue = 0;

volatile int pulse1 = 0;
volatile int choice1 = 0;
volatile int pulse2 = 0;
volatile int choice2 = 0;
volatile int pulse3 = 0;
volatile int choice3 = 0;
volatile int TIMER0_INT = 0;
volatile int TIMER2_INT = 0;
int distance1 = 0;
int distance2 = 0;
int distance3 = 0;

int8_t error = 0;
int8_t prev_error = 0;
int16_t sum_error = 0;
float correction = 0;
int16_t motorResponse = 0;
uint8_t LMspeed = 0; 
uint8_t RMspeed = 0;
uint8_t reverseSpeed = 0;
uint8_t inverseLevel = 0;
uint8_t prev_inverseLevel = 0;
uint8_t inverseFlag = 0;

// for maxSpeed 255:
// float Kp = 6.5;
// float Kd = 25;
// float Ki = 0;

// for maxSpeed 180-11.69V Final Flash 12V
float Kp = 2.6;
float Kd = 15;	//7
float Ki = 0.1;
 
// for maxSpeed 210 Final THis one i used
// float Kp = 3.8;
// float Kd = 3.8;
// float Ki = 0;

// for maxSpeed 210 Final
// float Kp = 5.5;
// float Kd = 15;
// float Ki = 0.1;

void sensorValueConversion(void);
void sensorMapping(void);
void PID_Correction(void);

void distanceMeasure(void);
void sonarGuideCave(void);
void TIMER0_2_INT_init(void);
void xINT_init(void);
void InterruptOff(void);

void Forward(void);
void Backward(void);
void Stop(void);
void RightSharp(void);
void LeftSharp(void);
void setRotationForward(void);
void setRotationRightSharp(void);
void setRotationLeftSharp(void);


int main(void)
{
	// Input-Output settings
	
	trig1_DDR |= (1 << trig1_PIN);
	trig2_DDR |= (1 << trig2_PIN);
	trig3_DDR |= (1 << trig3_PIN);
	echo1_DDR &= ~(1 << echo1_PIN);
	echo2_DDR &= ~(1 << echo2_PIN);
	echo3_DDR &= ~(1 << echo3_PIN);
	motor1a_DDR |= (1 << motor1a_PIN);
	motor1b_DDR |= (1 << motor1b_PIN);
	motor2a_DDR |= (1 << motor2a_PIN);
	motor2b_DDR |= (1 << motor2b_PIN);
	enable1_DDR |= (1 << enable1_PIN);
	enable2_DDR |= (1 << enable2_PIN);
	
	// Local Variables
	
//	uint8_t finishLevelCount = 0;
	
	// Initializations	
	
	adc_init();
	UART_init();
	PWM0_init();
	PWM2_init();
	setRotationForward();	
	Forward();
	enable1(maxSpeed);
	enable2(maxSpeed);
	
// 	xINT_init();
// 	TIMER0_2_INT_init();
// 	sei();
	Forward();
	_delay_ms(300);
//	int net_value = 0;
	
    while (1) 
    {
		
// 		for (int tempo = 0; tempo < 3; tempo++)
// 		{
			sensorValueConversion();
			sensorMapping();
// 			if (sensorValue == 0b11111111)
// 			{
// 				break;
// 			}
// 			net_value = net_value + mappedValue; 
// 		}
// 		if (sensorValue != 0b11111111)
// 		{
// 			mappedValue = net_value / 3;
// 		}
// 		
		if (mappedValue == 100 || mappedValue == -100)
		{
			for (int temp_loop = 0; temp_loop < 4;temp_loop++)
			{
				sensorValueConversion();
				sensorMapping();
				if (mappedValue != 100 && mappedValue != -100)
				{
					break;
				}
			}
		}
// 		net_value = 0;
		
// 		if (mappedValue == 100 || mappedValue == -100)
// 		{
// 			Stop();
// 			_delay_ms(300);
// 		}
// 				
// 		while (sensorValue == 0b00000000)			// No Line Sequence
// 		{
// // 			finishLevelCount++; 
// 			sensorValueConversion();
// // 			if(sensorValue != 0b00000000) finishLevelCount = 0;
// // 			if(finishLevelCount > 10) Stop();
// 		}
// 		
// // 		if (finishLevelCount > 10)
// // 		{
// // 			break;
// // 		}
// 
// 		while (sensorValue == 0b11111111)			// Horizontal Line Sequence
// 		{
// 			sensorValueConversion();
// 		}
//    		
// 		if (mappedValue == -100)									// Left 90 degree Turn Sequence
// 		{
// 			while (1)
// 			{
// 				sensorValueConversion();
// 				sensorMapping();
// 				if (sensorValue == 0b00000000)
// 				{
// 					mappedValue == 100;
// 					break;
// 				}
// 				else if (sensorValue == 0b11111111)
// 				{
// 					mappedValue = 0;
// 					break;
// 				}
// 			}
// 			PID_Correction();
// 			enable1(LMspeed);
// 			enable2(RMspeed);
// 			if (sensorValue == 0b11111111) _delay_ms(100);
// 		}
// 
// 
// 		else if (mappedValue == 100)								// Right 90 degree Turn Sequence
// 		{
// 			while (1)
// 			{
// 				sensorValueConversion();
// 				sensorMapping();
// 				if (sensorValue == 0b00000000) 
// 				{
// 					mappedValue == 100;
// 					break;
// 				}
// 				else if (sensorValue == 0b11111111) 
// 				{
// 					mappedValue = 0;
// 					break;
// 				}
// 			}
// 			
// 			PID_Correction();
// 			enable1(LMspeed);
// 			enable2(RMspeed);
// 			if (sensorValue == 0b11111111) _delay_ms(100);
// 
// 		}
// 		
// 		if (mappedValue == -110)														// Left Acute angle Turn Sequence
// 		{
// 			Stop();
// 			_delay_ms(200);
// 			while (mappedValue == -110)
// 			{
// 				sensorValueConversion();
// 				sensorMapping();
// 			}
// 			if (mappedValue == -120)
// 			{
// 				Forward();
// 				_delay_ms(400);
// 				mappedValue == 0;
// 			}
// 		}
// 
// 		else if (mappedValue == -120)														// Left Acute angle Turn Sequence
// 		{
// 			Stop();
// 			_delay_ms(200);
// 			while (mappedValue == -120)
// 			{
// 				sensorValueConversion();
// 				sensorMapping();
// 			}
// 			if (mappedValue == -110)
// 			{
// 				LeftSharp();
// 				_delay_ms(400);
// 				while (mappedValue != 0)
// 				{
// 					sensorValueConversion();
// 					sensorMapping();
// 				}
// 				sensorValueConversion();
// 				sensorMapping();
// 			}
// 			else mappedValue = 0;
// 		}
// 
// 		else if (mappedValue == 110)														// Right Acute angle Turn Sequence
// 		{
// 			Stop();
// 			_delay_ms(200);	
// 			while (mappedValue == 110)
// 			{
// 				sensorValueConversion();
// 				sensorMapping();
// 			}
// 			if (mappedValue == 120)
// 			{
// 				Forward();
// 				_delay_ms(400);
// 				mappedValue == 0;
// 			}
// 		}
// 
// 		else if (mappedValue == 120)														// Right Acute angle Turn Sequence
// 		{
// 			Stop();
// 			_delay_ms(200);
// 			while (mappedValue == 120)
// 			{
// 				sensorValueConversion();
// 				sensorMapping();
// 			}
// 			if (mappedValue == 110)
// 			{
// 				RightSharp();
// 				_delay_ms(400);
// 				while (mappedValue != 0)
// 				{
// 					sensorValueConversion();
// 					sensorMapping();
// 				}
// 				sensorValueConversion();
// 				sensorMapping();
// 			}
// 			else mappedValue = 0;
// 		}
//		
		if (mappedValue == .5)
		{
			Forward();
			_delay_ms(5000);
		}

		if (sensorValue == 0b00110111 || sensorValue == 0b11101100)
		{
			Forward();
			sensorValueConversion();
			sensorMapping();
		}


		if (mappedValue == 120/* || mappedValue == 110*/)
		{
// 			Stop();
// 			_delay_ms(500);
			for (int temp = 0; temp < 30; temp++)
			{
				sensorValueConversion();
				sensorMapping();
			}
			while (mappedValue == 120)
			{
				Stop();
				_delay_ms(200);
				RightSharp();
				_delay_ms(400);
				while(mappedValue > 15)
				{
					sensorValueConversion();
					sensorMapping();
				}
			}
		}
		
		if (mappedValue == -120/* || mappedValue == -110*/)
		{
// 			Stop();
// 			_delay_ms(500);
// 			for (int temp = 0; temp < 10; temp++)
// 			{
// 				sensorValueConversion();
// 				sensorMapping();
// 			}
			while (mappedValue == -120)
			{	
				Stop();
 				_delay_ms(200);
				LeftSharp();
				_delay_ms(400);
				while(mappedValue < -15)
				{
					sensorValueConversion();
					sensorMapping();
				}				
			}
		}
		
		
		PID_Correction();
		enable1(LMspeed);
		enable2(RMspeed);
		if (inverseLevel == prev_inverseLevel)
		{
			inverseLevel = 0;
		}
		else if (inverseLevel > 100)
		{
			inverseFlag = 1;
			inverseLevel = 0;
		}
		prev_inverseLevel = inverseLevel;
// 		Serial_sendInt(inverseLevel, DEC);
// 		Serial_sendString("\t");
// 		Serial_sendInt(inverseFlag, DEC);
// 		Serial_sendString("\n");
	}
}

// void sensorValueConversion(void)
// {
// 	int digitalValue = 0;
// 	sensorValue = 0;
// 	signed int sensorNumber = 0;
// 	
// 	for (sensorNumber = 7; sensorNumber >= 0; sensorNumber--)
// // 	for (sensorNumber = 0; sensorNumber <= 7; sensorNumber++)
// 	{
// 		int temp = adcRead(sensorNumber);
// 		if (temp < compareValue[sensorNumber]) digitalValue = 0;
// 		else digitalValue = 1;
// 		sensorValue |= (digitalValue << (7 - sensorNumber));
// // 		Serial_sendInt(temp, DEC);
// // 		Serial_sendString("\t");
// 	}
// // 	Serial_sendInt(sensorValue, BIN);
// // 	Serial_sendString("\n");
// }

void sensorValueConversion(void)
{
	int digitalValue = 0;
	sensorValue = 0;
	signed int sensorNumber = 0;
	if (inverseFlag == 0)
	{
		for (sensorNumber = 7; sensorNumber >= 0; sensorNumber--)
		// 	for (sensorNumber = 0; sensorNumber <= 7; sensorNumber++)
		{
			int temp = adcRead(sensorNumber);
			if (temp < compareValue[sensorNumber]) digitalValue = 0;
			else digitalValue = 1;
			sensorValue |= (digitalValue << (7 - sensorNumber));
// 	 		Serial_sendInt(temp, DEC);
// 	 		Serial_sendString("\t");
		}
// 	 	Serial_sendInt(sensorValue, BIN);
// 		Serial_sendString("\n");
	}
	else if (inverseFlag == 1)
	{
		for (sensorNumber = 7; sensorNumber >= 0; sensorNumber--)
		// 	for (sensorNumber = 0; sensorNumber <= 7; sensorNumber++)
		{
			int temp = adcRead(sensorNumber);
			if (temp > compareValue[sensorNumber]) digitalValue = 0;
			else digitalValue = 1;
			sensorValue |= (digitalValue << (7 - sensorNumber));
			// 		Serial_sendInt(temp, DEC);
			// 		Serial_sendString("\t");
		}
// 	Serial_sendInt(sensorValue, BIN);
// 	Serial_sendString("\t");
	}

	
}

void sensorMapping(void)
{
	prev_mappedValue = mappedValue;
	if (sensorValue == 0b00000000) mappedValue = prev_mappedValue;
	else if (sensorValue == 0b11111111) mappedValue = 120;
	
	else if ((sensorValue == 0b00011001) || (sensorValue == 0b00001001) || (sensorValue == 0b00010001) || (sensorValue == 0b00110001) || (sensorValue == 0b01110001) || (sensorValue == 0b01100001) || (sensorValue == 0b01100011)) mappedValue = 120;
	else if ((sensorValue == 0b00001011) || (sensorValue == 0b00010011) || (sensorValue == 0b00011011) || (sensorValue == 0b00001010) || (sensorValue == 0b00010010) || /*(sensorValue == 0b01110111) ||*/ (sensorValue == 0b01110011) ||/* eita (sensorValue == 0b00110111) ||*/ (sensorValue == 0b00011101) || (sensorValue == 0b01100111) || (sensorValue == 0b00001101) || (sensorValue == 0b00111001) || (sensorValue == 0b00110011)) mappedValue = 120;
	else if ((sensorValue == 0b00001111) || (sensorValue == 0b00011111 || sensorValue == 0b00111111 || sensorValue == 0b01111111)) mappedValue = 120;
	else if (sensorValue == 0b00000001) mappedValue = 70;
	else if (sensorValue == 0b00000011) mappedValue = 60;
	else if (sensorValue == 0b00000010 || sensorValue == 0b00000111 || sensorValue == 0b11111101) mappedValue = 30;
	else if (sensorValue == 0b00000110 || sensorValue == 0b11111001) mappedValue = 20;
	else if (sensorValue == 0b00000100 || sensorValue == 0b00001110) mappedValue = 15;
	else if (sensorValue == 0b11111011)
	{
		mappedValue = 15;
		inverseLevel++;
	}
	else if (sensorValue == 0b00001100) mappedValue = 10;
	else if (sensorValue == 0b11110011)
	{
		mappedValue = 10;
		inverseLevel++;
	}
	else if (sensorValue == 0b00001000 || sensorValue == 0b00011100) mappedValue = 5;
	else if (sensorValue == 0b11110111)
	{
		mappedValue = 5;
		inverseLevel++;
	}
	else if (sensorValue == 0b00011000 /*|| sensorValue == 0b11101110 || sensorValue == 0b01110111 || sensorValue == 0b01100110 || sensorValue == 0b10011001 || sensorValue == 0b00110111 || sensorValue == 0b11101100*/) mappedValue = 0;
	else if (sensorValue == 0b11100111)
	{
		mappedValue = 0;
		inverseLevel++;
	}
	else if (sensorValue == 0b11101110 || sensorValue == 0b01110111 || sensorValue == 0b01100110 || sensorValue == 0b10011001 /*|| sensorValue == 0b00110111 || sensorValue == 0b11101100*/ || sensorValue == 0b01111110 || sensorValue == 0b00111100 || sensorValue == 0b01111100 || sensorValue == 0b00111110 || sensorValue == 0b01110111 || sensorValue == 0b11101110)  mappedValue = .5;
	else if (sensorValue == 0b00010000 || sensorValue == 0b00111000) mappedValue = -5;
	else if (sensorValue == 0b11101111)
	{
		mappedValue = -5;
		inverseLevel++;
	}
	else if (sensorValue == 0b00110000) mappedValue = -10;
	else if (sensorValue == 0b11001111)
	{
		mappedValue = -10;
		inverseLevel++;
	}
	else if (sensorValue == 0b00100000 || sensorValue == 0b01110000) mappedValue = -15;
	else if (sensorValue == 0b11011111)
	{
		mappedValue = -15;
		inverseLevel++;
	}
	else if (sensorValue == 0b01100000 || sensorValue == 0b10011111) mappedValue = -20;
	else if (sensorValue == 0b01000000 || sensorValue == 0b11100000 || sensorValue == 0b10111111) mappedValue = -30;
	else if (sensorValue == 0b11000000) mappedValue = -60;
	else if (sensorValue == 0b10000000) mappedValue = -70;
	else if ((sensorValue == 0b11110000) || (sensorValue == 0b11111000 || sensorValue == 0b11111100 || sensorValue == 0b11111110)) mappedValue = -120;
	else if ((sensorValue == 0b11011000) || (sensorValue == 0b11010000) || (sensorValue == 0b11001000) || (sensorValue == 0b01010000) || (sensorValue == 0b01001000) || /*(sensorValue == 0b11101110) ||*/ (sensorValue == 0b11001110) /* eita || (sensorValue == 0b11101100)*/ || (sensorValue == 0b10111000) || (sensorValue == 0b11100110) || (sensorValue == 0b10110000) || (sensorValue == 0b10011100) || (sensorValue == 0b11001100)) mappedValue = -120;
	else if ((sensorValue == 0b10011000) || (sensorValue == 0b10010000) || (sensorValue == 0b10001000) || (sensorValue == 0b10001100) || (sensorValue == 0b10001110) || (sensorValue == 0b10000110) || (sensorValue == 0b11000110)) mappedValue = -120;
// 	Serial_sendInt(mappedValue, DEC);
// 	Serial_sendString("\n");
}

// void PID_Correction(void)
// {
// 	error = targetValue - mappedValue;
// 	sum_error = (sum_error + error) / 2;
// 	
// 	correction = ((Kp * error) + (Ki * sum_error) + (Kd * (error - prev_error)));
// 
// 	prev_error = error;
// 	motorResponse = (int)correction;
// 	
// // 	if(motorResponse > maxSpeed)
// // 	{
// // 		reverseSpeed = motorResponse - maxSpeed;
// // 		reverseSpeed = reverseSpeed * 15; 
// // 		if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
// // 		motorResponse = maxSpeed;
// // 		
// // 		setRotationLeftSharp();
// // 		
// // 		LMspeed = reverseSpeed;
// // 		RMspeed = maxSpeed;
// // 	}
// // 	
// // 	else if(motorResponse < -maxSpeed) 
// // 	{
// // 		reverseSpeed = -motorResponse - maxSpeed;
// // 		reverseSpeed = reverseSpeed * 15;
// // 		if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
// // 		motorResponse = -maxSpeed;
// // 		
// // 		setRotationRightSharp();
// // 		
// // 		LMspeed = maxSpeed;
// // 		RMspeed = reverseSpeed;
// // 	}
// 	
// 
// // 	if(motorResponse < maxSpeed && motorResponse > -maxSpeed)
// // 	{
// 		setRotationForward();
// 		
// 		LMspeed = maxSpeed - motorResponse;
// 		RMspeed = maxSpeed + motorResponse;
// //	}
// 	
// 	if(LMspeed > 254) LMspeed = 255;
// 	if(RMspeed > 254) LMspeed = 255;
// 	if(LMspeed < 0) LMspeed = 0;
// 	if(RMspeed < 0) LMspeed = 0;
// }

void PID_Correction(void) //my algo
{
	error = targetValue - mappedValue;
	sum_error = (sum_error + error) / 2;
	
	correction = ((Kp * error) + (Ki * sum_error) + (Kd * (error - prev_error)));

	prev_error = error;
	motorResponse = (int)correction;
	
	if(motorResponse > maxSpeed)
	{
		reverseSpeed = motorResponse - maxSpeed;
		if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
		motorResponse = maxSpeed;
		
		setRotationLeftSharp();
		
		LMspeed = reverseSpeed;
		RMspeed = maxSpeed;
	}
	
	else if(motorResponse < -maxSpeed)
	{
		reverseSpeed = -motorResponse - maxSpeed;
		if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
		motorResponse = -maxSpeed;
		
		setRotationRightSharp();
		
		LMspeed = maxSpeed;
		RMspeed = reverseSpeed;
	}
	
	else if(motorResponse < 0 && motorResponse > -maxSpeed)
	{
		setRotationForward();
		
		LMspeed = maxSpeed;
		RMspeed = maxSpeed + motorResponse;
	}
	else if(motorResponse > 0 && motorResponse < maxSpeed)
	{
		setRotationForward();
		
		LMspeed = maxSpeed - motorResponse;
		RMspeed = maxSpeed;
	}
}

void Stop(void)
{
	Backward();
	_delay_us(100);
	enable1(0);
	enable2(0);
	
	motor1a_PORT &= ~(1 << motor1a_PIN);
	motor1b_PORT &= ~(1 << motor1b_PIN);
	motor2a_PORT &= ~(1 << motor2a_PIN);
	motor2b_PORT &= ~(1 << motor2b_PIN);
}

void setRotationForward(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT |= (1 << motor1a_PIN);
	motor1b_PORT &= ~(1 << motor1b_PIN);
	motor2a_PORT |= (1 << motor2a_PIN);
	motor2b_PORT &= ~(1 << motor2b_PIN);
}

void setRotationLeftSharp(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT &= ~(1 << motor1a_PIN);
	motor1b_PORT |= (1 << motor1b_PIN);
	motor2a_PORT |= (1 << motor2a_PIN);
	motor2b_PORT &= ~(1 << motor2b_PIN);
}

void setRotationRightSharp(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT |= (1 << motor1a_PIN);
	motor1b_PORT &= ~(1 << motor1b_PIN);
	motor2a_PORT &= ~(1 << motor2a_PIN);
	motor2b_PORT |= (1 << motor2b_PIN);
}

void Forward(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT |= (1 << motor1a_PIN);
	motor1b_PORT &= ~(1 << motor1b_PIN);
	motor2a_PORT |= (1 << motor2a_PIN);
	motor2b_PORT &= ~(1 << motor2b_PIN);

	enable1(maxSpeed);
	enable2(maxSpeed);
}

void Backward(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT &= ~(1 << motor1a_PIN);
	motor1b_PORT |= (1 << motor1b_PIN);
	motor2a_PORT &= ~(1 << motor2a_PIN);
	motor2b_PORT |= (1 << motor2b_PIN);

	enable1(maxSpeed);
	enable2(maxSpeed);
}

void RightSharp(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT |= (1 << motor1a_PIN);
	motor1b_PORT &= ~(1 << motor1b_PIN);
	motor2a_PORT &= ~(1 << motor2a_PIN);
	motor2b_PORT |= (1 << motor2b_PIN);

	enable1(maxSpeed - 20);
	enable2(maxSpeed - 20);
}

void LeftSharp(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT &= ~(1 << motor1a_PIN);
	motor1b_PORT |= (1 << motor1b_PIN);
	motor2a_PORT |= (1 << motor2a_PIN);
	motor2b_PORT &= ~(1 << motor2b_PIN);

	enable1(maxSpeed - 20);
	enable2(maxSpeed - 20);
}

void sonarGuideCave(void)
{

	while (1)
	{
		distanceMeasure();
		Serial_sendInt(distance1, DEC);
		Serial_sendString("\t");
		Serial_sendInt(distance2, DEC);
		Serial_sendString("\t");
		Serial_sendInt(distance3, DEC);
		Serial_sendString("\n");
// 		sensorValueConversion();
// 		sensorMapping();
	}
// 	InterruptOff();
}

void distanceMeasure(void)
{	
	trig1_PORT &= ~(1 << trig1_PIN);
	trig2_PORT &= ~(1 << trig2_PIN);
	trig3_PORT &= ~(1 << trig3_PIN);
	trig1_PORT |= (1 << trig1_PIN);
	trig2_PORT |= (1 << trig2_PIN);
	trig3_PORT |= (1 << trig3_PIN);
	_delay_us(10);
	trig1_PORT &= ~(1 << trig1_PIN);
	trig2_PORT &= ~(1 << trig2_PIN);
	trig3_PORT &= ~(1 << trig3_PIN);
		
	distance1 = .0353 * pulse1 / 2;
	distance2 = .0353 * pulse2 / 2;
	distance3 = .0353 * pulse3 / 2;
		
}

void InterruptOff(void)
{
	cli();
	GICR &= ~((1 << INT0) | (1 << INT1) | (1 << INT2)); // Disable Interrupt 0, Interrupt 1, Interrupt 2
	TIMSK |= (1 << TOIE2) | (1 << TOIE0);				// Disable Timer Overflow Interrupt 0, Timer Overflow Interrupt 2
}

void xINT_init(void)
{
	MCUCR |= (1 << ISC00);								// Setting Interrupt 0 for any change
	MCUCR |= (1 << ISC10);								// Setting Interrupt 1 for any change
	MCUCSR |= (1 << ISC2);								// Setting Interrupt 2 for rising edge
	GICR = (1 << INT0) | (1 << INT1) | (1 << INT2);		// Enable Interrupt 0, Interrupt 1, Interrupt 2
}

void TIMER0_2_INT_init(void)
{
	TIMSK |= (1 << TOIE2) | (1 << TOIE0);				// Enable Timer Overflow Interrupt 0, Timer Overflow Interrupt 2
}

ISR (INT0_vect)
{
	if (choice1 == 0)
	{
		TCCR1B |= (1 << CS11);
		choice1 = 1;
	}
	else if (choice1 == 1)
	{
		TCCR1B = 0;
		pulse1 = TCNT1 / 2;
		TCNT1 = 0;
		choice1 = 0;
	}
}

ISR (INT1_vect)
{
	if (choice2 == 0)
	{
		TCCR0 |= 0b00000011;
		choice2 = 1;
	}
	else if (choice2 == 1)
	{
		TCCR0 = 0;
		pulse2 = (TCNT0 + (0xFF * TIMER0_INT)) * 4;
		TIMER0_INT = 0;
		TCNT0 = 0;
		choice2 = 0;
	}
}

ISR (INT2_vect)
{
	MCUCSR ^= (1 << ISC2);
	if (choice3 == 0)
	{
		TCCR2 |= 0b00000100;
		choice3 = 1;
	}
	else if (choice3 == 1)
	{
		TCCR2 = 0;
		pulse3 = (TCNT2 + (0xFF * TIMER2_INT)) * 4;
		TIMER2_INT = 0;
		TCNT2 = 0;
		choice3 = 0;
	}
}

ISR (TIMER2_OVF_vect)
{
	TIMER2_INT++;
}

ISR (TIMER0_OVF_vect)
{
	TIMER0_INT++;
}



