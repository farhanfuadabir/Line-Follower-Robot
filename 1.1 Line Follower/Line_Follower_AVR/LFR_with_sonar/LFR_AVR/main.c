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

#define maxSpeed				200
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

#define rightAngleConfidence	20
#define acuteAngleConfidence	20

// uint8_t maxSpeed = 200;			//170

const uint16_t compareValueWhiteSurface[8] = {550, 600, 600, 600, 600, 600, 600, 450};
// 	const uint16_t compareValue[8] = {550, 650, 650, 650, 650, 650, 650, 450};

// const uint16_t compareValueBlackSurface[8] = {950, 970, 970, 970, 970, 970, 970, 750};


volatile int sensorValue = 0;
volatile signed int mappedValue = 0;
volatile signed int prev_mappedValue = 0;

volatile int pulse1 = 0;
volatile int pulse2 = 0;
volatile int pulse3 = 0;
volatile int choice3 = 0;
volatile int TIMER0_INT = 0;
volatile int TIMER2_INT = 0;
uint16_t distance1 = 0;
int distance2 = 0;
int distance3 = 0;
uint8_t echo = 0;
uint8_t trig1_ack = 0;
uint8_t trig2_ack = 0;

int8_t error = 0;
int8_t prev_error = 0;
int16_t sum_error = 0;
float correction = 0;
int16_t motorResponse = 0;
uint8_t LMspeed = 0; 
uint8_t RMspeed = 0;
uint8_t reverseSpeed = 0;
// uint8_t inverseLevel = 0;
// uint8_t prev_inverseLevel = 0;
// uint8_t inverseFlag = 0;
uint8_t exception = 0;
uint8_t exception_1_Count = 0;
uint8_t exception_2_Count = 0;
int weightedValue = 0;
int prev_weightedValue = 0;
uint8_t weightCount = 0;
uint16_t straightLineCount = 0;
uint8_t allValueCount = 0;
uint8_t allValueCountFlag = 0;
uint8_t inverseLevel = 0;
uint8_t prev_inverseLevel = 0;
uint8_t inverseFlag = 0;
uint8_t acuteFlag = 0;
uint8_t rightFlag = 0;
uint8_t reverseFlag = 0;
uint8_t sonar1_ack = 0;
uint8_t sonar2_ack = 0;

// for maxSpeed 255 Final Sumo 12V
// float Kp = 8;
// float Kd = 0;
// float Ki = 0;
 
// for maxSpeed 255 Final Flash 6V
// float Kp = 6.5;
// float Kd = 3.3;
// float Ki = 0;


// for maxSpeed 170-11.69V Final Flash 12V 
// float Kp = 2.5;
// float Kd = 8;	//7
// float Ki = 0.1;

// for maxSpeed 180-11.69V Final Flash 12V ******** final silo eta
// float Kp = 2.6;
// float Kd = 15;	//7
// float Ki = 0.1;

// for maxSpeed 180-direct Final Flash 12V ******** final 
float Kp = 5;
float Kd = 17;	//7
float Ki = 0;

// for maxSpeed 200-direct Final Flash 12V ******** final
// float Kp = 2.5;
// float Kd = 25;	//7
// float Ki = 0;

// for maxSpeed 190-direct Final Flash 12V ******** final
// float Kp = 2.3;
// float Kd = 20;	//7
// float Ki = 0;


// for maxSpeed 200-11.69V Final Flash 12V
// float Kp = 2.8;
// float Kd = 25;	//7
// float Ki = 0.3;


// for maxSpeed 255 Final Flash 6V
// float Kp = 4;
// float Kd = 3.8;
// float Ki = 0;

// for maxSpeed 255 Final Flash 12V
// float Kp = 8;
// float Kd = 4;
// float Ki = 0;


void sensorValueConversion(void);
void sensorMapping(void);
void PID_Correction(void);
void exceptionHandling (void);

void distanceMeasure(void);
void sonarGuideCave(void);
void TIMER0_2_INT_init(void);
void xINT_init(void);
void InterruptOff(void);

void triggerSonar1(void);
void triggerSonar2(void);

void Forward(void);
void Backward(void);
void Stop(void);
void RightSharp(void);
void LeftSharp(void);
void setRotationForward(void);
void setRotationRightSharp(void);
void setRotationLeftSharp(void);

uint16_t sonar1Read(void);
uint16_t sonar2Read(void);



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
	
// 	uint8_t priorMappedValue = 0;
	
	uint8_t confidenceLevel = 0;
	
	// Initializations	
	
// 	adc_init();
// 	xINT_init();
// // 	TIMER0_2_INT_init();
// 	sei();

	UART_init();
	PWM0_init();
	PWM2_init();
	setRotationForward();	

// 	Forward();
// 	enable1(maxSpeed);
// 	enable2(maxSpeed);
	
	Forward();
// 	enable1(255);
// 	enable2(255);
// 	_delay_ms(500);
	
	
    while (1) 
    {	
		Serial_sendInt(sonar1Read(), DEC);
		Serial_sendString("\t");

		_delay_ms(50);
		
// 		Serial_sendInt(sonar2Read(), DEC);
// 		Serial_sendString("\n");
		


// 		PORTD = (1 << 3);
// 		Serial_sendInt(PORTD, BIN);
// 		Serial_sendString("\t");
		
// 		sensorValueConversion();
// 		sensorMapping();
// 		if (trig1_ack == 0)
// 		{
// 			triggerSonar1();
// 		}
// 		
// 		if (trig1_ack == 1 && sonar1_ack == 1)
// 		{
// 			distance1 = .0353 * pulse1 / 2;
// 			sonar1_ack = 0;
// 			Serial_sendString("D1 : ");
// 			Serial_sendInt(distance1, DEC);
// 			Serial_sendString("\n");
// 		}
		
		
// 		Serial_sendString("D1 : ");
// 		Serial_sendString("\n");
		
// 		if (trig2_ack == 0)
// 		{
// 			triggerSonar2();
// 		}
// 		
// 		else if (trig2_ack == 1 && sonar2_ack == 1)
// 		{
// 			distance2 = .0353 * pulse1 / 2;
// 			sonar2_ack = 0;
// 		}
//		distanceMeasure();
// 		sonarGuideCave();
// 		PID_Correction();
// 		enable1(LMspeed);
// 		enable2(RMspeed);

	}
}

uint16_t sonar1Read(void)
{
	sonar1_ack = 0;
	trig1_PORT &= ~(1 << trig1_PIN);
	trig1_PORT |= (1 << trig1_PIN);
	_delay_us(10);
	trig1_PORT &= ~(1 << trig1_PIN);
	
	while (sonar1_ack == 0)
	{
		TCCR1B |= (1 << CS11);
		if ((PIND & (1 << PIND3)))
		{
			TCNT1 = 0;
			TCCR1B |= (1 << CS11);
			while ((PIND & (1 << PIND3)))
			{
				if (TCNT1 > 5000)
				{
					TCCR1B = 0;
					TCNT1 = 0;

					return 111;
				}
			}
			TCCR1B = 0;
			pulse1 = TCNT1 / 2;
			TCNT1 = 0;
			sonar1_ack = 1;
		}
		else if (TCNT1 > 10000)
		{
			TCCR1B = 0;
			TCNT1 = 0;
			return 112;
		}

	}
	return (.035 * pulse1 / 2);
}

uint16_t sonar2Read(void)
{
	sonar2_ack = 0;
	trig2_PORT &= ~(1 << trig2_PIN);
	trig2_PORT |= (1 << trig2_PIN);
	_delay_us(10);
	trig2_PORT &= ~(1 << trig2_PIN);
	
	while (sonar2_ack == 0)
	{
		TCCR1B |= (1 << CS11);

		if ((PIND & (1 << PIND2)))
		{
// 			TCCR1B |= (1 << CS11);
			TCNT1 = 0;
			while ((PIND & (1 << PIND2)))
			{
				if (TCNT1 > 5000)
				{
					TCCR1B = 0;
					TCNT1 = 0;
					return 111;
				}
			}
			TCCR1B = 0;
			pulse2 = TCNT1 / 2;
			TCNT1 = 0;
			sonar2_ack = 1;
		}

		else if (TCNT1 > 10000)
		{
			TCCR1B = 0;
			TCNT1 = 0;
			return 112;
		}

	}
	return (.035 * pulse2 / 2);
}
void sensorValueConversion(void)
{
	int digitalValue = 0;
	sensorValue = 0;
	signed int sensorNumber = 0;
	weightedValue = 0;
	weightCount = 0;
	int8_t currentWeight = 0;
	int8_t weightRange = 0;
	int8_t maxWeight = 0;
	int8_t minWeight = 0;
	int8_t weightDiv = 0;
	int8_t weightTemp = 0;
	int8_t zero_count = 0;
	
	exception = 0;
	
	for (sensorNumber = 7; sensorNumber >= 0; sensorNumber--)
	{
		int temp = adcRead(sensorNumber);
		if (temp > compareValueWhiteSurface[sensorNumber]) digitalValue = 0; //black
		else
		{
			digitalValue = 1;
			int8_t tempVal = ((sensorNumber * 2) - 7);
// 			if (tempVal < 0) currentWeight = tempVal * (-tempVal + 10);
// 			else currentWeight = tempVal * (tempVal + 10);
			weightRange = (sensorNumber * 20) - 70;
			currentWeight = tempVal * 10;
			weightedValue += currentWeight;
			weightCount++;
			if (weightTemp == 0)
			{
				maxWeight = weightRange;
				weightTemp = 1;
			}
			minWeight = weightRange;
		}
		sensorValue |= (digitalValue << (7 - sensorNumber));
// 			Serial_sendInt(temp, DEC);
// 			Serial_sendString("\t");
	}
	


// 	Serial_sendInt(sensorValue, BIN);
// 	Serial_sendString("\n");
	if (sensorValue == 0x00)
	{
		if (allValueCount != 0)
		{
// 			Serial_sendInt(allValueCount, DEC);
// 			Serial_sendString("\n");
			if (allValueCount > 15 && allValueCount < 100)
			{
// 				Serial_sendString("\t\tOK\n");
				acuteFlag = 1;
//				exception_1_Count = 10;
			}
		}
		allValueCount = 0;
		weightedValue = prev_weightedValue;
	}
	else if (sensorValue == 0xFF)
	{
		if (allValueCountFlag == 0) allValueCount++;
		weightedValue = 0;
		exception = 0;
		Forward();
		_delay_ms(100);
		sensorValueConversion();
	}
	else 
	{ 
// 		if (allValueCount != 0)
// 		{
// // 			Serial_sendInt(allValueCount, DEC);
// // 			Serial_sendString("\n");
// 			if (allValueCount > 15 && allValueCount < 100)
// 			{
// // 				Serial_sendString("\t\tOK\n");
// 				acuteFlag = 1;
// 				exception_1_Count = 10;
// 			}
// 		}


		allValueCount = 0;
		weightDiv = maxWeight - minWeight;
		weightedValue = (weightedValue / weightCount);
		prev_weightedValue = weightedValue;
		if (weightDiv >= 60 && weightDiv <= 100)		//108-158
		{
			zero_count = (weightDiv / 20) + 1 - weightCount;
			if (zero_count == 0)
			{
				exception = 1; //Right angle
				exception_1_Count++;
				exception_2_Count = 0;
// 				if (weightedValue > 0) mappedValue = 100;
// 				else mappedValue = -100;
				
			}
			else if (zero_count >= 1 && zero_count < 4)
			{
				exception = 2; //Acute angle
				exception_1_Count = 0;
				exception_2_Count++;
			}
		}
// 		if ((weightCount == 6 || weightCount == 5) && weightedValue < 40 && weightedValue > -40) 
// 		{
// 			inverseLevel++;
// 		}
// 		else inverseFlag = 0;
	}
	if (exception == 0) 
	{
		exception_1_Count = 0;
		exception_2_Count = 0;
	}
	
	
// 	Serial_sendInt(mappedValue, DEC);
// 	Serial_sendString("\t");
// 
// // 	Serial_sendInt(weightedValue, DEC);
// // 	Serial_sendString("\t");
// 	Serial_sendInt(sensorValue, BIN);
// 	Serial_sendString("\n");	
}





void sensorMapping(void)
{
// 	Serial_sendString("OK\n");

	prev_mappedValue = mappedValue;
	if (sensorValue == 0b00000000) mappedValue = prev_mappedValue;
	else if (sensorValue == 0b11111111) mappedValue = 0;
	
	else if ((sensorValue == 0b00011001) || (sensorValue == 0b00001001) || (sensorValue == 0b00010001) || (sensorValue == 0b00110001) || (sensorValue == 0b01110001) || (sensorValue == 0b01100001) || (sensorValue == 0b01100011)) mappedValue = 120;
	else if ((sensorValue == 0b00001011) || (sensorValue == 0b00010011) || (sensorValue == 0b00011011) || (sensorValue == 0b00001010) || (sensorValue == 0b00010010) || (sensorValue == 0b01110111) || (sensorValue == 0b01110011) || (sensorValue == 0b00110111) || (sensorValue == 0b00011101) || (sensorValue == 0b01100111) || (sensorValue == 0b00001101) || (sensorValue == 0b00111001) || (sensorValue == 0b00110011)) mappedValue = 120;
	else if (/*(sensorValue == 0b00000111) ||*/ (sensorValue == 0b00001111) || (sensorValue == 0b00011111 || sensorValue == 0b00111111)) mappedValue = 125;
	else if (sensorValue == 0b00000001) mappedValue = 70;
	else if (sensorValue == 0b00000011) mappedValue = 60;
	else if (sensorValue == 0b00000010 || sensorValue == 0b11111101) mappedValue = 30;
	else if (sensorValue == 0b00000110 || sensorValue == 0b11111001) mappedValue = 20;
	else if (sensorValue == 0b00000100 || sensorValue == 0b00001110) mappedValue = 15;
	else if (sensorValue == 0b11111011)
	{
		mappedValue = 15;
		if (reverseFlag == 0)
		{
			inverseLevel++;
		}
		
	}
	else if (sensorValue == 0b00001100) mappedValue = 10;
	else if (sensorValue == 0b11110011)
	{
		mappedValue = 10;
		if (reverseFlag == 0)
		{
			inverseLevel++;
		}
	}
	else if (sensorValue == 0b00001000 || sensorValue == 0b00011100) mappedValue = 5;
	else if (sensorValue == 0b11110111)
	{
		mappedValue = 5;
		if (reverseFlag == 0)
		{
			inverseLevel++;
		}
	}
	else if (sensorValue == 0b00011000 /*|| sensorValue == 0b11101110 || sensorValue == 0b01110111 || sensorValue == 0b01100110 || sensorValue == 0b10011001 || sensorValue == 0b00110111 || sensorValue == 0b11101100*/) mappedValue = 0;
	else if (sensorValue == 0b11100111)
	{
		mappedValue = 0;
		if (reverseFlag == 0)
		{
			inverseLevel++;
		}
	}
	else if (sensorValue == 0b11101110 || sensorValue == 0b01110111 || sensorValue == 0b01100110 || sensorValue == 0b10011001 || sensorValue == 0b01111110 || sensorValue == 0b00111100 || sensorValue == 0b01111100 || sensorValue == 0b00111110 || sensorValue == 0b01110111 || sensorValue == 0b11101110)  mappedValue = .5;
	else if (sensorValue == 0b00010000 || sensorValue == 0b00111000) mappedValue = -5;
	else if (sensorValue == 0b11101111)
	{
		mappedValue = -5;
		if (reverseFlag == 0)
		{
			inverseLevel++;
		}
	}
	else if (sensorValue == 0b00110000) mappedValue = -10;
	else if (sensorValue == 0b11001111)
	{
		mappedValue = -10;
		if (reverseFlag == 0)
		{
			inverseLevel++;
		}
	}
	else if (sensorValue == 0b00100000 || sensorValue == 0b01110000) mappedValue = -15;
	else if (sensorValue == 0b11011111)
	{
		mappedValue = -15;
		if (reverseFlag == 0)
		{
			inverseLevel++;
		}
	}
	else if (sensorValue == 0b01100000 || sensorValue == 0b10011111) mappedValue = -20;
	else if (sensorValue == 0b01000000 || sensorValue == 0b10111111) mappedValue = -30;
	else if (sensorValue == 0b11000000) mappedValue = -60;
	else if (sensorValue == 0b10000000) mappedValue = -70;
	else if (/*(sensorValue == 0b11100000) ||*/ (sensorValue == 0b11110000) || (sensorValue == 0b11111000 || sensorValue == 0b11111100)) mappedValue = -125;
	else if ((sensorValue == 0b11011000) || (sensorValue == 0b11010000) || (sensorValue == 0b11001000) || (sensorValue == 0b01010000) || (sensorValue == 0b01001000) || (sensorValue == 0b11101110) || (sensorValue == 0b11001110) || (sensorValue == 0b11101100) || (sensorValue == 0b10111000) || (sensorValue == 0b11100110) || (sensorValue == 0b10110000) || (sensorValue == 0b10011100) || (sensorValue == 0b11001100)) mappedValue = -120;
	else if ((sensorValue == 0b10011000) || (sensorValue == 0b10010000) || (sensorValue == 0b10001000) || (sensorValue == 0b10001100) || (sensorValue == 0b10001110) || (sensorValue == 0b10000110) || (sensorValue == 0b11000110)) mappedValue = -120;
// 	Serial_sendInt(mappedValue, DEC);
// 	Serial_sendString("\n");
}


void PID_Correction(void) //my algo
{
	error = targetValue - weightedValue;
// 	if (error <= 40 && error >= -40) 
// 	{
// 		straightLineCount++;
// 		if (straightLineCount > 2000) maxSpeed = 230;
// 	}
// 	else 
// 	{
// 		straightLineCount = 0;
// 		maxSpeed = 200;
// 	}
// 	if (error < 0)
// 	{
// 		maxSpeed = 255 + error/*(error * 1.5)*/;
// 	}
// 	else if (error >= 0)
// 	{
// 		maxSpeed = 255 - error/*(error * 1.5)*/;
// 	}
// 
// 	if (error < 20 && error > -20) maxSpeed = 255;
// 	else maxSpeed = 170;

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

	enable1(130);
	enable2(130);
}

void RightSharp(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT |= (1 << motor1a_PIN);
	motor1b_PORT &= ~(1 << motor1b_PIN);
	motor2a_PORT &= ~(1 << motor2a_PIN);
	motor2b_PORT |= (1 << motor2b_PIN);

	enable1(150);
	enable2(150);
}

void LeftSharp(void)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT &= ~(1 << motor1a_PIN);
	motor1b_PORT |= (1 << motor1b_PIN);
	motor2a_PORT |= (1 << motor2a_PIN);
	motor2b_PORT &= ~(1 << motor2b_PIN);

	enable1(150);
	enable2(150);
}

void sonarGuideCave(void)
{

// 	while (1)
// 	{
// 		distanceMeasure();
		Serial_sendInt(distance1, DEC);
// 		Serial_sendString("\t");
// 		Serial_sendInt(distance2, DEC);
		Serial_sendString("\n");
// 		Serial_sendInt(distance3, DEC);
// 		Serial_sendString("\n");
// 		sensorValueConversion();
// 		sensorMapping();
// 	}
// 	InterruptOff();
}

void distanceMeasure(void)
{	
	trig1_PORT &= ~(1 << trig1_PIN);
// 	trig2_PORT &= ~(1 << trig2_PIN);
// 	trig3_PORT &= ~(1 << trig3_PIN);
	trig1_PORT |= (1 << trig1_PIN);
// 	trig2_PORT |= (1 << trig2_PIN);
// 	trig3_PORT |= (1 << trig3_PIN);
	_delay_us(10);
	trig1_PORT &= ~(1 << trig1_PIN);
// 	trig2_PORT &= ~(1 << trig2_PIN);
// 	trig3_PORT &= ~(1 << trig3_PIN);
	
// 	_delay_ms(1000);
	distance1 = .035 * pulse1 / 2;
// 	distance2 = .0353 * pulse2 / 2;
// 	distance3 = .0353 * pulse3 / 2;
	
	Serial_sendString("D1 : ");
	Serial_sendInt(distance1, DEC);
// 	Serial_sendString("\t");
// 	Serial_sendString("D2 : ");
// 	Serial_sendInt(distance2, DEC);
// 	Serial_sendString("\t");
// 	Serial_sendString("D3 : ");
// 	Serial_sendInt(distance3, DEC);
	Serial_sendString("\n");
		
}

// void triggerSonar1(void)
// {
// 	trig1_PORT &= ~(1 << trig1_PIN);
// 	// 	trig2_PORT &= ~(1 << trig2_PIN);
// 	// 	trig3_PORT &= ~(1 << trig3_PIN);
// 	trig1_PORT |= (1 << trig1_PIN);
// 	// 	trig2_PORT |= (1 << trig2_PIN);
// 	// 	trig3_PORT |= (1 << trig3_PIN);
// 	_delay_us(10);
// 	trig1_PORT &= ~(1 << trig1_PIN);
// 	// 	trig2_PORT &= ~(1 << trig2_PIN);
// 	// 	trig3_PORT &= ~(1 << trig3_PIN);
// 	
// 	// 	_delay_ms(1000);
// 	distance1 = .035 * pulse1 / 2;
// 	// 	distance2 = .0353 * pulse2 / 2;
// 	// 	distance3 = .0353 * pulse3 / 2;
// 	
// 	Serial_sendString("D1 : ");
// 	Serial_sendInt(distance1, DEC);
// 	// 	Serial_sendString("\t");
// 	// 	Serial_sendString("D2 : ");
// 	// 	Serial_sendInt(distance2, DEC);
// 	// 	Serial_sendString("\t");
// 	// 	Serial_sendString("D3 : ");
// 	// 	Serial_sendInt(distance3, DEC);
// 	Serial_sendString("\n");
// }

void triggerSonar1(void)
{
	trig1_PORT &= ~(1 << trig1_PIN);
	trig1_PORT |= (1 << trig1_PIN);
	_delay_us(10);
	trig1_PORT &= ~(1 << trig1_PIN);
// 	trig1_ack = 1;
}


void triggerSonar2(void)
{
	trig2_PORT &= ~(1 << trig2_PIN);
	_delay_us(10);
	xINT_init();
	sei();
	trig2_PORT &= ~(1 << trig2_PIN);
	trig2_ack = 1;
}
void InterruptOff(void)
{
//	cli();
	MCUCR &= ~(1 << ISC10);								// Setting Interrupt 1 for any change

	GICR &= ~((1 << INT1)); // Disable Interrupt 0, Interrupt 1, Interrupt 2
	TIMSK &= ~((1 << TOIE1));				// Disable Timer Overflow Interrupt 0, Timer Overflow Interrupt 2
}

void xINT_init(void)
{
// 	MCUCR |= (1 << ISC00);								// Setting Interrupt 0 for any change
	MCUCR |= (1 << ISC10);								// Setting Interrupt 1 for any change
// 	MCUCSR |= (1 << ISC2);								// Setting Interrupt 2 for rising edge
	GICR = (1 << INT1)/* | (1 << INT0) | (1 << INT2)*/;		// Enable Interrupt 0, Interrupt 1, Interrupt 2
}

void TIMER0_2_INT_init(void)
{
	TIMSK |= (1 << TOIE1)/* | (1 << TOIE1) | (1 << TOIE2)*/;				// Enable Timer Overflow Interrupt 0, Timer Overflow Interrupt 2
}

ISR (INT1_vect)
{
	if (echo == 0)
	{
		TCCR1B |= (1 << CS11)/* | (1 << CS10)*/;
		echo = 1;
	}
	else if (echo == 1)
	{
		TCCR1B = 0;
		pulse1 = TCNT1 / 2;
		
// 	Serial_sendString("T : ");
// 	Serial_sendInt(pulse1, DEC);
// 	 	Serial_sendString("\t");

		TCNT1 = 0;
		echo = 0;
		sonar1_ack = 1;
		trig1_ack = 0;
	}
}

// ISR (INT1_vect)
// {
// 	if (echo == 0)
// 	{
// 		TCCR1B |= (1 << CS11);
// 		echo = 1;
// 	}
// 	else if (echo == 1)
// 	{
// 		TCCR1B = 0;
// 		pulse1 = TCNT1 / 2;
// 		TCNT1 = 0;
// 		echo = 0;
// 		sonar2_ack = 1;
// 		trig2_ack = 0;
// 		InterruptOff();
// 	}
// }
// 
// ISR (INT2_vect)
// {
// 	MCUCSR ^= (1 << ISC2);
// 	if (choice3 == 0)
// 	{
// 		TCCR2 |= 0b00000100;
// 		choice3 = 1;
// 	}
// 	else if (choice3 == 1)
// 	{
// 		TCCR2 = 0;
// 		pulse3 = (TCNT2 + (0xFF * TIMER2_INT)) * 4;
// 		TIMER2_INT = 0;
// 		TCNT2 = 0;
// 		choice3 = 0;
// 	}
// }
// // 
// ISR (TIMER1_OVF_vect)
// {
// 	TIMER2_INT++;
// }
// 
// ISR (TIMER0_OVF_vect)
// {
// 	TIMER0_INT++;
// }



