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

//#define baseSpeed				180
#define setPoint				7

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


uint16_t threshold[8] = {700, 700, 700, 700, 700, 700, 700, 700};
uint16_t upperThreshold[8] = {547, 684, 618, 705, 679, 663, 723, 636};
uint16_t lowerThreshold[8] = {369, 486, 432, 497, 475, 459, 503, 438};
	
uint16_t proportionality[8] = {0, 0, 0, 0, 0, 0, 0, 0};

uint16_t SensorMaxValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t SensorMinValue[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};

uint16_t prev_SensorValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};

uint16_t timerOverflow = 0;
uint16_t duration = 0;
uint8_t timerFlag = 0;
uint16_t flashcount = 0;

float error = 0;
float prev_error = 0;
int16_t sum_error = 0;
float correction = 0;
int16_t motorResponse = 0;
uint8_t LMspeed = 0; 
uint8_t RMspeed = 0;
uint8_t reverseSpeed = 0;
int weightedValue = 0;
char flashMode = 0;
int stopFlag = 0;

// for baseSpeed 180-direct Final Flash 12V ******** final 

//uint8_t baseSpeed = 180;
// uint16_t Kp = 40;//45//38;//36;// 45 @255 // 35 @220 // 35@200
// uint8_t Kp_1 = 0;
// uint16_t Kd = 500;//650//4000;//700;//400;//7 @255 // 250 @ 220 // 206@200
// uint16_t Ki = 0;

// for baseSpeed 240

// uint8_t baseSpeed = 255;
// uint16_t Kp = 30;//45//38;//36;// 45 @255 // 35 @220 // 35@200
// uint8_t Kp_1 = 0;
// uint16_t Kd = 200;//650//4000;//700;//400;//7 @255 // 250 @ 220 // 206@200
// uint16_t Ki = 0;

uint8_t baseSpeed = 0;
uint16_t Kp = 0;
uint16_t Kd = 0;

void setTimer(uint16_t durationMicros);
void timerOff(void);
void autoCalibration(void);
void sensorValueConversion(void);
void sensorValuePrint(void);
void sensorMapping(void);
void PID_Correction(void);
void exceptionHandling (void);

void distanceMeasure(void);
void sonarGuideCave(void);
void TIMER0_2_INT_init(void);
void xINT_init(void);
void InterruptOff(void);

void Forward(void);
void Backward(void);
void Stop(void);
void RightSharp(uint8_t motorSpeed);
void LeftSharp(uint8_t motorSpeed);
void setRotationForward(void);
void setRotationRightSharp(void);
void setRotationLeftSharp(void);


int main(void)
{
	// Input-Output settings
	
	motor1a_DDR |= (1 << motor1a_PIN);
	motor1b_DDR |= (1 << motor1b_PIN);
	motor2a_DDR |= (1 << motor2a_PIN);
	motor2b_DDR |= (1 << motor2b_PIN);
	enable1_DDR |= (1 << enable1_PIN);
	enable2_DDR |= (1 << enable2_PIN);
	
	echo2_DDR &= ~(1 << echo2_PIN);
	trig2_DDR &= ~(1 << trig2_PIN);
	
	// Local Variables
	
	
	
	// Initializations	
	
	adc_init();
	UART_init();
	PWM0_init();
	PWM2_init();
	setRotationForward();
	
	
// 	if ((PIND & (1 << echo2_PIN)) == 0  && (PIND & (1 << trig2_PIN)) == 1)
// 	{
		_delay_ms(1000);
		autoCalibration();
		_delay_ms(1000);
// 	}
// 	Serial_sendInt(PIND, BIN);
	
	Forward();
	enable1(baseSpeed);
	enable2(baseSpeed);
    while (1) 
    {				
		sensorValueConversion();		
		if (stopFlag >= 50)
		{
			Backward();
			_delay_ms(50);
			Stop();
			break;
		}
		if (weightedValue != 255)
		{
			PID_Correction();
			enable1(LMspeed);
			enable2(RMspeed);
		}

//   		sensorValuePrint();
	}
}

void sensorValuePrint(void)
{
	int sensor = 0;
// 	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
// 	{
// 		Serial_sendInt(prev_SensorValue[sensorNumber], DEC);
// 		Serial_sendString("\t");
// 	}
// 	
// 	Serial_sendString("\t");
// 	Serial_sendInt(weightedValue, DEC);
// 	Serial_sendString("\t");
// 	Serial_sendInt(error, DEC);
// 	Serial_sendString("\t");
// 	Serial_sendInt(motorResponse, DEC);
// 	Serial_sendString("\t");
	
	Serial_sendInt(LMspeed, DEC);
	Serial_sendString("\t");
	Serial_sendInt(RMspeed, DEC);
	Serial_sendString("\t");
	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		sensor = adcRead(sensorNumber);
// 		if(sensor > SensorMaxValue[sensorNumber]) sensor = SensorMaxValue[sensorNumber];
// 		else if (sensor < SensorMinValue[sensorNumber]) sensor = SensorMinValue[sensorNumber];
// 		sensor = (sensor - SensorMinValue[sensorNumber]) * proportionality[sensorNumber]; 
		Serial_sendInt(sensor, DEC);
		Serial_sendString("\t");
	}
	Serial_sendString("\n");
	
}

// void sensorValueConversion(void)
// {
// 	uint8_t sum = 0;
// 	uint8_t sensorCount = 0;
// 
// 	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
// 	{
// 		if (prev_SensorValue[sensorNumber] == 0)
// 		{
// 			if(adcRead(sensorNumber) < lowerThreshold[sensorNumber])
// 			{
// 				sum += sensorNumber * 2;
// 				sensorCount++;
// 				prev_SensorValue[sensorNumber] = 1;
// 			}
// 			else prev_SensorValue[sensorNumber] = 0;
// 		}
// 		else if (prev_SensorValue[sensorNumber] == 1)
// 		{
// 			if(adcRead(sensorNumber) < upperThreshold[sensorNumber])
// 			{
// 				sum += sensorNumber * 2;
// 				sensorCount++;
// 				prev_SensorValue[sensorNumber] = 1;
// 			}
// 			else prev_SensorValue[sensorNumber] = 0;
// 		}
// 	}
// 	weightedValue = sum / sensorCount;
// }

void parameterDetection(void)
{
	
}

void sensorValueConversion(void)
{
	uint8_t sum = 0;
	uint8_t sensorCount = 0;

	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		if(adcRead(sensorNumber) < threshold[sensorNumber])
		{
			sum += sensorNumber * 2;
			sensorCount++;
		}
	}
	weightedValue = sum / sensorCount;
	if (sensorCount == 8)
	{
		stopFlag++;
	}
	else stopFlag = 0;
}

void autoCalibration(void)
{
	int currentValue = 0;
	setTimer(5000);
	
	RightSharp(140);
	while (timerFlag == 0)
	{
		for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
		{
			currentValue = adcRead(sensorNumber);
			if(currentValue > SensorMaxValue[sensorNumber]) SensorMaxValue[sensorNumber] = currentValue;
			if(currentValue < SensorMinValue[sensorNumber]) SensorMinValue[sensorNumber] = currentValue;
		}
	}
	timerFlag = 0;
	timerOff();
	Stop();
	Serial_sendString("MAX\t");
	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		Serial_sendInt(SensorMaxValue[sensorNumber], DEC);
		Serial_sendChar('\t');
	}
	Serial_sendChar('\n');
	
	Serial_sendString("MIN\t");
	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		Serial_sendInt(SensorMinValue[sensorNumber], DEC);
		Serial_sendChar('\t');
	}
	Serial_sendChar('\n');
	
	Serial_sendString("THR\t");
	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		threshold[sensorNumber] = (SensorMaxValue[sensorNumber] + SensorMinValue[sensorNumber]) / 2;
		Serial_sendInt(threshold[sensorNumber], DEC);
		Serial_sendChar('\t');
	}
	Serial_sendChar('\n');
	
	Serial_sendString("UTH\t");
	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		upperThreshold[sensorNumber] = threshold[sensorNumber] + ((SensorMaxValue[sensorNumber] - threshold[sensorNumber]) * 30 / 100); 
		Serial_sendInt(upperThreshold[sensorNumber], DEC);
		Serial_sendChar('\t');
	}
	Serial_sendChar('\n');
	
	Serial_sendString("LTH\t");
	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		lowerThreshold[sensorNumber] = threshold[sensorNumber] - ((threshold[sensorNumber] - SensorMinValue[sensorNumber]) * 30 / 100);
		Serial_sendInt(lowerThreshold[sensorNumber], DEC);
		Serial_sendChar('\t');
	}
	Serial_sendChar('\n');
		
	Serial_sendString("PRO\t");
	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		proportionality[sensorNumber] = 20000 / (SensorMaxValue[sensorNumber] - SensorMinValue[sensorNumber]);
		Serial_sendInt(proportionality[sensorNumber], DEC);
		Serial_sendChar('\t');
	}
	Serial_sendString("\n\n\n\n\n\n");
	
	
}

void setTimer(uint16_t durationMicros)
{
	duration = durationMicros;
	TIMSK |= (1 << OCIE1A);										// Set Timer1 Interrupt ON
	sei();														// Set Global Interrupt
	TCNT1 = 0;													// Reset Timer Count
	OCR1A = 0xF9;												// Compare Value for 1ms
	TCCR1B |= (1 << CS10) | (1 << CS11) | (1 << WGM12);			// Set Prescaler to 64 and CTC Mode
}

void timerOff(void)
{
	TCCR1B = 0x00;
	cli();
	TCNT1 = 0;
}

ISR(TIMER1_COMPA_vect)
{
	timerOverflow++;
	TCNT1 = 0;
	if (timerOverflow >= duration)
	{
		TCCR1B |= 0;
		timerFlag = 1;
		timerOverflow = 0;
		cli();
	}
}

void PID_Correction(void)
{
	error = weightedValue - setPoint;
//	parameterDetection();

	if (error >= 3 && error <= -3)
	{
		flashcount++;
		Stop();
		_delay_us(1000);
	}
	else if (error < 3 && error > -3)
	{
		flashcount = 0;
	}
	if (flashcount > 1)
	{
		flashMode = 1;
	}
	else flashMode = 0;


	if (flashMode == 0)
	{
		baseSpeed = 185;//180;
		Kp = 43;//40;
		Kd = 650;//600;
	
// 			baseSpeed = 255;
// 			Kp = 30;
// 			Kd = 200;
	}
	else if (flashMode == 1)
	{
		baseSpeed = 255;
		Kp = 40;
		Kd = 200;
	}

	correction = ((Kp * error) + (Kd * (error - prev_error))/* + (Ki * sum_error)*/);

	prev_error = error;
	motorResponse = (int)correction;

	if(motorResponse > baseSpeed)
	{
		reverseSpeed = (motorResponse - baseSpeed);
		if (reverseSpeed > baseSpeed) reverseSpeed = baseSpeed;
		motorResponse = baseSpeed;
		
		setRotationRightSharp();
		
		RMspeed = reverseSpeed;
		LMspeed = baseSpeed;
	}
	
	else if(motorResponse < -baseSpeed)
	{
		reverseSpeed = (-motorResponse - baseSpeed);
		if (reverseSpeed > baseSpeed) reverseSpeed = baseSpeed;
		motorResponse = -baseSpeed;
		
		setRotationLeftSharp();
		
		RMspeed = baseSpeed;
		LMspeed = reverseSpeed;
	}
	


	else if(motorResponse >= 0 && motorResponse < baseSpeed)
	{
		setRotationForward();
		
		RMspeed = baseSpeed - motorResponse;
		LMspeed = baseSpeed/* + motorResponse*/;
		
		//		if (baseSpeed + motorResponse > 255) LMspeed = 255;
	}

	else if(motorResponse < 0 && motorResponse > -baseSpeed)
	{
		setRotationForward();
		
		RMspeed = baseSpeed/* - motorResponse*/;
		LMspeed = baseSpeed + motorResponse;
		
		//		if (baseSpeed - motorResponse > 255) RMspeed = 255;
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

	enable1(baseSpeed);
	enable2(baseSpeed);
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

void RightSharp(uint8_t motorSpeed)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT |= (1 << motor1a_PIN);
	motor1b_PORT &= ~(1 << motor1b_PIN);
	motor2a_PORT &= ~(1 << motor2a_PIN);
	motor2b_PORT |= (1 << motor2b_PIN);

	enable1(motorSpeed);
	enable2(motorSpeed);
}

void LeftSharp(uint8_t motorSpeed)
{
	enable1(0);
	enable2(0);
	
	motor1a_PORT &= ~(1 << motor1a_PIN);
	motor1b_PORT |= (1 << motor1b_PIN);
	motor2a_PORT |= (1 << motor2a_PIN);
	motor2b_PORT &= ~(1 << motor2b_PIN);

	enable1(motorSpeed);
	enable2(motorSpeed);
}



