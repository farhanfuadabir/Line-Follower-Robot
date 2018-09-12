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

#define F_CPU 20000000UL

#define motor1a_DDR				DDRD
#define motor1a_PORT			PORTD
#define motor1a_PIN				6
#define motor1b_DDR				DDRD
#define motor1b_PORT			PORTD
#define motor1b_PIN				5
#define motor2a_DDR				DDRD
#define motor2a_PORT			PORTD
#define motor2a_PIN				7	
#define motor2b_DDR				DDRB
#define motor2b_PORT			PORTB
#define motor2b_PIN				0
#define enable1_DDR				DDRB
#define enable1_PORT			PORTB
#define enable1_PIN				1
#define enable2_DDR				DDRB
#define enable2_PORT			PORTB
#define enable2_PIN				2

//#define maxSpeed				180
#define setPoint				4

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>
#include "serial_lib.h"
#include "ADC_routine.h"
#include "PWM_routine.h"

#define enable1					PWM1A
#define enable2					PWM1B

#define	speedCoefficient		5.3


uint16_t threshold[8] = {700, 700, 700, 700, 700, 700, 700, 700};
uint16_t upperThreshold[8] = {547, 684, 618, 705, 679, 663, 723, 636};
uint16_t lowerThreshold[8] = {369, 486, 432, 497, 475, 459, 503, 438};
	
uint16_t proportionality[8] = {0, 0, 0, 0, 0, 0, 0, 0};

uint16_t SensorMaxValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t SensorMinValue[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};

uint16_t prev_SensorValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};

uint8_t sonar1_val[5], sonar2_val[5], sonar2_val[5];
uint8_t sonar1_readingFiltered = 0;
uint8_t sonar2_readingFiltered = 0;
uint8_t sonar3_readingFiltered = 0;
uint8_t sonar1_turn = 0;
uint8_t sonar2_turn = 0;
uint8_t sonar3_turn = 0;

uint16_t timerOverflow = 0;
uint16_t duration = 0;
uint8_t timerFlag = 0;
uint16_t flashcount = 0;

float error = 0;
float prev_error = 0;
int16_t sum_error = 0;
float correction = 0;
int16_t motorResponse = 0;
int16_t LMspeed = 0; 
int16_t RMspeed = 0;
uint8_t reverseSpeed = 0;
uint8_t weightedValue = 0;
uint8_t sensorCount = 0;
char flashMode = 0;
int stopFlag = 0;
uint8_t rightFlag = 0, leftFlag = 0, allFlag = 0, midFlag = 0;
uint8_t rightAcuteFlag = 0, leftAcuteFlag = 0;

uint8_t speedFlag = 0;

// const uint8_t maxSpeed = 255;
// const uint8_t baseSpeed = 122;
// // uint16_t Kp = 20; //70;		// 70 @ rmax 255
// // uint16_t Kd = 100; //250;		// 250 @ rmax 255
// uint16_t Kp = 18; //70;		// 70 @ rmax 255
// uint16_t Kd = 100; //250;		// 250 @ rmax 255

const uint8_t maxSpeed = 255;
uint8_t baseSpeed = 100;
// uint16_t Kp = 20; //70;		// 70 @ rmax 255
// uint16_t Kd = 100; //250;		// 250 @ rmax 255
uint16_t Kp = 20; //70;		// 70 @ rmax 255
uint16_t Kd = 80; //250;		// 250 @ rmax 255



void setTimer(uint16_t durationMicros);
void timercount(uint16_t durationmillis);
void timerOff(void);
void autoCalibration(void);
void sensorValueConversion(void);
void sensorValuePrint(void);
void sensorMapping(void);
void PID_Correction(void);
void exceptionHandling (void);

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
	
	DDRB |= (1 << PB3);
	// Initializations	
	
	adc_init();
	UART_init();
	PWM1_init();
	setRotationForward();
	
	
	_delay_ms(1000);
	autoCalibration();
	_delay_ms(1000);

	Stop();
	speedFlag = 1;
	
// 	Forward();
// 	enable1(maxSpeed);
// 	enable2(maxSpeed - (maxSpeed / 5.3));
    while (1) 
    {	

// 	while (1)
// 	{
// 		setRotationForward();
// 		enable1(120);
// 		enable2(110);
// 	}


		sensorValueConversion();

/*****************************************************Line Follow Turns****************************************************/

// 		if (leftFlag == 1 && weightedValue < 7 && weightedValue > 3)
// 		{
// 			Forward();
// 			while (leftFlag == 1)
// 			{
// 				sensorValueConversion();
// 				if (allFlag == 1 || rightFlag == 1) break;
// 			}
// 
// 			if (allFlag == 0 && rightFlag == 0)
// 			{
// 				_delay_ms(50);
// 				sensorValueConversion();
// // 				if (sensorCount == 0 && rightFlag == 0)
// // 				{
// // 					Backward();
// // 					_delay_ms(50);
// // 					RightSharp(150);
// // 					_delay_ms(200);
// // 					sensorValueConversion();
// // 				}
// 				if (sensorCount == 0 && rightFlag == 0)
// 				{
// 					LeftSharp(140);
// 					_delay_ms(100);
// 					while (sensorCount == 0 || weightedValue < 3)
// 					{
// 						sensorValueConversion();
// 						if (rightFlag == 1) break;
// 					}
// // 					Backward();
// // 					_delay_ms(400);
// // 					setRotationForward();
// 					sensorValueConversion();
// 				}
// 			}
// 		}
// 
// 		else if (rightFlag == 1 && weightedValue < 7 && weightedValue > 3)
// 		{
// 			Forward();
// 			while (rightFlag == 1)
// 			{
// 				sensorValueConversion();
// 				if (allFlag == 1 || leftFlag == 1) break;
// 			}
// 			
// 			if (allFlag == 0 && leftFlag == 0)
// 			{
// 				_delay_ms(80);
// // 				sensorValueConversion();
// 
// 				RightSharp(140);
// //				_delay_ms(100);
// 				
// 				while (rightFlag == 0) sensorValueConversion();
// 				
// 				while (sensorCount == 0 || weightedValue > 7 || weightedValue < 4) sensorValueConversion();
// 			}
// 		}
// 		
// 		else if (allFlag == 1 && weightedValue < 7 && weightedValue > 3)
// 		{
// 			Forward();
// 			while (allFlag == 1 && stopFlag < 100) sensorValueConversion();
// 
// // 			if (stopFlag >= 100)
// // 			{
// // 				Backward();
// // 				_delay_ms(100);
// // 				Stop();
// // 				_delay_ms(2000);
// // 				while (sensorCount == 6 && allFlag == 1) sensorValueConversion();
// // 			}
// // 
// // 			else
// // 			{
// // 				_delay_ms(20);
// 				sensorValueConversion();
// 
// 				RightSharp(140);
// 				_delay_ms(100);
// 			
// 				while (rightFlag == 0) sensorValueConversion();
// 			
// 				while (sensorCount == 0 || weightedValue > 7 || weightedValue < 4)	sensorValueConversion();
// // 			}			
// 		}
// 
		if (stopFlag > 120)
		{
			RightSharp(80);
			_delay_ms(1000);
			
			while (rightFlag == 0) sensorValueConversion();
			
			while (sensorCount == 0 || weightedValue == 4) sensorValueConversion();
		}
	
	
		if (leftFlag == 1 && weightedValue < 6 && weightedValue > 2)
		{
			Backward();
			_delay_ms(10);
			Forward();
			while (leftFlag == 1)
			{
				sensorValueConversion();
				if (allFlag == 1 || rightFlag == 1) break;
			}
			
// 			_delay_ms(20);
// 			sensorValueConversion();

			if (allFlag == 0 && rightFlag == 0 && midFlag == 0)
			{
				LeftSharp(80);
				//				_delay_ms(100);
				
				while (leftFlag == 0) sensorValueConversion();
				
				while (sensorCount == 0 || weightedValue == 4) sensorValueConversion();
// 				Forward();
// 				_delay_ms(20);
// 				allFlag = 0;
// 				leftFlag = 0;
// 				rightFlag = 0;
// 				setTimer(4000);
// 				baseSpeed = 120;

			}
		}

		if (rightFlag == 1 && weightedValue < 6 && weightedValue > 2)
		{
			Backward();
			_delay_ms(10);
			Forward();
			while (rightFlag == 1)
			{
				sensorValueConversion();
				if (allFlag == 1 || leftFlag == 1) break;
			}
			
			if (allFlag == 0 && leftFlag == 0)
			{
// 				Backward();
				_delay_ms(80);
				// 				sensorValueConversion();

				RightSharp(80);
				//				_delay_ms(100);
				
				while (rightFlag == 0) sensorValueConversion();
				
				while (sensorCount == 0 || weightedValue == 4) sensorValueConversion();
// 				Forward();
// 				_delay_ms(20);
// 				allFlag = 0;
// 				leftFlag = 0;
// 				rightFlag = 0;
// 				setTimer(4000);
// 				baseSpeed = 120;
				
			}
		}
		
		if (allFlag == 1 && weightedValue < 6 && weightedValue > 2)
		{
			Backward();
			_delay_ms(10);
			Forward();
			
			_delay_ms(80);
			// 				sensorValueConversion();

			RightSharp(80);
			//				_delay_ms(100);
				
			while (rightFlag == 0) sensorValueConversion();
				
				while (sensorCount == 0 || weightedValue == 4) sensorValueConversion();
			
		}

/*****************************************************Stop Sequence****************************************************/
		

/**********************************************************PID********************************************************/

		if (weightedValue != 255)
		{
			PID_Correction();
			enable1(RMspeed);
			enable2(LMspeed - 15);
		}
//		sensorValuePrint();
	}
}

// void lessenSpeed(uint8_t speed)
// {
// 	baseSpeed = speed;
// 	duration = durationMicros;
// 	TIMSK0 |= (1 << TOIE0);									// Set Timer1 Interrupt ON
// 	sei();														// Set Global Interrupt
// 	TCNT0 = 0;													// Reset Timer Count
// 	TCCR0B |= (1 << CS02);										// Set Prescaler to 256
// 	
// }



uint8_t medianFilter(uint8_t data[], uint8_t n)
{
	uint8_t i = 0, j = 0, key = 0;

	for(i = 1; i < n; i++)
	{
		key = data[i];
		j = i - 1;
		while(j >= 0 && data[j] > key)
		{
			data[j + 1] = data[j];
			j--;
		}
		data[j + 1] = key;
	}
	if (n % 2 == 1) return data[(n - 1) / 2];
	else return ((data[n / 2] + data[(n / 2) - 1]) / 2);
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
	Serial_sendString("\t");
	Serial_sendInt(weightedValue, DEC);
	Serial_sendString("\t");
	Serial_sendInt(error, DEC);
	Serial_sendString("\t");
	Serial_sendInt(motorResponse, DEC);
	Serial_sendString("\t\t");
	
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



void sensorValueConversion(void)
{
	uint8_t sum = 0;
	uint8_t maxSensor = 0;
	uint8_t minSensor = 5;
	sensorCount = 0;
	leftFlag = 0;
	rightFlag = 0;
	allFlag = 0;
	rightAcuteFlag = 0;
	leftAcuteFlag = 0;

	for(uint8_t sensorNumber = 0; sensorNumber < 8; sensorNumber++)
	{
		if(adcRead(sensorNumber) > threshold[sensorNumber])
		{
			if (sensorNumber == 5) leftFlag = 1;
			else if (sensorNumber == 6) rightFlag = 1;
			else if (sensorNumber == 4) midFlag = 1;
			else if (sensorNumber == 7)
			{
				if (4 < minSensor) minSensor = 4;
				else if (4 > maxSensor) maxSensor = 4;
				sum += 4 * 2;
				sensorCount++;
			}
			else
			{
				if ((3 - sensorNumber) < minSensor) minSensor = 3 - sensorNumber;
				else if ((3 - sensorNumber) > maxSensor) maxSensor = 3 - sensorNumber;
				sum += (3 - sensorNumber) * 2;
				sensorCount++;
			}
		}
	}
	
	if (sensorCount == (maxSensor - minSensor + 1))
	{
		if (maxSensor == 5 && minSensor > 1) 
		{
			rightAcuteFlag = 1;
// 			Serial_sendString("R\n");
		}
		if (minSensor == 0 && maxSensor < 4) 
		{
			leftAcuteFlag = 1;
// 			Serial_sendString("L\n");
		}
		if (rightAcuteFlag == 1 && leftAcuteFlag == 1)
		{
			rightAcuteFlag = 0; 
			leftAcuteFlag = 0;
		}
	}
	if (leftFlag == 1 && rightFlag == 1) 
	{
		leftFlag = 0;
		rightFlag = 0;
		allFlag = 1;
	}
	
	weightedValue = sum / sensorCount;
	
	if (sensorCount == 0 && allFlag == 0 && midFlag == 0 && leftFlag == 0 && rightFlag == 0) stopFlag++;
	else stopFlag = 0;
}

// void timercount(uint16_t durationmillis)
// {
// 	TCCR1B |= (1 << WGM12);
// 	unsigned long timerlimit = ((F_CPU / 1024) * (durationmillis / 1000)) - 1;
// 	OCR1A = timerlimit;
// 	TCCR1B |= (1 << CS12) | (1 << CS10);
// }

void autoCalibration(void)
{
	Forward();
	int currentValue = 0;
	setTimer(2000);
	
	RightSharp(170);
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
		threshold[sensorNumber] = SensorMinValue[sensorNumber] + (SensorMaxValue[sensorNumber] - SensorMinValue[sensorNumber]) * 2 / 3;
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
	TIMSK0 |= (1 << OCIE0A);									// Set Timer1 Interrupt ON
	sei();														// Set Global Interrupt
	TCNT0 = 0;													// Reset Timer Count
	OCR0A = 0x4D;												// Compare Value for 1ms
	TCCR0A |= (1 << WGM01);										// CTC Mode
	TCCR0B |= (1 << CS02);										// Set Prescaler to 256 
}

void timerOff(void)
{
	TCCR0B = 0x00;
	TIMSK0 = 0x00;
	cli();
	TCNT0 = 0;
}

ISR(TIMER0_COMPA_vect)
{
	if (speedFlag == 0)
	{
		timerOverflow++;
		TCNT0 = 0;
		if (timerOverflow >= duration)
		{
			TCCR0B = 0;
			timerFlag = 1;
			timerOverflow = 0;
			cli();
		}
	}
// 	else if (speedFlag == 1)
// 	{
// 		timerOverflow++;
// 		TCNT0 = 0;
// 		if (timerOverflow >= duration)
// 		{
// 			TCCR0B = 0;
// 			baseSpeed = 180;
// 			timerOverflow = 0;
// 			cli();
// 		}
// 		else if (timerOverflow > (duration * 3 / 4))
// 		{
// 			baseSpeed = 160;
// 		}
// 		else if (timerOverflow > (duration / 2))
// 		{
// 			baseSpeed = 140;
// 		}
// 		
// 	}
}

// ISR(TIMER0_OVF_vect)
// {
// 	baseSpeed = 180;
// }

void PID_Correction(void)
{
	error = weightedValue - setPoint;

	correction = ((Kp * error) + (Kd * (error - prev_error))/* + (Ki * sum_error)*/);

	prev_error = error;
	motorResponse = (int)correction;

// 	if(motorResponse > maxSpeed)
// 	{
// 		reverseSpeed = (motorResponse - maxSpeed);
// 		if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
// 		motorResponse = maxSpeed;
// 		
// 		setRotationRightSharp();
// 		
// 		RMspeed = reverseSpeed;
// 		LMspeed = maxSpeed/* - (maxSpeed / speedCoefficient)*/;
// 	}
// 	
// 	else if(motorResponse < -maxSpeed)
// 	{
// 		reverseSpeed = (-motorResponse - maxSpeed);
// 		if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
// 		motorResponse = -maxSpeed;
// 		
// 		setRotationLeftSharp();
// 		
// 		RMspeed = maxSpeed;
// 		LMspeed = reverseSpeed;
// 	}
// 	
// 
// 
// 	else if(motorResponse >= 0 && motorResponse < maxSpeed)
// 	{
// 		setRotationForward();
// 		
// 		RMspeed = maxSpeed - motorResponse/* - (maxSpeed / speedCoefficient)*/;
// 		LMspeed = maxSpeed;		
// 	}
// 
// 	else if(motorResponse < 0 && motorResponse > -maxSpeed)
// 	{
// 		setRotationForward();
// 		
// 		RMspeed = maxSpeed/* - (maxSpeed / speedCoefficient)*/;
// 		LMspeed = maxSpeed + motorResponse;
// 	}
	setRotationForward();
	
	RMspeed = baseSpeed - motorResponse;
	LMspeed = baseSpeed + motorResponse;
	
	if (RMspeed < 0) RMspeed = 0;
	if (RMspeed > maxSpeed) RMspeed = maxSpeed;
	if (LMspeed < 0) LMspeed = 0;
	if (LMspeed > maxSpeed) LMspeed = maxSpeed;

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



