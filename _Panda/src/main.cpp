/*
Team Panda
IUB Makers' Mania
13-09-18
-Fuad
*/

#include <Arduino.h>

/*          MOTOR PINS          */

#define rightMotorA 6
#define rightMotorB 5
#define leftMotorA 4
#define leftMotorB 3
#define rightMotor_power  7
#define leftMotor_power 2

/*          PID CONSTANT VALUES         */

#define kp 64
#define kd 120
#define ki 0

#define setPoint 7


/*          GLOBAL VARIABLES INITIALIZATION            */

int sum = 0, avg = 0;
boolean rightSensor = 0, leftSensor = 0;
int error = 0, prev_error = 0;
int motorResponse = 0, lmSpeed = 150, rmSpeed = 150, maxSpeed = 230, reverseSpeed = 0;
int correct = 0;
int timer = 0;
int count = 0;


int threshold[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int maxValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int minValue[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};


/*          BASIC MOVEMENT FUNCTIONS            */


void forward(uint8_t speed)
{
  digitalWrite(rightMotorA, HIGH);
  digitalWrite(rightMotorB, LOW);
  digitalWrite(leftMotorA, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(rightMotor_power, speed);
  analogWrite(leftMotor_power, speed);
}

void backward(uint8_t speed)
{
  digitalWrite(rightMotorB, HIGH);
  digitalWrite(rightMotorA, LOW);
  digitalWrite(leftMotorB, HIGH);
  digitalWrite(leftMotorA, LOW);
  analogWrite(rightMotor_power, speed);
  analogWrite(leftMotor_power, speed);
}

void halt()
{
  digitalWrite(rightMotorA , LOW);
  digitalWrite(rightMotorB , LOW);
  digitalWrite(leftMotorA , LOW);
  digitalWrite(leftMotorB , LOW);
  analogWrite(rightMotor_power, 0);
  analogWrite(leftMotor_power, 0);
}

void setRotationForward()
{
  digitalWrite(rightMotorA, HIGH);
  digitalWrite(rightMotorB, LOW);
  digitalWrite(leftMotorA, HIGH);
  digitalWrite(leftMotorB, LOW);
}

void leftSharp(uint8_t speed)
{
  digitalWrite(rightMotorA, HIGH);
  digitalWrite(rightMotorB, LOW);
  digitalWrite(leftMotorA, LOW);
  digitalWrite(leftMotorB, HIGH);
  analogWrite(rightMotor_power, speed);
  analogWrite(leftMotor_power, speed);
}


void setRotationLeftSharp()
{
  digitalWrite(rightMotorA, HIGH);
  digitalWrite(rightMotorB, LOW);
  digitalWrite(leftMotorA , LOW);
  digitalWrite(leftMotorB , HIGH);
}

void rightSharp(uint8_t speed)
{
  digitalWrite(rightMotorB, HIGH);
  digitalWrite(rightMotorA, LOW);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(leftMotorA, HIGH);
  analogWrite(rightMotor_power, speed);
  analogWrite(leftMotor_power, speed);
}

void setRotationRightSharp()
{
  digitalWrite(rightMotorB, HIGH);
  digitalWrite(rightMotorA, LOW);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(leftMotorA, HIGH);
}



/*               Sensor Functions                */



void sensorValuePrint()
{
  Serial.print(avg);
  Serial.print("\t");
//  Serial.print(error);
  Serial.print("\t");
  Serial.print(analogRead(A0));
  Serial.print('\t'); 
  Serial.print(analogRead(A1));
  Serial.print('\t');
  Serial.print(analogRead(A2));
  Serial.print('\t');
  Serial.print(analogRead(A3));
  Serial.print('\t');
  Serial.print(analogRead(A4));
  Serial.print('\t');
  Serial.print(analogRead(A5));
  Serial.print('\t');
  Serial.print(analogRead(A6));
  Serial.print('\t');
  Serial.println(analogRead(A7));
  //delay(10);
}


void sensorRead()
{ 
  leftSensor = 0;
  rightSensor = 0;
  count = 0;
  sum = 0;

  for(uint8_t i = 0 ; i < 8 ; i++)
  {
    if(analogRead(i) < threshold[i])
    {
      if(i == 0) rightSensor = 1;
      else if(i == 7) leftSensor = 1; 
      else
      {
        count++;
        sum += i * 2;
      }
    }   
  }
  avg = sum / count;
}

// void sensorRead()
// { 
//   leftSensor = 0;
//   rightSensor = 0;
//   count = 0;
//   sum = 0;
//   int temp = 0;

//   for(uint8_t i = 0 ; i < 8 ; i++)
//   {
//     temp = analogRead(i);
//     if(temp > maxValue[i]) temp = 10;
//     else if(temp < minValue[i]) temp = 0;
//     //else temp = (temp - minValue[i])*10;
//     else temp = ((temp - minValue[i]) * 10) / (maxValue[i] - minValue[i]);
//     Serial.print(temp);
//     Serial.print('\t');
//   }
//   Serial.print('\n');
// }

void autoCalibration(uint16_t calibrationTime, uint8_t percent)
{
  rightSharp(150);
  uint16_t startTime = millis();
  while(millis() - startTime <= calibrationTime)
  {
    for(uint8_t i = 0 ; i < 8 ; i++)
    {
      uint16_t temp = analogRead(i);
      if(temp > maxValue[i]) maxValue[i] = temp;
      else if(temp < minValue[i]) minValue[i] = temp;
    }
  }
  for(uint8_t i = 0 ; i < 8 ; i++) threshold[i] = ((maxValue[i] - minValue[i]) * percent / 100) + minValue[i];
  
  while(avg != (setPoint - 2)) sensorRead();
  leftSharp(200);
  delay(50);
  halt();

  // for(uint8_t i = 0 ; i < 8 ; i++) 
  // {
  //   Serial.print(threshold[i]);
  //   Serial.print('\t');
  // }  
}


void pid(void)
{
    error = setPoint - avg;    
    motorResponse = kp*error + kd*(error - prev_error);
    prev_error = error;
//    Serial.print(avg);
//    Serial.print('\t');
//    Serial.print(error);
//    Serial.print('\t');
//    Serial.print(motorResponse);
//    Serial.print('\t');
//    if (motorResponse >= 0)
//    {
//        lmSpeed = maxSpeed;
//        rmSpeed = maxSpeed - motorResponse;
//    }
//    else if (motorResponse < 0)
//    {
//        rmSpeed = maxSpeed;
//        lmSpeed = maxSpeed + motorResponse;
//    }

//    lmSpeed = maxSpeed + motorResponse;
//    rmSpeed = maxSpeed - motorResponse;
//    if(rmSpeed > maxSpeed)  rmSpeed = maxSpeed;
//    if(lmSpeed > maxSpeed)  lmSpeed = maxSpeed;
//    if(rmSpeed < 0) rmSpeed = 0;
//    if(lmSpeed < 0) lmSpeed = 0;  

  if(motorResponse > maxSpeed)
  {
    reverseSpeed = (motorResponse - maxSpeed);
    if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
    motorResponse = maxSpeed;
    
    setRotationRightSharp();
    
    rmSpeed = reverseSpeed;
    lmSpeed = maxSpeed;
  }
  
  else if(motorResponse < -maxSpeed)
  {
    reverseSpeed = (-motorResponse - maxSpeed);
    if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
    motorResponse = -maxSpeed;
    
    setRotationLeftSharp();
    
    rmSpeed = maxSpeed;
    lmSpeed = reverseSpeed;
  }
  


  else if(motorResponse >= 0 && motorResponse < maxSpeed)
  {
    setRotationForward();
    
    rmSpeed = maxSpeed - motorResponse;
    lmSpeed = maxSpeed;   
  }

  else if(motorResponse < 0 && motorResponse > -maxSpeed)
  {
    setRotationForward();
    
    rmSpeed = maxSpeed;
    lmSpeed = maxSpeed + motorResponse;
  }
    
//    Serial.print(lmSpeed);
//    Serial.print('\t');
//    Serial.println(rmSpeed);
}




void setup() {
  pinMode(rightMotorA , OUTPUT);
  pinMode(rightMotorB , OUTPUT);
  pinMode(leftMotorA , OUTPUT);
  pinMode(leftMotorB , OUTPUT);
  pinMode(rightMotor_power , OUTPUT);
  pinMode(leftMotor_power , OUTPUT);
  
  Serial.begin(9600);
  
  //forward();
  autoCalibration(2000, 50);  
}


void loop() {
  sensorRead();
//  sensorValuePrint();

//forward();
// delay(1000);
// rightSharp();
// delay(1000);
// leftSharp();
// delay(1000);

//   sensorRead();
  
//   if(prev_error < 2 && prev_error > -2 && rightFlag == 1 && count < 6)
//   {
//     forward();
//     timer = millis();
//     while ((millis() - timer) < 3000)
//     {
//       sensorRead();
//       if(count == 6 || count == 0) break;
//       rightFlag = 0;
//       leftFlag = 0;
//     }
    
//     if (count == 0)
//     {
//       rightSharp();
//     }
//     while ((millis() - timer) < 1000)
//     {
//       sensorRead();
//       if(avg != -1)
//       {
//           pid();
//           analogWrite(rightMotor_power , rmSpeed);
//           analogWrite(leftMotor_power , lmSpeed);
//       }
//     }
      
// //      sensorRead(); 
// //      pid();
// //      analogWrite(rightMotor_power , rmSpeed);
// //      analogWrite(leftMotor_power , lmSpeed);
      
//   }
//   else if(prev_error < 2 && prev_error > -2 && leftFlag == 1 && count < 6)
//   {
//     forward();
//     timer = millis();
//     while ((millis() - timer) < 3000)
//     {
//       sensorRead();
//       if(count == 6 || count == 0) break;
//       rightFlag = 0;
//       leftFlag = 0;
//     }
//     if(count == 0)
//     {
//       leftSharp();
//     }    
//     while ((millis() - timer) < 1000)
//     {
//       sensorRead();
//       if(avg != -1)
//     {
//           pid();
//           analogWrite(rightMotor_power , rmSpeed);
//           analogWrite(leftMotor_power , lmSpeed);
//       }
//     }

// //    sensorRead();
// //    pid();
// //    analogWrite(rightMotor_power , rmSpeed);
// //    analogWrite(leftMotor_power , lmSpeed); 
// }
//   if(count == 6)
//   {
//     forward();
//     allFlag++;
//     if(allFlag > 100) // stop condition
//     {
//       backward();
//       delay(20);
//       halt();
//       delay(5000);
//     }
//   }
//   else if(count < 6) allFlag = 0;

// //  //Serial.println(avg);
  if(avg != -1)
  {
      pid();
      analogWrite(rightMotor_power, rmSpeed);
      analogWrite(leftMotor_power, lmSpeed);
      Serial.print('\n');
      Serial.print(lmSpeed);
      Serial.print('\t');
      Serial.print(rmSpeed);
      Serial.print('\n');
  }  
}