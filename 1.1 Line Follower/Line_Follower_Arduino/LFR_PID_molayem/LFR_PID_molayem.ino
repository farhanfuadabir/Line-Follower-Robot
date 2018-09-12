#define targetValue                      0            
#define maxSpeed                       120
#define F                                0
#define R                                1
#define L                                2
#define S                                4


//int compareValue[6] = {70, 140, 120, 130, 140, 150};
//int compareValue[6] = {100, 175, 160, 120, 150, 120};
int compareValue[6] = {100, 90, 100, 100, 100, 90};
//int compareValue[6] = {500, 500, 500, 500, 500, 500};
//int compareValue[5] = {300, 350, 300, 300, 300};
//int compareValue[5] = {700, 700, 700, 700, 700};
int sensorValue = 0;
int mappedValue = 0; 

int error = 0;
int prev_error = 0;
int sum_error = 0;
float correction = 0;
int motorResponse = 0;

float Kp = 7.5;
float Ki = 8;
float Kd = 16;


int LMspeed, RMspeed;
int LMpin, RMpin;
int flag = 0;
int u_turn_flag = 0;

void sensorValueConversion(void);
void sensorMapping(void);
void PID_Correction(void);
void Forward(void);
void Stop(void);

void setup()
{
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  Forward();
  
  Serial.begin(9600);
}

void loop()
{
  Stop();

  delay(1000);

  Forward();
  
  while(1)
  {
    sensorValueConversion();
    sensorMapping();
/*
    while(mappedValue == 100)
    {
      sensorValueConversion();
      sensorMapping(); 
    }

    while(mappedValue == 111)
    {
      sensorValueConversion();
      sensorMapping(); 
    }
*/
    if((mappedValue < -40 && mappedValue > -60) || (mappedValue > 40 && mappedValue < 60))
    {
      Stop();
      //delay(100);
      Forward();
    }

    PID_Correction();
  
    analogWrite(10, LMspeed);
    analogWrite(11, RMspeed);

  
  Serial.print(analogRead(0));
  Serial.print("\t");
  Serial.print(analogRead(1));
  Serial.print("\t");
  Serial.print(analogRead(2));
  Serial.print("\t");
  Serial.print(analogRead(3));
  Serial.print("\t");
  Serial.print(analogRead(4));
  Serial.print("\t");
  Serial.print(analogRead(5));
  Serial.print("\t");
  Serial.print(sensorValue,BIN);
  Serial.print("\t");
  //Serial.println(mappedValue);
  Serial.print("\t");
  Serial.print(LMspeed);
  Serial.print("\t");
  Serial.print(RMspeed);
  Serial.print("\n");
  
  
    
  }
}

void sensorValueConversion(void)
{
  int digitalValue;
  sensorValue = 0;
    
  for (int sensorNumber = 5; sensorNumber >= 0; sensorNumber--)
  { 
    if (analogRead(sensorNumber) < compareValue[sensorNumber]) digitalValue = 0;
    else digitalValue = 1;
    sensorValue |= (digitalValue << sensorNumber);
  }
}

void sensorMapping(void)
{
  if (sensorValue == 0b000000) mappedValue = 100;
  else if (sensorValue == 0b111111) mappedValue = 111;
  
  else if (sensorValue == 0b001101) mappedValue = 50;
  else if (sensorValue == 0b001111) mappedValue = 50;
  else if (sensorValue == 0b000001) mappedValue = 50;
  else if (sensorValue == 0b000011) mappedValue = 40;
  else if (sensorValue == 0b000010) mappedValue = 30;
  else if (sensorValue == 0b000111) mappedValue = 30;
  else if (sensorValue == 0b000110) mappedValue = 20;
  else if (sensorValue == 0b001110) mappedValue = 10;
  else if (sensorValue == 0b001100) mappedValue = 0;
  else if (sensorValue == 0b011100) mappedValue = -10;
  else if (sensorValue == 0b011000) mappedValue = -20;
  else if (sensorValue == 0b111000) mappedValue = -30;
  else if (sensorValue == 0b010000) mappedValue = -30;
  else if (sensorValue == 0b110000) mappedValue = -40;
  else if (sensorValue == 0b100000) mappedValue = -50;
  else if (sensorValue == 0b111100) mappedValue = -50;
  else if (sensorValue == 0b101100) mappedValue = -50;
}

void PID_Correction(void)
{
  error = targetValue - mappedValue;
  sum_error += error;
  
  correction = ((Kp * error) + (Ki * (error + prev_error) / 2) + (Kd * (error - prev_error))); 

  prev_error = error;
  motorResponse = (int)correction;
  
  if(motorResponse > maxSpeed) motorResponse = maxSpeed;
  if(motorResponse < -maxSpeed) motorResponse = -maxSpeed;

  if(motorResponse < 0)
  {
    RMspeed = maxSpeed;
    LMspeed = maxSpeed + motorResponse;
  }
  else
  {
    RMspeed = maxSpeed - motorResponse;
    LMspeed = maxSpeed;
  }
}

void Stop(void)
{
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);

  analogWrite(10, 0);
  analogWrite(11, 0);
}

void Forward(void)
{
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
}

