#define targetValue                      0            
#define maxSpeed                       255


int sensePin1 = 0;
int sensePin2 = 1;
int sensePin3 = 2;
int sensePin4 = 3;
int sensePin5 = 4;
int sensePin6 = 5;

int compareValue[6] = {97, 140, 120, 130, 140, 150};
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

float Kp = 20;
float Ki = 0;
float Kd = 0;


int LMspeed, RMspeed;
int LMpin, RMpin;
int flag = 0;
int u_turn_flag = 0;

void sensorValueConversion(void);
void sensorMapping(void);
void PID_Correction(void);

void setup()
{
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
  
  //Serial.begin(9600);
}

void loop()
{
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  
  delay(1000);

  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
  
 while(1)
 {
  /*
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

*/
  
  sensorValueConversion();
  Serial.print(sensorValue,BIN);
  Serial.print("\t");
  sensorMapping();
  //Serial.println(mappedValue);

  PID_Correction();
  
  analogWrite(10, LMspeed);
  analogWrite(11, RMspeed);

/*
  
  Serial.print("\t");
  Serial.print(LMspeed);
  Serial.print("\t");
  Serial.print(RMspeed);
  Serial.print("\n");

*/
  
if(mappedValue == 100)
{
  
  sensorValueConversion();
  sensorMapping();
  PID_Correction();
  
  analogWrite(10, LMspeed);
  analogWrite(11, RMspeed); 
}


/*
  int discont = 0;
  while(mappedValue == 100)                     //Discontinuity sequence
  {
    discont++;
    sensorValueConversion();
    //Serial.println(sensorValue,BIN);
    sensorMapping();
    //Serial.println(mappedValue);
    if(discont > 1000)
    {
      LMspeed = 0;
      RMspeed = 0;
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
      digitalWrite(12, LOW);
      digitalWrite(13, LOW);
      delay(500);
    }
  } 
  
  /*
  Serial.print(LMspeed);
  Serial.print("\t");
  Serial.print(RMspeed);
  Serial.print("\t");
  Serial.print(flag);
  Serial.print("\n");
  */
  
 }

}

void sensorValueConversion(void)
{
  int digitalValue;
  sensorValue = 0;
    
  for (int sensorNumber = 6; sensorNumber > 0; sensorNumber--)
  { 
    if (analogRead(sensorNumber) < compareValue[sensorNumber]) digitalValue = 0;
    else digitalValue = 1;
    sensorValue |= (digitalValue << sensorNumber);
  }
  //Serial.println(sensorValue, BIN);
  
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

