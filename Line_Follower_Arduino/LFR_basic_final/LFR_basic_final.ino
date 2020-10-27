#define EnableLeftMotor       10
#define EnableRightMotor      5
#define LeftMotorA            8 
#define LeftMotorB            9
#define RightMotorA           7
#define RightMotorB           6

#define SenseLeftPin          0
#define SenseMidPin           1
#define SenseRightPin         2

#define maxSpeed            150
//#define threshold           500

int SenseLeftValue = 0;
int SenseMidValue = 1;
int SenseRightValue = 2;

int threshold[3] = {800, 800, 600};

void PrintSensorValue(void);
void ReadSensorValue(void);

void setup() {
  // put your setup code here, to run once:
  pinMode(EnableLeftMotor, OUTPUT);
  pinMode(EnableRightMotor, OUTPUT);
  pinMode(LeftMotorA, OUTPUT);
  pinMode(LeftMotorB, OUTPUT);
  pinMode(RightMotorA, OUTPUT);
  pinMode(RightMotorB, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  ReadSensorValue();
  PrintSensorValue();


  if(SenseLeftValue < threshold[0] && SenseMidValue < threshold[1] && SenseRightValue < threshold[2]);
  else if(SenseLeftValue > threshold[0] && SenseMidValue < threshold[1] && SenseRightValue > threshold[2])
  {
    Forward();
  }
  else if(SenseLeftValue < threshold[0] && SenseMidValue < threshold[1] && SenseRightValue > threshold[2])
  {
    LeftSharp();
  }
  else if(SenseLeftValue < threshold[0] && SenseMidValue > threshold[1] && SenseRightValue > threshold[2])
  {
    LeftSharp();
  }
  else if(SenseLeftValue > threshold[0] && SenseMidValue < threshold[1] && SenseRightValue < threshold[2])
  {
    RightSharp();
  }
  else if(SenseLeftValue > threshold[0] && SenseMidValue > threshold[1] && SenseRightValue < threshold[2])
  {
    RightSharp();
  }
  else if(SenseLeftValue > threshold[0] && SenseMidValue > threshold[1] && SenseRightValue > threshold[2]);
  else if(SenseLeftValue < threshold[0] && SenseMidValue > threshold[1] && SenseRightValue < threshold[2])
  {
    Forward();
  }
}

void ReadSensorValue(void)
{
  SenseLeftValue = analogRead(SenseLeftPin);
  SenseMidValue = analogRead(SenseMidPin);
  SenseRightValue = analogRead(SenseRightPin);
}

void PrintSensorValue(void)
{
  Serial.print(SenseLeftValue);
  Serial.print("\t");
  Serial.print(SenseMidValue);
  Serial.print("\t");
  Serial.print(SenseRightValue);
  Serial.print("\n");
}

void Forward(void)
{
  digitalWrite(LeftMotorA, HIGH);
  digitalWrite(LeftMotorB, LOW);
  digitalWrite(RightMotorA, HIGH);
  digitalWrite(RightMotorB, LOW);
  analogWrite(EnableLeftMotor, maxSpeed);
  analogWrite(EnableRightMotor, maxSpeed);
}

void LeftSharp(void)
{
  digitalWrite(LeftMotorA, LOW);
  digitalWrite(LeftMotorB, HIGH);
  digitalWrite(RightMotorA, HIGH);
  digitalWrite(RightMotorB, LOW);
  analogWrite(EnableLeftMotor, maxSpeed);
  analogWrite(EnableRightMotor, maxSpeed);
}

void RightSharp(void)
{
  digitalWrite(LeftMotorA, HIGH);
  digitalWrite(LeftMotorB, LOW);
  digitalWrite(RightMotorA, LOW);
  digitalWrite(RightMotorB, HIGH);
  analogWrite(EnableLeftMotor, maxSpeed);
  analogWrite(EnableRightMotor, maxSpeed);
}
