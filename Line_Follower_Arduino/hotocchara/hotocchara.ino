#define rightMotorA 4
#define rightMotorB 5
#define leftMotorA 6
#define leftMotorB 7
#define rightMotor_power 10
#define leftMotor_power 11
#define setPoint 5
//#define kp  10;
//#define kd  0;
//#define ki  0;
int i = 0, sum = 0, avg = 0;
int error = 0, prev_error = 0, cSum = 0;
int motor_res = 0, lmSpeed = 150, rmSpeed = 150, baseSpeed = 180;
int correct = 0;
int kp = 50, kd = 50, ki = 0;


void forward()
{
  digitalWrite(rightMotorA , HIGH);
  digitalWrite(rightMotorB , LOW);
  digitalWrite(leftMotorA , HIGH);
  digitalWrite(leftMotorB , LOW);
  analogWrite(rightMotor_power , rmSpeed);
  analogWrite(leftMotor_power , lmSpeed);
  
}
void leftSharp()
{
  digitalWrite(rightMotorA , HIGH);
  digitalWrite(rightMotorB , LOW);
  digitalWrite(leftMotorA , LOW);
  digitalWrite(leftMotorB , HIGH);
  analogWrite(rightMotor_power , 100);
  analogWrite(leftMotor_power , 100);
}

void rightSharp()
{
  digitalWrite(rightMotorB , HIGH);
  digitalWrite(rightMotorA , LOW);
  digitalWrite(leftMotorB , LOW);
  digitalWrite(leftMotorA , HIGH);
  analogWrite(rightMotor_power , 100);
  analogWrite(leftMotor_power , 100);
}

void ajairaSensor()
{
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
  Serial.println(analogRead(A5));
  delay(10);
}

void sensing()
{
  int count = 0;
  for(i = 0 ; i < 6 ; i++)
  {
    if(analogRead(i) > 500)
    { 
      count++;
      sum += i * 2;
    }   
  }
  avg = sum / count;
  sum = 0;
  

}
void pid(void)
{
  error = avg - setPoint;
  correct = kp*error + kd*(error - prev_error) + ki*cSum;
  prev_error = error;
  cSum += error;
  motor_res =  correct;
    
  lmSpeed = baseSpeed - motor_res;
  rmSpeed = baseSpeed + motor_res;
  if(rmSpeed > 255)
    rmSpeed = 255;
  if(lmSpeed > 255)
    lmSpeed = 255;
  if(rmSpeed < 0)
    rmSpeed = 0;
  if(lmSpeed < 0)
    lmSpeed = 0;
  
}




void setup() {
  // put your setup code here, to run once:

  pinMode(rightMotorA , OUTPUT);
  pinMode(rightMotorB , OUTPUT);
  pinMode(leftMotorA , OUTPUT);
  pinMode(leftMotorB , OUTPUT);
  pinMode(rightMotor_power , OUTPUT);
  pinMode(leftMotor_power , OUTPUT);
  Serial.begin(9600);
  forward();
}

void loop() {
  // put your main code here, to run repeatedly:
  ajairaSensor();
  sensing();
  while(avg == -1) sensing();
  
  pid();
  analogWrite(rightMotor_power , rmSpeed);
  analogWrite(leftMotor_power , lmSpeed);
  
}
