#define rightMotorA 8
#define rightMotorB 9
#define leftMotorA 7
#define leftMotorB 6
#define rightMotor_power  10
#define leftMotor_power 5
#define setPoint 5

#define kp 60
#define kd 250
#define ki 0


int i = 0, sum = 0, avg = 0;
int error = 0, prev_error = 0;
int motor_res = 0, lmSpeed = 150, rmSpeed = 150, maxSpeed = 220, reverseSpeed = 0;
int correct = 0;
int rightFlag = 0, leftFlag = 0, allFlag = 0;
int timer = 0;
int count = 0;

void forward()
{
  digitalWrite(rightMotorA , HIGH);
  digitalWrite(rightMotorB , LOW);
  digitalWrite(leftMotorA , HIGH);
  digitalWrite(leftMotorB , LOW);
  analogWrite(rightMotor_power , maxSpeed);
  analogWrite(leftMotor_power , maxSpeed);
}

void backward()
{
  digitalWrite(rightMotorB , HIGH);
  digitalWrite(rightMotorA , LOW);
  digitalWrite(leftMotorB , HIGH);
  digitalWrite(leftMotorA , LOW);
  analogWrite(rightMotor_power , maxSpeed);
  analogWrite(leftMotor_power , maxSpeed);
}

void halt()
{
  digitalWrite(rightMotorA , LOW);
  digitalWrite(rightMotorB , LOW);
  digitalWrite(leftMotorA , LOW);
  digitalWrite(leftMotorB , LOW);
  analogWrite(rightMotor_power , 0);
  analogWrite(leftMotor_power , 0);
}

void setRotationForward()
{
  digitalWrite(rightMotorA , HIGH);
  digitalWrite(rightMotorB , LOW);
  digitalWrite(leftMotorA , HIGH);
  digitalWrite(leftMotorB , LOW);
}

void leftSharp()
{
  digitalWrite(rightMotorA , HIGH);
  digitalWrite(rightMotorB , LOW);
  digitalWrite(leftMotorA , LOW);
  digitalWrite(leftMotorB , HIGH);
  analogWrite(rightMotor_power , 80);
  analogWrite(leftMotor_power , 80);
}


void setRotationLeftSharp()
{
  digitalWrite(rightMotorA , HIGH);
  digitalWrite(rightMotorB , LOW);
  digitalWrite(leftMotorA , LOW);
  digitalWrite(leftMotorB , HIGH);
}

void rightSharp()
{
  digitalWrite(rightMotorB , HIGH);
  digitalWrite(rightMotorA , LOW);
  digitalWrite(leftMotorB , LOW);
  digitalWrite(leftMotorA , HIGH);
  analogWrite(rightMotor_power , 80);
  analogWrite(leftMotor_power , 80);
}

void setRotationRightSharp()
{
  digitalWrite(rightMotorB , HIGH);
  digitalWrite(rightMotorA , LOW);
  digitalWrite(leftMotorB , LOW);
  digitalWrite(leftMotorA , HIGH);
}

void ajairaSensor()
{
//  Serial.print(avg);
//  Serial.print("\t");
//  Serial.print(error);
//  Serial.print("\t");
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
  //delay(10);
}

void sensing()
{
  leftFlag = 0;
  rightFlag = 0;

  count = 0;
  for(i = 0 ; i < 6 ; i++)
  {
      if(analogRead(i) < 800)
      { 
      count++;
      sum += i * 2;
      if(i == 0) leftFlag = 1;
      else if(i == 5) rightFlag = 1;
      }   
  }
  avg = sum / count;
  sum = 0;
}


void pid(void)
{
    error = avg - setPoint;    
    motor_res = kp*error + kd*(error - prev_error);
    prev_error = error;
//    Serial.print(avg);
//    Serial.print('\t');
//    Serial.print(error);
//    Serial.print('\t');
//    Serial.print(motor_res);
//    Serial.print('\t');
//    if (motor_res >= 0)
//    {
//        lmSpeed = maxSpeed;
//        rmSpeed = maxSpeed - motor_res;
//    }
//    else if (motor_res < 0)
//    {
//        rmSpeed = maxSpeed;
//        lmSpeed = maxSpeed + motor_res;
//    }

//    lmSpeed = maxSpeed + motor_res;
//    rmSpeed = maxSpeed - motor_res;
//    if(rmSpeed > maxSpeed)  rmSpeed = maxSpeed;
//    if(lmSpeed > maxSpeed)  lmSpeed = maxSpeed;
//    if(rmSpeed < 0) rmSpeed = 0;
//    if(lmSpeed < 0) lmSpeed = 0;  

    if(motor_res > maxSpeed)
  {
    reverseSpeed = (motor_res - maxSpeed);
    if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
    motor_res = maxSpeed;
    
    setRotationRightSharp();
    
    rmSpeed = reverseSpeed;
    lmSpeed = maxSpeed;
  }
  
  else if(motor_res < -maxSpeed)
  {
    reverseSpeed = (-motor_res - maxSpeed);
    if (reverseSpeed > maxSpeed) reverseSpeed = maxSpeed;
    motor_res = -maxSpeed;
    
    setRotationLeftSharp();
    
    rmSpeed = maxSpeed;
    lmSpeed = reverseSpeed;
  }
  


  else if(motor_res >= 0 && motor_res < maxSpeed)
  {
    setRotationForward();
    
    rmSpeed = maxSpeed - motor_res;
    lmSpeed = maxSpeed;   
  }

  else if(motor_res < 0 && motor_res > -maxSpeed)
  {
    setRotationForward();
    
    rmSpeed = maxSpeed;
    lmSpeed = maxSpeed + motor_res;
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
  forward();
}


void loop() {

//forward();
//delay(1000);
//rightSharp();
//delay(1000);
//leftSharp();
//delay(1000);

  sensing();
  
  if(prev_error < 2 && prev_error > -2 && rightFlag == 1 && count < 6)
  {
    forward();
    timer = millis();
    while ((millis() - timer) < 3000)
    {
      sensing();
      if(count == 6 || count == 0) break;
      rightFlag = 0;
      leftFlag = 0;
    }
    
    if (count == 0)
    {
      rightSharp();
    }
    while ((millis() - timer) < 1000)
    {
      sensing();
      if(avg != -1)
      {
          pid();
          analogWrite(rightMotor_power , rmSpeed);
          analogWrite(leftMotor_power , lmSpeed);
      }
    }
      
//      sensing(); 
//      pid();
//      analogWrite(rightMotor_power , rmSpeed);
//      analogWrite(leftMotor_power , lmSpeed);
      
  }
  else if(prev_error < 2 && prev_error > -2 && leftFlag == 1 && count < 6)
  {
    forward();
    timer = millis();
    while ((millis() - timer) < 3000)
    {
      sensing();
      if(count == 6 || count == 0) break;
      rightFlag = 0;
      leftFlag = 0;
    }
    if(count == 0)
    {
      leftSharp();
    }    
    while ((millis() - timer) < 1000)
    {
      sensing();
      if(avg != -1)
    {
          pid();
          analogWrite(rightMotor_power , rmSpeed);
          analogWrite(leftMotor_power , lmSpeed);
      }
    }

//    sensing();
//    pid();
//    analogWrite(rightMotor_power , rmSpeed);
//    analogWrite(leftMotor_power , lmSpeed); 
}
  if(count == 6)
  {
    forward();
    allFlag++;
    if(allFlag > 100) // stop condition
    {
      backward();
      delay(20);
      halt();
      delay(5000);
    }
  }
  else if(count < 6) allFlag = 0;

//  //Serial.println(avg);
  if(avg != -1)
  {
      pid();
      analogWrite(rightMotor_power , rmSpeed);
      analogWrite(leftMotor_power , lmSpeed);
//      Serial.print(lmSpeed);
//      Serial.print('\t');
//      Serial.println(rmSpeed);
  }
  
//ajairaSensor();

}
