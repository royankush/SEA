/* 
  DC brushed Motor Position control in Closed Loop PID Control
  Ankush Roy, Indian Institute of Technology Kharagpur

  Motor: Faulhaber DC Micromotor 2224006SR
  Motor V_max = 6V
  Encoder: IE2-512, 2 Channel
  Programming Board: Arduino UNO
  Motor Driver: L293d
  
*/

/* Encoder output to Arduino Interrupt pin */
#define encA 2
#define encB 3

/* For PWM  */
#define enable_1 6 
#define enable_2 9

/* For Motor A  */
#define input_1 4
#define input_2 5

/* Pulse counts from encoder */
long lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value

/* counter of milliseconds during interval for Encoder & Intervals for measurement (PID frequency) */
int sampleTime = 50;
long prevMillis = 0;
long currMillis = 0;

int displayTime = 100;
long now = 0;
long prev = 0;

/* Variables related to Motor RPM */
/* Max RPM measured from the motor at 6V is 7800 */
int RPM_max = 30;
float gearRatio = 298;
int motor_speed = 20; //Enter between 0-72 rpm

/* Motor encoder output pulse per rotation 
   Use 512 for normal & position control, 1024 for x2 and 2048 for x4 mode */
   /* we are using multipler as 1 for poisiton control since the update encoder fuction is using both falling and 
    *  rising edge to determine the position. Hence the max precision it can obtain is 512 ticks per revolution
    */
#define PPR_Encoder 7
#define Multiplier 4
#define PPR (PPR_Encoder*Multiplier)
#define max_ticks (PPR*gearRatio) //Number of ticks for 1 rev of output shaft

/* Process Variables */
int pwm;
long setpoint_ticks;
double setpoint_angle, measured_angle = 0.0;

/* For PID */
double Kp = 1.9;
double Ki = 0.03;
double Kd = 0.01;

double error = 0.0;
double integralError = 0.0;
double pid_output = 0.0;
double outputMax, outputMin;
double errorThreshold = 10;

#define FORWARD 0
#define REVERSE 1

int controllerDirection = FORWARD;


/* Various Flags and variables for cases */
char option; // for switch case
boolean loop_flag = false; // For setting new rpm
boolean startingTorque = false; // For running starting torque function once
boolean stall_flag = false; // To check if motor is stalled
boolean pid = false; // To start pid mode

void setup() {
  
  pinMode(enable_1, OUTPUT); // For the PWM
  pinMode(enable_2, OUTPUT); // For the PWM
  pinMode(input_1, OUTPUT);
  pinMode(input_2, OUTPUT);
  pinMode(encA, INPUT_PULLUP); //Set encoder input with internal pullup
  pinMode(encB, INPUT_PULLUP); //Set encoder input with internal pullup
  
  /* Attach interrupt */
  attachInterrupt(digitalPinToInterrupt(encA), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), updateEncoderB, CHANGE);

  /* Baud rate is very important! */
  Serial.begin(500000);
  Serial.println("Control Legend: 'm' to start moving, 's' for stop, 't' for stall ");
   
  OutputLimits(-1.5*max_ticks,1.5*max_ticks);

}


void loop() {
  
/* For setting new rpm to the motor  */
/*************************************/
  if(loop_flag == false)
  {
  
    String motor_angle = "Enter Motor Angle (0-360): ";
    Serial.println(motor_angle);
       
    while(!Serial.available()); //Wait until data is read
    
    setpoint_angle = (int)(Serial.parseInt());
    setpoint_ticks = map(setpoint_angle, 0, 360.0, 0, max_ticks);
      
    loop_flag = true;  
    
  }

/* Reading the control commands whenever availabale for CW, CCW rotations and stop  */
/************************************************************************************/ 
  if(Serial.available()>0)
  {
    option = Serial.read();
    Serial.println(option);   
  } 


/* PID Controller */
/*******************/

  currMillis = millis();
  
  if((currMillis - prevMillis) > sampleTime)
  {

   angleMeasurement();
     
   if(pid == true)
   { 
      PID_compute();
  
      pwm = map(abs(pid_output), 0, outputMax, 0, 255);
     
      if((error > -errorThreshold) && (error < errorThreshold)) /*If error is within a reasonable threshold, stop the motor */
      {
        stall();
        stall_flag = true;
        pid = false;
      }
      else
      {

        if(pwm < 100)
        {
          /*Give a starting torque, if rpm is too low */
          if(error < 0)
            start(false);
          else
            start(true);
        }

        if(error < 0)
        {
          analogWrite(enable_1, pwm);
          analogWrite(enable_2, pwm);
          reverse();
        }
        else
        {
          analogWrite(enable_1, pwm);
          analogWrite(enable_2, pwm);
          forward();
        }
      }
   }
   
    prevMillis = currMillis;
  } 
  

/* Motor Control cases - Forward, Reverse, Stop and RPM change */
/***************************************************************/

    if(option == 'm')
    { 
      
     pid = true;
     prevMillis = millis();
     
    }
    else if(option == 's')
    {
      brake();
      
      startingTorque = false;
      integralError = 0;
      pid = false;
    }
    else if(option == 'c')   /* to change motor rpm */
    {
      angleChange();
      
      startingTorque = false;
      stall_flag = false;
      /* Carefull with this */
      encoderValue = 0;
      integralError = 0;
      pid = false;
      
      
    }
    else if(option == 't')
    {
      stall();
      
      stall_flag = true;
      integralError = 0;
      pid = false;
      
    }
    else if(option == 'p')
    {
      Serial.println("Enter Kp: ");
      while(!Serial.available());
      double kp = (double)(Serial.parseFloat());
      Serial.println("Enter Ki: ");
      while(!Serial.available());
      double ki = (double)(Serial.parseFloat());
      Serial.println("Enter Kd: ");
      while(!Serial.available());
      double kd = (double)(Serial.parseFloat());
      
      SetPIDconstants(kp,ki,kd);
      
      Serial.println(" PID Constants Changed !");

      startingTorque = false;
      integralError = 0;
      stall_flag = false;
      pid = false;
    }

    now = millis();

    if(now - prev > displayTime)
    {
      print_data();
      prev = millis();
    }
  
}



/* -------------------------------------------------------------------------------------------*/
/*                                        FUNCTIONS                                           */
/* -------------------------------------------------------------------------------------------*/

/* Angle Measurement */
/* ************************/
void angleMeasurement()
{
   if(encoderValue >= 0)
      measured_angle = map(encoderValue, 0, max_ticks, 0.0, 360.0);
   else
      measured_angle = map(encoderValue, -max_ticks, 0, -360.0, 0.0);
}



/* PID Controller ****************************************************************************/
/*********************************************************************************************/

void PID_compute()
{  
  /* Compute all working error variables */
  
  error = setpoint_ticks - encoderValue; /* Proportional Error */
  integralError += (Ki*error); /* Integral Error */
  double dTicks = (encoderValue - lastEncoded); /* Differential Error */

  /* Constrain Integral Error */
  if(integralError > outputMax) integralError = outputMax;
  else if (integralError < outputMin) integralError = outputMin;


  /* Compute PID */
  pid_output = (Kp*error) + integralError - (Kd*dTicks);

  /* Constrain Output */
  if(pid_output > outputMax) pid_output = outputMax;
  else if (pid_output < outputMin) pid_output = outputMin; 
     
  lastEncoded = encoderValue;  

  /*
  Serial.print(" Error: ");
  Serial.print(error);
  */

}


void SetPIDconstants(double kp, double ki, double kd)
{
   if (kp<0 || ki<0|| kd<0) return;
 
  double sampleTimeInSec = ((double)sampleTime)/1000;
   Kp = kp;
   Ki = ki * sampleTimeInSec;
   Kd = kd / sampleTimeInSec;
 
  if(controllerDirection == REVERSE)
   {
      Kp = -Kp;
      Ki = -Ki;
      Kd = -Kd;
   }
}


void OutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outputMin = Min;
   outputMax = Max;
 
   if(pid_output > outputMax) pid_output = outputMax;
   else if(pid_output < outputMin) pid_output = outputMin;

   if(integralError > outputMax) integralError = outputMax;
   else if (integralError < outputMin) integralError = outputMin;
}



/* Interrupt serive routine function *********************************************************/
/*********************************************************************************************/

void updateEncoderA()
{
  // look for a low-to-high on channel A
  if (digitalRead(encA) == HIGH)
  { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encB) == LOW)
    {  
      encoderValue++;         // CW
    } 
    else
    {
      encoderValue--;       // CCW
    }
  }
  
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encB) == HIGH) 
    {   
      encoderValue++;          // CW
    } 
    else
    {
      encoderValue--;          // CCW
    }
  }

}

void updateEncoderB()
{
  // look for a low-to-high on channel B
  if (digitalRead(encB) == HIGH) 
  {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encA) == HIGH)
    {  
      encoderValue++;         // CW
    } 
    else
    {
      encoderValue--;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else
  { 
    // check channel A to see which way encoder is turning  
    if (digitalRead(encA) == LOW)
    {   
      encoderValue++;          // CW
    } 
    else 
    {
      encoderValue--;          // CCW
    }
  }
}


void print_data()
{
    Serial.print("Set Angle: ");
    Serial.print(setpoint_angle);
    Serial.print("\t Curr Angle: ");
    Serial.print(measured_angle);
    Serial.print("\t Set Ticks: ");
    Serial.print(setpoint_ticks);
    Serial.print("\t Encoder Ticks: ");
    Serial.print(encoderValue);
    Serial.print("\t Error: ");
    Serial.print(error);
    Serial.print("\t PID Output(pwm): ");
    Serial.println(pwm);
}

/* Motor Direction, Starting, stalling and Braking *****************************************************/
/*********************************************************************************************/

void forward()
{        
  digitalWrite(input_1, HIGH);
  digitalWrite(input_2, LOW);
}


void reverse()
{        
  digitalWrite(input_1, LOW);
  digitalWrite(input_2, HIGH);
}


void brake()
{
  digitalWrite(input_1, LOW);
  digitalWrite(input_2, LOW);
  Serial.println(" Motor Braking...");
  delay(500);
  
}


void stall()
{
  /*Let it slow down for fraction of a second */
  analogWrite(enable_1,0);
  analogWrite(enable_2,0);
  
  //digitalWrite(input_1, LOW);
  //digitalWrite(input_2, LOW);
  //delay(50);
  
  digitalWrite(input_1, HIGH);
  digitalWrite(input_2, HIGH);

  Serial.println(" Motor Stalled!!!");
  delay(500);
}


void start(boolean flag)
{
    /* Give a starting torque as a spike*/
    analogWrite(enable_1,255);
    analogWrite(enable_2,255);

    if(flag == true)
    {
      digitalWrite(input_1, HIGH);
      digitalWrite(input_2, LOW);
      delay(5);
    }
    else if(flag == false)
    {
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, HIGH);
      delay(5);
    }
    else
    {
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, LOW);
    }
    
}


void angleChange()
{
  digitalWrite(input_1, LOW);
  digitalWrite(input_2, LOW);
  loop_flag = false;
}
