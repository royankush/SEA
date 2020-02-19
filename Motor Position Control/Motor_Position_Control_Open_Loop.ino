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


int displayTime = 200;
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
double setpoint_angle, measured_angle = 0;



/* Various Flags and variables for cases */
char option; // for switch case
boolean loop_flag = false; // For setting new rpm
boolean startingTorque = false; // For running starting torque function once
boolean stall_flag = false; // To check if motor is stalled


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

    pwm = map(motor_speed, 0, RPM_max, 0, 255);
           
    loop_flag = true;  
    
  }

/* Reading the control commands whenever availabale for CW, CCW rotations and stop  */
/************************************************************************************/ 
  if(Serial.available()>0)
  {
    option = Serial.read();
    Serial.println(option);   
  } 


/* Motor Control cases - Forward, Reverse, Stop and RPM change */
/***************************************************************/

    if(abs(measured_angle) >= abs(setpoint_angle))
    {
      stall(); 
      stall_flag = true;        
    }

      
    if(option == 'm')
    { 

      if(startingTorque == false && stall_flag == false)
      {
        
        if(encoderValue >= 0)
          start(true);
        else
          start(false);

        startingTorque = true;
      }

      if(encoderValue >= 0)
      {
        analogWrite(enable_1,pwm);
        analogWrite(enable_2,pwm);
        forward();
      }
      else
      {
        analogWrite(enable_1,pwm);
        analogWrite(enable_2,pwm);
        reverse();
      }
     
    }
    else if(option == 's')
    {
      brake();
      startingTorque = false;

    }
    else if(option == 'c')   /* to change motor rpm */
    {
      angleChange();
      startingTorque = false;
      stall_flag = false;     
      encoderValue = 0;
      
    }
    else if(option == 't')
    {
      stall();
      stall_flag = true;
     
    }

   
    now = millis();

    if(now - prev > displayTime)
    {
      angleMeasurement();
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
    Serial.println(encoderValue);

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
      delay(100);
    }
    else if(flag == false)
    {
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, HIGH);
      delay(100);
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
