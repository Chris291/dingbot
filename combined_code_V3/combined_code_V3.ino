#include <string.h>
#include <Wire.h>
#include <math.h>

#define FEEDBACK_FREQUENCY 10// In Hz
#define FEEDBACK_FREQUENCY_COUNT 1000/FEEDBACK_FREQUENCY
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY
#define NUM_MOTORS 8
#define LENGTH_HEX_NUM_DIGITS 4
#define LENGTH_SEND_MAX 65536
#define LENGTH_MULT 10
#define RADIUS 20
#define SAMPLETIME (5000/FEEDBACK_FREQUENCY)

#define SEND_PREFIX_FEEDBACK 'f'
#define SEND_PREFIX_ERROR 'a'
#define RECEIVE_PREFIX_START 's'
#define RECEIVE_PREFIX_END 'e'
#define RECEIVE_PREFIX_INITIAL 'i'
#define RECEIVE_PREFIX_LENGTH_CMD 'l'
#define COMM_PREFIX_ACKNOWLEDGE 'a'

int timeCounter = 0;
int systemOn = 0;
double timeVal = 0.0;
double initLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};
double relLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};
double cableLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};
double cmdLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};
////////////////////////////////////////////////////initialize parameter motoers spec

int MOTOR_PINS[NUM_MOTORS] = {5, 6, 7, 8, 9, 10, 11, 12};
int maximum_pwm[NUM_MOTORS] = {1492, 1504, 1493, 1488, 1518, 1499, 1498, 1500};
int minimum_pwm[NUM_MOTORS] = {477, 485, 479, 476, 485, 480, 480, 485};
int clockwise_max[NUM_MOTORS] = {2400, 2400, 2400, 2400, 2450, 2450, 2450, 2450};
int clockwise_min[NUM_MOTORS] = {2090, 2090, 2070, 2070, 2100, 2090, 2090, 2090};
int anticlockwise_max[NUM_MOTORS] = {1750, 1650, 1700, 1650, 1650, 1650, 1700, 1650};
int anticlockwise_min[NUM_MOTORS] = {1850, 1870, 1850, 1850, 1880, 1870, 1850, 1870};
int crosspulse[NUM_MOTORS] = {0};
int changes[NUM_MOTORS];


int loopnumber[NUM_MOTORS] = {0};
int lastpos[NUM_MOTORS] = {0};
boolean left[NUM_MOTORS] = {0};
boolean right[NUM_MOTORS] = {0};
boolean cross[NUM_MOTORS] = {0};
unsigned long servo_pos[NUM_MOTORS];
double servo_deg[NUM_MOTORS]; 
double anglechange[NUM_MOTORS];

double gpos[NUM_MOTORS] = {0};
double *globalposition[NUM_MOTORS];
double destination[NUM_MOTORS];
double degree[NUM_MOTORS] = {0};
double initialpos[NUM_MOTORS];
double olddestination[NUM_MOTORS];
int motorarr[NUM_MOTORS] = {5, 3, 6, 2, 0, 4, 7, 1};
double globallength[NUM_MOTORS]; //

////////////////////////////////////////////////////Initialize parameter for motor
double ref, Kp, Ki, Kd, ITerm, lastErr, dInput;
unsigned long lastTime, lastTime2;
boolean before = false, after = false, stillmode = false, once = false, enableMotors = false;
double t = 0.0;
String str = "";
String *p = &str;
int counterha = 0;
int counterwa = 0;

// REMOVE THIS LATER
unsigned long isr_start = 0;

////////////////////////////////////////////////////Function precalling
void sendFeedback();
void setInitialLengths(String str);
void setCmdLengths(String str);
void readSerial();

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  // Setup timer0
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  setup_timer1();
  for(int i = 0; i < NUM_MOTORS; i++)
{
  globalposition[i] = &globallength[i];
}
}

void setup_timer1() {  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to desired timer count:
  OCR1A = (15624/FEEDBACK_FREQUENCY);
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12); 
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}
/////////////////////////////////////////////////////////// Feedback of motors
unsigned long posback(int servopin)
{
  unsigned long duration;
  digitalWrite(servopin, HIGH);
  delayMicroseconds(50);
  digitalWrite(servopin, LOW);
  duration = pulseIn(servopin, HIGH, 2000);
  return duration;
}

///////////////////////////////////////////////////////////Communication with Matlab

void sendFeedback()
{
  // Send the first character of F to indicate feedback
  Serial.print(SEND_PREFIX_FEEDBACK);
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    // Convert length from mm to mm * LENGTH_MULT to send
    double l_to_send = cableLengths[i] * LENGTH_MULT;
    // Round to nearest integer value
    int l_rounded = l_to_send + 0.5;
    char format[4];
    char s[LENGTH_HEX_NUM_DIGITS]; 
    sprintf(format, "%%.%dX", LENGTH_HEX_NUM_DIGITS);
    sprintf(s, format, l_rounded);
    Serial.print(s);
  }
  Serial.print("\n");
}

void readSerial()
{
  if (Serial.available() > 0)
  {
    String str = Serial.readStringUntil('\n');
    if (str[0] == COMM_PREFIX_ACKNOWLEDGE && str.length() == 1) //a
    {
      systemOn = 0;
      Serial.println(COMM_PREFIX_ACKNOWLEDGE);
    }
    
    else if (str[0] == RECEIVE_PREFIX_START && str.length() == 1) //s
    {
      systemOn = 1;
    }
    else if (str[0] == RECEIVE_PREFIX_END && str.length() == 1) //e
    {
      systemOn = 0;
      enableMotors = 1;
      
    }
    else if (str[0] == RECEIVE_PREFIX_INITIAL) //i
    {
      setInitialLengths(str);
      enableMotors = 1;
    }
    else if (str[0] == RECEIVE_PREFIX_LENGTH_CMD) //l
    {
      setCmdLengths(str);
      enableMotors = 1;
    }
    else
    {
    }
  }
}

void setInitialLengths(String str)
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    char tmp[LENGTH_HEX_NUM_DIGITS];
    for (int j = 0; j < LENGTH_HEX_NUM_DIGITS; j++)
    {
      tmp[j] = str[LENGTH_HEX_NUM_DIGITS*i + j + 1];
    }
    initLengths[motorarr[i]] = ((double)strtol(tmp, 0, 16))/LENGTH_MULT;
  }
}

void setCmdLengths(String str)
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    char tmp[LENGTH_HEX_NUM_DIGITS];
    for (int j = 0; j < LENGTH_HEX_NUM_DIGITS; j++)
    {
      tmp[j] = str[LENGTH_HEX_NUM_DIGITS*i + j + 1];
    }
    cmdLengths[motorarr[i]] = ((double)strtol(tmp, 0, 16))/LENGTH_MULT;
    // Serial.println(cmdLengths[motorarr[i]]);
  }
}

///////////////////////////////////////////////////////////Motors mechanism

int inverseMapping(int motor, double pwmValue)
{
  int min = minimum_pwm[motor];
  int max = maximum_pwm[motor];
  int difference = max - min;
  int degree = ((pwmValue - minimum_pwm[motor]) / difference *360);
  return degree;
}

void readFeedback() 
{
  for(int i = 0; i < NUM_MOTORS; i++)
  { int oldpulse = servo_pos[i];
    servo_pos[i] = posback(MOTOR_PINS[i]);
    if ((servo_pos[i] < 300) || (servo_pos[i]>2600)) {
      servo_pos[i] = oldpulse;
    }  
    servo_deg[i] = inverseMapping(i, servo_pos[i]);
  }
}

void globalpos () 
{
  for(int i=0; i< NUM_MOTORS; i++)
  {
  int min = minimum_pwm[i];
  int max = maximum_pwm[i];
  int difference = max - min;
   unsigned long currentpos = servo_pos[i];   
   int posChange = (currentpos - lastpos[i]);
   if(posChange >= 50)
   {
    posChange = -3;
   }
   if(posChange <= -50)
   {
    posChange = 3;
   }
  gpos[i] = (posChange * 2 * M_PI * RADIUS) / difference;
  *globalposition[i] += gpos[i];
   lastpos[i] = currentpos;
  }
}

void crossing() 
{
  for(int a = 0; a<NUM_MOTORS; a++)
  {
  degree[a] = rounding(degree[a]);
  }
  for(int i=0; i< NUM_MOTORS; i++)
  {
    if (left[i] == true) 
    { //    Serial.print(degree[i]);
      if (((degree[i] >= 1) && (degree[i] <= 20)) && (servo_deg[i] >= 300))
      {
 //    Serial.print("Case left true");
     cross[i] = true;
     crosspulse[i] = clockwise_min[i];
      }
       if((servo_deg[i] >= 2) && (servo_deg[i] <= 15)) 
        {
 //        Serial.print("exit");
          cross[i] = false;
          left[i] = false;
        }
    }
  if (right[i] == true)
   {
      if (((degree[i] <= 359) && (degree[i] >= 340)) && (servo_deg[i] <= 60))
      {
  //   Serial.print("Case right true");
     cross[i] = true;
     crosspulse[i] = anticlockwise_min[i];
      }
        if((servo_deg[i] <= 359) && (servo_deg[i] >= 340))
        {
    //     Serial.print("exit");
          cross[i] = false;
          right[i] = false;
        }
   }
  }
}
/////////////////////////////////////////////////////////Control of motors
void ctrl_motor(int motor, double degree)
{
//  int vmod = crossing(motor, rounding(degree));
  if(cross[motor] == true) 
  {
  servopulse(MOTOR_PINS[motor], crosspulse[motor]);
  digitalWrite(2, HIGH);
  }
  else
  {
  int pwm = mapping(motor, degree);
  servopulse(MOTOR_PINS[motor], pwm);
  digitalWrite(2, LOW);
  }
}

int mapping(int motor, double degree)
{ 
  int min = minimum_pwm[motor];
  int max = maximum_pwm[motor];
  int difference = max - min;
  degree = rounding(degree);
  int pwmValue= (difference * degree / 360 + min);
  pwmValue = deadzone(motor, pwmValue);
  pwmValue = control(pwmValue, motor);
  return pwmValue;
}

double rounding(double degree)
{
  if (degree < 0) 
  {
    degree = (360 -  fabs(fmod(degree, 360)));
  }
  if (degree > 360) 
  {
    degree = fmod(degree, 360);
  }
//  Serial.print("degree = ");
//  Serial.println(degree);
  return degree;
}

int deadzone (int motor, int pwmValue)
{
  //Serial.println(pwmValue);
  if ((pwmValue >= (maximum_pwm[motor] - 90)) || ((left[motor] == true) && (pwmValue < (minimum_pwm[motor] + 20))))
  {
   pwmValue = maximum_pwm[motor] - 15;
   if(right[motor] == false)
   {
   left[motor] = true;
  //        Serial.println("left");
   }
  }
  if((left[motor] == true) && ((pwmValue < (maximum_pwm[motor] - 20)) && (pwmValue > 1200)))
  {
    left[motor] = false;
   //      Serial.println("left false");
  }
 
  if ((pwmValue <= (minimum_pwm[motor] + 90)) || ((right[motor] == true) && (pwmValue > (maximum_pwm[motor] - 20))))
  {
   pwmValue = minimum_pwm[motor] + 15;
   if (left[motor] == false)
   {
   right[motor] = true;
   //            Serial.println("right");
   }
  }
  if((right[motor] == true) && ((pwmValue > (minimum_pwm[motor] + 20)) && (pwmValue < 600)))
  {
    right[motor] = false;
   //         Serial.println("right false");
  }
  return pwmValue;
}

void servopulse(int servopin, int pulsewidth)
{
  digitalWrite(servopin, HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servopin, LOW);
  delay(20 - pulsewidth / 1000);
}

double control(int ref, int motor) {
 double val=ref;
 double min = val-30;
 double max = val+30;
  if ((servo_pos[motor] < min) || (servo_pos[motor] > max)) 
    {
    ITerm = 0;
  // Serial.print("ITerm:");
  // Serial.println(ITerm);
    }
  else 
    {
     unsigned long now = millis();
    //Serial.print("now");
    //Serial.println(now);
     int timeChange = (now - lastTime);
     Kp = 1.0;
     Ki = 0.5;
     int error = ref - servo_pos[motor] ;
     //Serial.print("time change is ");
     //Serial.println(timeChange);
  if(timeChange >= (SAMPLETIME-50)) 
  {
     ITerm += (Ki * error);
     lastTime = now;
  }
  val = ref + Kp * error + ITerm;
  //Serial.print("val = ");
  //Serial.println(val);
  }
  
  return val;
}

double move() 
{ 
  if ( once == false)
  {
    for(int i=0; i< NUM_MOTORS; i++)
    {
      initialpos[i] = servo_deg[i];
    }
    once = true;
  }
  
  if (stillmode == false) 
  {
    for(int i = 0; i<NUM_MOTORS; i++)    
    { 
      changes[i] = olddestination[i] - cmdLengths[i];
      anglechange[i] = changes[i] * 180 / (M_PI * RADIUS);
      degree[i] = degree[i] - anglechange[i];
    }
  }
  if (stillmode == true)
  {
    for(int i = 0; i<NUM_MOTORS; i++)
    {
      if(servo_deg[i] >= (initialpos[i] - 1))
      {
      degree[i] = initialpos[i] -2;  
      }
    }
  }
}

/////////////////////////////////////////////////////////timer0 and timer1 loop
void loop()
{
readSerial();
}

SIGNAL(TIMER0_COMPA_vect)
{
  /*
  if (timeCounter < FEEDBACK_FREQUENCY_COUNT - 1)
  {
    timeCounter++;
  }
  else
  {
    timeCounter = 0;
    if (systemOn)
    {
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        cableLengths[i] = initLengths[i] + relLengths[i];
      }
      sendFeedback();
    }
    //timeVal += TIME_STEP;
    //Serial.println(timeVal);
  }*/
}

ISR(TIMER1_COMPA_vect) 
{
  readFeedback();
  globalpos();
  move();
  crossing();
  if(enableMotors)
  {
    for (int i = 0; i<NUM_MOTORS; i++)
    {
     ctrl_motor(i, degree[i]);
    }
  }
  if (systemOn)
    {
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        cableLengths[i] = initLengths[motorarr[i]] + relLengths[motorarr[i]];
      }
      sendFeedback();

    }
  for(int i = 0; i<NUM_MOTORS; i++)
  {
    olddestination[i] = cmdLengths[i];
    relLengths[i] = globallength[i];
  }

  t += TIME_STEP; 
  unsigned long int t_f = millis();
  //Serial.println(t_f - isr_start);
  isr_start = t_f;
    //timeVal += TIME_STEP;
    //Serial.println(t);
}

