/*
 * nano
 * 
 * receive via TX0
 * send via RX0
 * 
 * software serial baud rate: max. 57600 for 8MHz CPU, 
 * 115200 for 16MHz CPU (mega, nano)
 */

#include <string.h>
#include <Wire.h>
#include <math.h>

#define NANO_ID 6

#define FEEDBACK_FREQUENCY 40// In Hz
#define SAMPLETIME (5000.0/FEEDBACK_FREQUENCY)
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY
#define LENGTH_HEX_NUM_DIGITS 2
#define Kp 1.0 // Gain for proportional controller
#define Ki 0.5 // Gain for integrator
#define DELTA 10 // The quarter degree as freezing regions at crossing area

#define RECEIVE_ANGLE_CMD 'a'
#define RECEIVE_FEEDBACK_REQUEST 'f'
#define RECEIVE_TEST_REQUEST 't'

#define ASCII_MIDDLE_POINT 75
#define ASCII_DIFFERENCE 32


#define MOTOR_PIN 2
/////////////////////////// DEBUGGING AND TIMING VARIABLES //////////////

unsigned long int t_ref;
/////////////////////////// SERVO PARAMETERS ///////////////////////////

/// PWM scale for position feedback from servo ///
int maximumPWMFeedback = 1490; //1972 highest measured;
int minimumPWMFeedback = 480; //625 lowest measured;
double stepPWMFeedback = (float)(maximumPWMFeedback - minimumPWMFeedback) / 1440.0; //360 degree in quarter degree precision -> 1440 steps

/// PWM scale for position output to servo ///
int maximumPWMOutput = 1485; //1475 highest working, but sometimes errors
int minimumPWMOutput = 475; //460 lowest working, but sometimes errors
double stepPWMOutput = (float)(maximumPWMOutput - minimumPWMOutput) / 1440.0; //360 degree in quarter degree precision -> 1440 steps

/// scale for velocity output to servo ///
int clockwise_max = 2183;
int clockwise_min = 2083;
int anticlockwise_max = 1788;
int anticlockwise_min = 1883;

double initialDeg = -1;

int angularChangeReceived;
int angularChangeFeedback;


/////////////////////////// FEEDBACK VARIABLES ///////////////////////////

/// servo position as 'pwm' value (see _feedback for scale above) ///
int servoPWM;

/////////////////////////// OUTPUT VARIABLES ///////////////////////////

/// derived aim of angle ///
int destinationDeg = 0;
int destinationPWM;

/// control communication with serial and servo ///
boolean firstTime = 1;
boolean enableServo = 0;
boolean stillmode = 0;
boolean positive = 0;

/////////////////////////// FUNCTION PRECALLING ///////////////////////////

//void setup_timer1();
void sendFeedback();
void readSerial();
int readPositionFeedback();
void crossing();
void quitCrossing();
void ctrl_motor();
void mapping();
void limitDegree();
void deadzone();
void servoPulse(int servoPin, int pulseWidth);
void control();
void updateDestinationDeg();
void readAngularChange();


/////////////////////////// CONTROL VARIABLES ///////////////////////////

/// controlling of the method crossing() ///
boolean left = 0;
boolean right = 0;
boolean cross = 0;
int crossPulse = 0;
boolean check = 0;
/// control for errors ///
double ref, ITerm, lastErr, dInput;
unsigned long lastTime;
String strReceived;

void setup() {
  Serial.begin(115200);
}

void loop() {
  readSerial();
  if((millis() - t_ref) > TIME_STEP*1000){
    t_ref = millis();
    readPositionFeedback(); //reads position feedback from servo and calculates angle
    updateDestinationDeg(); //updates the destination degree from the serial monitor (if no input  readSerial();
    limitDegree(); //keeps the destinationDegree within 0 - 360 degree
    if (cross ==true)
    {
      crossing(); //
      quitCrossing();
    }
    if(enableServo) //if input has been received, transmission to servo is enabled
    {
      ctrl_motor(); //transmits the output signal towards the motor
    }
  }
}


void readSerial() //receive characterizing prefix (+ length in 2 digit Hex, with manipulation of first bit for sign)
{
  if (Serial.available() > 0){
    strReceived = Serial.readStringUntil('\n');    
    char command = strReceived[0];
    if(command == RECEIVE_ANGLE_CMD){
      Serial.println('a');
      if(firstTime){
        enableServo = 1;
        firstTime = 0;
      }
      readAngularChange();
    } 
    else if(command == RECEIVE_FEEDBACK_REQUEST){
      Serial.println('f');
      // FOR THE MOMENT THIS ONLY SUPPORTS 10 OPTIONS

      int receivedID = int(strReceived.charAt(1)) - '0';
      if(receivedID == NANO_ID){
        sendFeedback();
      }     
    }
    else if(command ==  RECEIVE_TEST_REQUEST){
      Serial.print(RECEIVE_TEST_REQUEST);
      Serial.println(NANO_ID);  
    }
  }
  // ADD CALIBRATION LATER
}


void readAngularChange(){
  char tmp[2];
  tmp[0] = strReceived[LENGTH_HEX_NUM_DIGITS*NANO_ID + 1];
  tmp[1] = strReceived[LENGTH_HEX_NUM_DIGITS*NANO_ID + 2];

  if(tmp[0] > ASCII_MIDDLE_POINT){
    tmp[0] -= ASCII_DIFFERENCE;       
    positive = 0;
  }  
  else positive = 1;

  if(positive){
    angularChangeReceived = strtol(tmp, 0, 16);
  } 
  else angularChangeReceived = -strtol(tmp, 0, 16);
}

int readPositionFeedback()
{//reads position feedback from servo and returns calculated angle
  int lastPWM = servoPWM; //temporarily stores last value as backup, if new measurement fails - not used anywhere else

  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);
  servoPWM = pulseIn(MOTOR_PIN, HIGH, 2000); //triggers servo, then measures time until next HIGH signal, cuts off after 3000us or 3ms

  if ((servoPWM < 300) || (servoPWM > 2000)) { //results outside these boundaries are faulty 
    servoPWM = lastPWM;
  }
  return inverseMapping(servoPWM); //converts the pwm value to an angle and returns it
}

void updateDestinationDeg(){//updates the destination degree from the serial monitor (if no input, no change)
  int servoDeg = readPositionFeedback();
  if (initialDeg < 0)
  { //first time this method is entered
    initialDeg = servoDeg;
  }
  if (angularChangeReceived != 0) 
  {
    destinationDeg = servoDeg + angularChangeReceived; // update the destination degree by using current position plus requested angular change
    Serial.println(servoDeg);
    Serial.println(angularChangeReceived);
    Serial.println(destinationDeg);
    angularChangeReceived = 0;
  }

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void limitDegree(){//keeps the destinationDegree within 0 - 360 degree, e.g. -100 for example 375 will map to 15, -100 will map to 260 
  if (destinationDeg < (-DELTA)) //negative degrees should be mapped from 360 downwards
  {
    destinationDeg = (1440 -  fabs(fmod(destinationDeg, 1440))); //the modulo ensures a value between -1 and -360, then subtracting the absolute value of this from 360
    cross = true;
  }
  if (destinationDeg > (1440 + DELTA))  //values above 360 will be mapped to 0 to 360
  {
    destinationDeg = fmod(destinationDeg, 1440); //modulo takes out all integral multiples of 360 to achieve correct mapping    //Serial.println("crossLeft");
    //Serial.println(positive);
    cross = true;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void crossing(){
  int Currentpos = readPositionFeedback();
  if(check !=1){
    if (positive == 0) // From right
    {  /*
      int deltaDegree = 1440 - (destinationDeg - Currentpos);
     Serial.println(deltaDegree);
     if(deltaDegree < 50)
     {
     crossPulse = anticlockwise_min;
     }
     else if (deltaDegree > 400)
     {
     crossPulse = anticlockwise_max;  
     }
     else
     {
     crossPulse = anticlockwise_min - deltaDegree*0.2857 ;  //0.667 is the ratio between 350 steps in deltaDegree and 100steps between max and min speed
     }
     check = 1;
     */
      crossPulse = anticlockwise_max;
    }
    else if (positive == 1) // From left
    { /*
      int deltaDegree = 1440 + (destinationDeg - Currentpos);
     Serial.println(deltaDegree);
     if(deltaDegree < 50)
     {
     crossPulse = clockwise_min;
     }
     else if (deltaDegree > 400)
     {
     crossPulse = clockwise_max;  
     }
     else
     {
     crossPulse = clockwise_min + deltaDegree*0.2857;  
     }
     check = 1;
     */
      crossPulse = clockwise_max; 
    }
  }
}

void quitCrossing(){
  int Currentpos = readPositionFeedback();
  if (positive == 0) // From right
  {  
    if((Currentpos < (1440 - DELTA))&&(Currentpos > 720 )){
      cross = false;
      check = 0;
    }
  }
  else if (positive == 1) // From left
  {
    if((Currentpos > DELTA)&&(Currentpos < 720 )){
      cross = false;
      check = 0;
    }
  }
}

void ctrl_motor(){//transmits the output signal towards the motor
  if (cross == true)//ding
  {
    servoPulse(MOTOR_PIN, crossPulse);
    quitCrossing();
  }
  else
  {
    mapping();
    servoPulse(MOTOR_PIN, destinationPWM);
  }
}

void servoPulse(int servoPin, int pulseWidth){
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(servoPin, LOW);
  delayMicroseconds(3000 - pulseWidth);
}

void mapping(){// maps the calculated destinationDeg onto the pwm_output scale
  destinationPWM = minimumPWMOutput + destinationDeg * stepPWMOutput; 
  //deadzone();
  control();
}

int inverseMapping(int pwmFeedback){
  //inversely maps detected pwm to a degree
  return ((double)(pwmFeedback - minimumPWMFeedback) / stepPWMFeedback) + 0.5;
}


/*void deadzone (){//ding
 if ((destinationPWM >= (maximumPWMFeedback - 10)) || ((left == true) && (destinationPWM < (minimumPWMFeedback + 10))))
 {
 destinationPWM = maximumPWMOutput - 10;
 if (right == false)
 {
 left = true;
 }
 }
 if ((left == true) && ((destinationPWM < (maximumPWMFeedback - 10)) && (destinationPWM > 1200)))
 {
 left = false;
 }
 
 if ((destinationPWM <= (minimumPWMFeedback + 10)) || ((right == true) && (destinationPWM > (maximumPWMFeedback - 10))))
 {
 destinationPWM = minimumPWMOutput + 10;
 if (left == false)
 {
 right = true;
 }
 }
 if ((right == true) && ((destinationPWM > (minimumPWMFeedback + 10)) && (destinationPWM < 800)))
 {
 right = false;
 }
 }
 */

void control() {// Control basically enhance the signal for motor to achieve the position that commanded to, consist of Proportional and integral parts.
  if ((servoPWM < destinationPWM - 30) || (servoPWM >  destinationPWM + 30))
  {
    ITerm = 0; // The integrater will trigger once the feedback is close enough(30pwm value) to the destinationPWM. 
  }
  else
  {
    unsigned long now = millis();
    int timeChange = (now - lastTime);
    int error = destinationPWM - servoPWM ;
    if (timeChange >= (SAMPLETIME - 50)) // the controller will trigger every 50millis
    {
      ITerm += (Ki * error);
      lastTime = now;
    }
    destinationPWM = destinationPWM + Kp * error + ITerm; // here is the final output on pwm signal
  }
}

void sendFeedback(){
  /*
  *general idea: first byte of 2-digit hex is manipulated to signal ccw or cw rotation
   *if first hex contains 0-9, A-F (ASCII 48-57, 65-70) -> clockwise
   *if first hex contains P-Y, a-f (ASCII 80-89, 97-102) -> counterclockwise
   *
   *
   */
  static int lastDeg;
  static int currentDeg;
  lastDeg = currentDeg;
  currentDeg = readPositionFeedback(); //calculates the changed in detected angle since last time feedback was sent

  angularChangeFeedback = currentDeg - lastDeg;
  if(angularChangeFeedback < 0){
    positive = 0;
    angularChangeFeedback = abs(angularChangeFeedback);
  } 
  else positive = 1;

  char sendFeedback[2];
  itoa(angularChangeFeedback, &sendFeedback[0], 16); //converts into hex, format: 0-9, a-f

  if(positive){
    /* out of 4 cases, only two have to be handled here, the others are:
     *  positive and standard conversion letters 0-9 -> not to be changed
     *  negative and standard conversion letters a-f -> not to be changed
     */
    if(sendFeedback[0] > '9'){ //this case represents letters a-f, but with a positive sign
      sendFeedback[0] -= ASCII_DIFFERENCE; //A-F after subtraction
    }
  } 
  else if (sendFeedback[0] < 'A'){ //this case represents 0-9, but with negative sign
    sendFeedback[0] += ASCII_DIFFERENCE; //P-Y after addition
  }


  Serial.print(NANO_ID); //as first byte of string send?
  Serial.print(sendFeedback[0]);
  Serial.println(sendFeedback[1]);
  Serial.println();
  Serial.flush();
}


