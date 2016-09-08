/**
 * MEGA
 * 
 * send via TX1
 * receive via digital pins
 * 
 * Not all pins on the Mega and Mega 2560 support change interrupts,
 * so only the following can be used for RX:
 * 10, 11, 12, 13, 14, 15, 50, 51, 52, 53,
 * A8 (62), A9 (63), A10 (64), A11 (65),
 * A12 (66), A13 (67), A14 (68), A15 (69).
 */

#include <SoftwareSerial.h>

// Defining constants
#define FEEDBACK_FREQUENCY 20// In Hz
#define FEEDBACK_FREQUENCY_COUNT 1000/FEEDBACK_FREQUENCY
#define SAMPLETIME (5000/FEEDBACK_FREQUENCY)
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY
#define NUMBER_CONNECTED_NANOS 8

#define HEX_DIGITS_ANGLE 2
#define HEX_DIGITS_LENGTH 4

#define ASCII_MIDDLE_POINT 75 //breakpoint between cw(0-9, A-F) and ccw (P-Y, a-f)
#define ASCII_DIFFERENCE 32 //difference for conversion between cw and ccw

#define RECEIVE_ANGLE 'a'

#define RADIUS 200 //spool, 20mm in 0.1mm precision

unsigned long int t_ref;

SoftwareSerial serialNano[8] = {
  SoftwareSerial (10, 22), // RX, TX
  SoftwareSerial (11, 23), //1
  SoftwareSerial (12, 24), //2
  SoftwareSerial (62, 25), //3*
  SoftwareSerial (63, 26), //4
  SoftwareSerial (64, 27), //5
  SoftwareSerial (50, 28), //6
  SoftwareSerial (51, 29)  //7
  };

  void setup() {
    Serial.begin(115200);  //USB
    Serial1.begin(115200); //broadcast
    for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) { //all the softwareSerials for arduino nano
      serialNano[i].begin(115200);
    }
    Serial.println("Mega is online."); 
    Serial.flush();
    t_ref = millis();

    /*
    *  Initialization needed:
     *    initialLength
     *    initialLengthFeedback
     */
  }
/*
  wait for command from MatLab, and pass the comand to nanos
 */
String receivedCommand;
String receivedFeedback;
String sendFeedback; //easier to have this as char[]?
unsigned int lastLength[NUMBER_CONNECTED_NANOS]; //unsigned int has 2 bytes, range 0 - 65535
unsigned int lastLengthFeedback[NUMBER_CONNECTED_NANOS]; //with .1 mm precision, this equals ~6.5m

void loop() {
  if (Serial.available() > 0) {  //USB
    receivedCommand = Serial.readStringUntil('\n');
  }

  if((millis() - t_ref) > TIME_STEP * 1000){
    t_ref = millis();

    /// REQUEST FEEDBACK FROM NANOS
    char feedbackNano[HEX_DIGITS_ANGLE];
    boolean positive = 1;
    int angularChangeReceived;
    unsigned int lengthFeedback;
    char feedbackMega[HEX_DIGITS_LENGTH];

    for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
      serialNano[i].listen();
      Serial1.println("f" + String(i)); //concatenates f and number of nano - sends feedback requests to the nanos
      Serial1.flush();
      if (serialNano[i].available() > 0) {
        receivedFeedback = serialNano[i].readStringUntil('\n'); 
        for (int j=0; j<HEX_DIGITS_ANGLE; j++){
          feedbackNano[j] = receivedFeedback[j+1]; //omits 'f' as feedback prefix
        }
        if(feedbackNano[0] > ASCII_MIDDLE_POINT){
          feedbackNano[0] -= ASCII_DIFFERENCE; //changes [0] to 0-9, A-F -> can be handled by hex conversion
          positive = 0; //detects sign of manipulated ASCII
        } 
        else positive = 1;

        if(positive){
          angularChangeReceived = strtol(feedbackNano, 0, 16);
        } 
        else angularChangeReceived = -strtol(feedbackNano, 0, 16); //uses sign to determine integer value from hex conversion
        
        lengthFeedback = lastLengthFeedback[i] + ((float)angularChangeReceived * (M_PI*RADIUS)) / 180.0; //converts to cable length
        lastLengthFeedback[i] = lengthFeedback;
        itoa(lengthFeedback, &feedbackMega[0], 16); //converts to hex to send it to MATLAB
        for(int j=0; j < HEX_DIGITS_LENGTH; j++){  //fills sendFeedback array at right position, no conversion necessary
          sendFeedback[HEX_DIGITS_LENGTH*i + j] = feedbackMega[j]; //any prefix while sending to MATLAB?
        }
      } 
      //if a nano gives no feedback, do nothing(keep the last feedback value in the combinedFeedback)
    }
    Serial.println(sendFeedback);
    Serial.flush();

    ///SEND RECEIVED COMMAND TO NANOS
    char sendCommand[HEX_DIGITS_ANGLE * NUMBER_CONNECTED_NANOS];
    int lengthChange;
    int angleChange;
    char commandNano[HEX_DIGITS_ANGLE];

    sendCommand[0] = RECEIVE_ANGLE;
    for(int i=0; i < NUMBER_CONNECTED_NANOS; i++){
      char tmp[HEX_DIGITS_LENGTH];
      for(int j=0; j < HEX_DIGITS_LENGTH; i++){
        tmp[j] = receivedCommand[HEX_DIGITS_LENGTH * i + j + 1]; //HEX_DIGITS_LENGTH*i gives position in array for respective ID, +1 omits command prefix  
      }
      lengthChange = strtol(tmp, 0, 16) - lastLength[i]; //strtol returns long int, lastLength is unsigned int (4byte - 2byte), changes will not be >int_max
      lastLength[i] += lengthChange; //update lastlength for next command
      if(lengthChange < 0){
        positive = 0;
        lengthChange = abs(lengthChange);
      } 
      else positive = 1;
      angleChange = ((float)lengthChange / (M_PI*RADIUS)) * 180.0;
      itoa(angleChange, &commandNano[0], 16);
      if(positive){
        /* out of 4 cases, only two have to be handled here, the others are:
         *  positive and standard conversion letters 0-9 -> not to be changed
         *  negative and standard conversion letters a-f -> not to be changed
         */
        if(commandNano[0] > '9'){ //this case represents letters a-f, but with a positive sign
          commandNano[0] -= ASCII_DIFFERENCE; //A-F after subtraction
        }
      } 
      else if (commandNano[0] < 'A'){ //this case represents 0-9, but with negative sign
        commandNano[0] += ASCII_DIFFERENCE; //P-Y after addition
      }
      for(int j=0; j < HEX_DIGITS_ANGLE; j++){
        sendCommand[HEX_DIGITS_ANGLE * i + j + 1] = commandNano[j];      
      }
    }
    Serial1.println(sendCommand);
    Serial1.flush();
  }
}






