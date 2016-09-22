/**
   MEGA

   send via TX1
   receive via digital pins

   Not all pins on the Mega and Mega 2560 support change interrupts,
   so only the following can be used for RX:
   10, 11, 12, 13, 14, 15, 50, 51, 52, 53,
   A8 (62), A9 (63), A10 (64), A11 (65),
   A12 (66), A13 (67), A14 (68), A15 (69).
*/

#include <SoftwareSerial.h>

// Defining constants
#define FEEDBACK_FREQUENCY 20// In Hz
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY
#define NUMBER_CONNECTED_NANOS 8
#define INITIAL_LENGTH_COMMAND "l00000000000000000000000000000000"
#define BAUD_RATE 74880

#define HEX_DIGITS_ANGLE 2
#define HEX_DIGITS_LENGTH 4

#define ASCII_MIDDLE_POINT 75 //breakpoint between cw(0-9, A-F) and ccw (P-Y, a-f)
#define ASCII_DIFFERENCE 32 //difference for conversion between cw and ccw

#define RECEIVE_ANGLE 'a'
#define REQUEST_FEEDBACK 'f'
#define REQUEST_ACCUMULATIVE_FEEDBACK 'z'

#define RADIUS 200 //spool, 20mm in 0.1mm precision

unsigned long int t_ref;
String receivedCommand;
String receivedFeedback;
String sendFeedback; //easier to have this as char[]?
unsigned int lastLengthCommand[NUMBER_CONNECTED_NANOS]; //unsigned int has 2 bytes, range 0 - 65535
unsigned int lastLengthFeedback[NUMBER_CONNECTED_NANOS]; //with .1 mm precision, this equals ~6.5m
char feedbackNano[HEX_DIGITS_ANGLE]; // Array to store the nano feedback
boolean positive = 1; // flag to indicate positive angle change
int angularChangeReceived; // change in angular value that was recieved
unsigned int lengthFeedback; // length value for feedback
char feedbackMega[HEX_DIGITS_LENGTH + 1]; // The mega feedback (to be given to the nano)

char sendCommand[HEX_DIGITS_ANGLE * NUMBER_CONNECTED_NANOS + 1];
int lengthChange;
int angleChange;
char commandNano[HEX_DIGITS_ANGLE + 1];
char tmpSend[HEX_DIGITS_LENGTH];


float piRdiv180 = (M_PI * RADIUS) / 180.0;
float piRmult180 = M_PI * RADIUS * 180.0;
int counter = 0;
boolean corruptedFeedback[NUMBER_CONNECTED_NANOS];

SoftwareSerial serialNano[8] = {
  SoftwareSerial (62, 19), // RX, TX - 0
  SoftwareSerial (63, 23), //1 THERE IS AN ISSUE WITH THIS PIN
  SoftwareSerial (64, 24), //2
  SoftwareSerial (65, 25), //3
  SoftwareSerial (66, 26), //4
  SoftwareSerial (67, 27), //5
  SoftwareSerial (68, 28), //6
  SoftwareSerial (69, 29)  //7
};

/* Setup 3 different serial lines.
   Serial for MATLAB
   Serial1. for transmission to Nano devices
   SoftwareSerial for receiving from individual Nanos
*/
void setup() {
  Serial.begin(BAUD_RATE);  //USB
  Serial1.begin(BAUD_RATE); //broadcast
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) { //all the softwareSerials for arduino nano
    serialNano[i].begin(BAUD_RATE);
    //serialNano[i].setTimeout(10);
  }
  Serial.println("SETUP");
  t_ref = millis();
  //receivedCommand = INITIAL_LENGTH_COMMAND;
  /*
     Initialization needed:
        initialLength
        initialLengthFeedback
  */
}

/* Main loop acts to interface with MATLAB (asynchronously) and nano at 20Hz */
void loop() {
  /*if (Serial.available() > 0) {  //MATLAB via USB
    receivedCommand = Serial.readStringUntil('\n');
    }
  */
  // Operate at roughly 20Hz time
  if ((millis() - t_ref) > TIME_STEP * 1000) {
    // Reset the time (AT A LATER DATE PROTECTION MAY BE NEEDED FOR OVERFLOW
    t_ref = millis();

    // Request Feedback from the nanos

    // Loop of feedback requests
    for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
      serialNano[i].listen();
      if (!corruptedFeedback[i]) {
        Serial1.println(REQUEST_FEEDBACK); //requests feedback from nano
      } else Serial1.println(REQUEST_ACCUMULATIVE_FEEDBACK); //requests accumulative feedback from nano
      Serial1.flush(); //waits for the sending of Serial to be complete before moving on

      counter = 0;
      while ((serialNano[i].available() == 0) && counter < 500) {
        counter++;
      }
      if (serialNano[i].available() > 0) {
        for (int j = 0; j <= HEX_DIGITS_ANGLE; j++) { // <= to include check sum byte
          feedbackNano[j] = serialNano[i].read();
        }
        while (serialNano[i].available() > 0) {
          serialNano[i].read(); //clears the buffer of any other bytes
        }
        if ((feedbackNano[0] + feedbackNano[1]) == feedbackNano[2]) { //checks against check sum byte - also check for ascii range?
          corruptedFeedback[i] = 0;
          feedbackNano[2] = '\0'; //cuts off the check sum bit for strtol, which uses the feedbackNano array

          if (feedbackNano[0] > ASCII_MIDDLE_POINT) {
            feedbackNano[0] -= ASCII_DIFFERENCE; //changes [0] to 0-9, A-F -> can be handled by hex conversion
            positive = 0; //detects sign of manipulated ASCII
          } else {
            positive = 1;
          }

          if (positive) {
            angularChangeReceived = strtol(feedbackNano, 0, 16);
          } else {
            angularChangeReceived = -strtol(feedbackNano, 0, 16); //uses sign to determine integer value from hex conversion
          }

          lengthFeedback = lastLengthFeedback[i] + ((float)angularChangeReceived * piRdiv180); //converts to cable length
          lastLengthFeedback[i] = lengthFeedback;
          itoa(lengthFeedback, feedbackMega, 16); //converts to hex to send it to MATLAB
          for (int j = 0; j < HEX_DIGITS_LENGTH; j++) { //fills sendFeedback array at right position, no conversion necessary
            sendFeedback[HEX_DIGITS_LENGTH * i + j] = feedbackMega[j]; //any prefix while sending to MATLAB?
          }
        } else corruptedFeedback[i] = 1; //if this is set to 1, it will request the accumulative feedback in the next loop

      }
      else corruptedFeedback[i] = 1; //if no feedback can be received, request accumulative feedback next loop
    }
    Serial.println(sendFeedback); //sendFeedback is never reset -> if corruption is detected or no feedback received, just keeps old value)
    Serial.flush();

    // Set up to send command for the nano
    sendCommand[0] = RECEIVE_ANGLE; // Should be an angle
    for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
      for (int j = 0; j < HEX_DIGITS_LENGTH; j++) {
        tmpSend[j] = receivedCommand[HEX_DIGITS_LENGTH * i + j + 1]; //HEX_DIGITS_LENGTH*i gives position in array for respective ID, +1 omits command prefix
      }
      lengthChange = strtol(tmpSend, 0, 16) - lastLengthCommand[i]; //strtol returns long int, lastLengthCommand is unsigned int (4byte - 2byte), changes will not be >int_max
      lastLengthCommand[i] += lengthChange; //update lastLengthCommand for next command

      if (lengthChange < 0) {
        positive = 0;
        lengthChange = abs(lengthChange);
      } else {
        positive = 1;
      }

      angleChange = (float)lengthChange / piRmult180;
      itoa(angleChange, commandNano, 16); //converts integer to string (ascii) in hex format
      if (commandNano[1] == '\0') { //if angular change value is <=15, only LSB is used - nano would receive this as MSB, if we don't shift
        commandNano[1] = commandNano[0]; //shifting the value to LSB position
        commandNano[0] = '0';
        commandNano[2] = '\0';
      }
      if (positive && (commandNano[0] > '9')) { //this case represents letters a-f, but with a is_positive sign
        commandNano[0] -= ASCII_DIFFERENCE; //A-F after subtraction
      } else if (!positive && (commandNano[0] < 'A')) { //this case represents 0-9, but with negative sign
        commandNano[0] += ASCII_DIFFERENCE; //P-Y after addition
      }
      /* out of 4 cases, only two have to be handled here, the others are:
        is_positive and standard conversion letters 0-9 -> not to be changed
        negative and standard conversion letters a-f -> not to be changed
      */

      for (int j = 0; j < HEX_DIGITS_ANGLE; j++) {
        sendCommand[HEX_DIGITS_ANGLE * i + j + 1] = commandNano[j];
      }
    }
    Serial1.println(sendCommand);
    Serial1.flush();
  }
}

