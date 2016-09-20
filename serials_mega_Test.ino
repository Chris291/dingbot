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
#define FEEDBACK_FREQUENCY_COUNT 1000/FEEDBACK_FREQUENCY
#define SAMPLETIME (5000/FEEDBACK_FREQUENCY)
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY
#define NUMBER_CONNECTED_NANOS 8
#define INITIAL_LENGTH_COMMAND "l00000000000000000000000000000000"
#define BAUD_RATE 115200

#define HEX_DIGITS_ANGLE 2
#define HEX_DIGITS_LENGTH 4

#define ASCII_MIDDLE_POINT 75 //breakpoint between cw(0-9, A-F) and ccw (P-Y, a-f)
#define ASCII_DIFFERENCE 32 //difference for conversion between cw and ccw

#define RECEIVE_ANGLE 'a'

#define RADIUS 200 //spool, 20mm in 0.1mm precision

unsigned long int t_ref; unsigned long int t_ref_receive;
String receivedCommand;
String receivedFeedback;
String sendFeedback; //easier to have this as char[]?
unsigned int lastLength[NUMBER_CONNECTED_NANOS]; //unsigned int has 2 bytes, range 0 - 65535
unsigned int lastLengthFeedback[NUMBER_CONNECTED_NANOS]; //with .1 mm precision, this equals ~6.5m

String combinedTest;
int counter = 0;
boolean led;

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
   SoftwareSerial for receiving from individual nanos
*/
void setup() {
  Serial.begin(BAUD_RATE);  //USB
  Serial1.begin(BAUD_RATE); //broadcast
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) { //all the softwareSerials for arduino nano
    serialNano[i].begin(BAUD_RATE);
  }
  Serial.println("SETUP");
  pinMode(13, OUTPUT);
  t_ref = millis();
  t_ref_receive = t_ref;
  combinedTest = "";
  led = false;
  //receivedCommand = INITIAL_LENGTH_COMMAND;
  /*
     Initialization needed:
        initialLength
        initialLengthFeedback
  */
}
/*
  wait for command from MatLab, and pass the comand to nanos
*/


/* Main loop acts to interface with MATLAB (asynchronously) and nano at 20Hz */
void loop() {
  Serial.print(".");
  /*if (Serial.available() > 0) {  //MATLAB via USB
    receivedCommand = Serial.readStringUntil('\n');
    }
  */
  // Operate at roughly 20Hz time
  if ((millis() - t_ref) > TIME_STEP * 1000) {
    // Reset the time (AT A LATER DATE PROTECTION MAY BE NEEDED FOR OVERFLOW
    t_ref = millis();
    /* Request Feedback from the nanos */
    /*char feedbackNano[HEX_DIGITS_ANGLE]; // Array to store the nano feedback
      boolean is_positive = 1; // flag to indicate positive angle change
      int angularChangeReceived; // The change in angular value that was recieved
      unsigned int lengthFeedback; // The length value for feedback
      char feedbackMega[HEX_DIGITS_LENGTH + 1]; // The mega feedback (to be given to the nano)
      // Loop of feedback requests
      //for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    */

    Serial1.println("aU0");
    

    for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
      serialNano[i].listen();
      Serial1.println('t' + String(i)); //concatenates f and number of nano - sends feedback requests to the nanos
      if (i == 0) {
        if (led) {
          digitalWrite(13, LOW);
        } else {
          digitalWrite(13, HIGH);
        }
        led = !led;
      }


      Serial1.flush();
      counter = 0;
      while ((serialNano[i].available() == 0) && counter < 1300) {
        //delayMicroseconds(150);
        counter++;
        if (counter > 1299) {
          Serial.println(counter);
        }
      }

      if (serialNano[i].available() > 0) {
        receivedFeedback = serialNano[i].readStringUntil('\n'); // check
        combinedTest = combinedTest + receivedFeedback;
        int x = receivedFeedback[0];
        if((x-'0') != i){
          Serial.println(x);
          Serial.println(counter);
          Serial.println("delay" + String(i));
          
        }
        //Serial.println(receivedFeedback);
        //Serial.flush();
        /*for (int j = 0; j < HEX_DIGITS_ANGLE; j++) {
          feedbackNano[j] = receivedFeedback[j + 2]; //omits 'f*' as feedback prefix
          }
          if (feedbackNano[0] > ASCII_MIDDLE_POINT) {
          feedbackNano[0] -= ASCII_DIFFERENCE; //changes [0] to 0-9, A-F -> can be handled by hex conversion
          is_positive = 0; //detects sign of manipulated ASCII
          } else {
          is_positive = 1;
          }
          //Serial.print(millis());
          //Serial.print("  ");
          if (is_positive) {
          angularChangeReceived = strtol(feedbackNano, 0, 16);
          } else {
          angularChangeReceived = -strtol(feedbackNano, 0, 16); //uses sign to determine integer value from hex conversion
          }
          }
        */
        
      }
    }
    Serial.print(combinedTest);
    combinedTest = "";
    Serial.println(millis() - t_ref_receive);
    if((millis() - t_ref_receive) > 75){
      Serial.println(counter);
    }
    t_ref_receive = millis();
    
    /*lengthFeedback = lastLengthFeedback[i] + ((float)angularChangeReceived * (M_PI*RADIUS)) / 180.0; //converts to cable length
      lastLengthFeedback[i] = lengthFeedback;
      itoa(lengthFeedback, feedbackMega, 16); //converts to hex to send it to MATLAB

      /
      itoa(angularChangeReceived, feedbackMega, 16);
      for(int j=0; j < HEX_DIGITS_LENGTH; j++){  //fills sendFeedback array at right position, no conversion necessary
      sendFeedback[HEX_DIGITS_LENGTH*i + j] = feedbackMega[j]; //any prefix while sending to MATLAB?

      }
      }
      //if a nano gives no feedback, do nothing(keep the last feedback value in the combinedFeedback)
      //}
      //Serial.println(sendFeedback);
      //Serial.flush();*/


    /* Set up send command for the nano */
    /*  char sendCommand[HEX_DIGITS_ANGLE * NUMBER_CONNECTED_NANOS + 1];
      int lengthChange;
      int angleChange;
      char commandNano[HEX_DIGITS_ANGLE + 1];

      sendCommand[0] = RECEIVE_ANGLE; // Should be an angle
      for(int i=0; i < NUMBER_CONNECTED_NANOS; i++){
      //for (int i = 0; i < 1; i++) {
        char tmp[HEX_DIGITS_LENGTH + 1];
        for (int j = 0; j < HEX_DIGITS_LENGTH; j++) {
          tmp[j] = receivedCommand[HEX_DIGITS_LENGTH * i + j + 1]; //HEX_DIGITS_LENGTH*i gives position in array for respective ID, +1 omits command prefix
        }
        // tmp should be the length command for this case.  Confirm that this is true.
        lastLength[i] = 30;
        lengthChange = strtol(tmp, 0, 16) - lastLength[i]; //strtol returns long int, lastLength is unsigned int (4byte - 2byte), changes will not be >int_max
        //lastLength[i] += lengthChange; //update lastlength for next command
        if (lengthChange < 0) {
          is_positive = 0;
          lengthChange = abs(lengthChange);
        } else {
          is_positive = 1;
        }
        angleChange = ((float)lengthChange / (M_PI * RADIUS)) * 180.0;
        //angleChange = 0; is_positive = 1;
        itoa(angleChange, commandNano, 16);
        if (commandNano[1] == '\0') {
          commandNano[1] = commandNano[0];
          commandNano[0] = '0';
          commandNano[2] = '\0';
        }
        if (is_positive) {
          /* out of 4 cases, only two have to be handled here, the others are:
            is_positive and standard conversion letters 0-9 -> not to be changed
            negative and standard conversion letters a-f -> not to be changed
    */

    /*    if (commandNano[0] > '9') { //this case represents letters a-f, but with a is_positive sign
          commandNano[0] -= ASCII_DIFFERENCE; //A-F after subtraction
        }
      } else if (commandNano[0] < 'A') { //this case represents 0-9, but with negative sign
        commandNano[0] += ASCII_DIFFERENCE; //P-Y after addition
      }
      for (int j = 0; j < HEX_DIGITS_ANGLE; j++) {
        sendCommand[HEX_DIGITS_ANGLE * i + j + 1] = commandNano[j];
      }
      }
      Serial1.println(sendCommand);
      Serial1.flush();
      }
    */

  }
}

