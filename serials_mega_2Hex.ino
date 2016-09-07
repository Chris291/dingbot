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

bool ledState = LOW;
unsigned long int t_ref;
unsigned long int loopTime;
int count = 0;
String printString = "l00000001001000110100010101100111";
char testByte = '0';

SoftwareSerial serialNano[8] = {SoftwareSerial (10, 22), // RX, TX
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

  //LED for debug uses
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.println("Mega is online."); 
  Serial.flush();
  //OCR0A = 0xAF;
  //TIMSK0 |= _BV(OCIE0A);
  //setup_timer1();
  t_ref = millis();
}

/*
  wait for command from MatLab, and pass the comand to nanos
*/
String command;
String request;
String feedback;
String combinedFeedback;
void loop() {
  if (Serial.available() > 0) {  //USB
    command = Serial.readStringUntil('\n');
    Serial1.println(command);    //broadcast command to nanos via Serial1
    Serial1.flush();
  }

  if((millis() - t_ref) > TIME_STEP * 1000){
    t_ref = millis();
    count++;
    if(count > 2000){
      Serial.println("STOP");
      Serial.flush();
    }else if(count % 10 == 1){
      //Serial.println(count - 1);
      //Serial.flush();
      //Serial.println("LENGTH_CHANGE");
      //Serial.flush();
      if(testByte == '9'){
        testByte = '0';
      } else {
        testByte++;
      }
      printString[3] = testByte;
      Serial1.println(printString);      
      Serial1.flush();
    } else {
      for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
        serialNano[i].listen();
        
        request = "f" + String(i);
        Serial1.println(request); 
        Serial1.flush();
        //Serial.println("broadcasted: " + request);  //for debugging
        //Serial.flush();                             //for debugging
        
        if (serialNano[i].available() > 0) {
          feedback = serialNano[i].readStringUntil('\n');
          ledState = !ledState;           //for debugging
          digitalWrite(13, ledState);     //for debugging
          //Put feedback into certain place inside combinedFeedback, corresponding to the Nano ID (to-do!)
          //Serial.println("From Nano" + String(i) + ":" + feedback);  //for debugging: pass Nano feedback to computer
          //Serial.flush();                                            //for debugging
          
        } //if a nano gives no feedback, do nothing(keep the last feedback value).
      }
    }

  //loopTime = millis() - t_ref;   //for debugging
  //Serial.println(loopTime);      //for debugging
  //Serial.flush();                //for debugging
    
  }
}

/*
   Periodically requests feedback from the nanos, collects and combine them, and pass to MatLab
*/

/*ISR(TIMER1_COMPA_vect) {
  //unsigned long int t_0 = micros();
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    serialNano[i].listen();
    
    request = "f" + String(i);
    Serial1.println(request); 
    Serial1.flush();
    Serial.println("broadcasted: " + request);  //for debugging
    Serial.flush();                             //for debugging
    
    if (serialNano[i].available() > 0) {
      feedback = serialNano[i].readStringUntil('\n');
      ledState = !ledState;           //for debugging
      digitalWrite(13, ledState);     //for debugging
      //Put feedback into certain place inside combinedFeedback, corresponding to the Nano ID (to-do!)
      Serial.println("From Nano" + String(i) + ":" + feedback);  //for debugging: pass Nano feedback to computer
      Serial.flush();                                            //for debugging
      
    } //if a nano gives no feedback, do nothing(keep the last feedback value).
  }
//  Serial.println(combinedFeedback);  //send combinedFeedback to MatLab via USB
//  Serial.flush();
}*/

/*void setup_timer1() {  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to desired timer count:
  OCR1A = (15624 / FEEDBACK_FREQUENCY);
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}*/



