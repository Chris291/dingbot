#define MOTOR_PIN 2

#define FEEDBACK_FREQUENCY 40// In Hz
#define SAMPLETIME (5000.0/FEEDBACK_FREQUENCY)
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY

int lastPWM, servoPWM;
int cmdPWM;
String strReceived;
unsigned long int t_ref;
char tmp[4];

void setup() {
  Serial.begin(74880);
  Serial.println("SETUP");
}

void loop() {
  if (Serial.available() > 0) {
    strReceived = Serial.readStringUntil('\n');
    for (int i = 0; i < 4; i++) {
      tmp[i] = strReceived[i];
    }
    cmdPWM = strtol(tmp, 0, 10);
  }

  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(cmdPWM);
  digitalWrite(MOTOR_PIN, LOW);
  delayMicroseconds(3000 - cmdPWM);


  if ((millis() - t_ref) > TIME_STEP * 1000) {
    t_ref = millis();

    lastPWM = servoPWM;
    digitalWrite(MOTOR_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(MOTOR_PIN, LOW);
    servoPWM = pulseIn(MOTOR_PIN, HIGH, 2000); //triggers servo, then measures time until next HIGH signal, cuts off after 3000us or 3ms

    if ((servoPWM < 300) || (servoPWM > 2000)) { //results outside these boundaries are faulty
      servoPWM = lastPWM;
    }
    Serial.print(cmdPWM);
    Serial.print(" cmd fb ");    
    Serial.print(servoPWM); //converts the pwm value to an angle and returns it
    Serial.print("     diff ");
    Serial.println(servoPWM - cmdPWM);
  }
}
