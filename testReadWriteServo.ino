#define MOTOR_PIN 5

int lastPWM, servoPWM;
int cmdPWM;
String strReceived;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(Serial.available() >0){
    strReceived = Serial.readStringUntil('\n');    
    char tmp[4];
    for(int i=0; i<4; i++){
      tmp[i] = strReceived[i];
    }
    cmdPWM = strtol(tmp, 0, 10);
    digitalWrite(MOTOR_PIN, HIGH);
    delayMicroseconds(cmdPWM);
    digitalWrite(MOTOR_PIN, LOW);
    delayMicroseconds(3000 - cmdPWM);
    Serial.println(cmdPWM);
  }
  lastPWM = servoPWM;
  
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);
  servoPWM = pulseIn(MOTOR_PIN, HIGH, 2000); //triggers servo, then measures time until next HIGH signal, cuts off after 3000us or 3ms

  if ((servoPWM < 300) || (servoPWM > 2000)) { //results outside these boundaries are faulty 
    servoPWM = lastPWM;
  }
  Serial.println(servoPWM); //converts the pwm value to an angle and returns it  
}
