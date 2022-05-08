int incomingByte = 0; // for incoming serial data
int onboardLedPin = 13;
int indicatorLaserPin = 3;

void setup() {
  Serial.begin(115200);
  pinMode(onboardLedPin, OUTPUT);
  pinMode(indicatorLaserPin, OUTPUT);
  digitalWrite(onboardLedPin, HIGH);
  delay(100);
  digitalWrite(onboardLedPin, LOW);
  delay(100);
}

void loop() {
  
  if (Serial.available() > 0) {

    int len = Serial.available();
    int buf[len];
    int i = 0;
    while (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }

    for (int j = 0; j < len; j++) {
      Serial.print(buf[j], DEC);
    }
    Serial.print(" ");
    Serial.println(len);
  }
}
