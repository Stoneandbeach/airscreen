byte incomingByte;
byte startByte = 0b00000001;
boolean incomingMessage = 0;
byte readIdx = 0;

void setup() {
  Serial.begin(115200);

}

void loop() {

  if (Serial.available()) {
    delay(50);
    incomingByte = Serial.read();
    /*if (incomingByte == 1) {
      Serial.write(incomingByte);  
    }*/
    delay(50);
    Serial.write(incomingByte);
  }
}
