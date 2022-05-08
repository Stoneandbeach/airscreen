byte incomingByte;
const int onboardLedPin = 13;
const int frameLen = 10;
byte frame[frameLen];
int i = 0;
const int COM_REQUEST_FRAME = 0x0f;
const int COM_FRAME_RECEIVED = 0x0a;

void setup() {
  pinMode(onboardLedPin, OUTPUT);
  Serial.begin(115200);
  digitalWrite(onboardLedPin, HIGH);
  delay(200);
  digitalWrite(onboardLedPin, LOW);
  delay(200);
  Serial.write(COM_REQUEST_FRAME);
  
}

void loop() {
  if (Serial.available() && (i < frameLen)) {
    int readInt = Serial.read();
    frame[i] = (byte) readInt;
    i++;
  }
  if (i >= frameLen) {
    digitalWrite(onboardLedPin, HIGH);
    i = 0;
    Serial.write(frame, frameLen);
    digitalWrite(onboardLedPin, LOW);
  }
}
