int fanSignalPin = 2;
float rps = 0.0;
int lastState = 0;
unsigned int lastTime = 0;
int stateChangeCount = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(fanSignalPin, INPUT);
  lastState = digitalRead(fanSignalPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  int fanSignal = digitalRead(fanSignalPin);
  if (fanSignal != lastState) { // We have state change on the signal pin
    stateChangeCount++;
    lastState = fanSignal;
  }
  unsigned int currentTime = millis();
  unsigned int dt = currentTime - lastTime;
  if (dt >= 1000) { // One second has passed since last rps estimate
    rps = stateChangeCount / (dt / 1000.0);
    stateChangeCount = 0;
    lastTime = millis();
    Serial.println(rps);
  }
  //Serial.println(fanSignal);
}
