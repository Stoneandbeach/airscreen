int fanSignalPin = 2;
float rps = 0.0;
unsigned long rpsEstimateTime = 1000000; // in microseconds
int lastState = 0;
unsigned long lastRPSTime = micros();
float microsPerRevolution = 1000000.0; // Initialized to some high value so we don't flash the laser until we have an rps estimate.
unsigned long lastLaserTime = micros();
int stateChangeCount = 0;
float revolutionCount = 0.0;
unsigned long lastFlashTime;
int laserPin = 4;
int laserState = LOW;
int flashDuration = 400; // in microseconds. Note: there appears to be some interrupt or something on the Arduino, which makes it so that after about 1000 microseconds of laser on (and sometimes earlier), the loop time is slowed down by a lot. This causes the flash to be over 10 milliseconds long instead.
int controlLoopsCounter = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(fanSignalPin, INPUT);
  pinMode(laserPin, OUTPUT);
  lastState = digitalRead(fanSignalPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  int fanSignal = digitalRead(fanSignalPin);
  if (fanSignal != lastState) { // We have state change on the signal pin
    stateChangeCount++;
    revolutionCount = stateChangeCount / 4;
    lastState = fanSignal;
  }
  unsigned long currentTime = micros();
  if (revolutionCount >= 10) {
    unsigned long dt = currentTime - lastRPSTime;
    rps = revolutionCount / (dt / 1000000.0); // Calculate rps.
    microsPerRevolution = dt / (revolutionCount);
    lastRPSTime = micros();
    Serial.println("New estimate:");
    Serial.println(revolutionCount);
    Serial.println(rps);
    Serial.println(microsPerRevolution);
    stateChangeCount = 0;
    revolutionCount = 0;
  }
  laser_control();
}

void laser_control() {
  static int flashDelay = 2000;
  unsigned long currentTime = micros();
  if (laserState) { // Check for how long the laser has been on. Turn it off after flashDuration microseconds.
    unsigned long laserOnTime = currentTime - lastFlashTime;
    if (laserOnTime >= flashDuration) {
      laser_toggle();
    }
  } else {
    unsigned long dt = currentTime - lastFlashTime;
    //Serial.println(dt);
    if (dt >= microsPerRevolution + flashDelay) { // One revolution has happened. Flash the laser.
      laser_toggle();
      //flashDelay += 5;
      Serial.println(flashDelay);
    }
  }

  
}

void laser_toggle() {
  if (laserState) {
    digitalWrite(laserPin, LOW);
    laserState = LOW;
  } else {
    digitalWrite(laserPin, HIGH);
    laserState = HIGH;
    lastFlashTime = micros();
  }
}
