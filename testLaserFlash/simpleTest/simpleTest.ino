int lastTime = millis();
int lastFlashTime;
int laserPin = 4;
int laserState = LOW;
int flashDuration = 1;

void setup() {
  // put your setup code here, to run once:
  pinMode(laserPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int currentTime = millis();
  int dt = currentTime - lastTime;
  if (dt >= 1000) { // One second has passed. Flash the laser.
    laser_toggle();
    lastTime = currentTime;
  }
  if (laserState) { // Check for how long the laser has been on. Turn it off after flashDuration milliseconds.
    int laserOnTime = currentTime - lastFlashTime;
    if (laserOnTime >= flashDuration) {
      laser_toggle();
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
    lastFlashTime = millis();
  }
}
