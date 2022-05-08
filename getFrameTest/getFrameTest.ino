uint8_t *frame;
int numLasers = 1;
int thisTime = 0;
int prevTime = 0;
int signalPin = 2;

/*void laser_showCol(int currentCol) {
  int column = frame[currentCol];
  int bitMask = 0b00000001; // THIS IS GOING TO BREAK IF THERE ARE MORE THAN 8 LASERS OR SOMETHING
  for (int i = 0; i < numLasers; i++) {
    int state = column & (bitMask << i);
    Serial.println(state);
    laser_setState(i, state);
  }
}*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(signalPin, INPUT);
}

void loop() {
  int currentCol = 0;
  Serial.print(digitalRead(signalPin));
}
