/*
  DigitalReadSerial

  Reads a digital input on pin 2, prints the result to the Serial Monitor

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/DigitalReadSerial
*/

// digital pin 2 has a pushbutton attached to it. Give it a name:
int fanSignalPin = 6;
long lowCount = 0;
long highCount = 0;
int lastTime = millis();
int time;
int fanSignal;
float rpm;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(fanSignalPin, INPUT_PULLUP);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  fanSignal = digitalRead(fanSignalPin);
  // print out the state of the button:
  if (fanSignal == 0) {
    lowCount++;
  } else
  highCount++;
  //Serial.println(fanSignal);
  time = millis();
  /*if (time > lastTime + 500) {
    lastTime = time;
    Serial.print("Low: ");
    Serial.println(lowCount);    
    Serial.print("High: ");
    Serial.println(highCount);
    Serial.println();
  }*/
  if (time > lastTime + 500) {
    
    float dt = time - lastTime;
    rpm = (float) highCount / dt;
    rpm = rpm * 2 * 60;
    Serial.print("Rpm: ");
    Serial.println(rpm);
    Serial.println(time);
    Serial.println(lastTime);
    Serial.println(highCount);
    Serial.println();
    lastTime = time;
    highCount = 0;
  }
}
