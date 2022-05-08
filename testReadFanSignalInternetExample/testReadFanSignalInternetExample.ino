//More information at: https://www.aeq-web.com/

const int SensorPin = 14;  //Define Interrupt Pin (2 or 3 @ Arduino Uno)

int InterruptCounter, rpm;

void setup(){
  delay(1000);
  Serial.begin(9600);
  Serial.print("Counting");
}

void loop() {
  meassure();
}

void meassure() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(SensorPin));
  rpm = (InterruptCounter / 2) * 60;
  display_rpm();
}

void countup() {
  InterruptCounter++;
}

void display_rpm() {
  Serial.print("Counts: ");
  Serial.print(InterruptCounter, 1);
  Serial.print(" RPM: ");
  Serial.println(rpm);
}
