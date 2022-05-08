// Canvas declarations
int canvas_signalPin = 2;
const int canvas_averageRpsOverCounts = 10 ;
volatile unsigned int canvas_signalHighCount = 0;
volatile unsigned int canvas_signalLowCount = 0;
volatile unsigned long canvas_highTimes[canvas_averageRpsOverCounts];
volatile unsigned long canvas_lowTimes[canvas_averageRpsOverCounts];
long prevTime = 0;
int prevCounts = 0;

// Canvas functions
float canvas_getRps() {
  float nrRevs = ((float) canvas_averageRpsOverCounts - 1) / 2; // The number of revolutions between the two times being compared
  float highRps = (canvas_highTimes[canvas_signalHighCount % canvas_averageRpsOverCounts] - canvas_highTimes[(canvas_signalHighCount + 1) % canvas_averageRpsOverCounts]);
  highRps = nrRevs / highRps;
  highRps = highRps * 1000000;
  float lowRps = (canvas_lowTimes[canvas_signalLowCount % canvas_averageRpsOverCounts] - canvas_lowTimes[(canvas_signalLowCount + 1) % canvas_averageRpsOverCounts]);
  lowRps = nrRevs / lowRps;
  lowRps = lowRps * 1000000;
  float rps = (highRps + lowRps) / 2;
  return rps;
}

// Interrupts
void canvas_signalInterrupt() {
  int state = digitalRead(canvas_signalPin);
  if (state) {
    canvas_signalHighCount++;
    canvas_highTimes[canvas_signalHighCount % canvas_averageRpsOverCounts] = micros();
  } else {
    canvas_signalLowCount++;
    canvas_lowTimes[canvas_signalLowCount % canvas_averageRpsOverCounts] = micros();
  }
}

void setup() {
  // Canvas
  pinMode(canvas_signalPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(canvas_signalPin), canvas_signalInterrupt, CHANGE);
  Serial.begin(9600);
}

void loop() {
  long time = micros();
  if (time - prevTime > 1000000) {
    float rps = canvas_getRps();
    Serial.print("Rps:");
    Serial.println(rps);
    prevTime = time;
  }
}
