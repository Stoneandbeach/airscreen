#include "math.h"

enum displayState_t {
  S_INIT = 0,
  S_SPINUP,
  S_READY,
  S_DARK,
  S_ACTIVE,
  S_STOPPED
};

enum displayCommand_t {
  NO_COMMAND = 0,
  GO_DARK,
  GO_ACTIVE,
  STOP
};

enum canvas_error_t {
  CANVAS_NO_ERROR = 0,
  CANVAS_TIMEOUT,       // This error is set if command GO_ACTIVE or GO_DARK is set while canvas_signalState == LOW
  CANVAS_NO_SIGNAL      // This error is set if the canvas motor has been asked to spin up but no canvas_signalState change is detected
};

// Display (i.e., overall) declarations
displayState_t displayState = S_INIT;
displayCommand_t displayCommand = NO_COMMAND;
int onboardLedPin = 13;
float display_radius = 75; // Radius of the swept volume in mm
float display_angleLimit = M_PI / 4; // Outer angle limit of the display volume, from axis parallel to laser beams
const int display_nrCols = 12;
const int display_colMax = display_nrCols - 1; // Index of the last column
float display_distanceFromAxis = 40; // Distance from canvas axis to display surface in mm
uint8_t frame[display_colMax]; // THIS NEEDS TO BE EXPANDED WHEN I HAVE MORE LASERS
int frameNr = 0;
int loadedFrameNr = 0;
long frameStartTime;
int prevCol = 0;

// Canvas declarations
float canvas_rps = 0;
float deltaRps;
float targetRps = 0;
volatile int canvas_signalCount = 0;
volatile int canvas_signalHighCount = 0;
volatile int canvas_signalLowCount = 0;
int canvas_prevSignalCount = 0;
int canvas_prevRpsSignalCount = 0;

const int canvas_averageRpsOverCounts = 10;
volatile long canvas_highTimes[canvas_averageRpsOverCounts];
volatile long canvas_lowTimes[canvas_averageRpsOverCounts];

const int canvas_signalPin = 2;

float steadyRpsChangeLimit = 0.05; // Percentage change limit to consider rps steady
int steadyRpsCountTarget = 10;
int steadyRpsCount = 0;

volatile int canvas_signalState;
canvas_error_t canvas_error = CANVAS_NO_ERROR;

bool fatalError = 0;

// Laser variables
const int numLasers = 5;
int laser_pins[numLasers] = {3, 4, 5, 6, 8};
int laser_states[numLasers];

// Display functions
void setCommand(displayCommand_t command) {
  if (!fatalError) {
    displayCommand = command;
  }
}

void changeState(displayState_t state) {
  if (!fatalError) {
    displayState = state;
  }
}

void display_loadFrame(uint8_t *frameArray, int nr) {
  uint8_t columns[11] = {0b00011111,
                         0b00000100,
                         0b00011111,
                         0b00000000,
                         0b00011111,
                         0b00010101,
                         0b00010101,
                         0b00000000,
                         0b00001001,
                         0b00010001,
                         0b00001111};
  /*if ((nr / 10) & 0b1) {
    uint8_t temp = dummy[0];
    dummy[0] = dummy[1];
    dummy[1] = temp;
  } */
  //Serial.print("Frame: ");
  for (int i = 0; i < display_nrCols; i++) {
    if (i < 11) {
      frameArray[i] = columns[i];
    } else {
      frameArray[i] = 0b00000000;
    }
    //Serial.println(frameArray[i]);
  }
}

// Canvas functions
void canvas_setError(canvas_error_t error) {
  canvas_error = error;
  switch (canvas_error) {
    case CANVAS_NO_SIGNAL:
      fatalError = 1;
      setCommand(STOP);
  }
}

float canvas_getRps() {
  if (canvas_signalCount > canvas_prevRpsSignalCount) {
    float nrRevs = ((float) canvas_averageRpsOverCounts - 1) / 2; // The number of revolutions between the two times being compared
    // Calculate delta-time between the latest and oldest recorded high signal times in the canvas_highTimes array.
    float highRps = (canvas_highTimes[canvas_signalHighCount % canvas_averageRpsOverCounts] - canvas_highTimes[(canvas_signalHighCount + 1) % canvas_averageRpsOverCounts]);
    if (highRps == 0) {
      return 0;
    }
    highRps = 1000000 * nrRevs / highRps;
    float lowRps = (canvas_lowTimes[canvas_signalLowCount % canvas_averageRpsOverCounts] - canvas_lowTimes[(canvas_signalLowCount + 1) % canvas_averageRpsOverCounts]);
    lowRps = 1000000 * nrRevs / lowRps;
    float rps = (highRps + lowRps) / 2;
    canvas_prevRpsSignalCount = canvas_signalCount;
    return rps;
  } else {
    return canvas_rps; // Rps has not updated since last polling, because we have gotten no new signals from the canvas
  }
}

int canvas_getCurrentCol(long frameTime, float rps) {
  // Calculate which column to display depending on rps and current frame time
  float t = ((float) frameTime) / 1000000; // Uptime of the current frame in seconds
  float w = rps * 2 * M_PI; // Angular frequency of the canvas
  float y = display_distanceFromAxis;
  float x = y / tan(w * t + display_angleLimit); // x position along the display surface. Angle is calculated in relation to the laser beam
  /*Serial.print(x);
  Serial.print(" ");
  Serial.println(y);
  Serial.print(" ");
  Serial.print(w);
  Serial.print(" ");
  Serial.print(t);
  Serial.print(" ");
  Serial.print(w * t + display_angleLimit);
  Serial.print(" ");
  Serial.println(tan(w * t + display_angleLimit));*/
  float D = 2 * display_radius * cos(display_angleLimit); // The width of the display surface
  int n = display_nrCols + 1; //+ Number of display columns, plus one last, dark, padded columns
  float colWidth = D / n;
  x = -x + y;
  for (int i = 0; i < display_nrCols; i++) {
    if ((x > i * colWidth) && (x < (i + 1) * colWidth)) { // Find the segment of D in which x lies
      return i;
    }
  }
  return -1; // We are in a dark, padded column
}

// Laser functions
void laser_setState(int laserIdx, int state) {
  laser_states[laserIdx] = state;
  digitalWrite(laser_pins[laserIdx], state);
}

int laser_getState(int laserIdx) { // This may never be needed.
  return laser_states[laserIdx];
}

void laser_showCol(int currentCol) {
  if (currentCol == -1) {
    for (int i = 0; i < numLasers; i++) {
      laser_setState(i, LOW);
    }
  } else {
    uint8_t column = frame[currentCol];
    uint8_t bitMask = 0b00000001; // THIS IS GOING TO BREAK IF THERE ARE MORE THAN 8 LASERS OR SOMETHING
    Serial.println();
    Serial.println(column);
    for (int i = 0; i < numLasers; i++) {
      uint8_t state = (column & (bitMask << i)) >> i;
      Serial.print(state);
      Serial.print(" ");
      laser_setState(i, state);
    }
    Serial.println();
  }
}

// Debugging

void debugPrint() {
  String debugStrings[] = {"displayState", "displayCommand", "signalState", "canvas_error", "frameNr", "loadedFrameNr"};
  int debugInts[] = {displayState, displayCommand, canvas_signalState, canvas_error, frameNr, loadedFrameNr};
  for (int i = 0; i < 6; i++) {
    Serial.print(debugStrings[i]);
    Serial.print(": ");
    Serial.print(debugInts[i]);
    Serial.print(" - ");
  }
  Serial.println();
}

long debugTimeNow = 0;
long debugTimeThen = 0;

void debugLasers() {
  debugTimeNow = micros() - canvas_highTimes[canvas_signalHighCount % canvas_averageRpsOverCounts];
  Serial.println(debugTimeNow);
  int col = canvas_getCurrentCol(debugTimeNow, canvas_rps);
  int debugLaserState = col % 2;
  /*float spr = 1000000 / canvas_rps;
  if (debugTimeNow - debugTimeThen > spr / 64) {
    debugLaserState = 1 - debugLaserState;
    debugTimeThen = debugTimeNow;
  }*/
  laser_setState(0, debugLaserState);
  /*Serial.print(" ");
  Serial.print(col);
  Serial.print(" ");
  Serial.println(digitalRead(canvas_signalPin));*/
}

// Interrupts
void canvas_signalInterrupt() {
  canvas_signalCount++;
  canvas_signalState = digitalRead(canvas_signalPin);
  if (canvas_signalState) {
    canvas_signalHighCount++;
    canvas_highTimes[canvas_signalHighCount % canvas_averageRpsOverCounts] = micros();
  } else {
    canvas_signalLowCount++;
    canvas_lowTimes[canvas_signalLowCount % canvas_averageRpsOverCounts] = micros();
  }
}

// Setup and main loop
void setup() {
  Serial.begin(115200);
  
  // Display
  pinMode(onboardLedPin, OUTPUT);
  
  // Canvas
  pinMode(canvas_signalPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(canvas_signalPin), canvas_signalInterrupt, CHANGE);
  // Here I will later initialize pins for controlling fan speed, if required.

  // Laser
  for (int i = 0; i < numLasers; i++) {
    pinMode(laser_pins[i], OUTPUT);
    laser_states[i] = LOW;
  }
}

void loop() {

  //debugLasers();
  
  switch (displayState) {
    case S_INIT:
      // Possibility of doing startup checks, like measuring feedback from diodes if available
      Serial.println("INIT");
      changeState(S_SPINUP);
      break;
    
    case S_SPINUP:
       if (canvas_signalCount > 2 * canvas_averageRpsOverCounts) {
         float prevRps = canvas_rps;
         canvas_rps = canvas_getRps();
         float deltaRpsChange = (fabs(canvas_rps - prevRps) - deltaRps) / deltaRps;
         if ((canvas_rps > 1) & (deltaRpsChange < steadyRpsChangeLimit)) {
            if (canvas_signalCount > canvas_prevSignalCount) {
              steadyRpsCount++;
              canvas_prevSignalCount = canvas_signalCount;
            }
            if (steadyRpsCount >= steadyRpsCountTarget) {
              Serial.print("Steady rps: ");
              Serial.println(canvas_rps);
              changeState(S_READY);
            }
         }
         deltaRps = canvas_rps - prevRps;
       }
       break;
    
    case S_READY:
      // Possibility of doing setup things, or going straight to S_DARK
      setCommand(GO_DARK);
      Serial.println("Ready");
      break;
    
    case S_DARK:
      if (loadedFrameNr != frameNr) {
        display_loadFrame(frame, frameNr);
        for (int i = 0; i < 8; i++) {
          Serial.println(frame[i]);
        }
        loadedFrameNr = frameNr;
      }
      canvas_rps = canvas_getRps();
      setCommand(GO_ACTIVE);
      break;
    
    case S_ACTIVE:
      long frameTime = micros() - frameStartTime;
      int currentCol = canvas_getCurrentCol(frameTime, canvas_rps);
      if (currentCol == -1) {
        setCommand(GO_DARK);
        prevCol = -1;
      }
      if (currentCol > prevCol) {
        laser_showCol(currentCol);
        prevCol = currentCol;
      }
      break;
    
    case S_STOPPED:
      break;
  }

  //debugPrint();

  switch (displayCommand) {
    case NO_COMMAND:
      break;
    
    case GO_DARK:
      if (canvas_signalState == LOW) {
        //debugPrint();
        if (displayState != S_READY) {
          canvas_setError(CANVAS_TIMEOUT);
        }
      } else {
        //debugPrint();
        frameNr++;
        laser_showCol(-1);
        changeState(S_DARK);
      }
      break;
    
    case GO_ACTIVE:
      if (canvas_signalState == HIGH) {
        frameStartTime = micros();
        changeState(S_ACTIVE);
      } else {
        canvas_setError(CANVAS_TIMEOUT);
      }
      break;
    
    case STOP:
      targetRps = 0;
      break;
  }
  setCommand(NO_COMMAND);
  
  // If I want variable rps, make a runMotor call here. Also, actually create that function.
  // canvas_runMotor(targetRps);

}
