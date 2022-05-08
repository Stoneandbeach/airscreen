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

enum display_error_t {
  DISPLAY_NO_ERROR = 0,
  DISPLAY_INCOMPLETE_FRAME_READ
};

enum coms_outgoing_msg_t {
  COM_REQUEST_FRAME = 1,
  COM_FRAME_RECEIVED = 2,
  COM_READY = 3
};

enum coms_incoming_msg_t {
  COM_NO_MSG = 0,
  COM_FRAME_AVAILABLE = 1
};


// Display (i.e., overall) declarations
displayState_t displayState = S_INIT;
displayCommand_t displayCommand = NO_COMMAND;
int onboardLedPin = 13;
float display_radius = 75; // Radius of the swept volume in mm
float display_angleLimit = M_PI / 4; // Outer angle limit of the display volume, from axis parallel to laser beams. TODO: It might be a good idea to change this, so that it describes the angle opening of the outer edges of the display volume instead.
const int display_nrCols = 1;
const int display_colMax = display_nrCols - 1; // Index of the last column
float display_distanceFromAxis = 40; // Distance from canvas axis to display surface in mm
uint8_t frame[display_nrCols]; // THIS NEEDS TO BE EXPANDED WHEN I HAVE MORE LASERS
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
  Serial.write(COM_REQUEST_FRAME);
  long now = micros();
  float waitTime = 1000000 * (1 / canvas_rps) / 8; // Microseconds to make 1/8 revolution
  int i = 0;
  while (micros() - now < waitTime && (i < display_nrCols)) {
    if (Serial.available()) {
      frameArray[i] = Serial.read();
      i++;
    }
  }
  Serial.write(COM_FRAME_RECEIVED);
}

uint8_t display_receiveComs() {
  if (!Serial.available()) {
    return 0;
  }
  uint8_t cmd = Serial.read();
  return cmd;
}

void display_processComs() {
  uint8_t cmd = display_receiveComs();
  switch (cmd) {
    case COM_NO_MSG:
      break;
    case COM_FRAME_AVAILABLE:
      if (loadedFrameNr != frameNr) {
        display_loadFrame(frame, frameNr);
        /*for (int i = 0; i < 8; i++) {
          Serial.println(frame[i]);
        }*/
        loadedFrameNr = frameNr;
      }
      break;
    default:
      // TODO: Set display_error unknown_command or similar
      break;
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
  float D = 2 * display_radius * cos(display_angleLimit); // The width of the display surface
  int n = display_nrCols + 1; //+ Number of display columns, plus one last, dark, padded column
  float colWidth = D / n;
  // TODO: Shit this is actually wrong! It shouldn't be -x + y, it should be -x + x_min which is = y * some tan thing. Figure it out!
  x = -x + y; // Shift x so that it goes 0 < x < y*tan(display angle)(? is this true?), i.e., conceptually the position and hence pixels go from left to right in the xy plane. This might not be the case in reality, but this x position is only used to calculate which pixel should be projected, not where that pixel is.
  for (int i = 0; i < display_nrCols; i++) {
    if ((x >= i * colWidth) && (x < (i + 1) * colWidth)) { // Find the segment of D in which x lies
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
    for (int i = 0; i < numLasers; i++) {
      uint8_t state = (column & (bitMask << i)) >> i;
      laser_setState(i, state);
    }
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
  laser_setState(0, debugLaserState);
}

void debugLed(int state) {
  digitalWrite(onboardLedPin, state);
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
  for (int i = 0; i < display_nrCols; i++) {
    frame[i] = 0; // Initialize the frame to all 0s.
  }
  
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

  long now = micros();
  
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
      Serial.write(COM_READY);
      break;
    
    case S_DARK:

      //digitalWrite(onboardLedPin, HIGH); // Debugging, probably
      //Serial.print(4);
      
      //display_processComs();
      canvas_rps = canvas_getRps();
      setCommand(GO_ACTIVE);
      break;
    
    case S_ACTIVE:

      //digitalWrite(onboardLedPin, LOW); // Debugging, probably
      //Serial.print(5);
      
      long frameTime = now - frameStartTime;
      int currentCol = canvas_getCurrentCol(frameTime, canvas_rps);
      if (currentCol == -1) {
        Serial.print(currentCol);
        Serial.write(canvas_signalState);
        setCommand(GO_DARK);
        prevCol = -1;
      }
      if (currentCol > prevCol) {
        Serial.print(currentCol);
        Serial.write(canvas_signalState);
        laser_showCol(currentCol);
        prevCol = currentCol;   // TODO: prevCol is not the best name for this. It should be displayedCol or something like that
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
          debugLed(1);
          canvas_setError(CANVAS_TIMEOUT);
        }
      } else {
        //debugPrint();
        debugLed(0);
        frameNr++;
        laser_showCol(-1);
        changeState(S_DARK);
      }
      break;
    
    case GO_ACTIVE:
      
      if (canvas_signalState == HIGH) {
        frameStartTime = micros();
        changeState(S_ACTIVE);
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
