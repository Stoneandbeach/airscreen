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

enum displayMode_t {
  MODE_EXT = 0,
  MODE_AUTO = 1
};

enum adminCommand_t {
  AC_NO_COMMAND = 0,
  AC_SPINUP = 0x01,
  AC_NEW_FRAME = 0x02,
  AC_RESTART = 0x03
};

enum display_error_t {
  DISPLAY_NO_ERROR = 0,
  DISPLAY_SERIAL_ERROR = 1
};

enum display_communication_t {
  COM_READY = 0x01,
  COM_FRAME_REQUEST = 0x02,
  COM_FRAME_RECEIVED = 0x03,
  COM_SERIAL_ERROR = 0x04,
  COM_CANVAS_TIMEOUT = 0x05
};

enum canvas_error_t {
  CANVAS_NO_ERROR = 0,
  CANVAS_TIMEOUT,       // This error is set if command GO_ACTIVE or GO_DARK is set while canvas_signalState == LOW
  CANVAS_NO_SIGNAL      // This error is set if the canvas motor has been asked to spin up but no canvas_signalState change is detected
};

// Display (i.e., overall) declarations
displayState_t displayState = S_INIT;
displayCommand_t displayCommand = NO_COMMAND;
displayMode_t displayMode = MODE_EXT;
display_error_t display_error = DISPLAY_NO_ERROR;
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
const long BAUD_RATE = 115200;
uint16_t stateTime = 0;

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
  int frameLen;
  if (nr == -1) { // Start-up frame
    frameLen = 5;
    Serial.println("Start-up frame");
    uint8_t columns[] = {0b00000100,
                                 0b00000000,
                                 0b00000100,
                                 0b00000000,
                                 0b00000100};
    frameFromColumns(frameArray, columns, frameLen);
    return;
  }
  if (displayMode = MODE_AUTO) { // Display some default message
    Serial.println("Auto mode");
    frameLen = 11;
    uint8_t columns[] =         {0b00011111,
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
    Serial.println("Made columns");
    for (int i = 0; i < display_nrCols; i++) {
      if (i < frameLen) {
        Serial.print("Loading column ");
        Serial.println(i);
        frameArray[i] = 0b00000001; //columns[i];
      } else {
        Serial.print("Loading empty column ");
        Serial.println(i);
        frameArray[i] = 0b00000000;
      }
    }
    Serial.println("LADDAD");
    //frameFromColumns(frameArray, columns, frameLen);
  } else { // Display frame incoming over serial port
    Serial.println("Reading frame from serial");
    frameLen = Serial.read();
    if (frameLen < 1) { // Frame is empty
      display_error = DISPLAY_SERIAL_ERROR;
      Serial.println("Error, frameLen < 1"); // TODO: Make this into a sendCommand()
      return;
    }
    uint8_t columns[frameLen];
    int readLen = Serial.readBytes(columns, frameLen);
    if (readLen != frameLen) { // Frame does not contain as many columns as it should
      display_error = DISPLAY_SERIAL_ERROR;
      Serial.println("Error, readLen != frameLen"); // TODO: Make this into a sendCommand()
      return;
    }
    frameFromColumns(frameArray, columns, frameLen);
    Serial.println("Frame received"); // TODO: Make this into a sendCommand()
  }
}

void frameFromColumns(uint8_t *frameArray, uint8_t *columns, int frameLen) {
  Serial.print("Frame from columns, frameLen = ");
  Serial.println(frameLen);
  for (int i = 0; i < display_nrCols; i++) {
    Serial.print(i);
    Serial.print(": ");
    if (i < frameLen) {
      Serial.println(columns[i]);
      Serial.println(columns[i+1]);
      frameArray[i] = columns[i];
      
    } else {
      Serial.println("Trying to load empty column");
      frameArray[i] = 0b00000000;
      Serial.print("Loaded empty column ");
      Serial.println(i);
    }
  }
  Serial.println("LADDAD");
}

int adminCommand() {
  /*AC_NO_COMMAND = 0,
  AC_SPINUP = 0x01,
  AC_NEW_FRAME = 0x02,
  AC_RESTART = 0x03*/
  int cmd = 0;
  if (Serial.available()) {
    cmd = Serial.read();
    int checkByte = Serial.read();
    if (checkByte != 1) { // Checkbyte should always be == 1. Some error has occured
      Serial.println("Error, checkByte != 1");
      Serial.println(cmd);
      sendCommunication(DISPLAY_SERIAL_ERROR);
      return -1;
    }
  }
  switch (cmd) {
    case 0x00:
      break;
    case 0x01:
      changeState(S_SPINUP);
      Serial.println("Spinning up");
      return 1; // Command handled
    case 0x02:
      display_loadFrame(frame, frameNr);
      return 1;
    case 0x03:
      changeState(S_INIT);
      Serial.println("Restarting");
      return 1;
    case 0x04: // Test command
      Serial.println("Test aknowledged");
      return 1;
    default:
      Serial.print("Unknown command ");
      Serial.println(cmd);
      return -1; // Unknown command
  return 0; // No command received
  }
}

int sendCommunication(int com) {
  return -1;
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
  Serial.begin(BAUD_RATE);
  Serial.println("Starting");
  
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
  //debugPrint();
  
  switch (displayState) {
    case S_INIT:
      // Possibility of doing startup checks, like measuring feedback from diodes if available
      adminCommand();
      if (millis() > 5000) { // If no spinup command has been received after 5 seconds, go to automatic mode
        Serial.println("Going to autonomous mode");
        displayMode = MODE_AUTO;
        changeState(S_SPINUP);
      }
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
      break;
    
    case S_DARK:
      if (displayMode = MODE_AUTO) {
        if (loadedFrameNr != frameNr) {
          display_loadFrame(frame, frameNr);
          Serial.print("Loaded frame ");
          Serial.println(frameNr);
          loadedFrameNr = frameNr;
        }
        break;
      }
      adminCommand();
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

  stateTime++;
  
  // If I want variable rps, make a runMotor call here. Also, actually create that function.
  // canvas_runMotor(targetRps);

}
