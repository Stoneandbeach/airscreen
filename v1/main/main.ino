#include <math.h>
#include <Arduino.h>
/*#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
*/

// Initialize high speed I2C for OLED screen
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

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

enum display_error_t {
  DISPLAY_NO_ERROR = 0,
  DISPLAY_UNKNOWN_COMS,
  DISPLAY_FRAME_READ_TIMEOUT
};

enum canvas_error_t {
  CANVAS_NO_ERROR = 0,
  CANVAS_TIMEOUT,       // This error is set if command GO_ACTIVE or GO_DARK is set while canvas_signalState == LOW
  CANVAS_NO_SIGNAL      // This error is set if the canvas motor has been asked to spin up but no canvas_signalState change is detected
};

enum com_inc_msg_t {
  COM_FRAME_AVAILABLE = 0b01000000
};

enum com_outg_msg_t {
  COM_READY = 0b01000000,
  COM_REQUEST_FRAME = 0b01000001,
  COM_FRAME_RECEIVED = 0b01000010
};

// Display (i.e., overall) declarations
displayState_t displayState = S_INIT;
displayCommand_t displayCommand = NO_COMMAND;
display_error_t display_error = DISPLAY_NO_ERROR;
const int ONBOARD_LED_PIN = 13;
const float DISPLAY_RADIUS = 75; // Radius of the swept volume in mm
const int DISPLAY_NR_COLS = 16; // Number of columns in the display
const int DISPLAY_NR_LAYERS = 3;
const float DISPLAY_DIST_FROM_AXIS[DISPLAY_NR_LAYERS] = {54, 47, 40}; // Distance from canvas axis to each layer in mm
float DISPLAY_ANGLE_LIMIT[DISPLAY_NR_LAYERS] = {0, 0, M_PI / 4}; // Outer angle limit of the display layers, from axis parallel to laser beams. The innermost layer limit is set, while the others are calculated from it.
uint8_t frame[DISPLAY_NR_COLS][DISPLAY_NR_LAYERS]; 
int frameNr = 0;
int loadedFrameNr = 0;
long frameStartTime;
long display_frameTime;
long stateStartTime = 0;
long display_stateTime;
int prevCol[DISPLAY_NR_LAYERS];
uint8_t display_frameReadTimeout = 0;

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

const int CANVAS_SIGNAL_PIN = 2;

float steadyRpsChangeLimit = 0.05; // Percentage change limit to consider rps steady
int steadyRpsCountTarget = 10;
int steadyRpsCount = 0;

volatile int canvas_signalState;
canvas_error_t canvas_error = CANVAS_NO_ERROR;

bool fatalError = 0;

// Laser variables
const int LASER_NR_LAYERS = DISPLAY_NR_LAYERS;
const int LASER_NR_ROWS = LASER_NR_LAYERS;
const int LASER_ROW_PINS[LASER_NR_ROWS] = {3, 4, 5};
const int LASER_LAYER_PINS[LASER_NR_LAYERS] = {6, 8, 9};
int laser_states[LASER_NR_ROWS][LASER_NR_LAYERS];

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
  display_stateTime = 0;
  stateStartTime = micros();
}

uint8_t columnIdx = 0;
uint8_t layerIdx = 0;

void display_loadFrame(uint8_t frameArray[DISPLAY_NR_COLS][DISPLAY_NR_LAYERS], int nr) {
  if (nr == -1) {
    display_loadDefaultFrame(frameArray);
  } else {
    float spr = 1 / canvas_rps;
    float upr = 1000000 * spr;
    float darkTime = upr / 4; // Microseconds per quarter revolution, i.e., how long the downtime between two ACTIVE periods is.
    if (darkTime < 0) { // This is to make sure darkTime is always positive regardless of when it is calculated. TODO: Check if this is necessary.
      darkTime *= -1;
    }
    if (!display_frameReadTimeout) {
      columnIdx = 0;
      layerIdx = 0;
    }
    while (display_stateTime < 0.8 * darkTime) {
      if (Serial.available()) {
        frameArray[columnIdx][layerIdx] = Serial.read();
        columnIdx++;
      }
      if (columnIdx == DISPLAY_NR_COLS) {
        columnIdx = 0;
        layerIdx++;
      }
      if (layerIdx == DISPLAY_NR_LAYERS) {
        darkTime = 0;
      }
    }
    if (layerIdx == DISPLAY_NR_LAYERS) {
      display_send(COM_FRAME_RECEIVED);
      display_frameReadTimeout = 0;
      //debugLed(0);
    } else {
      display_setError(DISPLAY_FRAME_READ_TIMEOUT);
      display_frameReadTimeout = 1;
      // TODO: Send a timeout message.
      // TODO: Also, make sure that the COMS don't break here, since COM_FRAME_RECEIVED won't be sent.
    }
  }
  // Serial.write(columnIdx);
}

void display_loadDefaultFrame(uint8_t frameArray[DISPLAY_NR_COLS][DISPLAY_NR_LAYERS]) {
  int state = 1;
  for (int l = 0; l < DISPLAY_NR_LAYERS; l++) {
    for (int c = 0; c < DISPLAY_NR_COLS; c++) {
      frameArray[c][l] = state;
      // state = state ^ 1;
    }
  }
}

void display_send(int cmd) {
  Serial.write(cmd);
//  debugScreen(cmd, 2);
}

void display_processComs() {
    
  if (Serial.available()) {
    //debugLed(1);
    if (display_frameReadTimeout) {
      display_loadFrame(frame, frameNr);
    } else {
      int coms = Serial.read();
//      debugScreen(coms, 1);
      switch (coms) {
        case COM_FRAME_AVAILABLE:
            display_send(COM_REQUEST_FRAME);
            display_loadFrame(frame, frameNr);
            if (!display_frameReadTimeout) { // TODO: The frame-counting logic is currently broken and also does nothing.
              loadedFrameNr = frameNr;
            }
          break;
        default:
          display_setError(DISPLAY_UNKNOWN_COMS);
      }
    }
  }
}

void display_setError(int error) {
  display_error = error;
//  debugScreen(error, 3);
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

const float twoPi = 2 * M_PI;
const float D = 2 * DISPLAY_DIST_FROM_AXIS[DISPLAY_NR_LAYERS - 1] * cos(DISPLAY_ANGLE_LIMIT[DISPLAY_NR_LAYERS - 1]); // The width of the display surface
const float Dhalf = D / 2;
const int n = DISPLAY_NR_COLS + 1; //+ Number of display columns, plus one last, dark, padded column
const float colWidth = D / n; // The width of a column in the display surface
int8_t currentCol[DISPLAY_NR_LAYERS]; // The currently active column
int currentLayer = 0; // The currently active layer

int canvas_getCurrentCol(long frameTime, float rps, int layer) {
  // Calculate which column to display depending on rps and current frame time, for the specified layer
  float t = ((float) frameTime) / 1000000; // Uptime of the current frame in seconds
  float w = rps * twoPi; // Angular frequency of the canvas
  float x = DISPLAY_DIST_FROM_AXIS[layer] / tan(w * t + DISPLAY_ANGLE_LIMIT[layer]); // x position along the display surface. Angle is calculated in relation to the laser beam
  x = -x + Dhalf; // Rescaling of x to 0 < x < D
  for (int i = currentCol[layer]; i < DISPLAY_NR_COLS; i++) {
    if ((x >= i * colWidth) && (x < (i + 1) * colWidth)) { // Find the segment of D in which x lies
      return i;
    }
  }
  return -1; // We are in a dark, padded column
}

// Laser functions
void laser_setState(int laserIdx, int layer, int state) {
  laser_states[laserIdx][layer] = state;
  digitalWrite(LASER_ROW_PINS[laserIdx], state);
}

void laser_show(int col, int layer) {
  if (col == -1) {
      laser_setActiveLayer(-1); // Set all layers to not active
  } else {
    laser_setActiveLayer(layer);
    uint8_t column = frame[col][layer];
    uint8_t bitMask = 0b00000001; // THIS IS GOING TO BREAK IF THERE ARE MORE THAN 8 LASERS OR SOMETHING
    for (int i = 0; i < LASER_NR_ROWS; i++) {
      uint8_t state = (column & bitMask) >> i;
      laser_setState(i, layer, state);
      bitMask = bitMask << 1;
    }
  }
}

void laser_setActiveLayer(int layer) {
  for (int l = 0; l < LASER_NR_LAYERS; l++) {
    digitalWrite(LASER_LAYER_PINS, (layer != l)); // Writes HIGH to all layers except the input layer.
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

void debugPrintTiming(int currentCol) {
  Serial.print(currentCol);
  Serial.print(",");
  Serial.print(canvas_signalState);
  Serial.print(",");
  Serial.println(displayState);
}

long debugTimeNow = 0;
long debugTimeThen = 0;

void debugLasers() {
  /* -- Doesn't work with 3D setup! --
  debugTimeNow = micros() - canvas_highTimes[canvas_signalHighCount % canvas_averageRpsOverCounts];
  Serial.println(debugTimeNow);
  int col = canvas_getCurrentCol(debugTimeNow, canvas_rps, 0);
  int debugLaserState = col % 2;
  laser_setState(0, debugLaserState);
  */
}

void debugLed(int state) {
  digitalWrite(ONBOARD_LED_PIN, state);
}

int debugScreenArray[] = {0, 0, 0}; // TODO: Make this a char array if that is possible

/*
// Print last sent message, last received message and last error on OLED screen
void debugScreen(int msg, int row) {
  return;
  debugScreenArray[row - 1] = msg;
  if (row == 3) {
    char msgStr[3];
    int cond1;
    u8g2.clearBuffer();
    for (int i = 0; i < 3; i++) {
      cond1 = debugScreenArray[i];
      itoa(cond1, msgStr, 10);
      int ypos = 15 * (i + 1);
      u8g2.drawStr(0, ypos, msgStr);  
    }
    u8g2.sendBuffer();
  }
}
*/

// Interrupts
void canvas_signalInterrupt() {
  canvas_signalCount++;
  canvas_signalState = digitalRead(CANVAS_SIGNAL_PIN);
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

/*
  u8g2.begin();
  //u8g2.setFont(u8g2_font_ncenB18_tf);
  u8g2.setFont(u8g2_font_ncenB12_tf);
  //u8g2.setFont(u8g2_font_ncenB08_tr);
*/
  
  // Display
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  // Calculate angle limits for all display layers, based on the predefined value for the innermost layer.
  for (int i = 0; i < DISPLAY_NR_LAYERS - 1; i++) {
    DISPLAY_ANGLE_LIMIT[i] = atan(DISPLAY_DIST_FROM_AXIS[i] / Dhalf);
    prevCol[i] = -1;
  }
  prevCol[DISPLAY_NR_LAYERS - 1] = -1;
  display_loadDefaultFrame(frame);
  
  // Canvas
  pinMode(CANVAS_SIGNAL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CANVAS_SIGNAL_PIN), canvas_signalInterrupt, CHANGE);
  // Here I will later initialize pins for controlling fan speed, if required.

  // Laser
  for (int i = 0; i < LASER_NR_ROWS; i++) {
    pinMode(LASER_ROW_PINS[i], OUTPUT);
    digitalWrite(LASER_LAYER_PINS[i], HIGH); // Initialize the column pins to HIGH, which means the transistors are open.
    for (int j = 0; j < LASER_NR_LAYERS; j++) {
      laser_states[i][j] = LOW;
    }
  }
  for (int i = 0; i < LASER_NR_LAYERS; i++) {
    pinMode(LASER_LAYER_PINS[i], OUTPUT);
  }
  
}

long lastDebugTime = micros();

void loop() {

  //debugLasers();

  long now = micros();
  display_stateTime = now - stateStartTime;
  
  // Main state machine
  switch (displayState) {
    case S_INIT:
      // Possibility of doing startup checks, like measuring feedback from diodes if available
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
              changeState(S_READY);
              display_send(COM_READY);
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
      if (canvas_signalState == LOW) {
        setCommand(GO_ACTIVE);
      }
      break;
    
    case S_ACTIVE:
      display_frameTime = now - frameStartTime;
      currentCol[currentLayer] = canvas_getCurrentCol(display_frameTime, canvas_rps, currentLayer);

      if ((currentLayer == DISPLAY_NR_LAYERS) && (currentCol[currentLayer] == -1)) {
        setCommand(GO_DARK);
        for (int l = 0; l < DISPLAY_NR_LAYERS; l++) {
          prevCol[currentLayer] = -1;
        }
      }
      if (currentCol[currentLayer] > prevCol[currentLayer]) {
        laser_show(currentCol[currentLayer], currentLayer);
        prevCol[currentLayer] = currentCol[currentLayer];
      }
      
      currentLayer++;
      if (currentLayer == DISPLAY_NR_LAYERS) {
        currentLayer = 0;
      }

      break;
    
    case S_STOPPED:
      break;
  }

  //debugPrint();
  //debugPrintTiming(currentCol);
  
  // Main command switch
  switch (displayCommand) {
    case NO_COMMAND:
      break;
    
    case GO_DARK:
      if (canvas_signalState == LOW) {
        if (displayState != S_READY) {
          canvas_setError(CANVAS_TIMEOUT);
          setCommand(NO_COMMAND);
        }
      } else {
        frameNr++;
        for (int l = 0; l < DISPLAY_NR_LAYERS; l++) {
          laser_show(-1, l);
        }
        changeState(S_DARK);
        setCommand(NO_COMMAND);
        //debugLed(0);
        canvas_rps = canvas_getRps();
        display_processComs(); // Read incoming serial coms and react accordingly
      }
      break;
    
    case GO_ACTIVE:
      if (canvas_signalState == HIGH) {
        changeState(S_ACTIVE);
        setCommand(NO_COMMAND);
        frameStartTime = now; // TODO: Change this so that frameStartTime = the latest canvas_highTime, since that gives the best reference for how long I've got before the active state should end.
        //debugLed(1);
      }
      break;
    
    case STOP:
      changeState(S_STOPPED);
      targetRps = 0;
      break;
  }
  
  // If I want variable rps, make a runMotor call here. Also, actually create that function.
  // canvas_runMotor(targetRps);

}
