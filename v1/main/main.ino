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
const int DISPLAY_NR_COLS = 16; // Number of columns in the display
const int DISPLAY_NR_LAYERS = 5;
const float DISPLAY_RADIUS = 75; // Radius of the swept volume in mm
const float DISPLAY_MAX_ANGLE = M_PI / 4; // Outer angle limit of innermost display layer
float DISPLAY_ANGLE_LIMIT[DISPLAY_NR_LAYERS] = {0., 0., 0., 0., 0.}; // Outer angle limit of the display layers, from axis tangential to laser beams
const float DISPLAY_MIN_DIST_FROM_AXIS = 30; // Distance from canvas axis to innermost display layer in mm
float DISPLAY_DIST_FROM_AXIS[DISPLAY_NR_LAYERS]; // Distance from canvas axis to display layers
const float DISPLAY_DIST_BETWEEN_LAYERS = 7; // Inter-layer distance in mm
const long DISPLAY_MAX_LAYER_TIME = 5000; // Light-time in microseconds for a layer before switching
uint8_t frame[DISPLAY_NR_COLS]; // THIS NEEDS TO BE EXPANDED WHEN I HAVE MORE LASERS
int frameNr = 0;
int loadedFrameNr = 0;
long frameStartTime;
long display_frameTime;
long stateStartTime = 0;
long display_stateTime;
long display_layerStartTime = 0;
long display_layerTime = 0;
int prevCol[DISPLAY_NR_LAYERS] = {0, 0, 0, 0, 0};
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

const float twoPi = 2 * M_PI;
const float Dhalf = DISPLAY_MIN_DIST_FROM_AXIS * tan(DISPLAY_MAX_ANGLE); // Half the width of the display volume
const float D = Dhalf * 2;
const int n = DISPLAY_NR_COLS + 1; //+ Number of display columns, plus one last, dark, padded columns
const float colWidth = D / n; // The width of a column in the display volume
int8_t currentCol = 0; // The currently active column
int currentLayer = 0; // The currently active layer

// Laser variables
const int LASER_NR_PINS = 6;
const int LASER_PINS[LASER_NR_PINS] = {3, 4, 5, 6, 7, 8};
int laser_states[LASER_NR_PINS];
const int LASER_NR_TRANSISTOR_PINS = DISPLAY_NR_COLS;
const int LASER_TRANSISTOR_PINS[LASER_NR_TRANSISTOR_PINS] = {17, 18, 19, 20, 21};

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
  
  // Canvas
  pinMode(CANVAS_SIGNAL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CANVAS_SIGNAL_PIN), canvas_signalInterrupt, CHANGE);
  // Here I will later initialize pins for controlling fan speed, if required.

  // Laser
  for (int i = 0; i < LASER_NR_PINS; i++) {
    pinMode(LASER_PINS[i], OUTPUT);
    laser_states[i] = LOW;
  }
  for (int i = 0; i < LASER_NR_TRANSISTOR_PINS; i++) {
    pinMode(LASER_TRANSISTOR_PINS[i], OUTPUT);
    digitalWrite(LASER_TRANSISTOR_PINS[i], HIGH);
  }
}

long lastDebugTime = micros();

void loop() {

  //debugLasers();

  long now = micros();
  display_stateTime = now - stateStartTime;

  if (now - lastDebugTime > 1000000) {
    debugPrint();
    lastDebugTime = now;
  }
  
  // Main state machine
  switch (displayState) {
    case S_INIT:
      // Possibility of doing startup checks, like measuring feedback from diodes if available
      Serial.println("Initializing...");
      canvas_initialize();
      
      Serial.println("Spinning up...");
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
      display_layerTime = now - display_layerStartTime;
      if (display_layerTime > DISPLAY_MAX_LAYER_TIME) {
        currentLayer = (currentLayer + 1) % DISPLAY_NR_LAYERS;
        laser_showLayer(currentLayer);
        display_layerStartTime = now;
      }
      // Loop over all layers and do the following. Probably need a prevCol array?
      float canvas_angularFrequency = canvas_rps * twoPi; // Angular frequency of the canvas
      currentCol = canvas_getCurrentCol(display_frameTime, canvas_angularFrequency, currentLayer);

      if (currentCol == -1) {
        prevCol[currentLayer] = -1;
      }
      if (currentCol > prevCol[currentLayer]) {
        laser_showCol(currentCol, currentLayer);
        prevCol[currentLayer] = currentCol;
      }
      
      // Check if the innermost layer is in a padded column. If so, this display period ends.
      if (prevCol[DISPLAY_NR_LAYERS - 1] = -1) {
        setCommand(GO_DARK);
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
        for (int layer = 0; layer < DISPLAY_NR_LAYERS; layer++) {
          laser_showCol(-1, layer);
        }
        changeState(S_DARK);
        setCommand(NO_COMMAND);
        debugLed(0);
        canvas_rps = canvas_getRps();
        display_loadDefaultFrame(frame, frameNr);
        //display_processComs(); // Read incoming serial coms and react accordingly
      }
      break;
    
    case GO_ACTIVE:
      if (canvas_signalState == HIGH) {
        changeState(S_ACTIVE);
        setCommand(NO_COMMAND);
        frameStartTime = now; // TODO: Change this so that frameStartTime = the latest canvas_highTime, since that gives the best reference for how long I've got before the active state should end.
        display_layerStartTime = now;
        currentLayer = 0;
        debugLed(1);
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
