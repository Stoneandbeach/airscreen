#define NUM_LED_ROWS 2
#define NUM_LED_COLS 2
#define MIN_COL_TIME 1 // Milliseconds
#define MAX_COL_TIME 100 // Milliseconds
#define ONBOARD_LED_PIN 13
uint8_t ledPins[NUM_LED_ROWS] = {2, 3};
uint8_t transistorPins[NUM_LED_COLS] = {4, 5};

uint8_t frame[2][2][2] = { // Frame, col, row
  {{1, 1}, {1, 1}},
  {{0, 1}, {1, 0}}
};

int now = 0;
int lastNow = 0;
int dt;
int colTime = MAX_COL_TIME;
int increment = -1;

uint8_t currentCol = 0;

void setup() {
  int i;
  for (i = 0; i < NUM_LED_ROWS; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW); // Initalize all leds to off
  }
  for (i = 0; i < NUM_LED_COLS; i++) {
    pinMode(transistorPins[i], OUTPUT);
    digitalWrite(transistorPins[i], HIGH); // Initialize all transistors to... open?
  }
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {

  now = millis();
  dt = now - lastNow;

  if (dt > colTime) {
    lastNow = now;
    colTime = colTime + increment;
    Serial.println(colTime);
    if (colTime < MIN_COL_TIME) {
      increment = increment * -1;
    } 
    else if (colTime > MAX_COL_TIME) {
      increment = increment * -1;
    }
    //currentCol = (++currentCol) % (NUM_LED_COLS - 1);
    if (currentCol == 0) {
      currentCol = 1;
      digitalWrite(ONBOARD_LED_PIN, HIGH);
    }
    else {
      currentCol = 0;
      digitalWrite(ONBOARD_LED_PIN, LOW);
    }
    updateImage();
  }
  
}

void updateImage() {
  openCol(currentCol);
  for (int r = 0; r < NUM_LED_ROWS; r++) {
    int state = frame[0][r][currentCol];
    setLaserState(r, state);
  }
}

void setLaserState(uint8_t led, uint8_t state) {
  digitalWrite(ledPins[led], state);
}

void openCol(uint8_t col) {
  for (int i = 0; i < NUM_LED_COLS; i++) {
    if (i == col) {
      digitalWrite(transistorPins[i], LOW);
    }
    else {
      digitalWrite(transistorPins[i], HIGH);
    }
  }
}
