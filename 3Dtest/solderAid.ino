#define NUM_LED_ROWS 3
#define NUM_LED_COLS 3
#define ONBOARD_LED_PIN 13

int rowPins[NUM_LED_ROWS] = {2, 3, 4};
int colPins[NUM_LED_COLS] = {5, 6, 7};

void setup() {
    Serial.begin(9600);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    for (int i = 0; i < NUM_LED_ROWS; i++) {
        pinMode(rowPins[i], OUTPUT);
        digitalWrite(rowPins[i], LOW);
    }
    for (int i = 0; i < NUM_LED_COLS; i++) {
        pinMode(colPins[i], OUTPUT);
        digitalWrite(colPins[i], LOW);
    }
}

int onboardLedState = 0;

void heartbeat() {
    onboardLedState = 1 - onboardLedState;
    Serial.println(onboardLedState);
    digitalWrite(ONBOARD_LED_PIN, onboardLedState);
}

void setOn(int row, int col) {
    openRow(row);
    openCol(col);
}

void setOff() {
    for (int i = 0; i < NUM_LED_ROWS; i++) {
        digitalWrite(rowPins[i], LOW);
    }
    for (int i = 0; i < NUM_LED_COLS; i++) {
        digitalWrite(colPins[i], LOW);
    }
}

void openRow(int row) {
    for (int i = 0; i < NUM_LED_ROWS; i++) {
        int state = i == row ? HIGH : LOW;
        digitalWrite(rowPins[i], state);
    }
}

void openCol(int col) {
    for (int i = 0; i < NUM_LED_COLS; i++) {
        int state = i == col ? HIGH : LOW;
        digitalWrite(colPins[i], state);
    }
}

unsigned long prevTick = millis();
int row;
int col;

void loop() {
    unsigned long now = millis();
    Serial.println("Hi");
    if (now - prevTick > 100) {
        prevTick = now;
        row = (row + 1) % NUM_LED_ROWS;
        if (row == 0) {
            col = (col + 1) % NUM_LED_COLS;
        }
        //setOn(row, col);
        heartbeat();
    }
}