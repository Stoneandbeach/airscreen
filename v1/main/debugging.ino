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
  Serial.print(currentLayer);
  Serial.print(",");
  Serial.print(canvas_signalState);
  Serial.print(",");
  Serial.println(displayState);
}

long debugTimeNow = 0;
long debugTimeThen = 0;

void debugLasers() {
  debugTimeNow = micros() - canvas_highTimes[canvas_signalHighCount % canvas_averageRpsOverCounts];
  Serial.println(debugTimeNow);
  int layer = 0;
  float canvas_angularFrequency = canvas_rps * twoPi;
  int col = canvas_getCurrentCol(debugTimeNow, canvas_angularFrequency, layer);
  int debugLaserState = col % 2;
  laser_setState(0, debugLaserState);
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