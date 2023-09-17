uint8_t columnIdx = 0;

void display_loadFrame(uint8_t *frameArray, int nr) {
  if (nr == -1) {
    display_loadDefaultFrame(frameArray, nr);
  } else {
    //long darkTime = canvas_highTimes[canvas_signalHighCount % canvas_averageRpsOverCounts] - canvas_lowTimes[canvas_signalLowCount % canvas_averageRpsOverCounts];
    float spr = 1 / canvas_rps;
    float upr = 1000000 * spr;
    float darkTime = upr / 4; // Microseconds per quarter revolution, i.e., how long the downtime between two ACTIVE periods is.
    if (darkTime < 0) { // This is to make sure darkTime is always positive regardless of when it is calculated. TODO: Check if this is necessary.
      darkTime *= -1;
    }
    if (!display_frameReadTimeout) {
      columnIdx = 0;
    }
    //while ((display_frameTime < 0.8 * darkTime) && (columnIdx < DISPLAY_NR_COLS)) {
    while (display_stateTime < 0.8 * darkTime) {
      if (Serial.available()) {
        frameArray[columnIdx] = Serial.read();
        columnIdx++;
      }
      if (columnIdx == DISPLAY_NR_COLS) {
        darkTime = 0;
      }
    }
    if (columnIdx == DISPLAY_NR_COLS) {
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
  Serial.write(columnIdx);
}

void display_loadDefaultFrame(uint8_t *frameArray, int nr) {
  uint8_t columns[11];
  //if ((nr / 10) % 2 == 0) {
  if (true) {
    uint8_t temp[11] = {0b00011111,
                        0b00010001,
                        0b00010001,
                        0b00010001,
                        0b00010001,
                        0b00010001,
                        0b00010001,
                        0b00010001,
                        0b00010001,
                        0b00010001,
                        0b00011111};
    for (int i = 0; i < 11; i++) {
      columns[i] = temp[i];
    }
  } else {
    uint8_t temp[11] = {0b00010101,
                           0b00001010,
                           0b00010101,
                           0b00001010,
                           0b00000000,
                           0b00000000,
                           0b00000000,
                           0b00000000,
                           0b00010101,
                           0b00001010,
                           0b00010101};
    for (int i = 0; i < 11; i++) {
      columns[i] = temp[i];
    }
  }
  for (int i = 0; i < DISPLAY_NR_COLS; i++) {
    if (i < 11) {
      frameArray[i] = columns[i];
    } else {
      frameArray[i] = 0b00000000;
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