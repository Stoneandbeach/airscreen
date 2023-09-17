// Canvas variable initialization
void canvas_initialize() {
  
  for (int layer = 0; layer < DISPLAY_NR_LAYERS; layer++) {
    DISPLAY_DIST_FROM_AXIS[layer] = DISPLAY_MIN_DIST_FROM_AXIS + DISPLAY_DIST_BETWEEN_LAYERS * (DISPLAY_NR_LAYERS - 1 - layer);
    //Serial.println(Dhalf);
    float angle = (float) atan(Dhalf / DISPLAY_DIST_FROM_AXIS[layer]);
    DISPLAY_ANGLE_LIMIT[layer] = angle;
    //Serial.print("Layer ");
    //Serial.print(layer);
    //Serial.print(" at dist ");
    //Serial.print(DISPLAY_DIST_FROM_AXIS[layer]);
    //Serial.print(" and angle ");
    //Serial.println(angle);
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

int canvas_getCurrentCol(long frameTime, float w, int layer) {
  // Calculate which column to display, for the given layer, depending on rps and current frame time
  float t = ((float) frameTime) / 1000000; // Uptime of the current frame in seconds
  float y = tan(w * t - DISPLAY_ANGLE_LIMIT[layer]) * DISPLAY_DIST_FROM_AXIS[layer]; // y position along the display volume
  for (int i = prevCol[layer]; i < DISPLAY_NR_COLS; i++) {
    if ((y >= i * colWidth - Dhalf) && (y < (i + 1) * colWidth - Dhalf)) { // Find the segment of D in which y lies
      return i;
    }
  }
  return -1; // We are in a dark, padded column
}