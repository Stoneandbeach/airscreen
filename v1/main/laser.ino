// Laser functions
void laser_setState(int laserIdx, int state) {
  laser_states[laserIdx] = state;
  digitalWrite(LASER_PINS[laserIdx], state);
}

int laser_getState(int laserIdx) { // This may never be needed.
  return laser_states[laserIdx];
}

void laser_showLayer(int layer) {
  for (int i = 0; i < LASER_NR_TRANSISTOR_PINS; i++) {
    digitalWrite(LASER_TRANSISTOR_PINS[i], i == layer ? LOW : HIGH); // Set every transistor to HIGH (= no laser) except the one for layer
  }
}

void laser_showCol(int currentCol, int layer) {
  if (currentCol == -1) {
    for (int i = 0; i < LASER_NR_PINS; i++) {
      laser_setState(i, LOW);
    }
  } else {
    uint8_t column = frame[currentCol];
    uint8_t bitMask = 0b00000001; // THIS IS GOING TO BREAK IF THERE ARE MORE THAN 8 LASERS OR SOMETHING
    for (int i = 0; i < LASER_NR_PINS; i++) {
      uint8_t state = (column & (bitMask << i)) >> i;
      laser_setState(i, state);
    }
  }
}