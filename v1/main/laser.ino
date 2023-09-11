// Laser functions
void laser_setState(int laserIdx, int state) {
  laser_states[laserIdx] = state;
  digitalWrite(LASER_PINS[laserIdx], state);
}

int laser_getState(int laserIdx) { // This may never be needed.
  return laser_states[laserIdx];
}

void laser_showCol(int currentCol) {
  if (currentCol == -1) {
    for (int i = 0; i < NR_LASERS; i++) {
      laser_setState(i, LOW);
    }
  } else {
    uint8_t column = frame[currentCol];
    uint8_t bitMask = 0b00000001; // THIS IS GOING TO BREAK IF THERE ARE MORE THAN 8 LASERS OR SOMETHING
    for (int i = 0; i < NR_LASERS; i++) {
      uint8_t state = (column & (bitMask << i)) >> i;
      laser_setState(i, state);
    }
  }
}