// Laser functions
layer_t make_layer(int cs_pin, int gpio_addr, int num_leds, int bit_0_pin, int order) {
  layer_t layer;
  layer.cs_pin = cs_pin;
  layer.gpio_addr = gpio_addr;
  layer.num_leds = num_leds;
  layer.bit_0_pin = bit_0_pin;
  layer.order = order; // order = 1 signifies reverse bit order
  return layer;
}

void init_layers() {
  layers[0] = make_layer(CS1, GPIOA, 6, 7, 1);
  layers[1] = make_layer(CS1, GPIOB, 6, 5, 1);
  //layers[2] = make_layer(CS2, GPIOA, 6, 2, 0);
  //layers[3] = make_layer(CS2, GPIOB, 6, 0, 0);
  //layers[4] = make_layer(CS3, GPIOA, 6, 7, 1);
}

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

void show_data_on_layer(layer_t &layer, uint8_t data) {
  if (data == 0) {
    write_reg(layer.gpio_addr, data, layer.cs_pin);
    return;
  }
  if (layer.bit_0_pin != 0) {
    uint8_t temp = 0;
    uint8_t getmask = 0;
    if (layer.order == 1) {
      // Reverse bit order  
      for (int i = 0; i < layer.num_leds; i++) {
        getmask = 1 << i;
        temp = temp | (((data & getmask) >> i) << (layer.bit_0_pin - i));
      }
      data = temp;
    }
    else {
      data = data << layer.bit_0_pin;
    }
  }
  write_reg(layer.gpio_addr, data, layer.cs_pin);
}