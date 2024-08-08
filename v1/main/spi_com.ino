uint16_t write_reg(uint8_t addr, uint8_t msg, uint8_t cs_pin) {
  return expander_com(addr, msg, 0, cs_pin);
}

uint16_t read_reg(uint8_t addr, uint8_t cs_pin) {
  return expander_com(addr, 0, 1, cs_pin);
}

uint16_t expander_com(uint8_t addr, uint8_t msg, uint8_t mode, uint8_t cs_pin) {
  uint8_t control = 0b01000000;
  uint8_t device_addr = 0b00000000;

  uint8_t buff = control | device_addr | mode;
  uint8_t buff2 = addr;
  uint8_t buff3 = msg;
  
  digitalWrite(cs_pin, LOW);
  //delay(1);
  SPI.transfer(buff);
  //delay(1);
  buff2 = SPI.transfer(buff2);
  //delay(1);
  buff3 = SPI.transfer(buff3);
  //delay(1);
  digitalWrite(cs_pin, HIGH);
  //delay(10);

  uint16_t ret = (buff2 << 8) | buff3;
  return ret;
}