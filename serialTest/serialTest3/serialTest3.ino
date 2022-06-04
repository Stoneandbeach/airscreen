#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

enum com_inc_msg_t {
  COM_FRAME_AVAILABLE = 1
};

enum com_outg_msg_t {
  COM_READY = 1,
  COM_REQUEST_FRAME = 2,
  COM_FRAME_RECEIVED = 3
};

void setup() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB18_tf);
  
  Serial.begin(115200);
  Serial.write(COM_READY);
}

void loop() {
  if (Serial.available()) {
    
    int coms = Serial.read();
    
    char comStr[3];
    itoa(coms, comStr, 10);
    u8g2.clearBuffer();
    u8g2.drawStr(20, 20, comStr);
    u8g2.sendBuffer();
    
    switch (coms) {
      case COM_FRAME_AVAILABLE:
        Serial.write(COM_REQUEST_FRAME);
        break;
      case 4:
        Serial.write(COM_FRAME_RECEIVED);
        break;
      default:
        break;
    }
  }

  delay(500);
  
}
