#define WR_INPUT_SOURCE_SELECT                       0x5
#define         SOURCE_SELECT_EXTERNAL_VIDEO_PORT    0x0
#define WR_EXTERNAL_VIDEO_SOURCE_FORMAT_SELECT       0x7
#define         SOURCE_FORMAT_RGB888_24PIN           0x43
#define WR_IMAGE_CROP                                0x10
#define WR_DISPLAY_SIZE                              0x12
#define WR_DISPLAY_IMAGE_ORIENTATION                 0x14
#define WR_DISPLAY_IMAGE_CURTAIN                     0x16
#define WR_EXTERNAL_INPUT_IMAGE_SIZE                 0x2e
#define WR_LED_OUTPUT_CONTROL_METHOD                 0x50
#define         LED_OUTPUT_CONTROL_MANUAL            0x0
#define         LED_OUTPUT_CONTROL_CAIC              0x1
#define WR_RGB_LED_ENABLE                            0x52
#define         RGB_LED_ENABLE_RGB                   0x7
#define WR_RGB_LED_CURRENT                           0x54
#define RD_CONTROLLER_DEVICE_ID                      0xd4  // Should be 0x05
#define RD_DMD_DEVICE_ID                             0xd5  // Should be 0x60, 0x0d, 0x00, 0x68
#define WR_IMAGE_FREEZE                              0x1a
#define BR_TENS                                      0x32 // tens digits for brightness
#define BR_HUNDREDS                                  0x02 // Hundreds digits for brightness
#define PROJ_I2C_ADDR (0x36 >> 1)

//Projector TI chip. WR write, RD read
// Register addresses can be anything from 1-8 byte wide, be careful
// Config exactly based off of reference board.
// Reference: https://www.ti.com/lit/ug/dlpu020d/dlpu020d.pdf?ts=1632742451506&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDLPC3438 
//      REGISTER                               WIDTH, VALUE
byte dlpc3438_config[] = {
       WR_DISPLAY_IMAGE_CURTAIN,               1, 1, // Enable
       WR_IMAGE_FREEZE,                        1, 1, // Enable
       WR_IMAGE_CROP,                          8, 0, 0, 0, 0, 0x00, 0x05, 0xd0, 0x02,     // (0,0) - (1280,720)
       // WR_DISPLAY_SIZE,                        4, 0x00, 0x05, 0xd0, 0x02, // 1280x720, attempt at manual
       WR_DISPLAY_SIZE,                        8, 0x00,0x00,0x00,0x00,0x00, 0x05, 0xd0, 0x02, // 1280x720
       WR_EXTERNAL_INPUT_IMAGE_SIZE,           4, 0x00, 0x05, 0xd0, 0x02, // 1280x720, reference board based
       WR_DISPLAY_IMAGE_ORIENTATION,           1, 0x06, // Flip over short and long axis (face was upside down)
       WR_EXTERNAL_VIDEO_SOURCE_FORMAT_SELECT, 1, SOURCE_FORMAT_RGB888_24PIN,      
       WR_INPUT_SOURCE_SELECT,                 1, SOURCE_SELECT_EXTERNAL_VIDEO_PORT,
       WR_IMAGE_FREEZE,                        1, 0, // Enable
       WR_DISPLAY_IMAGE_CURTAIN,               1, 0, // Enable
       WR_LED_OUTPUT_CONTROL_METHOD,           1, LED_OUTPUT_CONTROL_MANUAL,
       WR_RGB_LED_ENABLE,                      1, RGB_LED_ENABLE_RGB,
       WR_RGB_LED_CURRENT,                     6, BR_TENS, BR_HUNDREDS, BR_TENS, BR_HUNDREDS, BR_TENS, BR_HUNDREDS, // 0x0100 for R, G & B . This value is 100, as an example.
       0xFF, 0xFF, 0xFF, 0xFF // STOP
};

void i2c_read(byte addr, byte reg, int len) {
  addr = addr >> 1; // 7-bit addr
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len);
  while (Wire.available()) {
    byte c = Wire.read();
    Serial.print(c, HEX);
  }
  Wire.endTransmission();
  Serial.println("");
}

void dlpc3438_show_red_test_pattern() {
  // Red test pattern
  Wire.beginTransmission(PROJ_I2C_ADDR);
  Wire.write(0x0b);
  Wire.write(0x80);
  Wire.write(0x11);
  Wire.endTransmission();

  Wire.beginTransmission(PROJ_I2C_ADDR);
  Wire.write(WR_INPUT_SOURCE_SELECT);
  Wire.write(0x01);
  Wire.endTransmission();
}

void dlpc3438_init_reg() {
  int i = 0;
  byte len = 0x00;
  byte reg = 0x00;
  byte val = 0x00;

  while (true) {
    // Stop at 0xFF,0xFF,0xFF,0xFF
    if (dlpc3438_config[i] == 0xFF && dlpc3438_config[i+1] == 0xFF && dlpc3438_config[i+2] == 0xFF && dlpc3438_config[i+3] == 0xFF) {
      break;
    }

    reg = dlpc3438_config[i++];
    len = dlpc3438_config[i++];

    Wire.beginTransmission(PROJ_I2C_ADDR);
    Wire.write(reg);

    for (int j = 0; j < len; j++) {
      Wire.write(dlpc3438_config[i++]);
    }
    
    Wire.endTransmission();
  }
}
