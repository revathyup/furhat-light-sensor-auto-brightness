// Board: SparkFun Pro Micro, 3.3V 8MHz
// NOTE: Needs patching of pins_arduino.h:
//       TX_RX_LED_INIT, TXLED0, TXLED1 and LED_BUILTIN_TX
//       needs to be cleared and not touch PD5

#include <FastLED.h>

#include "adv7611.h"
#include "dlpc3438.h"

 // Digital pin assignments:
 #define PIN_ROTARY_A                     0 // PD2 - D0
 #define PIN_ROTARY_B                     1 // PD3 - D1
 #define PIN_ROTARY_SWITCH               19 // PF6 - A6 
 
 #define PIN_FAN_PROJ                     6 // PD7 - D6
 
 #define PIN_LED                          8 // LED_DATA - PB4 - D8
 #define NUM_LEDS                        86
 
 #define PIN_AMP_OFF                      7 // PE6 - D7
 
 #define PIN_RFID_RESET                  20 // PF5/ADC5 - A2/D20
 #define PIN_RFID_SS                     10 // PB6 - D10

 #define PIN_PROJ_TEMP_ADC               A5 // PF0/ADC0 - A5/D23
 #define PIN_PROJ_LED_CTRL               21 // PF4/ADC4 - A3/D21
 
 #define PIN_5V_12V_ENABLE               13 // PC7 - D13
 
 // NOTE: Needs patching of pins_arduino.h:
 //       TX_RX_LED_INIT, TXLED0, TXLED1 and LED_BUILTIN_TX
 //       needs to be cleared and not touch PD5
 #define PIN_PROJ_ON                     30 // PD5 -  D30/TX Led - Needs patching of pins_arduino.h
 
 #define PIN_I2C_SDA                      2 // PD1 - D2
 #define PIN_I2C_SCL                      3 // PD0 - D3
 #define PIN_I2C_SEL                     18 // PF7/ADC7 - A0/D18 - 0 = HDMI, 1 = PROJ

 #define PIN_MOTOR_SENSOR_OUT            A4 // PF1/ADC1 - A4/D22 
 #define PIN_MOTOR_DIRECTION              4 // PD4 - D4 //sets direction
 #define PIN_MOTOR_STEP                  11 // PB7 - D11 // moves step in above direction
 #define PIN_MOTOR_nSLEEP                 5 // PC6 - D5


 #define PIN_ADV7611_nRESET               9 // PB5 - D9
 #define PIN_SPI_MISO                    14 // PB3 - MISO - D14
 #define PIN_SPI_MOSI                    16 // PB2 - MOSI - D16
 #define PIN_SPI_SCK                     15 // PB1 - SCK  - D15

 #define PIN_USB_VBUS                  VBUS
 #define PIN_NUC_POWER_LED               12 // PD6 - D12

 #define VERSION_MAJOR                    2
 #define VERSION_MINOR                    2
 #define VERSION_STEP                     0

// State machine states for reading the multibyte commands
// coming from the firmware host server:

 #define STATE_START                      0
 #define STATE_LED_RED_VALUE              1
 #define STATE_LED_GREEN_VALUE            2
 #define STATE_LED_BLUE_VALUE             3

 // Are we in LED stream state or not? 
 
 // Set all the LEDs in the array to the same value:
 #define STATE_STREAM_OFF                 0
 
 // Set each pixel in the array to its own value that is
 // streamed from the host:
 #define STATE_STREAM_ON                  1

 // Projector temperature thresholds
 #define PROJ_THERM_SHUTDOWN 70 // Thermal shutdown around 52C
 #define PROJ_THERM_WARNING  60 // Send thermal shutdown warning
 #define PROJ_THERM_FAN_MAX  45 // Maximum fan speed around 45C
 #define PROJ_THERM_FAN_MIN  30 // Start ramping fan from around 30C // 100 steps inbetween, allows scaling from 0-255 in fan speed
 
 // USB VBUS / NUC power state
 unsigned char                            vbus_state;
 
 // Variables to hold the button/direction of the rotary encoder:
 
 unsigned char                            rc_state;
 unsigned char                            rc_interrupt = 0;
 unsigned char                            rc_direction;
 unsigned char                            rc_button_state;
 unsigned char                            rc_last_button_state;
 unsigned char                            input_byte;
 unsigned char                            state;
 unsigned char                            stream_state;
 unsigned char                            motor_nsleep = 0;
 
 //Bootup sequence variables
 unsigned char                            nuc_power_on_counter = 0;
 unsigned char                            contacted_by_nuc = 0;
 unsigned char                            bootup_brightness = 25;
 unsigned char                            bootup_direction = 1;
 unsigned char                            boot_sequence_ongoing = 0;

 // RGB values parsed:
 unsigned char                            red_value;
 unsigned char                            green_value;
 unsigned char                            blue_value;
 
 // RGB LED array stream counter
 unsigned char                            stream_counter;

 // Array to hold all pixel color values for the LED strip:
 CRGB leds[NUM_LEDS];

 // Amp related variables
 unsigned char                            amp_powerOn;
 int                                      amp_lastMillis;
 int                                      amp_lastFrame;
 int                                      amp_usbTimeout;
 
 void projector_on () {
   // Power on projector
   delay(500); // minimum time from when power has been supplied

   // Hold projector PIN37 low for 1 second
   digitalWrite(PIN_PROJ_ON, LOW);
   delay (1000);
   
   // Pull projector PIN37 high
   digitalWrite(PIN_PROJ_ON, HIGH);

   // Select I2C bus before talking to dlpc3438
   digitalWrite(PIN_I2C_SEL, HIGH);
   
   // Wait at least 2 seconds before
   delay(2000);
   dlpc3438_init_reg();

   // Pull projector PIN48
   //No noticable off, low, or high
   //digitalWrite(PIN_PROJ_LED_CTRL, HIGH); // Maybe not needed?, could potentially needed for manual brightness control? No one knows what this is supposed to do.
 }

 void projector_off() {
   digitalWrite(PIN_PROJ_LED_CTRL, LOW);
   delay(1000); // Maybe needed for poweroff
   digitalWrite(PIN_PROJ_ON, LOW);
 }

 void focus_step(int direction) {
   // Focus motor, period time 16 and 30 ms on reference board
   digitalWrite(PIN_MOTOR_nSLEEP, HIGH);
   digitalWrite(PIN_MOTOR_DIRECTION, direction);
   digitalWrite(PIN_MOTOR_STEP, HIGH);
   delay(30);
   digitalWrite(PIN_MOTOR_STEP, LOW);
   delay(30);
   motor_nsleep = 2; // Delayed nSLEEP in main loop / fan_control - seems to need more time
 }

 // Projector temp sensor schematics
 //
 //                      3V3
 //                       |
 //                      [R] 10k
 // PIN_PROJ_TEMP_ADC-----|
 //                      [R] 10k NTC
 //
 // Projector NTC resistance values from datasheet
 // for conversion between deg C and raw ADC values
 //  C    ohm  raw
 // 10  17926  657
 // 15  14674  609
 // 20  12081  560
 // 25  10000  512
 // 30   8315  465
 // 35   6948  420
 // 40   5834  377
 // 45   4917  338
 // 50   4161  301
 // 55   3535  267
 // 60   3014  237
 //
 // ADC raw values for measured temperatures
 // with projector powered off and on:
 //  C - OFF - ON
 // 23 - 168 - 516
 // 25 - 134 - 432
 // 32 - 135 - 420
 // 35 - 129 - 418 (433)
 // 37 - 125 - 409
 // 40 - 116 - 369
 // 46 - 106 - 349
 //


 // Returns raw adc value of the projector's temperature
 // Values are constrained to 3 digit numbers due to messaging protocols.
 int read_proj_temp_raw() {
  int raw_adc = analogRead(PIN_PROJ_TEMP_ADC);
  //Clamp values to three digits for UART protocol reasons
  return constrain(raw_adc,100,999);
 }

//Returns the best celcius approximation of the projector's temperature
 int read_proj_temp_celc(){
  int raw_adc = analogRead(PIN_PROJ_TEMP_ADC);
  float voltage_start = 3.3;
  float percentage = raw_adc / 1024.0;
  float voltage_out = percentage * voltage_start;
  float r2_ohms = (10000 * voltage_out) / (voltage_start - voltage_out);
  float estimated_temp = 311-24.8*log(r2_ohms);
  int temp_integer = estimated_temp + 0.5;

  return constrain(temp_integer,10,99);
 }

 // Fan and temperature monitoring and control
 void fan_control() {
   static unsigned int i = 1;
   if (i++ % 100000 != 0) { // Don't run too often --> Every 1.5 sec in average, possible to be every 5 sec
     return;
   }
   
   int proj_temp = read_proj_temp_celc();

   if (proj_temp > PROJ_THERM_FAN_MAX) {
     // Always run fan at full speed above threshold
     analogWrite (PIN_FAN_PROJ, 255);
     if (proj_temp > PROJ_THERM_SHUTDOWN) { 
       //Serial.print("Thermal protection - shutting down projector!\n");
       Serial.print("po");
       projector_off();
     } else if (proj_temp > PROJ_THERM_WARNING) {
       Serial.print("wt");
     }
   } else if (proj_temp > PROJ_THERM_FAN_MIN) {
     /** Ramp up fan speed based on temperature readings
      * Starts ramping at 30 degrees, up to 45 where it pulls always max (pin to 255)
      * For now, starts 85% of max speed from 30 deg, but could be scaled in a better way, from 40% for instance :
      * speed_start_percentage = 40/100
      * origin_value = 1-(1-speed_start_percentage)*PROJ_THERM_FAN_MAX/(PROJ_THERM_FAN_MAX-PROJ_THERM_FAN_MIN)
      * fanspeed = (origin_value + (temp * (1-speed_start_percentage)/(PROJ_THERM_FAN_MAX-PROJ_THERM_FAN_MIN))) * 255
      * 
      * If we put just 0 as a start, it will ramp linearly from PROJ_THERM_FAN_MIN to PROJ_THERM_FAN_MAX
      */
     int fan_speed = (100 - (PROJ_THERM_FAN_MAX - proj_temp)) * 2.55;
     analogWrite (PIN_FAN_PROJ, fan_speed);
   } else {
     // Turn off fan for low temperatures
     analogWrite (PIN_FAN_PROJ, 0);
   }

   // Disable stepper motor current when idle.
   if (motor_nsleep && --motor_nsleep == 0) {
    digitalWrite(PIN_MOTOR_nSLEEP, LOW); 
   }
 }
 
 // Rotary encoder clock interrupt
 void rc_interrupt_func() {
   if (digitalRead(PIN_ROTARY_A)) {
     rc_interrupt = 1;
     rc_direction = digitalRead(PIN_ROTARY_B);
   }
 }

 // Any setup code that is supposed to have run once the main game loop starts
 // goes here:
 // All pins set to ensure that modes are correct, due to differences that this isn't standard arduino chip.
 void setup() { 
   // Turn off amp by default. Run this is as early as possible to prevent clicking
   amp_powerOn = 0;
   amp_lastFrame = 10;
   pinMode(PIN_AMP_OFF, OUTPUT);

   // Misc pins, output
   pinMode(PIN_I2C_SEL,       OUTPUT);
   pinMode(PIN_FAN_PROJ,      OUTPUT);
   pinMode(PIN_LED,           OUTPUT);
   pinMode(PIN_RFID_RESET,    OUTPUT);
   pinMode(PIN_RFID_SS,       OUTPUT);
   pinMode(PIN_5V_12V_ENABLE, OUTPUT);
   pinMode(PIN_I2C_SEL,       OUTPUT);

   
   // Misc pins, input
   pinMode(PIN_NUC_POWER_LED, INPUT);

   // Focus motor control pins
   pinMode(PIN_MOTOR_SENSOR_OUT, INPUT);
   pinMode(PIN_MOTOR_DIRECTION,  OUTPUT);
   pinMode(PIN_MOTOR_STEP,       OUTPUT);
   pinMode(PIN_MOTOR_nSLEEP,     OUTPUT);


   digitalWrite(PIN_AMP_OFF, HIGH);

   // Set rotary encoder as inputs
   pinMode (PIN_ROTARY_A,       INPUT);
   pinMode (PIN_ROTARY_B,       INPUT);
   pinMode (PIN_ROTARY_SWITCH,  INPUT);
   attachInterrupt(digitalPinToInterrupt(PIN_ROTARY_A), rc_interrupt_func, RISING);

   // Set projector control pins
   pinMode (PIN_PROJ_ON, OUTPUT);
   pinMode (PIN_PROJ_LED_CTRL, OUTPUT);
   pinMode (PIN_PROJ_TEMP_ADC, INPUT);
   
   // Not needed on ATMega with USB UART...
   //Serial.begin (115200);

   // Reads the initial state of the outputA
   rc_last_button_state = digitalRead(PIN_ROTARY_SWITCH);

   // LED library setup:
   FastLED.addLeds<NEOPIXEL, PIN_LED>(leds, NUM_LEDS);
   
   // Initial: Set all LEDs to black:
   for (int i=0; i<NUM_LEDS; i++) {
          leds[i] = 0;
   }
   FastLED.show();
   
   // We start in the state being no characters are read:
   state        = STATE_START;
   stream_state = STATE_STREAM_OFF;

   // Join I2C bus as i2c master
   Wire.begin();
   Wire.setClock(10000);

   // Enable 12V and 5V, should be controlled together
   digitalWrite(PIN_5V_12V_ENABLE, LOW);
   delay(2000); // Block and wait for voltages to get stable

   // Init ADV7611
   digitalWrite(PIN_ADV7611_nRESET, HIGH);
   delay(1000);
   digitalWrite(PIN_I2C_SEL, LOW);
   adv7611_init_reg();
   digitalWrite(PIN_I2C_SEL, HIGH);
 } 



















 
 // Main game loop of the firmware. This never terminates and lasts for some µs (1-5)
 void loop() {
  unsigned char nuc_power_pin = digitalRead(PIN_NUC_POWER_LED);
  // After cable-switching: 111....1111 when off, 000....00000 when on (according to Nick)
  // Before cable-switching: First batch of FPS PCB -> 11111...1111 when on, 010011011... random when off.

  if (nuc_power_pin == 0) {
    if(nuc_power_on_counter < 20){
      nuc_power_on_counter++;
    }
    else if(nuc_power_on_counter >= 20 && contacted_by_nuc == 0){ 
      // For robots have the cable-switch, the LEDs will light up to indicate bootup
      boot_sequence_ongoing = 1;
      for (int i=0; i<NUM_LEDS; i++) {
        leds[i].setRGB (bootup_brightness/2, bootup_brightness/2, bootup_brightness);
      }
      FastLED.show();
      bootup_brightness += bootup_direction;
      if(bootup_brightness == 128) {
        bootup_direction = -1;
      } else if(bootup_brightness == 10){
        bootup_direction = 1;
      }
      delay(10);
    }
  } else if (nuc_power_pin == 1){
    nuc_power_on_counter = 0;
    contacted_by_nuc = 0; //Reset nuc contact flag when power is off.
    if(boot_sequence_ongoing == 1 ){
      boot_sequence_ongoing = 0;
      nuc_power_on_counter++;
      for (int i=0; i<NUM_LEDS; i++) {
        leds[i].setRGB ((int)0, (int)0, (int) 0);
      }
      FastLED.show();
    }
  }
  
   // Make sure projector is turned off if NUC is shut down
   // TODO: Possible update in a future firmware revision, where we keep track of which hardware batch we are using, just uncomment the code below.
   // Note that ArduinoCore currently also sends this signal when it shuts down. 
//   if (digitalRead(PIN_NUC_POWER_LED) == 1 &&
//       digitalRead(PIN_PROJ_ON)       == 1) {
//     projector_off();
//     delay(2000);
//     return;
//   }

   // Rotary encoder: rotation
   if (rc_interrupt) {
     delay(5); // Delay reading 5 ms according to rotary encoder datasheet, could be adjusted to get fast spin... Test with multiple encoders to ensure solid value.
     if (digitalRead(PIN_ROTARY_A)) {
       rc_interrupt = 0;
       if (rc_direction) { 
         Serial.print("ru");     
       } else {
         Serial.print("rd");
       }
     }
   }
   
   // Rotary encoder: press
   rc_button_state = digitalRead(PIN_ROTARY_SWITCH);
   if (rc_button_state != rc_last_button_state){
     if (rc_button_state) {
        Serial.print("bu");
     } else {
        Serial.print("bd");
     }
     // Saving the current state of the rotary encoder button pin for
     // comparing against during the next iteration of the loop:
     rc_last_button_state = rc_button_state;
   }
   
   fan_control();

   while(Serial.available()) {
    contacted_by_nuc = 1;
    input_byte = Serial.read();

    if (state == STATE_START) {

        // If first character is 'l' , the next character will be the LEDVALUE
        if (input_byte == 'l') {
          Serial.print("l");
          state = STATE_LED_RED_VALUE;

        // If the first character is 's', the next character will be red value of the
        // first series of r,g,b - values in a long array of pixel values:
        } else if (input_byte == 's') {
          Serial.print("s");
          stream_state   = STATE_STREAM_ON;
          state          = STATE_LED_RED_VALUE;
          stream_counter = 0;
        
        } else if (input_byte == 'v') {
          Serial.print("vFW"); // echo the last character sent 'v' and then 'FW' to indicate that the rest is not random characters from the stream
          Serial.print((char) VERSION_MAJOR); // but an actual three-byte firmware version.
          Serial.print((char) VERSION_MINOR);
          Serial.print((char) VERSION_STEP);
          state = STATE_START;
        } else if (input_byte == 'a') {
          Serial.print("a");
          digitalWrite(PIN_AMP_OFF, HIGH);   // HIGH = Shutdown on, amp off
          amp_powerOn = false;
        } else if (input_byte == 'A') {
          Serial.print("A");
          digitalWrite(PIN_AMP_OFF, LOW);   // LOW = Shutdown off, amp on
          amp_powerOn = true;
        } else if (input_byte == 'f') {  // Change focus direction 0
          Serial.print("f");
          focus_step(0);
        } else if (input_byte == 'F') {  // Change focus direction 1
          Serial.print("F");
          focus_step(1);
        } else if (input_byte == 'p') {
          Serial.print("p");
          projector_off();
        } else if (input_byte == 'P') {
          Serial.print("P");
          projector_on();
        } else if (input_byte == 'b') {
          Serial.print("b");
          analogWrite (PIN_FAN_PROJ, 0);
        } else if (input_byte == 'B') {
          Serial.print("B");
          analogWrite (PIN_FAN_PROJ, 255);
        } else if (input_byte == 't') {
          int proj_temp = read_proj_temp_raw();
          Serial.print("t");
          Serial.print(proj_temp, DEC);
        } else if (input_byte == 'T'){
          int proj_temp = read_proj_temp_celc();
          Serial.print("T");
          Serial.print(proj_temp, DEC);
        } else {
          state = STATE_START;
        }
    } else {
      // Echo the value to the host:
      Serial.print((unsigned char)input_byte);
      
      if (state == STATE_LED_RED_VALUE) {
        red_value = (unsigned char)input_byte;
        state = STATE_LED_GREEN_VALUE; 
      
      } else if (state == STATE_LED_GREEN_VALUE) {
        green_value = (unsigned char)input_byte;
        state = STATE_LED_BLUE_VALUE;
      
      } else if (state == STATE_LED_BLUE_VALUE) {
        blue_value = (unsigned char)input_byte;

        // The last byte of the RGB triplet has been received...

        // There are two possibilities here: Either we are in single LED state, and then we assign all the LEDs in
        // the array to the same RGB triplet value, or we are in STREAM_ARRAY state were we loop over all
        // of the LEDs in the array, streaming values from the serial port to fill the array.
        
        // Alternative 1: We are in LED streaming mode: Go to the next pixel and increment the pixel counter until we run
        //                out of pixels, and then go back to the starting state.
        
        if (stream_state == STATE_STREAM_ON) {
          
          leds[stream_counter].setRGB((int)red_value,(int)green_value,(int)blue_value);
          
          // next pixel. increment the array counter in the pixel array and go back to setting a RED value:
          stream_counter ++;
          state = STATE_LED_RED_VALUE;
         
          if (stream_counter == NUM_LEDS) {
            state = STATE_START;
            stream_state = STATE_STREAM_OFF;
            FastLED.show();
          }

        // Alternative 2: We are in non-streaming state. Loop through the whole LED array and set all of the pixels to 
        //                the same value:
        
        } else {
          for (int i=0; i<NUM_LEDS; i++) {
              leds[i].setRGB ((int)red_value, (int)green_value, (int)blue_value);
          }
          FastLED.show();
          state = STATE_START;
        }
      }
    }
   }
 }
