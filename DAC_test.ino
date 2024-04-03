/////////////////////////////////////////////////////////////////////
// DAC_test v 0.02
// Brandon Doyle
// 3/27/2024
// 
// Trying to get both the AD5310 or DAC8531 working.
//
// I intend to apply this in my contribution to the class project
// for Uwe Konopka's electronics course.
//
// v 0.02
// working so far:
//   - Use a potentiometer to select a voltage between 0 and 5 V
//   - Report the selected voltage to a computer via serial (to be removed?)
//   - Display the selected voltage on an I2C OLED screen
//   - Ask DAC8531 to output selected voltage
// Not working yet:
//   - Want to also be able to use AD5310
//   - Read value from DAC and also display it on the OLED?
//
// Physical setup:
// I'm using an Elegoo Uno R3 with the following pin connections:
// A0 <- potentiometer signal
// A4 -> OLED SDA
// A5 -> OLED SCL
//
// Note: I'm using my knockoff Uno R3 because the provided LGT8F328P
// (purple knockoff nano) isn't supported by the native Serial library.
// But I'm sure someone more dedicated than I am could get it working.
//
/////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/////////////Begin OLED setup stuff///////////////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Define a splash screen. This is the Adafruit logo. For the laser project,
//I vote we replace this with a bmp of the Death Star.
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };
/////////////End OLED setup stuff///////////////

/////////////Begin SPI setup stuff///////////////
const uint16_t mode_run   = 0x0000;
const uint16_t mode_kToG  = 0x1000;
const uint16_t mode_MToG  = 0x2000;
const uint16_t mode_three = 0x3000;

uint16_t DAC_mode = mode_run;
int csPin = 10;
/////////////End SPI setup stuff///////////////

int potPin = A0;
uint16_t potVal = 0;
float potToVolt = 5.0/1023; //Conversion from pot value to V
float volt = 0;
char* msg = new char[50];

void setup() {
  // SPI chip select pin should start HIGH:
  pinMode(csPin,OUTPUT);
  digitalWrite(csPin,HIGH);

  //SPI requires begin() and THEN beginTransaction()
  SPI.begin();
  SPI.beginTransaction(SPISettings(30000000, MSBFIRST, SPI_MODE1));

  //Begin Serial for reporting to a computer's serial monitor:
  Serial.begin(9600);

  //Begin I2C communication to the OLED screen:
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show splash screen:
  display.display();
  delay(1000); // Pause for 1 second

  // Clear the OLED buffer
  display.clearDisplay();
}

void loop() {
  // Read pot and convert to volts:
  potVal = analogRead(potPin);
  volt = potVal*potToVolt;
  
  // Make a cstring to report voltage:
  // Annoyingly, sprintf float formatting isn't supported in Arduino,
  // so I have to do it this weird way.
  msg = dtostrf(volt,5,2,msg);
  msg = strcat(msg," V\0");
  
  // Report via serial connection:
  //Serial.println(msg);

  //Display to OLED screen:
  OLED_show(msg);

  //Update DAC:
  update_DAC(potVal); //potVal is a 10 bit value 0->1023
}

void OLED_show(char* msg){
  display.clearDisplay();

  display.setTextSize(3);             // Double the normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);// Draw white text (monochrome board, but doesn't work without this)
  display.setCursor(0,16);            // In pixels (x,y) from top left corner
                                      // The top 16-ish pixels on my OLED are yellow
                                      // and below that, they're blue.

  display.print(msg);
  display.display();
}

void update_DAC(uint16_t val){
  //AD5310 protocol:
  //uint16_t to_send = val<<2 | DAC_mode;
  //digitalWrite(csPin,LOW);
  //SPI.transfer16(to_send);
  //digitalWrite(csPin,HIGH);

  //DAC8531 protocol:
  uint8_t mode_8bit = DAC_mode >> 12; //Translate mode byte to DAC8531 protocol
  digitalWrite(csPin,LOW);            //Begin communication with Chip Select pin
  SPI.transfer(mode_8bit);            //Transfer 8 bit mode byte
  SPI.transfer16(val<<6);             //Transfer 16 bit value (bit-shifted from 10 bit)
  digitalWrite(csPin,HIGH);           //End communication
}