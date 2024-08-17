#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

//global variables
enum LOG_LEVEL {
  DEBUG = 1,
  INFO = 2,
  WARNING = 3,
  ERROR = 4
};

// pinout
#define RADAR 26
#define LED_DATA 27
#define LED_CLOCK 14
#define NETWORK_LED 2
#define MANUAL_BTN 34
#define AP_BTN 19
#define NOW_BTN 18
#define AMB_TEMP 32
#define NUMPIXELS 24

float ambient_t_reading = 0.0;
bool last_output_state = false;

//oneWire and DallasTemperature
OneWire ow_ambient_t(AMB_TEMP);
DallasTemperature ambient_t_sensor(&ow_ambient_t);

//RTC
RTC_DS3231 RTC;

//logger level definition.
LOG_LEVEL logger_level = DEBUG;

//neopixel
Adafruit_NeoPixel pixels(NUMPIXELS, LED_CLOCK, NEO_GRB + NEO_KHZ400);

//logger function.
void logger(char *message, LOG_LEVEL msg_level){
  unsigned long now = millis();
  switch (msg_level)
  {
  case DEBUG:
    /* code */
    if (logger_level <= DEBUG) {
      Serial.print("# debug @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
      Serial.flush();
    }
    break;

  case INFO:
    if (logger_level <= INFO) {
      Serial.print("- info @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
      Serial.flush();
    }
    break;

  case WARNING:
    if (logger_level <= WARNING) {
      Serial.print("% warning @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
      Serial.flush();
    }
    break;

  case ERROR:
    if (logger_level <= ERROR) {
      Serial.print("$ error @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
      Serial.flush();
    }
    break;
  
  default:
  Serial.println("not implemented..");
    break;
  }
  return;
}

int myFunction(int x, int y) {
  return x + y;
}

void get_ambient_temperature() {
  char word_buffer[30];
  char temp_buffer[8];
  //--
  logger("getting temperature from OneWire sensor..", DEBUG);
  ambient_t_sensor.requestTemperaturesByIndex(0);
  float tempC = ambient_t_sensor.getTempCByIndex(0); // temp of the first sensor.
  if (tempC == -127)
  {
    logger("Error -127 from OneWire sensor - [Ambient temperature]", ERROR);
  }
  if (tempC == 85) {
    logger("Got 85 deg. as tempC reading", WARNING);
  }

  //-- logging
  dtostrf(tempC, 5, 2, temp_buffer);
  sprintf(word_buffer, "Temperature value: %s", temp_buffer);
  logger(word_buffer, DEBUG);
  //--
  return;
}


void update_inputs(){
// #define RADAR 26;
// #define MANUAL_BTN 34;
// #define AP_BTN 19;
// #define NOW_BTN 18;
char word_buffer[40];
sprintf(word_buffer, "input states: RADAR[%d], MANU[%d], AP[%d], NOW[%d]", 
digitalRead(RADAR), digitalRead(MANUAL_BTN), digitalRead(AP_BTN), digitalRead(NOW_BTN));
logger(word_buffer, DEBUG);
return;
}

void update_outputs(){
  //--
  switch (last_output_state)
  {
  case true:
    digitalWrite(NETWORK_LED, LOW);
    last_output_state = false;
    break;
  
  default:
    digitalWrite(NETWORK_LED, HIGH);
    last_output_state = true;
  }
  return;
}

void update_pixels(){
  pixels.clear();

  for(int i=0; i<NUMPIXELS; i++) {

    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show();
    delay(42); // approx. 1sec.
  }
}

void setup() {
  // put your setup code here, to run once:
  logger("Welcome! Setting up device.", INFO);
  //--Serial.
  Serial.begin(115200);
  //--RTC
  logger("finding RTC on I2C bus", DEBUG);
  if (!RTC.begin())
  {
    logger("RTC not found in I2C bus, please reboot", ERROR);
    Serial.flush();
    while (1) delay(10);
  }
  logger("RTC found!", DEBUG);
  if (RTC.lostPower())
  {
    logger("RTC lost power!, setting up sketch compiled datetime.", DEBUG);
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  logger("RTC configured!", DEBUG);
  //--DS18B20
  logger("Setting up OneWire sensor", DEBUG);
  ambient_t_sensor.begin();
  ambient_t_sensor.setResolution(9); //9 bits resolution.
  delay(500);
  logger("first temp. reading..", DEBUG);
  get_ambient_temperature();
  //-- I-O
  pinMode(RADAR, INPUT);
  pinMode(MANUAL_BTN, INPUT);
  pinMode(AP_BTN, INPUT);
  pinMode(NOW_BTN, INPUT);
  pinMode(NETWORK_LED, OUTPUT);

  //-- NeoPixel
  pixels.begin();
  pixels.show();
  pixels.setBrightness(50); //50% brightness

  logger("setup finished. 1sec", DEBUG);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //get DS18B20 temperature and print to serial logger.
  get_ambient_temperature();
  update_inputs();
  update_outputs();
  update_pixels();

  //--delay and back again.
  delay(500);
}