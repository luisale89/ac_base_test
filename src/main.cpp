#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>

//global variables
enum LOG_LEVEL {
  DEBUG,
  INFO,
  WARNING,
  ERROR,
};

//logger level definition.
LOG_LEVEL logger_level = DEBUG;

// pinout
#define RADAR 26
#define LED_DATA 27
#define NETWORK_LED 2
#define MANUAL_BTN 34
#define AP_BTN 19
#define NOW_BTN 18
#define AMB_TEMP 32
#define NUMPIXELS 24

float ambient_temperature = 0.0;
bool last_output_state = false;
int sensor_resolution = 12; // bits
unsigned long lastTempRequest = 0;
unsigned long lastIOUpdate = 0;
int delayInMillis = 0;
const unsigned long IOUpdateInterval = 5L * 1000L; // 5 second.


//oneWire and DallasTemperature
OneWire ow_ambient_t(AMB_TEMP);
DallasTemperature ambient_t_sensor(&ow_ambient_t);

DeviceAddress ambientSensorAddress;

//RTC
RTC_DS3231 RTC;

//neopixel

//logger function.
void logger(char *message, LOG_LEVEL msg_level){
  unsigned long now = millis();
  switch (msg_level)
  {
  case DEBUG:
    /* code */
    if (logger_level <= DEBUG) {
      Serial.print("- [D] @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
    }
    break;

  case INFO:
    if (logger_level <= INFO) {
      Serial.print("- [I] @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
    }
    break;

  case WARNING:
    if (logger_level <= WARNING) {
      Serial.print("- [W] @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
    }
    break;

  case ERROR:
    if (logger_level <= ERROR) {
      Serial.print("- [E] @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
    }
    break;
  
  default:
    Serial.println("not implemented..");
    Serial.flush();
    break;
  }
  return;
}

void get_ambient_temperature() {
  char word_buffer[40];
  char temp_buffer[8];
  //--

  if (millis() - lastTempRequest >= delayInMillis) {
    // waited long enough?
    logger("reading temperature from DS18B20 sensor..", DEBUG);
    float tempC = ambient_t_sensor.getTempCByIndex(0); // temp of the first sensor.
    if (tempC == -127)
    {
      logger("Error -127 while reading sensor; [Ambient temperature]", ERROR);
    }

    //-- logging
    dtostrf(tempC, 5, 2, temp_buffer);
    sprintf(word_buffer, "Temperature value: %s °C", temp_buffer);
    logger(word_buffer, DEBUG);
    //--

    //request new readings.
    ambient_t_sensor.requestTemperatures();
    lastTempRequest = millis();
  }
  return;
}


void update_IO(){
  // io updates.
  char word_buffer[40];
  
  if (millis() - lastIOUpdate >= IOUpdateInterval) {
    //-
    // get datetime
    DateTime now = RTC.now();
    char buf2[] = "YYMMDD-hh:mm:ss";
    logger("printing datetime ->", DEBUG);
    Serial.println(now.toString(buf2));

    sprintf(word_buffer, "inputs: RAD[%d] MAN[%d] AP[%d] NOW[%d]", 
    digitalRead(RADAR), digitalRead(MANUAL_BTN), digitalRead(AP_BTN), digitalRead(NOW_BTN));
    logger(word_buffer, DEBUG);
    switch (last_output_state)
    {
    case true:
      digitalWrite(NETWORK_LED, LOW);
      last_output_state = false;
      break;
    
    default:
      digitalWrite(NETWORK_LED, HIGH);
      last_output_state = true;
      break;
    }

    lastIOUpdate = millis();
  }

  return;
}

void setup() {
  // put your setup code here, to run once:
  logger("Welcome! Setting up device.", INFO);
  //--Serial.
  Serial.begin(115200);
  //--RTC
  logger("searching RTC on I2C bus", DEBUG);
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
  logger("Setting up DS18B20 sensor.", DEBUG);
  ambient_t_sensor.begin();
  ambient_t_sensor.getAddress(ambientSensorAddress, 0);
  ambient_t_sensor.setResolution(ambientSensorAddress, sensor_resolution);
  ambient_t_sensor.setWaitForConversion(false);
  logger("request for DS18B20 temp. readings.", DEBUG);
  ambient_t_sensor.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - sensor_resolution));
  lastTempRequest = millis();
  logger("DS18B20 settings done.", DEBUG);

  //-- I-O
  logger("setting up I-O pins", DEBUG);
  pinMode(RADAR, INPUT);
  pinMode(MANUAL_BTN, INPUT);
  pinMode(AP_BTN, INPUT);
  pinMode(NOW_BTN, INPUT);
  pinMode(NETWORK_LED, OUTPUT);

  // finish.
  logger("setup finished!", INFO);
}

void loop() {
  //get DS18B20 temperature and print to serial logger.
  get_ambient_temperature();
  update_IO();
}