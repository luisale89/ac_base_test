#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <esp_log.h>

//global variables
static const char* TAG = "main_module";

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

void get_ambient_temperature() {
  char word_buffer[40];
  char temp_buffer[8];
  //--

  if (millis() - lastTempRequest >= delayInMillis) {
    // waited long enough?
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "reading temperature from DS18B20 sensor..");
    float tempC = ambient_t_sensor.getTempCByIndex(0); // temp of the first sensor.
    if (tempC == -127)
    {
      ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "Error -127 while reading sensor; [Ambient temperature]");
    }

    //-- logging
    dtostrf(tempC, 5, 2, temp_buffer);
    sprintf(word_buffer, "Temperature value: %s °C", temp_buffer);
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "%s", word_buffer);
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
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "DateTime: %s", now.toString(buf2));
    //--
    sprintf(word_buffer, "inputs: RAD[%d] MAN[%d] AP[%d] NOW[%d]", 
    digitalRead(RADAR), digitalRead(MANUAL_BTN), digitalRead(AP_BTN), digitalRead(NOW_BTN));
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "%s", word_buffer);
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
      break;
    }

    lastIOUpdate = millis();
  }

  return;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Welcome!");
  esp_log_level_set("*", ESP_LOG_DEBUG);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Welcome! Setting up device.");
  //--Serial.
  //--RTC
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "searching RTC on I2C bus");
  if (!RTC.begin())
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "RTC not found in I2C bus, please reboot");
    Serial.flush();
    while (1) delay(10);
  }
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "RTC found!");
  if (RTC.lostPower())
  {
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "RTC lost power!, setting up sketch compiled datetime.");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "RTC configured!");
  

  //--DS18B20
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "Setting up DS18B20 sensor.");
  ambient_t_sensor.begin();
  ambient_t_sensor.getAddress(ambientSensorAddress, 0);
  ambient_t_sensor.setResolution(ambientSensorAddress, sensor_resolution);
  ambient_t_sensor.setWaitForConversion(false);
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "request for DS18B20 temp. readings.");
  ambient_t_sensor.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - sensor_resolution));
  lastTempRequest = millis();
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "DS18B20 settings done.");

  //-- I-O
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "setting up I-O pins");
  pinMode(RADAR, INPUT);
  pinMode(MANUAL_BTN, INPUT);
  pinMode(AP_BTN, INPUT);
  pinMode(NOW_BTN, INPUT);
  pinMode(NETWORK_LED, OUTPUT);

  // finish.
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "setup finished!");
}

void loop() {
  //get DS18B20 temperature and print to serial logger.
  get_ambient_temperature();
  update_IO();
}