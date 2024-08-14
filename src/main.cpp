#include <Arduino.h>

//global variables
enum LOG_LEVEL {
  DEBUG = 1,
  INFO = 2,
  WARNING = 3,
  ERROR = 4
};

LOG_LEVEL logger_level = DEBUG;

// void logger(char *message, LOG_LEVEL msg_level);
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
    }
    break;

  case INFO:
    if (logger_level <= INFO) {
      Serial.print("- info @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
    }
    break;

  case WARNING:
    if (logger_level <= WARNING) {
      Serial.print("% warning @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
    }
    break;

  case ERROR:
    if (logger_level <= ERROR) {
      Serial.print("$ error @ ");
      Serial.print(now);
      Serial.print(" > msg: ");
      Serial.println(message);
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

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(115200);
  Serial.println("hello world!");
}

void loop() {
  // put your main code here, to run repeatedly:
  logger("debug message", DEBUG);
  logger("info message", INFO);
  logger("warning message", WARNING);
  logger("error message", ERROR);
  delay(1000);
}