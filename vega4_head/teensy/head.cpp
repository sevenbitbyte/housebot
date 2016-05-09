#include "Arduino.h"

#include <ros.h>
#include <sensor_msgs/Image.h>

const int ledPin = 13;

ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;

void setup() {
  pinMode(ledPin, OUTPUT);

  nh.initNode();
  nh.getHardware()->setBaud(115200);
}


void loop() {
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(100);

  nh.spinOnce();
}

