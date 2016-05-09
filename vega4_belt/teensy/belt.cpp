#include "Arduino.h"

#include <ros.h>
#include <std_msgs/Float32.h>

const int ledPin = 13;
const int vMonPin = 14;
const int iMonPin = 15;
const int powerEAPin = 2;

#define V_MON_COEF (16.35f / 658)

ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;

std_msgs::Float32 voltageMsg;
std_msgs::Float32 currentMsg;

ros::Publisher voltagePub("voltage", &voltageMsg);
ros::Publisher currentPub("current", &currentMsg);

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(powerEAPin, OUTPUT);
  digitalWrite(powerEAPin, LOW);

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(voltagePub);
  nh.advertise(currentPub);
}


void loop() {
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(100);

  voltageMsg.data = analogRead(vMonPin) * V_MON_COEF;
  voltagePub.publish(&voltageMsg);

  if(voltageMsg.data < 12.0){
    digitalWrite(powerEAPin, LOW);
  }
  else{
    digitalWrite(powerEAPin, HIGH);
  }

  currentMsg.data = analogRead(iMonPin);
  currentPub.publish(&currentMsg);

  nh.spinOnce();

}

