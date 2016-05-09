#include "Arduino.h"

#include <ros.h>
#include <std_msgs/Float32.h>

const int ledPin = 13;
const int vMonPin = 14;
const int iMonPin = 15;
const int powerEAPin = 2;

#define V_MON_COEF (16.35f / 658)

ros::NodeHandle nh;

std_msgs::Float32 voltageMsg;
std_msgs::Float32 currentMsg;

ros::Publisher voltagePub("voltage", &voltageMsg);
ros::Publisher currentPub("current", &currentMsg);

void setup() {
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
  pinMode(powerEAPin, OUTPUT);
  digitalWrite(powerEAPin, LOW);

  nh.initNode();
  nh.advertise(voltagePub);
  nh.advertise(currentPub);
}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {
  digitalWrite(ledPin, HIGH);   // set the LED on
  delay(100);                  // wait for a second
  digitalWrite(ledPin, LOW);    // set the LED off
  delay(100);                  // wait for a second

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

