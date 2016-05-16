#include "Arduino.h"

#include <Adafruit_NeoPixel.h>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8MultiArray.h>

const int ledPin = 13;
const int vMonPin = 14;
const int iMonPin = 15;
const int powerEAPin = 2;

#define V_MON_COEF (16.35f / 658)

Adafruit_NeoPixel beltPixels = Adafruit_NeoPixel(23, 20, NEO_RGBW + NEO_KHZ800);

ros::NodeHandle_<ArduinoHardware, 25, 25, 4096, 4096> nh;

std_msgs::Float32 voltageMsg;
std_msgs::Float32 currentMsg;

ros::Publisher voltagePub("voltage", &voltageMsg);
ros::Publisher currentPub("current", &currentMsg);

void setStripMultiArray(Adafruit_NeoPixel& strip, const std_msgs::Int8MultiArray& pixels){
  for(int i=0; i<strip.numPixels() && i<pixels.data_length; i++){
    int idx = i*4;	//NOTE: We expect RGBW data only

    byte red = pixels.data[idx];
    byte green = pixels.data[idx + 1];
    byte blue = pixels.data[idx + 2];
    byte white = pixels.data[idx + 3];

    strip.setPixelColor(i, strip.Color( red, green, blue, white));
  }

  strip.show();
}

void beltCb(const std_msgs::Int8MultiArray& pixels){
  setStripMultiArray(beltPixels, pixels);
}

void setStrip(Adafruit_NeoPixel* strip, int red, int green, int blue, int white, int gamma) {

  for (int x = 0; x < strip->numPixels(); x++ ) {
    strip->setPixelColor(x, red, green, blue, white);
  }

  strip->show();
}

ros::Subscriber<std_msgs::Int8MultiArray> beltSub("/belt/belt_led_rgbw", &beltCb );

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  pinMode(powerEAPin, OUTPUT);
  digitalWrite(powerEAPin, LOW);

  beltPixels.setBrightness(50);
  beltPixels.begin();
  setStrip(&beltPixels, 0, 0 ,0, 0, 50);

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(voltagePub);
  nh.advertise(currentPub);
  nh.subscribe(beltSub);
}


void loop() {

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

  if(nh.connected()){

  }
  else{
    float i = (((float)(millis()%3500)) / 3500) * M_PI;
    uint8_t x = (sin(i) * 256.0) + 0.0;

    beltPixels.setPixelColor(0, x, x, x, x);

    beltPixels.show();
  }
}

