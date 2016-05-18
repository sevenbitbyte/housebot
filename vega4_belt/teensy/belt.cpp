// Internal libraries
#include "Arduino.h"

//Externel libraries
#include <Adafruit_NeoPixel.h>

//ROS libraries
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>

//Power
const int ledPin = 13;
const int vMonPin = 14;
const int iMonPin = 15;
const int powerEAPin = 2;

#define V_MON_COEF (16.35f / 658)

#define CONNECTED_LED_BLINK_HZ 10
#define BATTERY_PUB_RATE_HZ 5

//Pump
#define PUMP_TIMOUT_MS 5000

//Limit switch
#define LIMIT_SWITCH_PIN 6
#define LIMIT_SWITCH_PUB_RATE_HZ 1

//NeoPixel
Adafruit_NeoPixel beltPixels = Adafruit_NeoPixel(23, 20, NEO_RGBW + NEO_KHZ800);

//ROS
ros::NodeHandle_<ArduinoHardware, 25, 25, 4096, 4096> nh;

std_msgs::Bool lowerLimitMsg;
std_msgs::Float32 voltageMsg;
std_msgs::Float32 currentMsg;

ros::Publisher lowerLimitPub("/gantry/lower_limit", &lowerLimitMsg);
ros::Publisher voltagePub("voltage", &voltageMsg);
ros::Publisher currentPub("current", &currentMsg);

// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
  Serial3.write(0x83);
}

// speed should be a number from -3200 to 3200
void setMotorSpeed(int speed)
{
  if (speed < 0)
  {
    Serial3.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    Serial3.write(0x85);  // motor forward command
  }
  Serial3.write(speed & 0x1F);
  Serial3.write(speed >> 5);
}

int calculate_breath() {
  float i = (((float)(millis()%3500)) / 3500) * M_PI;
  return (sin(i) * 256.0) + 0.0;
}

void setStripMultiArray(Adafruit_NeoPixel& strip, const std_msgs::UInt8MultiArray& pixels){
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

void beltCb(const std_msgs::UInt8MultiArray& pixels){
  setStripMultiArray(beltPixels, pixels);
}

long lastPumpCbMS = 0;
void pumpCb(const std_msgs::Int16& msg) {

  setMotorSpeed(msg.data);
  lastPumpCbMS = millis();

}

void setStrip(Adafruit_NeoPixel* strip, int red, int green, int blue, int white, int gamma) {

  for (int x = 0; x < strip->numPixels(); x++ ) {
    strip->setPixelColor(x, red, green, blue, white);
  }

  strip->show();
}

ros::Subscriber<std_msgs::UInt8MultiArray> beltSub("/belt/belt_led_rgbw", &beltCb );
ros::Subscriber<std_msgs::Int16> pumpSub("/belt/pump", &pumpCb );

void setup() {

  // Turn on Connect led
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Turn on power supply
  pinMode(powerEAPin, OUTPUT);
  digitalWrite(powerEAPin, LOW);

  pinMode(LIMIT_SWITCH_PIN, OUTPUT);

  // Pump setup
  Serial3.begin(19200);
  delay(5);  //Motor Controller requires 5ms startup head time
  Serial3.write(0xAA);
  exitSafeStart();
  setMotorSpeed(0);

  // Setup Adafruit Neopixel
  beltPixels.setBrightness(50);
  beltPixels.begin();
  setStrip(&beltPixels, 0, 0 ,0, 0, 50);

  // ROS_serial initiation
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(voltagePub);
  nh.advertise(currentPub);
  nh.advertise(lowerLimitPub);
  nh.subscribe(beltSub);
  nh.subscribe(pumpSub);
}

//Super loop and timout vars, TODO replace with ROS::time
unsigned long lastEndstopPubMS = 0;
unsigned long lastBlinkMS = 0;
unsigned long lastBatteryPubMS = 0;
bool blinkDelta = false;
bool endstopDelta = false;
unsigned long _millis = 0;

void loop() {

  // Edge case for rolling of the millis register if left on too long
  if(millis() < _millis) {
     _millis = millis();
     lastEndstopPubMS = 0;
     lastBlinkMS = 0;
     lastBatteryPubMS = 0;
     lastPumpCbMS = 0;
     return;
  }
  _millis = millis();

  // Battery monitoring
  voltageMsg.data = analogRead(vMonPin) * V_MON_COEF;
  currentMsg.data = analogRead(iMonPin);

  if(voltageMsg.data < 12.0){
    digitalWrite(powerEAPin, LOW);
  }
  else{
    digitalWrite(powerEAPin, HIGH);
  }

  // Pump defaults to off after timeout
  if( _millis - lastPumpCbMS > PUMP_TIMOUT_MS ) {
    setMotorSpeed(0);
  }

  if(nh.connected()) {

    digitalWrite(ledPin, HIGH);

    // Publish battery msgs at rate
    if((_millis - lastBatteryPubMS) > (1000 / BATTERY_PUB_RATE_HZ) ) {
      voltagePub.publish(&voltageMsg);
      currentPub.publish(&currentMsg);
      lastBatteryPubMS = _millis;
    }

    // Publish endstop if delta or if rate triggers
    bool lowerLimitPinState = digitalRead(LIMIT_SWITCH_PIN);
    if((endstopDelta != lowerLimitPinState) ||
      (_millis - lastEndstopPubMS) > (1000 / LIMIT_SWITCH_PUB_RATE_HZ) ) {
      lowerLimitMsg.data = lowerLimitPinState;
      lowerLimitPub.publish(&lowerLimitMsg);
      endstopDelta = lowerLimitPinState;
      lastEndstopPubMS = _millis;
      }
  } else { // LED breath if n/c

    setMotorSpeed(0); // Turn off pump if n/c

    // Blink onboard LED if not connected
    if( _millis - lastBlinkMS > 1000 / CONNECTED_LED_BLINK_HZ ) {
        digitalWrite(ledPin, blinkDelta);
        blinkDelta = !(blinkDelta);
        lastBlinkMS = _millis;
    }

    int breath = calculate_breath();
    beltPixels.setPixelColor(0, 0, breath, 0, 0 );
    beltPixels.show();
  }

  nh.spinOnce();
}
