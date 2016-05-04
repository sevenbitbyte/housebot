#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_NeoPixel.h>



#define VEGA_NUM_LEDS 47
#define MAX_LED_FRAMES 124
#define PIXEL_RING_PIN 4
#define BYTES_PER_PIXEL 4
#define BYTES_PER_PIXEL_STRING "BYTES_PER_PIXEL"
#define TODO_MSG_FRAMES 1

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(VEGA_NUM_LEDS, PIXEL_RING_PIN, NEO_RGBW + NEO_KHZ800);
ros::NodeHandle_<ArduinoHardware, 1, 1, 30720, 512>  nh;

int autoScale( int originalMin, int originalMax, int newBegin, int
newEnd, int inputValue){

  long zeroRefOriginalMax = 0;
  long zeroRefnewEnd = 0;
  long zeroRefCurVal = 0;
  long rangedValue = 0;
  boolean invFlag = 0;

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  zeroRefOriginalMax = originalMax - originalMin;

  if (newEnd > newBegin){ 
    zeroRefnewEnd = newEnd - newBegin;
  }
  else
  {
    zeroRefnewEnd = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;


 // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

   if (invFlag == 0){
    rangedValue =  ((zeroRefCurVal * zeroRefnewEnd) /
      zeroRefOriginalMax) + newBegin ; 
  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - ((zeroRefCurVal * zeroRefnewEnd) /
      zeroRefOriginalMax)  ; 
  }

  return rangedValue;
}

void cmd_vel( const geometry_msgs::Twist& msg) {

  int x = autoScale(-1000, 1000, -254, 255, msg.linear.x * 1000.0);



    for(int y = 0; y < 47; y++) {
      if( x > 0 ) {
        pixel.setPixelColor(y, 0, 0, x);
      } else {
        pixel.setPixelColor(y, 0, x, 0);
      }
    
  }

  pixel.show();

}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmd_vel );



void setStrip(Adafruit_NeoPixel* pix, int color) {

  for (int x = 0; VEGA_NUM_LEDS > x; x++ ) {

    pix->setPixelColor(x, color, color, color, color);

  }

}

int calculate_breath() {

  return  (int)((exp( sin( millis() / 2000.0 * PI )) - 0.36787) * 108.0);
}

void setup() {

    pixel.begin();
    setStrip(&pixel, 0);
    pixel.show();

    nh.initNode();
    nh.subscribe(sub);

    while (!nh.connected()) {
    int breath = calculate_breath();
    pixel.setPixelColor(0, breath, 0, 0, 0 );
    pixel.show();
    nh.spinOnce();
  }
}

void loop() {

  nh.spinOnce();
  delay(50);
  

}
