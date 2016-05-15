#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8MultiArray.h>
#include <Adafruit_NeoPixel.h>

#include <math.h>

#include <SmartMatrix3.h>
//#include "Adafruit_GFX/Adafruit_GFX.h"   // Core graphics library
//#include "RGB-matrix-Panel-master/RGBmatrixPanel.h" // Hardware-specific library

// Similar to F(), but for PROGMEM string pointers rather than literals
//#define F2(progmem_ptr) (const __FlashStringHelper *)progmem_ptr

#define VEGA_NUM_LEDS 24
#define COLOR_DEPTH 24                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
const uint8_t kMatrixWidth = 32;        // known working: 32, 64, 96, 128
const uint8_t kMatrixHeight = 16;       // known working: 16, 32, 48, 64
const uint8_t kRefreshDepth = 36;       // known working: 24, 36, 48
const uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save memory, more to keep from dropping frames and automatically lowering refresh rate
const uint8_t kPanelType = SMARTMATRIX_HUB75_16ROW_MOD8SCAN;   // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels
const uint8_t kMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);      // see http://docs.pixelmatix.com/SmartMatrix for options
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);
const uint8_t kScrollingLayerOptions = (SM_SCROLLING_OPTIONS_NONE);
const uint8_t kIndexedLayerOptions = (SM_INDEXED_OPTIONS_NONE);
SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);
SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);
SMARTMATRIX_ALLOCATE_INDEXED_LAYER(indexedLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kIndexedLayerOptions);
const rgb24 defaultBackgroundColor = {0, 0, 0};

#define CLK 11  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2

Adafruit_NeoPixel pixelr = Adafruit_NeoPixel(24, 11, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel pixell = Adafruit_NeoPixel(24, 12, NEO_RGB + NEO_KHZ800);
//RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, true);

bool isIdle=true;

ros::NodeHandle_<ArduinoHardware, 5, 5, 4096, 4096>  nh;

sensor_msgs::Image image_msg;
ros::Publisher img_echo("/head/echo", &image_msg);

void faceCb( const sensor_msgs::Image& image){

  backgroundLayer.fillScreen(defaultBackgroundColor);
  //backgroundLayer.swapBuffers();

  image_msg.height = image.height;
  image_msg.width = image.width;

  image_msg.data_length = image.data_length;
  image_msg.st_data = image.st_data;

  image_msg.data = (uint8_t*)realloc(image_msg.data, image_msg.data_length * sizeof(uint8_t));

  memcpy(image_msg.data, image.data, image_msg.data_length);
  

  //image_msg.data = image.data;
  image_msg.encoding = image.encoding;
  image_msg.is_bigendian = image.is_bigendian;
  image_msg.step = image.step;


  img_echo.publish(&image_msg);

  for(int x=0; x<32 /*|| x<image.width*/; x++){
    for(int y=0; y<16 /*|| y<image.height*/; y++){

        int idx = (y * image.step) + (x * 3);
	//int idx = (x * 3);
        rgb24 pixel = {(uint8_t)image.data[idx], (uint8_t)image.data[idx+1], (uint8_t)image.data[idx+2]};
	
        //rgb24 pixel = {(uint8_t)image.data[idx], (uint8_t)image.data[idx+1], (uint8_t)image.data[idx+2]};
        backgroundLayer.drawPixel(31-x,15-y, pixel);
    }
  }

  backgroundLayer.swapBuffers();
}

void leftEarCb( const std_msgs::Int8MultiArray& pixels){
  //
}

void rightEarCb( const std_msgs::Int8MultiArray& pixels){
  //
}

void setStrip(Adafruit_NeoPixel* pix, int color) {

  for (int x = 0; VEGA_NUM_LEDS > x; x++ ) {
    pix->setPixelColor(x, color, color, color, color);
  }
}

ros::Subscriber<sensor_msgs::Image> faceSub("/head/image", &faceCb );
ros::Subscriber<std_msgs::Int8MultiArray> leftEarSub("/head/left_led_rgb", &leftEarCb );
ros::Subscriber<std_msgs::Int8MultiArray> rightEarSub("/head/right_led_rgb", &rightEarCb );

void setup() {
  pixelr.begin();
  setStrip(&pixelr, 0);
  pixelr.show();
  pixell.begin();
  setStrip(&pixell, 0);
  pixell.show();
  matrix.addLayer(&backgroundLayer);
  matrix.begin();

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(faceSub);
  nh.subscribe(leftEarSub);
  nh.subscribe(rightEarSub);
  nh.advertise(img_echo);
}

void loop() {
             
  nh.spinOnce();

  if(true){

    if(nh.connected()){

	// No op

    }
    else{
      //backgroundLayer.swapBuffers();

      backgroundLayer.fillScreen(defaultBackgroundColor);

      float i = (((float)(millis()%3500)) / 3500) * M_PI;
      uint8_t x = (sin(i) * 256.0) + 0.0;
      rgb24 pixelColor = {x, x, x};
      backgroundLayer.drawPixel(31,0, pixelColor);
      backgroundLayer.swapBuffers();
    }

  }

}
