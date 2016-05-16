#include <math.h>
#include <SmartMatrix3.h>
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8MultiArray.h>



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

Adafruit_NeoPixel leftEarPixels = Adafruit_NeoPixel(24, 11, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel rightEarPixels = Adafruit_NeoPixel(24, 12, NEO_RGB + NEO_KHZ800);

ros::NodeHandle_<ArduinoHardware, 5, 5, 4096, 4096>  nh;

//sensor_msgs::Image image_msg;
//ros::Publisher img_echo("/head/echo", &image_msg);

void faceCb( const sensor_msgs::Image& image){

  backgroundLayer.fillScreen(defaultBackgroundColor);

  /*image_msg.height = image.height;
  image_msg.width = image.width;

  image_msg.data_length = image.data_length;
  image_msg.st_data = image.st_data;

  image_msg.data = (uint8_t*)realloc(image_msg.data, image_msg.data_length * sizeof(uint8_t));

  memcpy(image_msg.data, image.data, image_msg.data_length);

  image_msg.encoding = image.encoding;
  image_msg.is_bigendian = image.is_bigendian;
  image_msg.step = image.step;


  img_echo.publish(&image_msg);*/

  for(int x=0; x<32; x++){
    for(int y=0; y<16; y++){

        int idx = (y * image.step) + (x * 3);
        rgb24 pixel = {(uint8_t)image.data[idx], (uint8_t)image.data[idx+1], (uint8_t)image.data[idx+2]};
        backgroundLayer.drawPixel(31-x,15-y, pixel);
    }
  }

  backgroundLayer.swapBuffers();
}


void setStripMultiArray(Adafruit_NeoPixel& strip, const std_msgs::Int8MultiArray& pixels){
  for(int i=0; i<strip.numPixels() && i<pixels.data_length; i++){
    int idx = i*3;	//NOTE: We expect RGB data only

    byte red = pixels.data[idx];
    byte green = pixels.data[idx + 1];
    byte blue = pixels.data[idx + 2];

    strip.setPixelColor(i, strip.Color( red, green, blue ));
  }

  strip.show();
}

void leftEarCb(const std_msgs::Int8MultiArray& pixels){
  setStripMultiArray(leftEarPixels, pixels);
}

void rightEarCb(const std_msgs::Int8MultiArray& pixels){
  setStripMultiArray(rightEarPixels, pixels);
}


void setStrip(Adafruit_NeoPixel* strip, int red, int green, int blue, int gamma) {

  for (int x = 0; x < strip->numPixels(); x++ ) {
    strip->setPixelColor(x, strip->Color(red, green, blue, gamma));
  }

  strip->show();
}

ros::Subscriber<sensor_msgs::Image> faceSub("/head/image", &faceCb );
ros::Subscriber<std_msgs::Int8MultiArray> leftEarSub("/head/left_led_rgb", &leftEarCb );
ros::Subscriber<std_msgs::Int8MultiArray> rightEarSub("/head/right_led_rgb", &rightEarCb );

void setup() {

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  leftEarPixels.setBrightness(50);
  leftEarPixels.begin();
  setStrip(&leftEarPixels, 0, 0 ,0 ,50);

  rightEarPixels.setBrightness(50);
  rightEarPixels.begin();
  setStrip(&rightEarPixels, 0, 0, 0, 50);

  matrix.addLayer(&backgroundLayer);
  matrix.begin();

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(faceSub);
  nh.subscribe(leftEarSub);
  nh.subscribe(rightEarSub);
  //nh.advertise(img_echo);
}

void loop() {
  nh.spinOnce();


  if(nh.connected()){

  }
  else{

    backgroundLayer.fillScreen(defaultBackgroundColor);

    float i = (((float)(millis()%3500)) / 3500) * M_PI;
    uint8_t x = (sin(i) * 256.0) + 0.0;
    rgb24 pixelColor = {x, x, x};
    backgroundLayer.drawPixel(31,0, pixelColor);
    backgroundLayer.swapBuffers();

    leftEarPixels.setPixelColor(0, leftEarPixels.Color(x, x, x));
    rightEarPixels.setPixelColor(0, rightEarPixels.Color(x, x, x));

    leftEarPixels.show();
    rightEarPixels.show();
  }
}
