#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <housebot_msgs/draw.h>

#include <math.h>
#include "Time/Time.h"
#include "Adafruit_GFX/Adafruit_GFX.h"   // Core graphics library
#include "RGB-matrix-Panel-master/RGBmatrixPanel.h" // Hardware-specific library

// Similar to F(), but for PROGMEM string pointers rather than literals
//#define F2(progmem_ptr) (const __FlashStringHelper *)progmem_ptr

#define CLK 11  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2

RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, true);

bool isIdle=true;

ros::NodeHandle_<ArduinoHardware, 5, 5, 512, 512>  nh;
unsigned char drawCmdTemp[512];
ros::Time currentDrawStart;
housebot_msgs::drawRequest currentDrawCmd;


bool doDrawCommand(ros::Time t, housebot_msgs::DrawCommand& layer){
//  ros::Time layerStart = ros::Time(t.sec, t.nsec) += layer.delay;
//  ros::Time layerEnd = (ros::Time(t.sec, t.nsec) += layer.delay) += layer.duration;

  // Load color info
  uint16_t color = 0;
  if(layer.color_space == housebot_msgs::DrawCommand::COLOR_888 && layer.color_length == 3){
    color = matrix.Color888(layer.color[0], layer.color[1], layer.color[2], true);
  }
  else if(layer.color_space == housebot_msgs::DrawCommand::COLOR_HSV && layer.color_length == 3){
    color = matrix.ColorHSV(layer.color[0], layer.color[1], layer.color[2], true);
  }
  else{
    color = matrix.Color888(layer.color[0], layer.color[1], layer.color[2], true);
  }
  
  // Parse shape parameters
  if(layer.shape == housebot_msgs::DrawCommand::SHAPE_SCREEN){
    matrix.fillScreen(color);
  }
  else if(layer.shape == housebot_msgs::DrawCommand::SHAPE_PIXEL && layer.shape_data_length == 2){
    matrix.drawPixel(layer.shape_data[0], layer.shape_data[1], color);
  }
  else if(layer.shape == housebot_msgs::DrawCommand::SHAPE_LINE && layer.shape_data_length == 4){
    matrix.drawLine(layer.shape_data[0], layer.shape_data[1],
                    layer.shape_data[2], layer.shape_data[3], color);
  }
  else if(layer.shape == housebot_msgs::DrawCommand::SHAPE_V_LINE && layer.shape_data_length == 3){
    matrix.drawFastVLine(layer.shape_data[0], layer.shape_data[1],
                    layer.shape_data[2], color);
  }
  else if(layer.shape == housebot_msgs::DrawCommand::SHAPE_H_LINE && layer.shape_data_length == 3){
    matrix.drawFastHLine(layer.shape_data[0], layer.shape_data[1],
                    layer.shape_data[2], color);
  }
  else if(layer.shape == housebot_msgs::DrawCommand::SHAPE_RECT && layer.shape_data_length == 4){
    if(layer.fill){
      matrix.fillRect(layer.shape_data[0], layer.shape_data[1],
                    layer.shape_data[2], layer.shape_data[3], color);
    }
    else{
      matrix.drawRect(layer.shape_data[0], layer.shape_data[1],
                    layer.shape_data[2], layer.shape_data[3], color);
    }
  }
  else if(layer.shape == housebot_msgs::DrawCommand::SHAPE_CIRCLE && layer.shape_data_length == 3){
    if(layer.fill){
      matrix.fillCircle(layer.shape_data[0], layer.shape_data[1],
                    layer.shape_data[2], color);
    }
    else{
      matrix.drawCircle(layer.shape_data[0], layer.shape_data[1],
                    layer.shape_data[2], color);
    }
  }
  else if(layer.shape == housebot_msgs::DrawCommand::SHAPE_TRIANGLE && layer.shape_data_length == 3){
    if(layer.fill){
      matrix.fillTriangle(layer.shape_data[0], layer.shape_data[1],
                          layer.shape_data[2], layer.shape_data[3],
                          layer.shape_data[4], layer.shape_data[5], color);
    }
    else{
      matrix.drawTriangle(layer.shape_data[0], layer.shape_data[1],
                          layer.shape_data[2], layer.shape_data[3],
                          layer.shape_data[4], layer.shape_data[5], color);
    }
  }
  else{
    matrix.fillScreen(color);
  }

  if(layer.swap_buffers){
    matrix.swapBuffers(false);
    //isIdle=true;
  }

  return true;
}


void drawCb(const housebot_msgs::drawRequest& req, housebot_msgs::drawResponse& res){
  req.serialize(drawCmdTemp);
  currentDrawCmd.deserialize(drawCmdTemp);
  currentDrawStart = nh.now();
  
  bool activeLayer = false;
  for(int i=0; i<currentDrawCmd.layers_length; i++){
    activeLayer |= doDrawCommand(currentDrawStart, currentDrawCmd.layers[i]);
  }
  
  res.success = true;
  isIdle=false;
}


ros::ServiceServer<housebot_msgs::drawRequest, housebot_msgs::drawResponse> drawSrv("~/head/draw", drawCb);

void setup() {
  matrix.begin();
  matrix.setTextWrap(false); // Allow text to run off right edge
  matrix.setTextSize(1);
  matrix.fillScreen(0);  
  matrix.swapBuffers(false);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertiseService(drawSrv);
}

void loop() {
  nh.spinOnce();
  
  if(isIdle){
    ros::Time t = nh.now();
    int h = hour(t.sec - (7 * 3600));
    int m = minute(t.sec - (7 * 3600));
    int s = second(t.sec - (7 * 3600));
    
    matrix.fillScreen(0); 
    

    if(nh.connected()){
	    int i=0;
      matrix.setTextColor(matrix.Color888(255, 255, 255, true));
      matrix.setCursor(1, 4);
      if(h < 10){ matrix.print(String(0, DEC)); }
      matrix.print(String(String(h, DEC)+":"));
      if(m < 10){ matrix.print(String(0, DEC)); }
      matrix.print(String(m, DEC));
      i = map(s, 0, 60, 0, 16);
			
			matrix.drawFastHLine(16, 0, i, matrix.ColorHSV(0, 0, 80, true));
			matrix.drawFastHLine(16-i, 0, i, matrix.ColorHSV(0, 0, 80, true));
			matrix.drawFastHLine(16, 15, i, matrix.ColorHSV(0, 0, 80, true));
			matrix.drawFastHLine(16-i, 15, i, matrix.ColorHSV(0, 0, 80, true));
    }
    else{
      float i = (((float)(millis()%3500)) / 3500) * M_PI;
			int x = (sin(i) * 127.0) + 127.0;
      matrix.drawPixel(31,15, matrix.Color888(x,x,x,true));
    }

  
    matrix.swapBuffers(false);
  }
  else{
    ros::Time t = nh.now();
    bool activeLayer = false;
    for(int i=0; i<currentDrawCmd.layers_length; i++){
      activeLayer |= doDrawCommand(t, currentDrawCmd.layers[i]);
    }
    
    if(!activeLayer){
      isIdle = true;
    }
  }
}
