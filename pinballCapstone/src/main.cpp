#include <Arduino.h>


//UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
#define FRAMESIZE FRAMESIZE_CIF
const int IMG_W = 400;
const int IMG_H = 296;
const int ROI_X0 = 50;
const int ROI_W = 90;
const int ROI_X1 = ROI_X0 + ROI_W;
const int ROI_Y = 296 - 235;
const int ROI_H = 50;

uint8_t lastFrame[IMG_H * IMG_W];

const float AVG_W = 10.0;  // number of frames for rolling average (integer value cast as float)
#include "configCamera.h"

const int PIN_L = 20;
const int PIN_R = 21;
const int UPTIME = 500;     // ms to activate flipper
const int DOWNTIME = 1000;  // ms of mandatory solenoid relaxation
const float THRESH = 1.0;

unsigned long leftFlipT, rightFlipT; // set to millis() time to set LOW
bool leftUp = false, rightUp = false;       // flipper state

int imgX, imgY, imgI;     // for indexing through frames
float sumDiff, thisMean;  // for statistics calculation
float lDif, rDif;         // running difference averages
float lBase, rBase;       
unsigned long lastTick;   // timing
camera_fb_t *fb;

const int DELAY_INIT = 10000;

int count;

float checkROI(int roiX, int roiW, int roiY, int roiH, camera_fb_t *fb);

void setup() {
  Serial.begin(9600);
  pinMode(PIN_L, OUTPUT);
  pinMode(PIN_R, OUTPUT);

  configCamera();

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE);

// let things normalize before enabling flippers
  leftFlipT = millis() + DELAY_INIT;
  rightFlipT = millis() + DELAY_INIT;
  count = 0;
}

void loop() {
  lastTick = millis();

// lower flippers if appropriate
  if (leftUp && lastTick > leftFlipT){
    leftUp = false;
    digitalWrite(PIN_L, LOW);
  }
  if (rightUp && lastTick > rightFlipT){
    rightUp = false;
    digitalWrite(PIN_R, LOW);
  }

  fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }

  lDif = checkROI(ROI_X0, ROI_W, ROI_Y, ROI_H, fb);
  lBase = (lBase * (AVG_W - 1)  + lDif) / AVG_W;
  rDif = checkROI(ROI_X0 + ROI_W, ROI_W, ROI_Y, ROI_H, fb);
  rBase = (rBase * (AVG_W - 1)  + rDif) / AVG_W;
  if (lastTick > leftFlipT + DOWNTIME){
    if (10*(lDif / lBase - 1) > THRESH){
      leftUp = true;
      leftFlipT = lastTick + UPTIME;
      digitalWrite(PIN_L, HIGH);    
    }
  }
  if (lastTick > rightFlipT + DOWNTIME){
    if (10*(rDif / rBase - 1) > THRESH){
      rightUp = true;
      rightFlipT = lastTick + UPTIME;
      digitalWrite(PIN_R, HIGH);
    }
  }
  if (leftFlipT > lastTick){
    Serial.printf("\rL%4d       ", leftFlipT - lastTick);
  } else {
    Serial.printf("\r            ");
  }
  if (rightFlipT > lastTick){
    Serial.printf("R%4d       ", rightFlipT - lastTick);
  } else {
    Serial.printf("            ");
  }
  Serial.printf(" %6.3f / %6.3f (%d ms)    ", 10*(lDif / lBase - 1), 10*(rDif / rBase - 1), millis() - lastTick);

  std::copy(fb->buf, fb->buf + IMG_W * IMG_H, lastFrame);

  esp_camera_fb_return(fb);
}

// returns average pixel deviation between current frame and last
float checkROI(int roiX, int roiW, int roiY, int roiH, camera_fb_t *fb){
  sumDiff = 0;
  for (imgY = roiY; imgY < roiY + roiH; imgY++){
    for (imgX = roiX; imgX < roiX + roiW; imgX++){
      imgI = imgY * IMG_W + imgX;
      sumDiff += abs((int)fb->buf[imgI] - (int)lastFrame[imgI]);
    }
  }
  return(sumDiff / (float)(roiW * roiH));
}
