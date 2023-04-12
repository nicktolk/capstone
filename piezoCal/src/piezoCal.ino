/*
 * Project piezoCal
 * Description:
 * Author:
 * Date:
 */
SYSTEM_MODE(SEMI_AUTOMATIC)


#include <math.h>
#include <iostream>

#include "credentials.h"

const int PIEZO_PIN_L = A2;
const int PIEZO_PIN_R = A3;


const int SERIAL_TIMEOUT = 10*1000;   // ms to wait for serial connection - may be absent

const uint16_t SAMPLES = 1024;
uint8_t lIn[SAMPLES];
uint8_t rIn[SAMPLES];

unsigned lastTick = 0;
int elapsed;
int dataIndex = 0;

#ifndef ONBOARD_LED
#define ONBOARD_LED D7
#endif


int piezoL, piezoR;
bool onboardOn = false;

int piezoI;
float sum;
float mLeft, mRight;
float lAvg, rAvg;
int dMax;
int lMax, rMax;
int highCount;
int lHigh, rHigh;

int piezoMax(uint8_t *data);
int piezoHigh(uint8_t *data, float thresh);

void setup() {
  Serial.begin(9600);

  pinMode(PIEZO_PIN_L, INPUT); 
  pinMode(PIEZO_PIN_R, INPUT); 


// wait for serial, and blink onboard LED irregularly
  pinMode(ONBOARD_LED, OUTPUT); 
  while(!Serial.available() && millis() - lastTick < SERIAL_TIMEOUT){
    digitalWrite(ONBOARD_LED, onboardOn);
    if (random(2)){             // flip on/off chaotically
      onboardOn = !onboardOn;
    }
    delay(200);
    Serial.begin(9600);
  }
  digitalWrite(ONBOARD_LED, LOW);
  if (Serial.available()){
    Serial.println("Serial is up!");
  }
  lastTick = millis();
}

void loop() {

  lIn[dataIndex] = analogRead(PIEZO_PIN_L);
  rIn[dataIndex] = analogRead(PIEZO_PIN_R);
  lAvg = piezoAvg(lIn);
  rAvg = piezoAvg(rIn);
  lHigh = piezoHigh(lIn, lAvg);
  rHigh = piezoHigh(rIn, rAvg);
  lMax = piezoMax(lIn);
  rMax = piezoMax(rIn);
  Serial.printf("\r%6.2f (%d), %6.2f (%d), %d, %d        ", lAvg, lHigh, rAvg, rHigh, lMax, rMax);

  dataIndex = (dataIndex + 1) % SAMPLES;
}

int piezoMax(uint8_t *data){
  dMax = 0;
  for (piezoI = 0; piezoI < SAMPLES; piezoI++){
    if (dMax < data[piezoI]){
      dMax = data[piezoI];
    }
  }
  return(dMax);
}

int piezoHigh(uint8_t *data, float thresh){
  highCount = 0;
  for (piezoI = 0; piezoI < SAMPLES; piezoI++){
    if (data[piezoI] > thresh){
      highCount++;
    }
  }
  return(highCount);
}

float piezoAvg(uint8_t *data){
  sum = 0;
  for (piezoI = 0; piezoI < SAMPLES; piezoI++){
    sum += data[piezoI];
  }
  return(sum / (float)SAMPLES);
}
