/*
 * Project plinkoPiezo
 * Description:
 * Author:
 * Date:
 */
//#define LIVE 1
#ifdef LIVE
const bool liveRun = true;
#else
SYSTEM_MODE(SEMI_AUTOMATIC)
const bool liveRun = false;
#endif
SYSTEM_THREAD(ENABLED)

#include <math.h>
#include <iostream>
#include <vector>
#include <string>
//#include <SPI.h>
//#include <SdFat.h>
// these are used for image publication to Adafruit dashboard
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include <JsonParserGeneratorRK.h>

#include "credentials.h"

const int PIEZO_PIN_L = A2;
const int PIEZO_PIN_R = A3;

const int PIEZO_MAX_O = -100;
const int PIEZO_MIN_O = 100;
const float PIEZO_MAX_I = 4.0;
const float PIEZO_MIN_I = -1.5;
const float PIEZO_THRESH_L =  90.0;
const float PIEZO_THRESH_R =  90.0;
const int PIEZO_TIMEOUT = 500*1000;    // us to consider an impact missed or erroneous

const int SERIAL_TIMEOUT = 10*1000;   // ms to wait for serial connection - may be absent
const int PUB_DELAY = 1*1000;

const uint16_t SAMPLES = 4;
uint8_t lIn[SAMPLES];
uint8_t rIn[SAMPLES];
system_tick_t tIn[SAMPLES];

unsigned lastTick = 0;
int elapsed;
int dataIndex = 0;

#ifndef ONBOARD_LED
#define ONBOARD_LED D7
#endif

void mqtt_setup();
void MQTT_connect();
bool MQTT_ping();
//void createEventPayLoad(int x, int y, int t);
void createEventPayLoad(float x, float y, float tDiff, long tLeft, long tRight, float t);

TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plinko");
Adafruit_MQTT_Publish pubFeedPos = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plinkoLR");

int piezoL, piezoR;
long t;
long tLeft, tRight;
int tDiff;
bool onboardOn = false;

int milliStart;
unsigned long microStart;
long microNow;
int cnt = 0;
char strRand[64];
std::vector<String> mqttVector;
std::vector<int> tVector;

int piezoI;
float sum;
float mLeft, mRight;
float lAvg, rAvg;

void setup() {
  Serial.begin(9600);
  // mPub will publish Strings in mqttVector to ''
  new Thread("mPub", mPub);

  pinMode(PIEZO_PIN_L, INPUT); 
  pinMode(PIEZO_PIN_R, INPUT); 


// wait for serial, and blink onboard LED irregularly
  pinMode(ONBOARD_LED, OUTPUT); 
  if (!liveRun){
    while(!Serial.available() && millis() - lastTick < SERIAL_TIMEOUT){
      digitalWrite(ONBOARD_LED, onboardOn);
      if (random(2)){             // flip on/off chaotically
        onboardOn = !onboardOn;
      }
      delay(200);
      Serial.begin(9600);
    }
  }
  digitalWrite(ONBOARD_LED, LOW);
  if (Serial.available()){
//    Serial.setDebugOutput(true);
    Serial.println("Serial is up!");
  }
  if (liveRun){
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
      Serial.printf(".");
      delay(100);
    }
  }
  Serial.printf("\n\n");
  lastTick = millis();
  microStart = micros();
  tLeft = 0;
  tRight = 0;
}

void loop() {
  if (liveRun){
    MQTT_connect();
    MQTT_ping();
  }

  lIn[dataIndex] = analogRead(PIEZO_PIN_L);
  rIn[dataIndex] = analogRead(PIEZO_PIN_R);
  tIn[dataIndex] = millis();
  if (tLeft == 0){
    lAvg = piezoAvg(lIn);
  }
  if (tRight == 0){
    rAvg = piezoAvg(rIn);
  }
  if (!liveRun){
//    Serial.printf("\r%6.2f, %6.2f, %f", lAvg, rAvg, (micros()-microStart)/1000000.0);
  }

  microNow = micros() - microStart;
  if ((tLeft == 0) && (lAvg > PIEZO_THRESH_L)){
    mLeft = lAvg;
    tLeft = microNow;
  }
  if ((tRight == 0) && (rAvg > PIEZO_THRESH_R)){
    mRight = rAvg;
    tRight = microNow;
  }
  if (tRight != 0 && tLeft != 0){
    if (millis() - lastTick > PUB_DELAY){
      elapsed = millis() - lastTick;
      lastTick = millis();
//      sprintf(strRand, "%d", random(256));
      tDiff = tLeft - tRight;
      createEventPayLoad(mLeft, mRight, tDiff/1000.0, tLeft, tRight, elapsed / 1000.0);
      tLeft = 0;
      tRight = 0;
      for (piezoI = 0; piezoI < SAMPLES; piezoI++){
        lIn[piezoI] = 0;
        rIn[piezoI] = 0;
      }
      Serial.printf("\nSz:%d, Ele:%s\n", mqttVector.size(), mqttVector.front().c_str());
      Serial.printf("XY: %d, Z:%f\n", 
        (int)round(map(tDiff/1000.0, PIEZO_MIN_I, PIEZO_MAX_I, (float)PIEZO_MIN_O, (float)PIEZO_MAX_O)),
        sqrt(mLeft * mLeft + mRight * mRight - PIEZO_THRESH_L * PIEZO_THRESH_L - PIEZO_THRESH_R * PIEZO_THRESH_R));
    }
    delay(PIEZO_TIMEOUT / 1000.0);
  }

  if ((tLeft != 0 && (microNow - tLeft) > PIEZO_TIMEOUT) || 
    (tRight != 0 && (microNow - tRight) > PIEZO_TIMEOUT)){
    Serial.printf("Unmatched event\n");
    Serial.printf("l:%f, r:%f\n", mLeft, mRight);
    for (piezoI = 0; piezoI < SAMPLES; piezoI++){
      lIn[piezoI] = 0;
      rIn[piezoI] = 0;
    }
    tLeft = 0;
    tRight = 0;
    mLeft = 0;
    mRight = 0;
  }

  dataIndex = (dataIndex + 1) % SAMPLES;
  if (tLeft == 0 && tRight == 0){
//    microStart = micros();
  }
}

float piezoAvg(uint8_t *data){
  sum = 0;
  for (piezoI = 0; piezoI < SAMPLES; piezoI++){
    sum += data[piezoI];
  }
  return(sum / (float)SAMPLES);
}

String strOut;
int tOut;
// runs detached watching for stepsOut to be anything but 0, then turns motor and resets stepsOut
void mPub(){
  system_tick_t lastThreadTime = 0;
  while (true){
    if(mqttVector.size() > 0){
      strOut = mqttVector.front();
      mqttVector.erase(mqttVector.begin());
      tOut = tVector.front();
      tVector.erase(tVector.begin());
      if (liveRun){
        if(mqtt.Update()) {
          pubFeed.publish(strOut);
          pubFeedPos.publish(tOut);
        }
      } 
    }
    os_thread_delay_until(&lastThreadTime, 10);
  }
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds...\n");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus = 0;

  if ((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    pingStatus = mqtt.ping();
    if(!pingStatus) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }
  return pingStatus;
}

void createEventPayLoad(float x, float y, float tDiff, long tLeft, long tRight, float t) {
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);

    jw.insertKeyValue ("l", x);
    jw.insertKeyValue ("r", y);
    jw.insertKeyValue ("tDiff", tDiff);
    jw.insertKeyValue ("tL", tLeft);
    jw.insertKeyValue ("tR", tRight);
    jw.insertKeyValue ("t", t);
  }
  tVector.push_back(round(map(tDiff, PIEZO_MIN_I, PIEZO_MAX_I, (float)PIEZO_MIN_O, (float)PIEZO_MAX_O)));
  mqttVector.push_back(jw.getBuffer());
}