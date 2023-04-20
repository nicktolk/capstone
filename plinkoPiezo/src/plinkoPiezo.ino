/*
 * Project plinkoPiezo
 * Description: Uses a pair of piezo elements attached to a Plinko board to triangulate and publish position of falling puck
 * Author:  Nick Tolk
 * Date:    10-APR-2023
 */
//#define LIVE
#ifdef LIVE
const bool liveRun = true;
#else
SYSTEM_MODE(SEMI_AUTOMATIC)
const bool liveRun = false;
#endif
SYSTEM_THREAD(ENABLED)


const int TEST_POINTS = 11;
const int TEST_ROWS[] = {0, 1, 2, 2, 3, 3, 4, 5, 6, 7, 7};
const int TEST_COLS[] = {3, 2, 2, 1, 1, 2, 2, 1, 1, 0, 1};
int testIndex = 0;


#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include "neopixel.h"
//#include <SPI.h>
//#include <SdFat.h>
// these are used for image publication to Adafruit dashboard
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include <JsonParserGeneratorRK.h>

#include "credentials.h"

const int PIXEL_PIN = D2;
const int PIXEL_COUNT = 2;
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);


const int PIEZO_PIN_L = A3;
const int PIEZO_PIN_R = A4;
const int EVENT_LED = D4;

const int PEG_ROWS = 8;   // starting from top (0), even rows have 5 pegs and odd rows have 4

// coordinates on Plaza for mapping
const double LAT_TOP = 35.08799050651442;
const double LAT_BOTTOM = 35.08560415806543;
const double LON_RIGHT = -106.65038177579693;
const double LON_LEFT = -106.65203434020583;

const int PIEZO_MIN_O = -100;
const int PIEZO_MAX_O = 100;
const float PIEZO_MIN_I = -1.5;
const float PIEZO_MAX_I = 4.0;

const float PIEZO_THRESH_L =  60.0;
const float PIEZO_THRESH_R =  60.0;
const int PIEZO_TIMEOUT = 60*1000;    // us to consider an impact missed or erroneous

const int NEW_GAME_T = 500;           // ms to condider a game restarted
const int EVENT_T = 200;              // ms to keep event LED lit

const int SERIAL_TIMEOUT = 10*1000;   // ms to wait for serial connection - may be absent
const int PUB_DELAY = 200;            // ms to wait between MQTT publishes
unsigned long lastPub;

const uint16_t SAMPLES = 4;
uint8_t lIn[SAMPLES];
uint8_t rIn[SAMPLES];
system_tick_t tIn[SAMPLES];

unsigned long lastTick = 0;
int elapsed;
int dataIndex = 0;

#ifndef ONBOARD_LED
#define ONBOARD_LED D7
#endif

//void mqtt_setup();
void MQTT_connect();
bool MQTT_ping();
//void createEventPayLoad(int x, int y, int t);
void createEventPayLoad(float x, float y, float tDiff, long tLeft, long tRight, float t);
void createEventPayLoad(float lat, float lon);

TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/latlon");
//Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plinko");
Adafruit_MQTT_Publish pubFeedPos = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plinkoLR");

// current readings from piezo inputs
int piezoL, piezoR;
//long t;
// tLeft and tRight are >= 0 to indicate event time. -1 means nothing currentle registered.
long tLeft, tRight;
// difference between left and right impact times; used for triangulation
int tDiff;
// used to blink onboard LED erradically during Serial startup and indicate events
bool eventOn = false;

system_tick_t tNew;   // tracks last impact to decide if a new game has been started
int lr, lrLast, mag;
double latOut, lonOut;

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

uint8_t r, g, b;

bool playing;     // true when game is in play
int row, col;
void resetLeft();
void resetRight();
void setLights(int lr, int mag);

void setup() {
  Serial.begin(9600);
  // mPub will publish Strings in mqttVector to "/feeds/plinko" and "/feeds/plinkoLR"
  new Thread("mPub", mPub);

  pinMode(PIEZO_PIN_L, INPUT); 
  pinMode(PIEZO_PIN_R, INPUT); 
  pinMode(EVENT_LED, OUTPUT);

  pixel.begin();
  pixel.setBrightness ( 15 );
  pixel.setPixelColor(0, 0x00ffff);
  pixel.setPixelColor(1, 0xff00ff);
  pixel.show();


// wait for serial, and blink onboard LED irregularly
  pinMode(ONBOARD_LED, OUTPUT); 
  if (!liveRun){
    while(!Serial.available() && millis() - lastTick < SERIAL_TIMEOUT){
      digitalWrite(ONBOARD_LED, eventOn);
      digitalWrite(EVENT_LED, eventOn);
      if (random(2)){             // flip on/off chaotically
        eventOn = !eventOn;
      }
      delay(EVENT_T);
      Serial.begin(9600);
    }
  }
  eventOn = false;
  digitalWrite(ONBOARD_LED, eventOn);
  digitalWrite(EVENT_LED, eventOn);
  if (Serial.available()){
    delay(200);
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
  tLeft = -1;
  tRight = -1;

  playing = false;
  row = -1;
}

void loop() {
  if (liveRun){
    MQTT_connect();
    MQTT_ping();
  }

// see if we've timed out to new game
  if (playing && millis() - tNew > NEW_GAME_T){
    playing = false;
    row = -1;
    resetLeft();
    resetRight();
  }

// reset event LED if necessary
  if (eventOn && millis() - tNew > EVENT_T){
    eventOn = false;
    digitalWrite(EVENT_LED, eventOn);
  }

// check piezos
  lIn[dataIndex] = analogRead(PIEZO_PIN_L);
  rIn[dataIndex] = analogRead(PIEZO_PIN_R);
  tIn[dataIndex] = millis();
  if (tLeft < 0){
    lAvg = piezoAvg(lIn);
  }
  if (tRight < 0){
    rAvg = piezoAvg(rIn);
  }
  if (!liveRun){
//    Serial.printf("\r%6.2f, %6.2f, %f", lAvg, rAvg, (micros()-microStart)/1000000.0);
  }

// if we're not playing, the clock hasn't started
  if (!playing && tLeft < 0 && tRight < 0){
    microStart = micros();
    microNow = 0;
  } else {
    microNow = micros() - microStart;
  }

// register times for events if one of our piezos is above the cutoff
  if ((tLeft < 0) && (lAvg > PIEZO_THRESH_L)){
    mLeft = lAvg - PIEZO_THRESH_L;
    tLeft = microNow;
  }
  if ((tRight < 0) && (rAvg > PIEZO_THRESH_R)){
    mRight = rAvg - PIEZO_THRESH_R;
    tRight = microNow;
  }

// check for registered events on both piezos  
  if (tRight >= 0 && tLeft >= 0){
    tNew = millis();              // keep reset game clock current
    if (!playing){
      playing = true;             // if we weren't playing, we are now
      microStart = micros();      // and we'll reset the clock for the last time this game
      lastTick = microStart;
    }
    row++;
    if (!eventOn){
      eventOn = true;
      digitalWrite(EVENT_LED, HIGH);
    }

    elapsed = micros() - lastTick;  // time since last registered event
    lastTick = micros();
    tDiff = tLeft - tRight;         // difference to determine position
    lr = (int)round(map(tDiff/1000.0, PIEZO_MIN_I, PIEZO_MAX_I, (float)PIEZO_MIN_O, (float)PIEZO_MAX_O));
    mag = (int)round(sqrt(mLeft * mLeft + mRight * mRight));
    if (row == 0){
      col = map(lr, PIEZO_MIN_O, PIEZO_MAX_O, 0, 4);
    } else {
      if (row % 2){   // 4 pegs on odd rows
        if (lr < lrLast && col > 0){
          col--;
        }
      } else {        // 5 pegs on even rows
        if (lr > lrLast && col < 4){
          col++;
        }
      }
    }
    col = (col > 4) ? 4 : (col < 0) ? 0 : col;

if (testIndex < TEST_POINTS){
  row = TEST_ROWS[testIndex];
  col = TEST_COLS[testIndex++];
}




    lrLast = lr;
    latOut = map((float)row, 0.0, (float)PEG_ROWS - 1, (float)LAT_TOP, (float)LAT_BOTTOM);
    if (row % 2){   // 4 pegs on odd rows
      lonOut = map((float)col + 0.5, 0.5, 3.5, LON_LEFT, LON_RIGHT);
    } else {        // 5 pegs on even rows
      lonOut = map((float)col, 0.0, 4.0, LON_LEFT, LON_RIGHT);
    }
//    createEventPayLoad(mLeft, mRight, tDiff/1000.0, tLeft, tRight, elapsed / 1000*1000.0);
    createEventPayLoad(latOut, lonOut);
    if (!liveRun){
    Serial.printf("%s\n", mqttVector.front().c_str());
//      Serial.printf("XY: %5d, Z: %5d, tL:%ld     \n", lr, mag, tLeft);
//      Serial.printf("R: %d, C: %d, Lat: %8.4f, Lon: %8.4f\n", row, col, latOut, lonOut);
    }
    setLights(lr, mag);
    // reset impact times and levels
    resetLeft();
    resetRight();
    delay(PIEZO_TIMEOUT / 1000.0);    // pause so we don't double-count
  }

  if (tLeft >= 0 && (microNow - tLeft) > PIEZO_TIMEOUT){
//    Serial.printf("Unmatched event\n");
//    Serial.printf("l:%f, r:%f\n", mLeft, mRight);
    resetLeft();
  }
  if (tRight >= 0 && (microNow - tRight) > PIEZO_TIMEOUT){
//    Serial.printf("Unmatched event\n");
//    Serial.printf("l:%f, r:%f\n", mLeft, mRight);
    resetRight();
  }

  dataIndex = (dataIndex + 1) % SAMPLES;
}

void setLights(int lr, int mag){
  lr = (lr < PIEZO_MIN_O) ? PIEZO_MIN_O : (lr > PIEZO_MAX_O) ? PIEZO_MAX_O : lr;  // bounds check
  r = 3 * mag;
  r = (r > 0xff) ? 0xff : r;
  b = map(lr, PIEZO_MIN_O, PIEZO_MAX_O, 0, 0xff);
  g = map(lr, PIEZO_MIN_O, PIEZO_MAX_O, 0xff, 0);
  pixel.setPixelColor(0, (((r << 8) | g) << 8) | b);
  b = 0xff - b;
  g = 0xff - g;
  pixel.setPixelColor(1, (((r << 8) | g) << 8) | b);

  pixel.show();
}

// returns average of values in *data over SAMPLES elements
float piezoAvg(uint8_t *data){
  sum = 0;
  for (piezoI = 0; piezoI < SAMPLES; piezoI++){
    sum += data[piezoI];
  }
  return(sum / (float)SAMPLES);
}

// sets tLeft and mLeft to 0 and empties lIn[]
void resetLeft(){
  if (!liveRun){
//    Serial.printf("RL\n");
  }
  for (piezoI = 0; piezoI < SAMPLES; piezoI++){
    lIn[piezoI] = 0;
  }
  tLeft = -1;
  mLeft = 0;
}

// sets tRight and mRight to 0 and empties rIn[]
void resetRight(){
  if (!liveRun){
//    Serial.printf("RR\n");
  }
  for (piezoI = 0; piezoI < SAMPLES; piezoI++){
    rIn[piezoI] = 0;
  }
  tRight = -1;
  mRight = 0;
}

String strOut;
int tOut;
// runs detached watching for mqttVector to have at least one member, and publishes while honoring the publication throttle delay
void mPub(){
  system_tick_t lastThreadTime = 0;
  while (true){
    if(mqttVector.size() > 0 && millis()- lastPub > PUB_DELAY){
      lastPub = millis();
      strOut = mqttVector.front();
      mqttVector.erase(mqttVector.begin());
      if (tVector.size() > 0){
        tOut = tVector.front();
        tVector.erase(tVector.begin());
      }
      if (liveRun){
        if(mqtt.Update()) {
          pubFeed.publish(strOut);
//          pubFeedPos.publish(tOut);
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

// crafts JSON packet from arguments, then pushes that onto mqttVector to be handled elsewhere
// also (for now) creates separate packet pushed to tVector for timing debugging
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

void createEventPayLoad(float lat, float lon) {
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);

    jw.insertKeyValue ("lat", lat);
    jw.insertKeyValue ("lon", lon);
  }
  mqttVector.push_back(jw.getBuffer());
}