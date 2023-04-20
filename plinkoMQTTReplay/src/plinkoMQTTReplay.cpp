/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/nick/Documents/IoT/capstone/plinkoMQTTReplay/src/plinkoMQTTReplay.ino"
/*
 * Project plinkoMQTTReplay
 * Description: Reads previously exported JSON packets then publishes them to Adafruit feed
 * Author:      Nick Tolk
 * Date:        14-APR-2023
 */

void setup();
void loop();
#line 8 "c:/Users/nick/Documents/IoT/capstone/plinkoMQTTReplay/src/plinkoMQTTReplay.ino"
#ifndef ONBOARD_LED
#define ONBOARD_LED D7
#endif

#include "credentials.h"
#include "messages.h"

#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"

const int SERIAL_TIMEOUT = 10*1000;   // ms to wait for serial connection
const int PUB_DELAY = 1000;           // delay between MQTT pushes

system_tick_t lastTick;

TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plinkoLatLon");

void MQTT_connect();
bool MQTT_ping();

bool startSerial();

void setup() {

  int messageI;       // index into messages array

  pinMode(ONBOARD_LED, OUTPUT); 
  startSerial();

  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
    delay(100);
  }
  Serial.printf("\n\n");

  for (messageI = 0; messageI < NUM_MESSAGES; messageI++){
    while((millis() - lastTick < PUB_DELAY)){}  // wait
    MQTT_connect();
    MQTT_ping();
    if(mqtt.Update()) {
      pubFeed.publish(messages[messageI]);
    }

    lastTick = millis();                        // reset timer
  }
}

void loop() {
  delay(100);
}

bool startSerial(){
  Serial.begin(9600);
  bool eventOn = true;

  while(!Serial.available() && millis() - lastTick < SERIAL_TIMEOUT){
    digitalWrite(ONBOARD_LED, eventOn);
    if (random(2)){             // flip on/off chaotically
      eventOn = !eventOn;
    }
    delay(200);
    Serial.begin(9600);
  }
  if (Serial.available()){
    delay(200);
    Serial.println("Serial is up!");
    return(true);
  } else {
    return(false);
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
