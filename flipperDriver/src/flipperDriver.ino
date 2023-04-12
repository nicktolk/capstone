/*
 * Project flipperDriver
 * Description: Handles detached threads for driving a pair of push-pull solenoids
 * Author:      Nick Tolk
 * Date:        07-APR-2023
 */

SYSTEM_MODE(SEMI_AUTOMATIC)
SYSTEM_THREAD(ENABLED)

const int PIN_L = D7;
const int PIN_R = D8;

const int IMGW = 320, IMGH = 240;
// region of interest {x0, y0, x1, y1}
const int ROIL[] = {100, 180, 160, 220};
const int ROIR[] = {160, 180, 220, 220};
const float THRESH = 130.0;               // average pixel value for "ball detected"
const int DOWNTIME = 500;   // ms of mandatory solenoid relaxation
// simulated image from camera
uint8_t imgIn[IMGW][IMGH];

String sOut;

unsigned int leftFlipT = 0, rightFlipT = 0;
bool checkLeft();   // returns true if ball is detected in region
bool checkRight();
//void flipLeft();
void flipRight();

// fills image with random data
void getFrame();

void setup() {
  Serial.begin(9600);
  pinMode(PIN_L, OUTPUT);
  pinMode(PIN_R, OUTPUT);

  // these threads actuate the solenoids
  new Thread("flipLeft", flipLeft);

}

void loop() {
  sOut = "";
  getFrame();
  if (checkLeft()){
    sOut += "LEFT  ";
    leftFlipT = millis() + 500;
    digitalWrite(PIN_L, HIGH);
  } else {
    sOut += "      ";
  }
  if (checkRight()){
    sOut += "RIGHT ";
    rightFlipT = 500;
    digitalWrite(PIN_R, HIGH);
  } else {
    sOut += "      ";
  }
//  Serial.printf("\r%5d L, %5d R", leftFlipT, rightFlipT);
  Serial.printf("\r%s", sOut.c_str());
  if (millis() > leftFlipT){
    digitalWrite(PIN_L, LOW);
  }
  if (millis() > rightFlipT){
    digitalWrite(PIN_R, LOW);
  }
//  delay(1000);
}

bool checkLeft(){
  static int x, y;
  static float sum;
  sum = 0;
  for (x = ROIL[0]; x < ROIL[2]; x++){
    for (y = ROIL[1]; y < ROIL[3]; y++){
      sum += imgIn[x][y];
    }
  }
  sum /= (ROIL[2] - ROIL[0]) * (ROIL[3] - ROIL[1]);
  return(sum > THRESH);
}

bool checkRight(){
  static int x, y;
  static float sum;
  sum = 0;
  for (x = ROIR[0]; x < ROIR[2]; x++){
    for (y = ROIR[1]; y < ROIR[3]; y++){
      sum += imgIn[x][y];
    }
  }
  sum /= (ROIR[2] - ROIR[0]) * (ROIR[3] - ROIR[1]);
  return(sum > THRESH);
}

void getFrame(){
  static int x, y;
//  static int i;
//  i = 0;
  for (x = 0; x < IMGW; x++){
    for (y = 0; y < IMGH; y++){
//      (uint_8*)(imgIn + i++) = random(0xff);
      imgIn[x][y] = random(0xff);
    }
  }
}


// runs detached watching for leftFlipT to be anything but 0, then actuvates solenoid for as long as requested
void flipLeft(){
  system_tick_t lastThreadTime = 0;
  while (true){
    if (leftFlipT != 0){
      lastThreadTime = millis();
      Serial.printf("Left!\n");
      digitalWrite(PIN_L, HIGH);
      os_thread_delay_until(&lastThreadTime, leftFlipT);
      digitalWrite(PIN_L, LOW);
      os_thread_delay_until(&lastThreadTime, leftFlipT + DOWNTIME);
      leftFlipT = 0;
    }
  }
}