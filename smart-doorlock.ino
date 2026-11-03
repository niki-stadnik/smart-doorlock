#include <Arduino.h>
#include <WiFi.h>
#include "WebSocketsClient.h"
#include "StompClient.h"
#include "SudoJSON.h"
#include <ArduinoOTA.h>

#define NAME "smart-doorlock"
/*
ESP32S3 Dev Module
Tools → Partition Scheme → Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
Tools → Flash Size → confirm 4MB
*/

//debug
#define DEBUG 0 //1 = debug messages ON; 0 = debug messages OFF

#if DEBUG == 1
#define debugStart(x) Serial.begin(x)
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debugStart(x)
#define debug(x)
#define debugln(x)
#endif

#include "config.h"
const char* wlan_ssid = WIFI;
const char* wlan_password =  PASS;
const char * ws_host = HOSTPI;
const uint16_t ws_port = PORT;
const char* ws_baseurl = URL; 
bool useWSS = USEWSS;
const char * key = KEY;
// VARIABLES
WebSocketsClient webSocket;
Stomp::StompClient stomper(webSocket, ws_host, ws_port, ws_baseurl, true);
unsigned long keepAlive = 0;

unsigned long sendtimeing = 0;
unsigned long mesure = 0;


#include <TMCStepper.h>
#include <AccelStepper.h>

String debugString = "";

//the potentiometer pin is the middle of the pins. the other 2 are connected to vcc and gnd

#define DOOR_PIN 13
#define POT_PIN 4
#define EN_PIN 2
#define DIR_PIN 6
#define STEP_PIN 7

//Uart driver settings
#define UART_TX 8
#define UART_RX 5
#define R_SENSE   0.11f     // Sense resistor value, typically 0.11Ω  
TMC2209Stepper driver = TMC2209Stepper(&Serial1, R_SENSE, 0);  // HardwareSerial1 for UART

//Controller setup
AccelStepper stepper (1, STEP_PIN, DIR_PIN);


int potVal = 0;
int maxPot = 13700;
uint32_t sum = 0;
int i = 0;
boolean flag = false;


void setup() {
  // setup serial
  debugStart(115200);
  // flush it - ESP Serial seems to start with rubbish
  debugln();

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  pinMode(DOOR_PIN, INPUT_PULLUP);


  // --- TMC2209 Init ---
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX); // RX=-1 disables receive
  delay(100);
  driver.begin();
  delay(100);
  driver.toff(5);
  delay(100);
  driver.tbl(2); 
  delay(100);
  driver.rms_current(4000);       // Set motor current in mA (motor is rated for 4.1V 1A so 4.1W - at 12V that is 0.34A but for short bursts its ok)
  driver.microsteps(8); //8 default
  delay(100);
  //driver.en_spreadCycle(false);  // Use stealthChop //default false, to be silent
  //delay(100);
    /*It enables or disables the SpreadCycle PWM mode on the TMC2209 stepper driver.
    SpreadCycle is a PWM current control mode designed for high torque and performance.
    The alternative mode is stealthChop, optimized for quiet and smooth operation.*/
  driver.pwm_autoscale(true);
  delay(100);
    /*PWM Auto Scale is a feature that automatically adjusts the PWM amplitude to keep the motor current within desired limits.
    It monitors the back-EMF and adjusts the PWM duty cycle accordingly to maintain smooth current control.
    This helps prevent motor stalls and improves torque consistency, especially at low speeds.*/
  driver.pwm_autograd(true);           // Disable smoothing
  delay(100);
    /*It stands for PWM Auto Gradient.
    It is a feature of the TMC2209’s stealthChop mode.
    It automatically adjusts the PWM gradient to optimize motor noise and torque.*/
  
  // --- AccelStepper Init ---
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(9000);
  stepper.setCurrentPosition(0);
  stepper.setSpeed(9000);


  // connect to WiFi
  debugln("Logging into WLAN: " + String(wlan_ssid));
  debug(" ...");
  WiFi.begin(wlan_ssid, wlan_password);
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    i++;
    delay(200);
    debug(".");
    if (i >= 25) ESP.restart(); // 5s / 200ms = 25
  }
  debugln(" success.");
  debug("IP: "); debugln(WiFi.localIP());
  
  //OTA upload
  ArduinoOTA.setHostname(NAME); // shows up with this name
  ArduinoOTA.setPassword("admin"); // optional security
  ArduinoOTA.onStart([]() {
    debugln("OTA update starting...");
  });
  ArduinoOTA.onEnd([]() {
    debugln("\nOTA update complete!");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    debugln("Progress: " + (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    debugln("Error" + error);
  });

  ArduinoOTA.begin();
  debugln("OTA ready");


  stomper.onConnect(subscribe);
  stomper.onError(error);
  // Start the StompClient
  if (useWSS) {
    stomper.beginSSL();
  } else {
    stomper.begin();
  }
}

boolean calFlag = true;

void loop() {
  ArduinoOTA.handle();
  
  if (flag){
    sendData();
    flag = false;
    sum = 0;
    i = 0;
  }
  
  if(millis() >= keepAlive + 600000){  //if no messages are recieved in 10min - restart esp
    ESP.restart();
    keepAlive = millis();
  }

  if (calFlag){
    //calibratePot(1000, 12000, 12); // the value must be 1000 before upload
    calFlag = false;
  }

  int x = analogReadMilliVolts(POT_PIN);  // Reads 0 - 4095 on ESP32
  sum += x;
  i++;
  if(millis() >= mesure + 500){ //every 0.5s
    float mv = sum / (float)i;
    stepper.setCurrentPosition(mvToPos(mv));
    sendData();
    i = 0;
    sum = 0;
    debugln(mv);
    mesure = millis();
  }

  webSocket.loop();
}

void sendData(){
  // Construct the STOMP message
  SudoJSON json;
  int position = stepper.currentPosition();
  boolean door = digitalRead(DOOR_PIN);
  json.addPair("position", position);
  json.addPair("door", door);
  json.addPair("debug", debugString.c_str());
  // Send the message to the STOMP server
  stomper.sendMessage("/app/device/doorlock", json.retrive());   //this is the @SendTo anotation
  debugln("sent");
}

void getData(String input){
  SudoJSON json = SudoJSON(input);
  int move = json.getPairI("move");
  int acc = json.getPairI("acc");
  int minSpeed = json.getPairI("minSpeed");
  int maxSpeed = json.getPairI("maxSpeed");
  int i11 = json.getPairI("i11");
  int i12 = json.getPairI("i12");
  int i21 = json.getPairI("i21");
  int i22 = json.getPairI("i22");
  int i31 = json.getPairI("i31");
  int i32 = json.getPairI("i32");
  moveToPos(move, acc, minSpeed, maxSpeed, i11, i12, i21, i22, i31, i32);
}

void move(int pos, int acc, int minSpeed, int maxSpeed, int i11, int i12, int i21, int i22, int i31, int i32){
  unsigned long lastProgressTime = millis();
  long lastPosition = stepper.currentPosition();

  while (stepper.currentPosition() != pos) {
    long currentPos = stepper.currentPosition();

    // --- Variable speed zones ---
    if ((currentPos >= i11 && currentPos <= i12) ||
        (currentPos >= i21 && currentPos <= i22) ||
        (currentPos >= i31 && currentPos <= i32)) {
      stepper.setMaxSpeed(minSpeed);
    } else {
      stepper.setMaxSpeed(maxSpeed);
    }

    stepper.run();  //makes 1 step

    // --- Stall detection ---
    if (currentPos != lastPosition) {
      // motor moved, reset the timer
      lastPosition = currentPos;
      lastProgressTime = millis();
    } else {
      // motor hasn't moved since last check
      if (millis() - lastProgressTime >= 2000) {
        // stalled for 2 seconds straight - abort
        digitalWrite(EN_PIN, HIGH); // disable stepper
        flag = true;
        return;
      }
    }
  }
}

void moveToPos(int pos, int acc, int minSpeed, int maxSpeed, int i11, int i12, int i21, int i22, int i31, int i32) {
  int check1 = (i21+i12)/2; //4500 //5600
  int check2 = (i31+i22)/2; //7800 //8800
  stepper.setAcceleration(acc);
  digitalWrite(EN_PIN, LOW); //enable stepper
  //setPosAverage();
  stepper.setCurrentPosition(mvToPos(readMv()));

  if (pos > i32){  //unlocking
    if (stepper.currentPosition() < check1){
      stepper.moveTo(check1);
      move(check1, acc, minSpeed, maxSpeed, i11, i12, i21, i22, i31, i32);
      //setPosAverage();
      stepper.setCurrentPosition(mvToPos(readMv()));
    }
    if (stepper.currentPosition() < check2){
      stepper.moveTo(check2);
      move(check2, acc, minSpeed, maxSpeed, i11, i12, i21, i22, i31, i32);
      //setPosAverage();
      stepper.setCurrentPosition(mvToPos(readMv()));
    }
  } else if (pos < i11){ //locking
    if (stepper.currentPosition() > check2){
      stepper.moveTo(check2);
      move(check2, acc, minSpeed, maxSpeed, i11, i12, i21, i22, i31, i32);
      //setPosAverage();
      stepper.setCurrentPosition(mvToPos(readMv()));
    }
    if (stepper.currentPosition() > check1){
      stepper.moveTo(check1);
      move(check1, acc, minSpeed, maxSpeed, i11, i12, i21, i22, i31, i32);
      //setPosAverage();
      stepper.setCurrentPosition(mvToPos(readMv()));
    }
  }

  //setPosAverage();
  stepper.setCurrentPosition(mvToPos(readMv()));
  stepper.moveTo(pos); //set desired move: number of stepps  // 6 full rotations, 1600 1 rotation
  move(pos, acc, minSpeed, maxSpeed, i11, i12, i21, i22, i31, i32);
  //setPosAverage();
  stepper.setCurrentPosition(mvToPos(readMv()));
  
  if (stepper.currentPosition() < pos - 50 || stepper.currentPosition() > pos + 50) {
    moveToPos(pos, acc, minSpeed, maxSpeed, i11, i12, i21, i22, i31, i32);
  }
  digitalWrite(EN_PIN, HIGH); // disable stepper
  flag = true;
}
/*
void setPosAverage(){
  delay(10);
  int sum = 0;
  for (int i=0; i<40; i++){
    potVal = analogRead(POT_PIN);  // Reads 0 - 4095 on ESP32
    sum = sum + potVal;
    delayMicroseconds(500); 
  }
  int avg = sum / 40;
  stepper.setCurrentPosition(map(avg, 4095, 0, 0, maxPot));  //map(value, fromLow, fromHigh, toLow, toHigh)
  //stepper.setCurrentPosition(mvToPos(readMv()));
}
*/

//calibration function is called from position 1000, and a table is generated in the debug message.
//tha table is pasted here
const int CAL_MAX = 16;
int   calN = 13;
long  calStart = 1000;
long  calSteps[CAL_MAX] = {0,1000,2000,3000,4000,5000,6000,7000,8000,9000,10000,11000,12000};
float calMv[CAL_MAX]    = {3040.6f, 2804.2f, 2536.1f, 2282.8f, 2049.6f,
                           1804.2f, 1540.9f, 1286.6f, 1036.8f, 803.2f, 562.5f, 324.7f, 105.2f};

long mvToPos(float mv) {
  int i = 0;
  while (i < calN - 2 && mv < calMv[i + 1]) i++;       // mV falls as position rises
  float t = (mv - calMv[i]) / (calMv[i + 1] - calMv[i]);
  return calStart + lroundf(calSteps[i] + t * (calSteps[i + 1] - calSteps[i]));
}
float readMv() {
  delay(10);                                   // settle
  uint32_t s = 0;
  for (int i = 0; i < 30; i++) { s += analogReadMilliVolts(POT_PIN); delayMicroseconds(500); }
  return s / 30.0;
}
void calibratePot(long startPos, long span, int n) {
  move(1000, 1500, 400, 400, 4250, 4300, 7250, 7300, 10350, 10400);
  if (n < 2 || n >= CAL_MAX) return;
  float up[CAL_MAX], down[CAL_MAX];
  long stepSize = span / n;

  digitalWrite(EN_PIN, LOW);
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(1500);
  stepper.setCurrentPosition(0);                       // relative steps

  for (int k = 0; k <= n; k++) {                       // up pass
    stepper.runToNewPosition(k * stepSize);
    up[k] = readMv();
  }
  for (int k = n; k >= 0; k--) {                       // down pass
    stepper.runToNewPosition(k * stepSize);
    down[k] = readMv();
  }
  digitalWrite(EN_PIN, HIGH);

  // average both passes, track the worst disagreement
  float maxDiff = 0;
  calN = n + 1;
  calStart = startPos;
  for (int k = 0; k <= n; k++) {
    calSteps[k] = k * stepSize;
    calMv[k] = (up[k] + down[k]) / 2.0f;
    float d = fabsf(up[k] - down[k]);
    if (d > maxDiff) maxDiff = d;
  }
  stepper.setCurrentPosition(startPos);                // motor is back at the start

  // paste-ready output, no newlines (safe inside JSON)
  debugString += String("calN=") + calN + ";calStart=" + calStart + ";steps={";
  for (int k = 0; k < calN; k++) debugString += String(calSteps[k]) + (k < calN - 1 ? "," : "");
  debugString += "};mv={";
  for (int k = 0; k < calN; k++) debugString += String(calMv[k], 1) + "f" + (k < calN - 1 ? "," : "");
  debugString += String("};maxdiff=") + String(maxDiff, 1) + ";";
  flag = true;                                         // loop() sends it
}

/*
обороти за врата мотор 
3
1.5
6

2.5 оборота на средното е целия диапазон на потенциометъра
половин преди и половин след
половин оборот на средното е 2 оборота
*/



// Once the Stomp connection has been made, subscribe to a topic

void subscribe(Stomp::StompCommand cmd) {
  stomper.subscribe("/topic/doorlock", Stomp::CLIENT, handleMessage);    //this is the @MessageMapping("/test") anotation so /topic must be added
  stomper.subscribe("/topic/keepAlive", Stomp::CLIENT, handleKeepAlive);
}

Stomp::Stomp_Ack_t handleMessage(const Stomp::StompCommand cmd) {
  getData(cmd.body);
  return Stomp::CONTINUE;
}
Stomp::Stomp_Ack_t handleKeepAlive(const Stomp::StompCommand cmd) {
  keepAlive = millis();
  return Stomp::CONTINUE;
}

void error(const Stomp::StompCommand cmd) {
}

