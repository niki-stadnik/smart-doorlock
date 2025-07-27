#include <Arduino.h>
#include <WiFi.h>
#include "WebSocketsClient.h"
#include "StompClient.h"
#include "SudoJSON.h"

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

//the potentiometer pin is the middle of the pins. the other 2 are connected to vcc and gnd

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
int potValOld = 0;
int sum = 0;
int i = 0;


//onboard led
#include <FastLED.h>
#define NUM_LEDS 1
#define DATA_PIN 48
CRGB leds[NUM_LEDS];


void setup() {

  //onboard led
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);



  WiFi.begin(wlan_ssid, wlan_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  stomper.onConnect(subscribe);
  stomper.onError(error);
  // Start the StompClient
  if (useWSS) {
    stomper.beginSSL();
  } else {
    stomper.begin();
  }


  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);


  // --- TMC2209 Init ---
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX); // RX=-1 disables receive
  driver.begin();
  driver.toff(5);
  driver.tbl(1); 
  driver.rms_current(1500);       // Set motor current in mA (motor is rated for 4.1V 1A so 4.1W - at 12V that is 0.34A but for short bursts its ok)
  driver.microsteps(8);
  driver.en_spreadCycle(false);  // Use stealthChop
    /*It enables or disables the SpreadCycle PWM mode on the TMC2209 stepper driver.
    SpreadCycle is a PWM current control mode designed for high torque and performance.
    The alternative mode is stealthChop, optimized for quiet and smooth operation.*/
  driver.pwm_autoscale(true);
    /*PWM Auto Scale is a feature that automatically adjusts the PWM amplitude to keep the motor current within desired limits.
    It monitors the back-EMF and adjusts the PWM duty cycle accordingly to maintain smooth current control.
    This helps prevent motor stalls and improves torque consistency, especially at low speeds.*/
  driver.pwm_autograd(true);           // Disable smoothing
    /*It stands for PWM Auto Gradient.
    It is a feature of the TMC2209’s stealthChop mode.
    It automatically adjusts the PWM gradient to optimize motor noise and torque.*/

  // --- AccelStepper Init ---
  stepper.setMaxSpeed(9000);
  stepper.setAcceleration(9000);
  stepper.setCurrentPosition(0);

 
  //calibrate(); //mesured as 13000 //pot = steps ; 4095 = 0 ; 0 = 13000


  stepper.setSpeed(9000);
}



void loop() {
  if(millis() >= keepAlive + 600000){  //if no messages are recieved in 10min - restart esp
    ESP.restart();
    keepAlive = millis();
  }

  int x = analogRead(POT_PIN);  // Reads 0 - 4095 on ESP32
  sum = sum + x;
  i++;
  if(millis() >= mesure + 500){ //every 0.5s
    potVal = sum / i;
    i = 0;
    sum = 0;
    stepper.setCurrentPosition(map(potVal, 4095, 0, 0, maxPot));
    if (potVal < potValOld - 30 || potVal > potValOld + 30){
      sendData();
      leds[0] = CRGB::Green; FastLED.show();
    }
    sendData();
    potValOld = potVal;
    mesure = millis();
  }
  

  if(millis() >= sendtimeing + 60000){  //every minute
    sendData();  
    sendtimeing = millis();
  }

  webSocket.loop();
}

void sendData(){
  // Construct the STOMP message
  webSocket.loop();
  SudoJSON json;
  int position = stepper.currentPosition();
  json.addPair("position", position);
  // Send the message to the STOMP server
  stomper.sendMessage("/app/doorlock", json.retrive());   //this is the @SendTo anotation
  webSocket.loop();
}

void getData(String input){
  SudoJSON json = SudoJSON(input);
  int move = json.getPairI("move");
  moveToPos(move);
}


void mapPos() {
  int potVal = analogRead(POT_PIN);  // Reads 0 - 4095 on ESP32
  stepper.setCurrentPosition(map(potVal, 4095, 0, 0, maxPot));  //map(value, fromLow, fromHigh, toLow, toHigh)
  if (potVal < potValOld - 30 || potVal > potValOld + 30){
    sendData();
  }
  potValOld = potVal;
}

void calibrate() {
  digitalWrite(EN_PIN, LOW);
  stepper.setSpeed(-5000);
  while (analogRead(POT_PIN) < 4095) {
    stepper.runSpeed();
    delay(10);
  }
  stepper.setCurrentPosition(0);
  delay(1000);
  stepper.setSpeed(5000);
  delay(1000);
  while (analogRead(POT_PIN) > 0){
    stepper.runSpeed();
    delay(10);
  }
  maxPot = stepper.currentPosition();
  digitalWrite(EN_PIN, HIGH);
}

void moveToPos(int pos) {
  leds[0] = CRGB::Red; FastLED.show();
  digitalWrite(EN_PIN, LOW); //enable stepper
  //mapPos();
  //----
  int potVal = analogRead(POT_PIN);  // Reads 0 - 4095 on ESP32
  stepper.setCurrentPosition(map(potVal, 4095, 0, 0, maxPot));  //map(value, fromLow, fromHigh, toLow, toHigh)
  //----
  stepper.moveTo(pos); //set desired move: number of stepps  // 6 full rotations, 1600 1 rotation
  while (stepper.currentPosition() != pos){ //returns the current position...
    stepper.run(); //makes 1 step
    webSocket.loop();
  }/*
  mapPos();
  if (stepper.currentPosition() < pos - 50 || stepper.currentPosition() > pos + 50) {
    moveToPos(pos);
  }
  */
  digitalWrite(EN_PIN, HIGH); // disable stepper
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

