#include <WiFiNINA.h>
#include <PubSubClient.h>
#include "wifi_secrets.h"
#include <AccelStepper.h>
#include <TMC2209.h>
#include "wiring_private.h"

// === Configuration ===
// Replace with your network credentials in wifi_secrets.h
const char* MQTT_BROKER_IP = "192.168.50.9";
const uint16_t MQTT_PORT = 1883;

// === Pins and Constants ===
#define STEP_PIN_X 3
#define DIR_PIN_X 2
#define STEP_PIN_Z 5
#define DIR_PIN_Z 6
#define ENABLE_X_PIN 10
#define ENABLE_Z_PIN 9
#define LIMIT_SWITCH_PIN 11  // Normally‐closed X‐axis limit switch

#define X_SERIAL Serial1

// Custom Serial2 on pins 13(RX), 8(TX) for Z driver (avoid SPI pins)
Uart Z_SERIAL(&sercom3, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);

const long jogStep = 100;    // step increment for discrete moves (if used)
const long jogSpeed = 1200;  // steps/sec for constant‐velocity jog/pan
const int X_RIGHT_LIMIT = -83700;

// === State Variables ===
bool xJogging = false;
bool zJogging = false;
bool xLimitTriggered = false;

long pointA = 0, pointB = 0;    // X‐axis set points
long zPointA = 0, zPointB = 0;  // Z‐axis set points

volatile bool clientDisconnectedFlag = false;

// === Command Parsing ===
enum CommandType {
  CMD_NONE,
  CMD_AXIS_X,  // absolute X‐axis move (if ever used)
  CMD_AXIS_Z,  // absolute Z‐axis move (if ever used)

  CMD_JOG_START_LEFT_X,
  CMD_JOG_START_RIGHT_X,
  CMD_JOG_STOP_X,

  CMD_SET_A,
  CMD_SET_B,
  CMD_START_TRAVEL,
  CMD_HOME,
  CMD_STOP_ALL,

  CMD_PAN_START_LEFT_Z,
  CMD_PAN_START_RIGHT_Z,
  CMD_PAN_STOP_Z,

  CMD_Z_SET_A,
  CMD_Z_SET_B,
  CMD_Z_START_TRAVEL
};

CommandType parseCommand(const String& t) {
  // X‐axis
  if (t.endsWith("/axis/x/command")) return CMD_AXIS_X;
  if (t.endsWith("/button/x-jog-start-left/pressed")) return CMD_JOG_START_LEFT_X;
  if (t.endsWith("/button/x-jog-start-right/pressed")) return CMD_JOG_START_RIGHT_X;
  if (t.endsWith("/button/x-jog-stop/pressed")) return CMD_JOG_STOP_X;
  if (t.endsWith("/button/x-set-A/pressed")) return CMD_SET_A;
  if (t.endsWith("/button/x-set-B/pressed")) return CMD_SET_B;
  if (t.endsWith("/button/x-start-travel/pressed")) return CMD_START_TRAVEL;
  if (t.endsWith("/button/home/pressed")) return CMD_HOME;

  // Z‐axis pan & set/travel
  if (t.endsWith("/axis/z/command")) return CMD_AXIS_Z;
  if (t.endsWith("/button/z-pan-start-left/pressed")) return CMD_PAN_START_LEFT_Z;
  if (t.endsWith("/button/z-pan-start-right/pressed")) return CMD_PAN_START_RIGHT_Z;
  if (t.endsWith("/button/z-pan-stop/pressed")) return CMD_PAN_STOP_Z;
  if (t.endsWith("/button/z-set-A/pressed")) return CMD_Z_SET_A;
  if (t.endsWith("/button/z-set-B/pressed")) return CMD_Z_SET_B;
  if (t.endsWith("/button/z-start-travel/pressed")) return CMD_Z_START_TRAVEL;

  // General

  if (t.endsWith("/button/stop/pressed")) return CMD_STOP_ALL;

  return CMD_NONE;
}

// === Debug Macros ===
#define DEBUG 1
#ifdef DEBUG
#define Sprint(a) Serial.print(a)
#define Sprintln(a) Serial.println(a)
#else
#define Sprint(a)
#define Sprintln(a)
#endif

// === Stepper & Driver Objects ===
TMC2209 driverX, driverZ;
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_PIN_Z, DIR_PIN_Z);


// === Wi-Fi & MQTT Objects ===
WiFiClient net;
PubSubClient client(net);

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi “");
  Serial.print(WIFI_SSID);
  Serial.print("”...");
  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (client.connect("nano33-client")) {
      Serial.println(" connected!");
      client.subscribe("camera-slider/axis/+/command");
      client.subscribe("camera-slider/button/+/pressed");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" – retrying in 2s");
      delay(2000);
    }
  }
}

// === Limit‐Switch Interrupt Handler ===
void handleXLimitInterrupt() {
  // Only trigger if we are actively moving right:
  if (xJogging && stepperX.speed() > 0) {
    xLimitTriggered = true;
    stepperX.stop();
    digitalWrite(ENABLE_X_PIN, HIGH);  // disable the X driver
    detachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN));
    Sprintln("X limit switch opened driver disabled until jog left");
  }
}

// === MQTT Callback ===
void callback(char* topic, byte* payload, unsigned int length) {
  String msg, t = String(topic);
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  CommandType cmd = parseCommand(t);
  Serial.print("→ MQTT recvd: ");
  Serial.print(t);
  Serial.print("  → cmd=");
  Serial.println(cmd);

  switch (cmd) {
    //-----------------------------------------
    // X‐axis jog/pan
    case CMD_JOG_START_LEFT_X:
      // // If limit was previously triggered, clear and re-enable:
      if (xLimitTriggered) {
        xLimitTriggered = false;
        digitalWrite(ENABLE_X_PIN, LOW);  // re-enable driver
        attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN),
                        handleXLimitInterrupt, RISING);
        Sprintln(" X limit cleared; driver re-enabled.");
      }
      Sprintln(stepperX.currentPosition());
      if (stepperX.currentPosition() >  X_RIGHT_LIMIT){
      xJogging = true;
      stepperX.setSpeed(-jogSpeed);
      Sprintln("← X Jog Start Left");
      }
      break;

    case CMD_JOG_START_RIGHT_X:

      xJogging = true;
      stepperX.setSpeed(+jogSpeed);
      Sprintln("← X Jog Start Right");

      break;

    case CMD_JOG_STOP_X:
      xJogging = false;
      stepperX.setSpeed(0);
      Sprintln("← X Jog Stop");
      break;

    //-----------------------------------------
    // X‐axis Set/A/B/Travel
    case CMD_SET_A:
      pointA = stepperX.currentPosition();
      Sprintln("← X Set A = " + String(pointA));
      break;

    case CMD_SET_B:
      pointB = stepperX.currentPosition();
      Sprintln("← X Set B = " + String(pointB));
      break;

    case CMD_START_TRAVEL:
      xJogging = false;  // cancel any X jog
      stepperX.moveTo(pointA);
      Sprintln("← X Start Travel → A");
      break;

    //-----------------------------------------
    // Z‐axis pan (constant velocity)
    case CMD_PAN_START_LEFT_Z:
      zJogging = true;
      stepperZ.setSpeed(-jogSpeed);
      Sprintln("← Z Pan Start Left");
      break;

    case CMD_PAN_START_RIGHT_Z:
      zJogging = true;
      stepperZ.setSpeed(+jogSpeed);
      Sprintln("← Z Pan Start Right");
      break;

    case CMD_PAN_STOP_Z:
      zJogging = false;
      stepperZ.setSpeed(0);
      Sprintln("← Z Pan Stop");
      break;

    //-----------------------------------------
    // Z‐axis Set/A/B/Travel
    case CMD_Z_SET_A:
      zPointA = stepperZ.currentPosition();
      Sprintln("← Z Set A = " + String(zPointA));
      break;

    case CMD_Z_SET_B:
      zPointB = stepperZ.currentPosition();
      Sprintln("← Z Set B = " + String(zPointB));
      break;

    case CMD_Z_START_TRAVEL:
      zJogging = false;  // cancel any Z pan
      stepperZ.moveTo(zPointA);
      Sprintln("← Z Start Travel → A");
      break;

    //-----------------------------------------
    // Home & Emergency Stop
    case CMD_HOME:

      Sprintln("← Homing X-axis...");

      // Move towards the limit switch
      stepperX.setSpeed(150);
      while (!digitalRead(LIMIT_SWITCH_PIN)) {
        stepperX.runSpeed();
      }

      // Immediately stop upon limit switch activation
      stepperX.stop();
      stepperX.runToPosition();

      // Set current position to zero (home position)
      stepperX.setCurrentPosition(0);

      Sprintln("X-axis homed at position 0.");

      Sprintln("← Homing ");
      break;

    case CMD_STOP_ALL:
      xJogging = false;
      zJogging = false;
      stepperX.stop();
      stepperZ.stop();
      Sprintln("← Emergency STOP All");
      break;

    default:
      break;
  }
}

// === Setup & Loop ===
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // 1) Connect to Wi-Fi & MQTT:
  connectToWiFi();
  client.setServer(MQTT_BROKER_IP, MQTT_PORT);
  client.setCallback(callback);

  // 2) X‐axis driver setup:
  X_SERIAL.begin(9600);
  driverX.setup(X_SERIAL);
  driverX.enable();
  driverX.setRunCurrent(50);
  Sprintln("Driver X ready");


  stepperX.setMaxSpeed(2500);
  stepperX.setAcceleration(50);
  pinMode(ENABLE_X_PIN, OUTPUT);






  digitalWrite(ENABLE_X_PIN, LOW);  // enable X driver

  // Limit switch on X (NC → RISING when switch opens)
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN),
                  handleXLimitInterrupt, RISING);

  // 3) Z‐axis driver setup:
  pinPeripheral(8, PIO_SERCOM_ALT);   // RX
  pinPeripheral(13, PIO_SERCOM_ALT);  // TX
  Z_SERIAL.begin(9600);
  driverZ.setup(Z_SERIAL);
  driverZ.enable();
  driverZ.setRunCurrent(75);
  Sprintln("★ Driver Z ready");

  stepperZ.setMaxSpeed(5000);
  stepperZ.setAcceleration(800);
  pinMode(ENABLE_Z_PIN, OUTPUT);
  digitalWrite(ENABLE_Z_PIN, LOW);  // enable Z driver
}

void loop() {
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();

  // X‐axis: if jogging, runSpeed(); else service any pending moveTo()
  if (xJogging) {
    stepperX.runSpeed();
  } else {
    stepperX.run();
  }

  // Z‐axis: if panning, runSpeed(); else service any pending moveTo()
  if (zJogging) {
    stepperZ.runSpeed();
  } else {
    stepperZ.run();
  }
}

#ifdef DEBUG
void printWifiStatus() {
  Sprint("Access point SSID: ");
  Sprintln(WiFi.SSID());
  Sprint("IP address: ");
  Sprintln(WiFi.localIP());
  Sprint("Gateway: ");
  Sprintln(WiFi.gatewayIP());
  Sprint("Netmask: ");
  Sprintln(WiFi.subnetMask());
}
#endif
