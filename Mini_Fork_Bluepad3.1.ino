#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>   // by Richard Quesada

// All mast Controll on the Right Stick, no more DPad, dynamic MastTilt with Right stick left an Right

ControllerPtr myController;

#define steeringServoPin 23
#define mastTiltServoPin 22

#define mastMotor0 25     // Used for controlling mast movement
#define mastMotor1 26     // Used for controlling mast movement
#define lightsAttach0 18  // Used for controlling headlight control
#define lightsAttach1 17  // Used for controlling headlight control

#define leftMotor0 21     // Used for controlling the left motor movement
#define leftMotor1 19     // Used for controlling the left motor movement
#define rightMotor0 33    // Used for controlling the right motor movement
#define rightMotor1 32    // Used for controlling the right motor movement

#define throttleDeadZone 40
#define steeringDeadZone 30
#define mastDeadZone 20
#define mastMoveSpeed 1
#define mastTiltMin 60
#define mastTiltMax 180

#define steeringMaxSpeed 3

#define wiggleCountMax 6

// <<< geändert: neue Defines für Tilt >>>
#define mastTiltDeadZone   70
#define mastTiltAxisMax   512   // typical value -512..512
#define mastTiltMinHz     10    // minimale Step-Frequenz (Stick little above center )
#define mastTiltMaxHz     180    // maximale Step-Frequenz (Stick full)

Servo steeringServo;
Servo mastTiltServo;

int lightSwitchTime = 0;
float steeringValue = 90;
float steeringAdjustment = 1;
int steeringTrim = 0;
int mastTiltValue = 115;
bool lightsOn = false;
unsigned long lastWiggleTime = 0;
int wiggleCount = 0;
int wiggleDirection = 1;
long wiggleDelay = 100;
bool shouldWiggle = false;

// <<< geändert: Variablen für Tilt-State >>>
enum TiltState { TILT_IDLE, TILT_UP, TILT_DOWN };
TiltState tiltState = TILT_IDLE;
unsigned long mastTiltLastStep = 0;
float mastTiltStepIntervalMs = 1000.0 / mastTiltMinHz;

void onConnectedController(ControllerPtr ctl) {
  ControllerProperties properties = ctl->getProperties();
  auto btAddress = properties.btaddr;
  if (myController == nullptr) {
    Serial.printf("CALLBACK: Controller is connected, ADDR=%2X:%2X:%2X:%2X:%2X:%2X\n",
                  btAddress[0], btAddress[1], btAddress[2],btAddress[3], btAddress[4], btAddress[5]);
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                  properties.product_id);
    myController = ctl;
    BP32.enableNewBluetoothConnections(false);
    ctl->playDualRumble(0, 250, 0x80, 0x40);
    shouldWiggle = true;
    processLights(true);
  }
  else {
    Serial.printf("CALLBACK: Controller connected, ADDR=%2X:%2X:%2X:%2X:%2X:%2X, but another controller already connected.\n",
                  btAddress[0], btAddress[1], btAddress[2],btAddress[3], btAddress[4], btAddress[5]);
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  ControllerProperties properties = ctl->getProperties();
  auto btAddress = properties.btaddr;
  if (myController == ctl) {
    Serial.printf("CALLBACK: Controller disconnected ADDR=%2X:%2X:%2X:%2X:%2X:%2X\n",
                  btAddress[0], btAddress[1], btAddress[2],btAddress[3], btAddress[4], btAddress[5]);
    myController = nullptr;
    BP32.enableNewBluetoothConnections(true);
  }
  else {
    Serial.printf("CALLBACK: Controller disconnected, ADDR=%2X:%2X:%2X:%2X:%2X:%2X, but not active controller\n",
                  btAddress[0], btAddress[1], btAddress[2],btAddress[3], btAddress[4], btAddress[5]);
  }
}

void processGamepad(ControllerPtr ctl) {
  //Steering
  processSteering(ctl->axisX());
  //Throttle
  processThrottle(ctl->axisY());
  //Rasing and lowering of mast
  processMast(ctl->axisRY());
  //MastTilt
  // <<< changed:  dpad() -> axisRX() >>>
  processMastTilt(ctl->axisRX());
  //Aux
  processLights(ctl->thumbR() | ctl->a());

  processWiggle(ctl->b());

  processTrimRight(ctl->r1());
  processTrimLeft(ctl->l1());
}

void processWiggle(bool value) {
  if (value) {
    shouldWiggle = true;
    processLights(true);
  }
}

void wiggle() {                  
  unsigned long currentTime = millis();
  if (abs((int)(currentTime - lastWiggleTime)) >= wiggleDelay) {
    lastWiggleTime = currentTime;
    wiggleDirection = -wiggleDirection;
    wiggleCount++;
    moveMotor(rightMotor0, rightMotor1, wiggleDirection * 100);
    moveMotor(leftMotor0, leftMotor1, -1 * wiggleDirection * 100);
    if (wiggleCount >= wiggleCountMax) {
      moveMotor(leftMotor0, leftMotor1, 0);
      moveMotor(rightMotor0, rightMotor1, 0);
      wiggleCount = 0;
      shouldWiggle = false;
      processLights(true);
    }
  }
}

void processThrottle(int newValue) {
  if (abs(newValue) <= throttleDeadZone) {
    moveMotor(leftMotor0, leftMotor1, 0);
    moveMotor(rightMotor0, rightMotor1, 0);
  } else {
    float throttleValue = newValue / 2;
    if (steeringValue > 100) {
      moveMotor(leftMotor0, leftMotor1, throttleValue * steeringAdjustment);
      moveMotor(rightMotor0, rightMotor1, throttleValue);
    } else if (steeringValue < 80) {
      moveMotor(leftMotor0, leftMotor1, throttleValue);
      moveMotor(rightMotor0, rightMotor1, throttleValue * steeringAdjustment);
    } else {
      moveMotor(leftMotor0, leftMotor1, throttleValue);
      moveMotor(rightMotor0, rightMotor1, throttleValue);
    }
  }
}

void processMast(int newValue) {
  int mastValue = newValue / 2;
  if (mastValue > mastDeadZone) {
    mastValue -= mastDeadZone;
    moveMotor(mastMotor0, mastMotor1, mastValue);
  } else if (mastValue < -1 * mastDeadZone) {
    mastValue += mastDeadZone;
    moveMotor(mastMotor0, mastMotor1, mastValue);
  } else {
    moveMotor(mastMotor0, mastMotor1, 0);
  }
}

void processTrimRight(int trimValue) {
  if (trimValue == 1 && trimValue < 20) {
    steeringTrim++;
    delay(100);
  }
}

void processTrimLeft(int trimValue) {
  if (trimValue == 1 && trimValue > -20) {
    steeringTrim--;
    delay(100);
  }
}

void processSteering(int newValue) {
  if (abs(newValue) < steeringDeadZone) {
    newValue = 0;
  }
  else if (newValue > 0) {
    newValue -= steeringDeadZone;
  }
  else {
    newValue += steeringDeadZone;
  }

  int targetValue = (90 - (newValue / 10));
  int delta = steeringValue - targetValue;
  if (abs(delta) > steeringMaxSpeed) {
    delta = steeringMaxSpeed * (delta > 0 ? 1 : -1);
  }
  steeringValue -= delta;
  steeringServo.write(steeringValue - steeringTrim);

  steeringAdjustment = 1 - (abs(newValue) / (768.0 - steeringDeadZone));
}

// <<< geändert: Neue Tilt-Implementierung >>>
void processMastTilt(int axisRX) {
  int a = axisRX;
  int sign = 0;
  if (a > mastTiltDeadZone) sign = +1;
  else if (a < -mastTiltDeadZone) sign = -1;
  else sign = 0;

  if (sign == 0) {
    tiltState = TILT_IDLE;
  } else {
    // <<< Change if movement is inverted>>>
    // Links (negativ) = UP / vor
    // Rechts (positiv) = DOWN / zurück
    tiltState = (sign < 0) ? TILT_UP : TILT_DOWN;

    int absA = abs(a) - mastTiltDeadZone;
    int absMax = mastTiltAxisMax - mastTiltDeadZone;
    if (absMax < 1) absMax = 1;
    float x = (float)absA / (float)absMax;  // 0..1

    float hz = mastTiltMinHz + x * (mastTiltMaxHz - mastTiltMinHz);
    if (hz < 1.0f) hz = 1.0f;
    mastTiltStepIntervalMs = 1000.0f / hz;
  }

  unsigned long now = millis();
  if (tiltState != TILT_IDLE && (now - mastTiltLastStep) >= mastTiltStepIntervalMs) {
    mastTiltLastStep = now;

    if (tiltState == TILT_UP && mastTiltValue < mastTiltMax) {
      mastTiltValue = min(mastTiltValue + mastMoveSpeed, mastTiltMax);
      mastTiltServo.write(mastTiltValue);
    } 
    else if (tiltState == TILT_DOWN && mastTiltValue > mastTiltMin) {
      mastTiltValue = max(mastTiltValue - mastMoveSpeed, mastTiltMin);
      mastTiltServo.write(mastTiltValue);
    }
  }
}

void processLights(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchTime) > 200) {
    if (lightsOn) {
      digitalWrite(lightsAttach0, LOW);
      digitalWrite(lightsAttach1, LOW);
      lightsOn = false;
    } else {
      digitalWrite(lightsAttach0, LOW);
      digitalWrite(lightsAttach1, HIGH);
      lightsOn = true;
    }
    lightSwitchTime = millis();
  }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 1) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -1) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}

void processController() {
  if (myController && myController->isConnected() && myController->hasData()) {
    if (myController->isGamepad()) {
      processGamepad(myController);
    } else {
      Serial.println("Unsupported controller");
    }
  }
}

void setup() {
  pinMode(mastMotor0, OUTPUT);
  pinMode(mastMotor1, OUTPUT);
  pinMode(lightsAttach0, OUTPUT);
  pinMode(lightsAttach1, OUTPUT);
  digitalWrite(lightsAttach0, LOW);
  digitalWrite(lightsAttach1, LOW);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);

  steeringServo.attach(steeringServoPin);
  steeringServo.write(steeringValue);
  mastTiltServo.attach(mastTiltServoPin);
  mastTiltServo.write(mastTiltValue);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processController();
  }
  if (shouldWiggle) {
    wiggle();
  }
  else { vTaskDelay(1); }
}
