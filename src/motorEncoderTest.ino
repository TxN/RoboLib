#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "PinChangeInterrupt.h"
#include "MotorLib.h"
#include "CommLib.h"

#define MOTOR_COUNT         2
#define NUM_LEDS            8
#define LEDS_PIN            16
#define VOLTAGE_PIN         A0
#define FAST_CYCLE_TIME     5
#define SLOW_CYCLE_TIME     100
#define RANGE_GET_TIMEOUT   100
#define VCC_VOLTAGE         5130

unsigned long lastFastCycleTimeMillis = 0;
unsigned long lastSlowCycleMillis     = 0;
unsigned long receivedCommandCount    = 0;
unsigned int  lastReadRange           = 0;
unsigned int  rawBatteryVoltage       = 0;

CRGB             cLeds[NUM_LEDS];
ControlableMotor Motors[MOTOR_COUNT];
VL53L0X          ranger;

void MotorATick() {
  Motors[0].Tick();
}

void MotorBTick() {
  Motors[1].Tick();
}

void setup() {
 pinMode(VOLTAGE_PIN, INPUT);
 Wire.begin();
 ranger.init();
 ranger.setTimeout(RANGE_GET_TIMEOUT);
 FastLED.addLeds<WS2812, LEDS_PIN>(cLeds, NUM_LEDS);
 FastLED.show();
 PrepareComm();
 comm_recv_basic = COMM_RECV_ANY_CALLBACK;
 RegSerialCommand(COMM_STATUS, COMM_STATUS_CALLBACK);
 RegSerialCommand(COMM_LED_SET, COMM_LED_SET_CALLBACK);
 RegSerialCommand(COMM_LED_SET_COLOR_ALL, COMM_LED_SET_COLOR_ALL_CALLBACK);
 RegSerialCommand(COMM_FLASHLIGHT, COMM_FLASHLIGHT_CALLBACK);
 RegSerialCommand(COMM_MOTOR_GET_SPEED, COMM_MOTOR_GET_SPEED_CALLBACK);
 RegSerialCommand(COMM_MOTOR_GET_TICKS, COMM_MOTOR_GET_TICKS_CALLBACK);
 RegSerialCommand(COMM_MOTOR_SET_MOVEMENT, COMM_MOTOR_SET_MOVEMENT_CALLBACK);
 RegSerialCommand(COMM_RANGE_GET, COMM_RANGE_GET_CALLBACK);
 RegSerialCommand(COMM_BATTERY_VOLTAGE_GET, COMM_BATTERY_VOLTAGE_GET_CALLBACK);

 Motors[0] = ControlableMotor();
 Motors[0].Init(L_MOTOR, 10, 9, 15, 32, MotorATick); //old enc = 3, new enc = 15
 Motors[1] = ControlableMotor();
 Motors[1].Init(R_MOTOR, 5, 6, 14, 32, MotorBTick); //old enc = 2, new enc = 14
}
 
void loop(){
  auto loopMillis = millis();
  CheckComm();
  FastCycle(loopMillis);
  SlowCycle(loopMillis);
}

void SlowCycle(unsigned long loopMillis) {
  if ( loopMillis >= (lastSlowCycleMillis + SLOW_CYCLE_TIME) ) { 
    lastSlowCycleMillis = loopMillis;
    //Doing some slow cycle stuff:
    UpdateRanger();
    UpdateBatteryVoltage();
  }
}

void FastCycle(unsigned long loopMillis) {
  if ( loopMillis >= (lastFastCycleTimeMillis + FAST_CYCLE_TIME) ) { 
    lastFastCycleTimeMillis = loopMillis;
    //Doing some fast cycle stuff:
    UpdateMotors();
  }
}

void UpdateRanger() {
  ranger.timeoutOccurred(); // reset timeout status if occured.  
}

void UpdateBatteryVoltage() {
  rawBatteryVoltage = analogRead(VOLTAGE_PIN);
}

void UpdateMotors() {
  for (int i = 0; i< MOTOR_COUNT; i++) {
    Motors[i].Update();
  }
}

void SetAllLedsRGB(byte r, byte g, byte b) {
  for (byte i = 0; i < NUM_LEDS; i++) {
    cLeds[i].setRGB(r, g, b);
  }
  FastLED.show();  
}

void COMM_RECV_ANY_CALLBACK() {
  receivedCommandCount++;
}

void COMM_STATUS_CALLBACK() {
  SERIAL_PORT.print("ROBOT_MIDDLEWARE_STATUS_OK: ");
  SERIAL_PORT.println(receivedCommandCount);
  cLeds[1].g = 255;
  cLeds[1].b = 0;
  cLeds[1].r = 0;
 FastLED.show();
}

void COMM_LED_SET_CALLBACK() {
  auto curLed = 0;
  for (byte i=0;i<commandPayloadLength;i+=3) {
    if ( curLed >= NUM_LEDS ) {
      break;
    }
    cLeds[curLed].r = commandBuffer[i];
    cLeds[curLed].g = commandBuffer[i + 1];
    cLeds[curLed].b = commandBuffer[i + 2];
    curLed++;
  }
  FastLED.show();
}

void COMM_LED_SET_COLOR_ALL_CALLBACK() {
  auto r = commandBuffer[0];
  auto g = commandBuffer[1];
  auto b = commandBuffer[2];
  SetAllLedsRGB(r, g, b);
}

void COMM_FLASHLIGHT_CALLBACK() {
  auto luminance = commandBuffer[0];
  SetAllLedsRGB(luminance, luminance, luminance);
}

void COMM_MOTOR_GET_TICKS_CALLBACK() {
  auto motorID = commandBuffer[0];
  motorID = constrain(motorID, 0, 1);
  SERIAL_PORT.print("MOTOR_TICKS ");
  SERIAL_PORT.print(motorID);
  SERIAL_PORT.print(" ");
  SERIAL_PORT.println(Motors[motorID].GetTicks());
}

void COMM_MOTOR_GET_SPEED_CALLBACK() {
  auto motorID = commandBuffer[0];
  motorID = constrain(motorID, 0, 1);
  SERIAL_PORT.print("MOTOR_SPEED ");
  SERIAL_PORT.print(motorID);
  SERIAL_PORT.print(" ");
  SERIAL_PORT.println(Motors[motorID].GetSpeedTPS());
}

void COMM_MOTOR_SET_MOVEMENT_CALLBACK() {
  auto motorID = commandBuffer[0];
  auto actType = commandBuffer[1];
  auto dir     = commandBuffer[2];
  auto speed   = commandBuffer[3];
  auto par1    = word(commandBuffer[4], commandBuffer[5]);
  auto par2    = word(commandBuffer[6], commandBuffer[7]);
  auto controlBoth = false;
  if ( motorID == 2 ) {
    controlBoth = true;
  }
  motorID = constrain(motorID, 0, 1);
  if ( !controlBoth ) {
    auto motor = Motors[motorID];
    motor.SetMovement( (MOTOR_MODE) actType, (MOTOR_DIRECTION) dir,speed, par1,par2);
  } else {
    Motors[0].SetMovement( (MOTOR_MODE) actType, (MOTOR_DIRECTION) dir,speed, par1,par2);
    Motors[1].SetMovement( (MOTOR_MODE) actType, (MOTOR_DIRECTION) dir,speed, par1,par2);
  }
}

void COMM_RANGE_GET_CALLBACK() {
  auto needRefresh = commandBuffer[0] > 0;
  if ( needRefresh ) {
    lastReadRange = ranger.readRangeSingleMillimeters();
  }
  SERIAL_PORT.print("RANGE ");
  SERIAL_PORT.println(lastReadRange);
}

void COMM_BATTERY_VOLTAGE_GET_CALLBACK() {
  long calcVolts = ((long) VCC_VOLTAGE * (long) rawBatteryVoltage) / 1024;
  SERIAL_PORT.print("BATT_V ");
  SERIAL_PORT.println(calcVolts);
}

