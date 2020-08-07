#include <Arduino.h>
#include "PinChangeInterrupt.h"

#define L_MOTOR 1
#define R_MOTOR 2

#define AVERAGING_FACTOR 5
#define MEASURE_SPD_EVERY_MS 50

enum MOTOR_MODE : unsigned char {
  IDLING    = 0,
  TIME_RUN  = 1,
  TICK_RUN  = 2,
  REV_RUN   = 3,
  ANGLE_RUN = 4,
  CONT_RUN  = 5
};

enum MOTOR_DIRECTION : unsigned char {
	STOP     = 0,
	FORWARD  = 1,
	BACKWARD = 2
};

class ControlableMotor {
public:
  void Init(unsigned char id, unsigned char out1, unsigned char out2, unsigned char encPin, unsigned char revTicks, void(*f) ());
  void Tick();
  void Update();
  int  GetSpeedTPS();
  unsigned long GetTicks();
  void SetMovement(MOTOR_MODE mode, MOTOR_DIRECTION dir, unsigned char speed, int par1, int par2);
  MOTOR_MODE GetState();
private:
  MOTOR_MODE    curMode = IDLING;
  byte motorID;
  unsigned long tickCount = 0;
  byte outPin1;
  byte outPin2;
  byte ticksPerRev = 32;
  unsigned long lastSpdMeasureTime = 0;
  unsigned long lastSpdMeasureTick = 0;
  byte rollAverageCursor = 0;
  byte lastTicks[AVERAGING_FACTOR];
  unsigned long moveEndPos = 0;

  void WriteMovement(MOTOR_DIRECTION dir, unsigned char speed);
};

void ControlableMotor::Init(byte id, byte out1, byte out2, byte encPin, byte revTicks, void(*f) ()) {
	motorID = id;
	tickCount = 0;
	outPin1 = out1;
	outPin2 = out2;
	ticksPerRev = revTicks;
	pinMode(encPin, INPUT_PULLUP);
	attachPCINT(digitalPinToPCINT(encPin), f, CHANGE);
	//attachInterrupt(digitalPinToInterrupt(encPin), f, CHANGE); //for old interrupt logic
}

void ControlableMotor::Tick() {
  tickCount++;
}

int ControlableMotor::GetSpeedTPS() {
	int sum = 0;
	for (auto i = 0; i < AVERAGING_FACTOR; i++) {
		sum += lastTicks[i];
	}
	int spd = (sum / AVERAGING_FACTOR) * (1000 / MEASURE_SPD_EVERY_MS);
	return spd;
}

unsigned long ControlableMotor::GetTicks() {
	return tickCount;
}

void ControlableMotor::Update() {
	auto curTime = millis();
	if (curTime >= lastSpdMeasureTime + MEASURE_SPD_EVERY_MS) {
		lastSpdMeasureTime = curTime;
		int tickDelta = tickCount - lastSpdMeasureTick;
		lastTicks[rollAverageCursor] = tickDelta;
		rollAverageCursor++;
		if (rollAverageCursor >= AVERAGING_FACTOR) {
			rollAverageCursor = 0;
		}
		lastSpdMeasureTick = tickCount;
	}
	if ( curMode == TIME_RUN ) {
		if ( curTime > moveEndPos ) {
			SetMovement(IDLING, STOP, 0,0,0);
		}
	} else if ( curMode == TICK_RUN ) {
		if (tickCount >= moveEndPos ) {
			SetMovement(IDLING, STOP, 0,0,0);
		}
	}
}

void ControlableMotor::WriteMovement(MOTOR_DIRECTION dir, byte speed) {
	if (dir == STOP) {
		digitalWrite(outPin1, 0);
		digitalWrite(outPin2, 0);
		return;
	}
	if (dir == FORWARD) {
		analogWrite(outPin1, speed);
		analogWrite(outPin2, 0);
	} else {
		analogWrite(outPin1, 0);
		analogWrite(outPin2, speed);
	}
}

void ControlableMotor::SetMovement(MOTOR_MODE mode, MOTOR_DIRECTION dir, byte speed, int par1, int par2) {
	curMode    = mode;
	moveEndPos = 0;
	if (mode == IDLING ) {
		WriteMovement(STOP, 0);
		return;
	} else if ( mode == CONT_RUN ) {
		WriteMovement(dir, speed);
	} else if ( mode == TIME_RUN ) {
		WriteMovement(dir,speed);
		auto curTime = millis();
		moveEndPos = curTime + par1 - 10; 
	} else if ( mode == TICK_RUN ) {
		WriteMovement(dir,speed);
		moveEndPos = tickCount + par1;
	} else if ( mode == REV_RUN ) {
		curMode = TICK_RUN;
		WriteMovement(dir,speed);
		moveEndPos = tickCount + par1*ticksPerRev;
	} else if ( mode == ANGLE_RUN ) {
		curMode = TICK_RUN;
		WriteMovement(dir,speed);
		auto anglePerTick = 360 / ticksPerRev;
		moveEndPos = tickCount + (par1 / anglePerTick);
	}

}

MOTOR_MODE ControlableMotor::GetState() {
	return curMode;
}