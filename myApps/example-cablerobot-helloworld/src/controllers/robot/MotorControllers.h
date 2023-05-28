#pragma once

#include "ofMain.h"
#include "pubSysCls.h"
#include "Motor.h"

enum MoveType {
	POS,
	VEL
};

enum MoveZone {
	FINE,
	SMALL,	// 5MM
	MEDIUM,	// 10MM
	LARGE	// 20MM
};

enum MotorState {
	DISABLED,
	PLAY,
	PAUSE,
	HOMING,
	E_STOP
};

using namespace sFnd;

class MotorControllers {
private:
	Motor* motor;
	MotorState state = DISABLED;
public:
	MotorControllers();
    MotorControllers(SysManager& SysMgr, INode* node);
	~MotorControllers();
	bool initialize();
	void update();
	void shutdown();

	Motor* get_motor();
	int get_motor_id();
	bool run_homing_routine();
	void set_target(float dist_mm);

	void set_state(MotorState state);
};