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
	void set_state(MotorState state);

	Motor* get_motor();
	int get_motor_id();
	bool run_homing_routine();

	//virtual void enable();
	//virtual void disable();
	//virtual bool is_enabled();
	//virtual bool is_ready();

	//virtual bool run_homing_routine(int timeout);
	//virtual bool is_homed();

	//virtual void stop();
	//virtual void set_e_stop(bool val);
	//virtual bool is_e_stopped();

	//virtual float get_velocity();
	//virtual float set_velocity(float val);
	//virtual float get_acceleration();
	//virtual float set_acceleration(float val);

	//virtual float get_position();
	//virtual void move_position(float pos, bool absolute);
	//virtual void move_velocity(float vel);

	//virtual bool run_shutdown_routine();
};