#include "MotorControllers.h"


MotorControllers::MotorControllers()
{
}

MotorControllers::MotorControllers(SysManager& SysMgr, INode* node)
{
	motor = new Motor(SysMgr, node);
}

MotorControllers::~MotorControllers()
{
}

Motor* MotorControllers::get_motor() {
	return motor;
}

int MotorControllers::get_motor_id() {
	return int(motor->get()->Info.Ex.Addr());
}