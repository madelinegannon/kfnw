#include "MotorController.h"


MotorController::MotorController()
{
}

MotorController::MotorController(SysManager& SysMgr, INode* node)
{
	motor = new Motor(SysMgr, node);
}

MotorController::~MotorController()
{
}

Motor* MotorController::get_motor() {
	return motor;
}

int MotorController::get_motor_id() {
	return int(motor->get()->Info.Ex.Addr());
}