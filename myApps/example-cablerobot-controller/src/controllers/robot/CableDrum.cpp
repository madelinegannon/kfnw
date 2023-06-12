#include "CableDrum.h"

void CableDrum::initialize(Groove direction, float diameter_drum, float length, int turns)
{
	this->direction = direction;
	this->diameter_drum = diameter_cable;
	this->length = length;
	this->turns = turns;
	this->circumference = PI * diameter_drum;
}

void CableDrum::draw()
{
}
