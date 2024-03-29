#pragma once

#include "ofMain.h"

enum Groove {
    NONE = 0,
    LEFT_HANDED, // UP=CCW(+), DOWN=CW(-)
    RIGHT_HANDED // UP=CW(-),  DOWN=CCW(+)
};

class CableDrum
{
private:
    float diameter_drum;
    float diameter_cable;
    float length;
    int turns;
public:
    CableDrum() = default;
    void initialize(Groove direction, float diameter_drum = 99.95, float length = 30, int turns = 30);
    void draw();
    Groove direction = Groove::NONE;
    float get_diameter() { return diameter_drum; };
    
    float circumference = 0;
};