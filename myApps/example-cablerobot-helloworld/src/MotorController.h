#pragma once
#include "ofMain.h"
#include "pubSysCls.h"
#include "Axis.h"

using namespace sFnd;


class MotorController :
	public ofThread
{

public:

	MotorController();
	~MotorController();

	void update(bool use_sine);
	void close();

	void threadedFunction();


	bool play = false;

	void pause();

	int get_num_motors() { return axes.size(); };

	Axis* get_motor(int index) { return axes[index]; }
	ofxPanel* get_gui(int index){ return &axes[index]->panel; };

	void set_target(int index, float dist_mm) { target_positions[index] = dist_mm; };

private:
	SysManager* myMgr;
	vector<Axis*> axes;
	vector<float> target_positions;

	bool initialize();
};

