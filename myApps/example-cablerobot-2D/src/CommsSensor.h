#pragma once

#include "ofMain.h"
#include "ofxOsc.h"

class CommsSensor : ofThread
{
public:
	CommsSensor();

	void setup(vector<ofNode*> data, int port);
	void threadedFunction();

	vector<ofNode*> get_data() { return data; }

private:
	ofxOscReceiver receiver;
	void check_for_message();
	int port = 12345;

	vector<ofNode*> data;

	int K4A_JOINT_COUNT = 32;
};