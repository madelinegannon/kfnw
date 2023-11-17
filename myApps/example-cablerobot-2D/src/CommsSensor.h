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

	glm::vec3 get_incoming_pt() { return pt; }

private:
	ofxOscReceiver receiver;
	void check_for_message();
	int port = 12345;

	vector<ofNode*> data;
	glm::vec3 pt = glm::vec3(0,0,0);

	int K4A_JOINT_COUNT = 32;
};