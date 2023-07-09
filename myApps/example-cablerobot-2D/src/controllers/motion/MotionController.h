#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOsc.h"
#include "ofxEasing.h"

class MotionController
{
private:

	ofPolyline path;
	ofNode centroid;

	void setup_gui();

	glm::vec3 target;
	float evaluate_percent = 0;

public:
	MotionController();
	~MotionController();

	void setup();
	void update();
	void draw();
    void keyPressed(int key);

	void create_path(glm::vec3 centroid=glm::vec3(), float radius=250, float resolution=30);

	glm::vec3 get_targets() { return target; }


	ofxPanel panel;
	ofParameterGroup params;
	ofParameter<glm::vec3> pos;
	ofParameter<float> radius;
	ofParameter<float> resolution;
	ofParameter<float> speed;
	ofParameter<bool> play;
	ofParameter<void> reset;

	void on_pos_changed(glm::vec3& val);
	void on_radius_changed(float& val);
	void on_resolution_changed(float& val);
	void on_reset();
};

