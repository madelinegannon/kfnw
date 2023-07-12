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
	float offset_z = -75;

	void setup_gui();

	vector<glm::vec3*> targets;

	glm::vec3 target;
	float evaluate_percent = 0;

public:
	MotionController();
	~MotionController();

	void setup();
	void update();
	void draw();
    void keyPressed(int key);

	vector<glm::vec3*> get_targets();


	ofxPanel panel;
	ofParameterGroup params;
	ofParameter<glm::vec3> pos;
	ofParameter<float> radius;
	ofParameter<float> resolution;
	ofParameter<float> offset_theta = 0;
	ofParameter<float> speed;
	ofParameter<bool> play;
	ofParameter<void> reset;

	void on_pos_changed(glm::vec3& val);
	void on_parameter_changed(float& val);
	void on_reset();

	struct MotionPath
	{
		ofPolyline path;
		ofNode centroid;
		glm::vec3 target;

		float radius = 250;
		float resolution = 30;
		float offset_theta = 0;

		void reset() {
			path.clear();
			float theta = 360.0 / resolution;
			for (int i = 0; i <= resolution; i++) {
				glm::vec3 pt = glm::rotateZ(glm::vec3(radius, 0, 0), ofDegToRad(theta * i + offset_theta));
				pt += centroid.getGlobalPosition();
				path.addVertex(pt);
			}
			target = path.getVertices()[0];
		};

		void draw() {
			ofPushStyle();

			centroid.draw();

			ofNoFill();
			ofSetLineWidth(5);
			ofSetColor(255, 100);
			path.draw();

			ofFill();
			ofSetColor(ofColor::red);
			ofDrawEllipse(target, 30, 30);

			ofPopStyle();
		}
	};
	vector<MotionPath> paths;
};

