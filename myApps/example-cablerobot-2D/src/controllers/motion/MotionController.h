#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOsc.h"
#include "ofxEasing.h"

class MotionController
{
private:

	ofNode* origin;      // World reference frame 
	vector<glm::vec3> bases;

	ofPolyline path;
	ofNode centroid;
	float offset_z = -75;

	void setup_gui();

	vector<glm::vec3*> targets;

	glm::vec3 target;
	float evaluate_percent = 0;

	bool showGUI = true;
	bool debugging = true;

public:
	MotionController();
	MotionController(vector<glm::vec3> bases, ofNode* _origin, float offset_z=-75);
	~MotionController();

	void setup();
	void update();
	void draw();
	void draw_gui();
    void keyPressed(int key);

	vector<glm::vec3*> get_targets();


	ofxPanel panel;
	ofParameterGroup params;
	ofParameter<glm::vec3> pos;
	ofParameter<float> radius;
	ofParameter<float> resolution;
	ofParameter<float> offset_theta = 0;
	ofParameter<float> speed;
	ofParameter<float> duration;
	ofParameter<bool> play;
	ofParameter<void> reset;

	void on_play(bool& val);
	void on_pos_changed(glm::vec3& val);
	void on_parameter_changed(float& val);
	void on_reset();

	enum PATH_TYPE {
		POLYGON,
		LINE,
		TRAJECTORY
	};

	struct MotionPath
	{
		ofPolyline path;
		ofNode centroid;
		glm::vec3 target;

		PATH_TYPE path_type;

		float radius = 250;
		float resolution = 30;
		float offset = 0;
		float offset_theta = 0;

		void reset() {
			path.clear();
			if (path_type == PATH_TYPE::POLYGON) {
				float theta = 360.0 / resolution;
				for (int i = 0; i <= resolution; i++) {
					glm::vec3 pt = glm::rotateZ(glm::vec3(radius, 0, 0), ofDegToRad(theta * i + offset_theta));
					pt += centroid.getGlobalPosition();
					path.addVertex(pt);
				}
				target = path.getVertices()[0];
			}
			else if (path_type == PATH_TYPE::LINE) {
				auto pos = centroid.getGlobalPosition();
				path.addVertex(glm::vec3(pos.x - offset, pos.y, pos.z));
				path.addVertex(glm::vec3(pos.x + offset, pos.y, pos.z));
				target = path.getVertices()[0];
			}
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
	
	MotionPath create_polygon(ofNode centroid=ofNode(), float radius = 250, float resolution = 30, float offset_theta = 0);
	MotionPath create_line(ofNode centroid=ofNode(), float offset = 250);

	float timer_start = 0;
};

