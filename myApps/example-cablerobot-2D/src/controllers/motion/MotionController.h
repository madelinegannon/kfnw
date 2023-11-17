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

	void rotate(float theta);
	float rotation_theta = 0;
	void scale(float scalar);
	float scalar_percent = 1;


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

	ofParameterGroup params_motion;
	ofParameterGroup params_motion_line;
	float motion_line_rotation = 0;
	ofParameter<glm::vec3> motion_pos;
	glm::vec3 motion_pos_prev = glm::vec3();
	ofParameter<float> motion_line_length = 2000;
	ofParameter<float> motion_theta = 0;
	ofParameter<bool> motion_drawing_follow = false;
	ofParameter<bool> motion_line_follow = false;
	ofParameter<bool> motion_circle_follow = false;
	ofParameter<void> motion_reset;
	ofParameter<bool> motion_spin_enable = false;
	ofParameter<float> motion_spin_speed = 0;
	ofParameterGroup params_motion_circle;
	ofParameter<float> motion_circle_radius = 1000;
	ofParameter<float> motion_circle_angle_start = 0;
	ofParameter<float> motion_circle_angle_end = 180;

	void on_motion_drawing_follow(bool& val);
	void on_motion_line_follow(bool& val);
	void on_motion_circle_follow(bool& val);
	void on_motion_circle_radius(float& val);
	void on_motion_circle_angle_start(float& val);
	void on_motion_circle_angle_end(float& val);

	void rotate(ofPolyline* path, float theta);

	void on_motion_pos_changed(glm::vec3& val);
	void on_motion_line_length_changed(float& val);
	void on_motion_theta_changed(float& val);
	void on_motion_reset();

	void calculate_theta(ofPolyline path);

	ofPolyline motion_line;
	void setup_motion_line();
	void draw_motion_line();
	void update_motion_line();

	ofPolyline motion_circle;
	void setup_motion_circle();
	void draw_motion_circle();
	void update_motion_circle(float start_angle = 0, float end_angle = 180, float resolution = 3);




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

