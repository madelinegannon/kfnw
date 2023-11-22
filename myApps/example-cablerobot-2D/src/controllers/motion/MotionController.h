#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOsc.h"
#include "ofxEasing.h"


#include "controllers/agent/AgentController.h"

class MotionController
{
private:

	ofNode* origin;      // World reference frame 
	vector<glm::vec3> bases;

	ofPolyline path;
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

	ofNode centroid;
	vector<glm::vec3*> get_targets();

	void rotate(float theta);
	float rotation_theta = 0;
	void scale(float scalar);
	float scalar_percent = 1;

	vector<ofPolyline*> paths_drawing;
	void setup_paths(int count=4);
	void update_paths();
	void draw_paths();
	void add_to_path(int i, glm::vec3 pt);
	void update_targets(vector<glm::vec3> actual);

	void update_path(ofPolyline* path, glm::vec3 pt);
	void clear_paths();
	void draw_path(ofPolyline* path);

	//AgentController* agents;
	//void setup_agents();
	//void draw_agents();
	//void update_agents();


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
	ofParameter<glm::vec3> motion_pos;
	glm::vec3 motion_pos_prev = glm::vec3();
	ofParameter<float> motion_line_length = 2000;
	ofParameter<float> motion_theta = 0;
	ofParameter<bool> motion_drawing_follow = false;
	ofParameter<bool> motion_line_follow = false;
	ofParameter<bool> motion_circle_follow = false;
	//ofParameter<bool> motion_agents_follow = false;
	ofParameter<void> motion_reset;
	ofParameter<bool> motion_spin_enable = false;
	ofParameter<float> motion_spin_speed = 0;

	ofParameterGroup params_motion_drawing;
	ofParameter<float> motion_drawing_offset = 150;
	ofParameter<float> motion_drawing_accuracy = 50;
	ofParameter<float> motion_drawing_length_max = 500;

	ofParameterGroup params_motion_line;
	float motion_line_rotation = 0;

	ofParameterGroup params_motion_circle;
	ofParameter<float> motion_circle_radius = 1000;
	ofParameter<float> motion_circle_angle_start = 0;
	ofParameter<float> motion_circle_angle_end = 180;

	ofParameterGroup params_motion_sine;
	ofParameter<bool> enable_sine_wave = true;
	ofParameter<float> sine_wave_speed = 0.01;
	ofParameter<float> sine_wave_amplitude = 50;
	ofParameter<float> sine_wave_offset = 0.01;
	float sine_wave_counter = 0;

	void on_enable_pendulum(bool& val);
	ofParameterGroup params_motion_pendulum;
	ofParameter<bool> enable_pendulum = true;
	ofParameter<float> pendulum_speed = 0.001;
	ofParameter<float> pendulum_offset = 0.01;
	float pendulum_counter = 0;

	void on_enable_eyes(bool& val);
	void on_eye_spacing(float& val);
	ofParameterGroup params_eyes;
	ofParameter<bool> enable_eyes = false;
	ofParameter<float> eye_spacing = 250;
	ofParameter<float> eye_radius = 250;
	ofParameter<float> pupil_radius = 125;
	void setup_eyes();
	void update_eyes();
	void draw_eyes();
	vector<ofNode*> rigging_eyes;
	ofNode center = ofNode();
	ofNode left_eye = ofNode();
	ofNode left_pupil = ofNode();
	ofNode right_eye = ofNode();
	ofNode right_pupil = ofNode();

	void on_motion_drawing_follow(bool& val);
	void on_motion_line_follow(bool& val);
	void on_motion_circle_follow(bool& val);
	//void on_motion_agents_follow(bool& val);
	void on_motion_circle_radius(float& val);
	void on_motion_circle_angle_start(float& val);
	void on_motion_circle_angle_end(float& val);

	void rotate(ofPolyline* path, float theta);
	void rotateLine(ofPolyline* path, float theta);
	void rotateCircle(ofPolyline* path, float theta);

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

