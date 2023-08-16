#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGizmo.h"
#include "ofxXmlSettings.h"
#include "CableRobot.h"
#include "Trajectory.h"
#include "OneEuroFilter.h"


class CableRobot2D 
{
private:

	int id;
	vector<CableRobot*> robots;
	Trajectory trajectory;
	vector<Trajectory*> trajectories_2D;
	void update_trajectories_2D();
	void draw_trajectories_2D();

	void draw_cables_actual(glm::vec3 start_0, glm::vec3 end_0, float dist_0, glm::vec3 start_1, glm::vec3 end_1, float dist_1);
	void draw_cables_2D();

	vector<vector<glm::vec3>> ee_path;
	void draw_ee_path();

	ofNode* origin;     // World reference frame 
	ofNode* ee;			// World reference 
	ofxGizmo gizmo_ee;
	
	glm::vec3 base_top_left;
	glm::vec3 base_top_right;
	ofRectangle bounds;

	void setup_gui();
	string state_names[5] = { "NOT_HOMED", "HOMING", "ENABLED", "DISABLED", "E_STOP" };
   
	bool debugging = true;
public:

	//CableRobot2D(vector<CableRobot> robots);
	CableRobot2D() {};
	CableRobot2D(CableRobot* top_left, CableRobot* top_right, ofNode* _origin, glm::vec3 base_top_left, glm::vec3 base_top_right, int id);

	void update();
	void draw();
	void update_gui(ofxPanel* _panel);
	void draw_gui();
	void shutdown();

	void get_status();

	ofxGizmo* get_gizmo() { return &gizmo_ee; }
	bool override_gizmo = false;
	void update_gizmo();

	void key_pressed(int key);


	void move_position(float target_pos, bool absolute = true);
	void move_velocity(float target_pos);

	bool is_estopped();
	bool is_homed();
	bool is_enabled();
	bool run_homing_routine(int timeout = 20);

	void stop();
	void set_e_stop(bool val);
	void set_enabled(bool val);


	struct TimeSeriesPlot
	{
		string name = "";
		ofColor color_0 = ofColor::red;
		ofColor color_1 = ofColor::blue;
		int width = 500;
		int height = 100;
		int resolution = width;
		float min = -1;
		float max = 1;
		vector<vector<float>> data;

		vector<vector<float>> data_filtered;
		
		void update(float incoming_0, float incoming_1, float incoming_filtered_0, float incoming_filtered_1) {
			vector<float> temp;
			temp.push_back(incoming_0);
			temp.push_back(incoming_1);
			data.push_back(temp);

			vector<float> temp_filtered;
			temp_filtered.push_back(incoming_filtered_0);
			temp_filtered.push_back(incoming_filtered_1);
			data_filtered.push_back(temp_filtered);

			if (data.size() > resolution) {
				data.erase(data.begin());
				data_filtered.erase(data_filtered.begin());
			}

		}

		void draw() {
			ofPushStyle();

			// Draw Labels
			ofDrawBitmapStringHighlight(name, 0, -15);
			ofFill();
			ofSetColor(255, 40);
			ofDrawRectangle(0, 0, width, height);
			ofDrawLine(0, height / 2, width, height / 2);
			float offset_y = 7;
			ofDrawBitmapStringHighlight(ofToString(max), -20, 0 + offset_y);
			ofDrawBitmapStringHighlight(ofToString((max + min) / 2), -20, height / 2 + offset_y);
			ofDrawBitmapStringHighlight(ofToString(min), -23, height + offset_y);

			// Draw Raw Data
			ofSetLineWidth(5);
			ofNoFill();
			ofSetColor(color_0, 80);
			float step = width / resolution;
			ofBeginShape();
			for (int i = 0; i < data.size(); i++) {
				float x = i * step;
				float y = ofMap(data[i][0], min, max, 0, height);
				ofVertex(x, y);
			}
			ofEndShape();

			ofSetColor(color_1, 80);
			ofBeginShape();
			for (int i = 0; i < data.size(); i++) {
				float x = i * step;
				float y = ofMap(data[i][1], min, max, 0, height);
				ofVertex(x, y);
			}
			ofEndShape();

			// Draw Filtered Data
			ofSetLineWidth(2);
			ofSetColor(ofColor::yellow);
			ofBeginShape();
			for (int i = 0; i < data_filtered.size(); i++) {
				float x = i * step;
				float y = ofMap(data_filtered[i][0], min, max, 0, height);
				ofVertex(x, y);
			}
			ofEndShape();

			ofSetColor(ofColor::cyan);
			ofBeginShape();
			for (int i = 0; i < data_filtered.size(); i++) {
				float x = i * step;
				float y = ofMap(data_filtered[i][1], min, max, 0, height);
				ofVertex(x, y);
			}
			ofEndShape();

			ofPopStyle();
		}

		void reset() {
			data.clear();
			data_filtered.clear();
		}
	};
	vector<TimeSeriesPlot> plots_rpm;




	// GUI Listeners
	void on_enable(bool& val);
	void on_e_stop(bool& val);
	void on_run_homing();
	void on_run_shutdown();
	void on_bounds_changed(float& val);
	void on_torque_limits_changed(float& val);
	void on_vel_limit_changed(float& val);
	void on_accel_limit_changed(float& val);

	void on_ee_offset_changed(float& val);
	void on_base_offset_changed(float& val);

	void on_move_to_changed(glm::vec2& val);
	void on_move_to_pos();
	void on_move_to_vel(bool& val);
	void on_zone_changed(float& val);

	ofxPanel panel;
	ofParameterGroup params_control;
	ofParameter<string> status;
	ofParameter<bool> enable;
	ofParameter<bool> e_stop;
	ofParameter<void> btn_run_homing;
	ofParameter<void> btn_run_shutdown;

	ofParameterGroup params_limits;
	ofParameter<float> vel_limit;
	ofParameter<float> accel_limit;
	ofParameter<float> bounds_min;
	ofParameter<float> bounds_max;
	ofParameter<float> torque_min;
	ofParameter<float> torque_max;

	ofParameterGroup params_kinematics;
	ofParameter<float> base_offset;
	ofParameter<float> ee_offset;
	ofParameter<float> x_offset_max;
	void on_x_offset_max_changed(float& val);

	ofParameterGroup params_motion;
	ofParameter<float> accel_rate;
	ofParameter<float> decel_radius;
	ofParameter<float> zone;

	ofParameterGroup params_move;
	ofParameter<glm::vec2> move_to;
	ofParameter<void> move_to_pos;
	ofParameter<bool> move_to_vel;

	ofColor mode_color_enabled;
	ofColor mode_color_disabled;
	ofColor mode_color_not_homed;
	ofColor mode_color_estopped;

	double frequency = 60; // Hz
	double mincutoff = 1.0; // Hz
	double beta = 0.1;
	double dcutoff = 1.0;
	OneEuroFilter filter_0;
	OneEuroFilter filter_1;

	// TEMP PID Controller
	float PID_error = 0;
	float previous_error = 0;
	float elapsedTime, Time, timePrev;
	int PID_value = 0;
	//PID constants
	int kp = 10.0;   int ki = 0.3;   int kd = 1000.5;
	int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
};