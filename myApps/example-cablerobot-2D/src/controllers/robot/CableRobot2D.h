#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGizmo.h"
#include "ofxXmlSettings.h"
#include "CableRobot.h"


class CableRobot2D 
{
private:

	int id;
	vector<CableRobot*> robots;

	ofNode* origin;      // World reference frame 
	ofNode ee = ofNode();
	ofxGizmo gizmo_ee;
	
	ofRectangle bounds;

	void setup_gui();
	string state_names[5] = { "NOT_HOMED", "HOMING", "ENABLED", "DISABLED", "E_STOP" };

	void check_status();
   
	bool debugging = true;
public:

	//CableRobot2D(vector<CableRobot> robots);
	CableRobot2D() {};
	CableRobot2D(CableRobot* top_left, CableRobot* top_right, ofNode* _origin, int id);

	void setup();
	void update();
	void draw();
	void update_gui(ofxPanel* _panel);
	void draw_gui();
	void shutdown();


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




	// GUI Listeners
	void on_enable(bool& val);
	void on_e_stop(bool& val);
	void on_run_homing();
	void on_run_shutdown();
	void on_bounds_changed(float& val);
	void on_vel_limit_changed(float& val);
	void on_accel_limit_changed(float& val);
	void on_accel_rate_changed(float& val);
	void on_decel_radius_changed(float& val);
	void on_base_offset_changed(float& val);

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

	ofParameterGroup params_motion;
	ofParameter<float> accel_rate;
	ofParameter<float> decel_radius;

	ofParameterGroup params_kinematics;
	ofParameter<float> base_offset;
	ofParameter<float> ee_offset;

	ofColor mode_color_enabled;
	ofColor mode_color_disabled;
	ofColor mode_color_not_homed;
	ofColor mode_color_estopped;
};