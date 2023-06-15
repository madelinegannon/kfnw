#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGizmo.h"
#include "CableRobot.h"


class CableRobot2D 
{
private:

	vector<CableRobot*> robots;

	ofNode* origin;      // World reference frame 
	ofNode ee = ofNode();
	ofxGizmo gizmo_ee;
	
	ofRectangle bounds;

   
public:

	CableRobot2D(vector<CableRobot> robots);

	void setup();
	void update();
	void draw();
	void draw_gui();
	void shutdown();

	void key_pressed(int key);
	void key_pressed_gizmo(int key);
	bool disable_camera();
	void windowResized(int w, int h);


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

	ofColor mode_color_disabled;
	ofColor mode_color_eStop;
	ofColor mode_color_normal;
	ofColor mode_color_playing;   
};