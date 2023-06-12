#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOsc.h"
//#include "pubSysCls.h"	
//#include "Axis.h"
#include "MotorController.h"

//using namespace sFnd;

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	MotorController* controller;

	// Sine Curve
	ofPolyline curve;
	float crv_width;
	int crv_res = 1000;
	glm::vec3 get_projected_point(ofPolyline crv, float x);
	void update_sine_curve();
	void draw_sine_curve();

	// OSC Controller
	void setup_osc_controller();
	void update_osc_controller();
	void draw_osc_controller();
	void reset_osc_controller();
	ofVec3f start, end;

	int port = 12345;
	ofxOscReceiver receiver;
	void checkForOSCMessage();
	int key_count = 0;

	// CPM Parameters
	//SysManager* myMgr;
	int numPorts;
	bool initialize();
	void close();

	//vector<Axis*> axes;
	//glm::vec3 origin;
	//glm::vec3 desired_pos;
	//glm::vec3 actual_pos;	
	//float actual_dist_mm;
	//float desired_dist_mm;
	//float desired_vel;
	//float pid_vel = 0;



	// GUI
	void setup_gui();
	ofxPanel panel;
	ofParameter<bool> showGUI;
	ofParameterGroup params_system;
	ofParameter<string> systemStatus;
	ofParameter<string> numHubs;

	ofParameterGroup params_motion_control;
	ofParameter<bool> play;
	ofParameter<bool> pause;
	ofParameter<float> vel_time_step;
	ofParameter<float> vel_max;

	ofParameterGroup params_hub;
	ofParameter<string> comPortNumber;
	ofParameter<string> numNodes;
	ofParameter<bool> enable_all;
	ofParameter<bool> eStopAll;

	ofParameterGroup params_motion;
	ofParameter<float> vel_all;
	ofParameter<float> accel_all;

	ofParameterGroup params_macros;
	ofParameter<bool> move_trigger_all;
	ofParameter<float> move_target_all;
	ofParameter<bool> move_absolute_pos_all;
	ofParameter<bool> move_zero_all;
	ofParameter<bool> home_all;   // <-- @TODO

	ofxPanel panel_curve;
	ofParameterGroup params_curve;
	ofParameter<bool> use_sine_curve;
	ofParameter<bool> use_osc_controller;
	ofParameter<float> crv_amp = 120.0;
	ofParameter<float> crv_speed = 0.1;
	ofParameter<float> crv_period = 500;
	ofParameter<float> crv_theta = 0;


	// GUI Listeners
	void on_play(bool& val);
	void on_pause(bool& val);

	//void on_enable_all(bool& val);
	//void on_eStop_all(bool& val);
	//void on_vel_all(float& val);
	//void on_accel_all(float& val);

	//void on_move_trigger_all(bool& val);
	//void on_move_absolute_pos_all(bool& val);
	//void on_move_target_all(float& val);
	//void on_move_zero_all(bool& val);

	void on_use_sine_curve(bool& val);
	void on_use_osc_controller(bool& val);


	ofColor mode_color_disabled;
	ofColor mode_color_eStop;
	ofColor mode_color_normal;
	ofColor mode_color_playing;


};
