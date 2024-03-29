#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOsc.h"
#include "ofxGizmo.h"
#include "CommsSensor.h"

#include "controllers/robot/RobotController.h"
#include "controllers/motion/MotionController.h"
#include "controllers/agent/AgentController.h"

#define DEBUG


class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void key_pressed_gizmo(int key);
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

	RobotController* robots;
	MotionController* motion;
	AgentController* agents;

	ofNode origin;
	ofEasyCam cam;


	CommsSensor sensor_comms;

	void setup_comms();
	ofxOscReceiver osc_receiver;
	void check_for_messages();
	void check_for_messages(ofxOscReceiver* receiver);

	ofxOscReceiver osc_receiver_skeleton;
	int port_skeleton = 12345;

	void setup_camera();
	void key_pressed_camera(int key);
	void disable_camera(bool val);
	//void key_pressed_gizmo(int key);

	void on_set_camera_view(glm::vec3 position, glm::vec3 target, float distance = 2250);
	void on_print_camera_view();

	glm::vec3 camera_top = glm::vec3(0, 0, 0);
	glm::vec3 camera_side = glm::vec3(1000, -1000, 0);
	glm::vec3 camera_front = glm::vec3(0, -1000, 2000);
	glm::vec3 camera_perspective = glm::vec3(5000, -3000, 5000);

	glm::vec3 camera_target = glm::vec3(0, -1000, 0);

	ofColor background_inner = ofColor(238);
	ofColor background_outer = ofColor(118);

	void setup_gui();

	ofxPanel panel;
	ofParameterGroup params;
	ofParameter<int> osc_port_listening = 55555;
	ofParameter<void> osc_connect;
	ofParameter<string> osc_status;

	ofParameterGroup params_zones;
	ofParameterGroup params_zone_sensor;
	ofParameter<glm::vec3> zone_pos;
	ofParameter<float> zone_width = 3000;
	ofParameter<float> zone_height = 1000;

	ofParameterGroup params_zone_drawing;
	ofParameter<glm::vec3> zone_drawing_pos;
	ofParameter<float> zone_drawing_width = 3000;
	ofParameter<float> zone_drawing_height = 1000;
	ofParameter<int> zone_drawing_length = 10;
	ofParameter<float> zone_drawing_accuracy = 25; // zone (in mm) to move to next  point
	ofParameter<bool> zone_drawing_follow = true;
	ofParameter<float> zone_drawing_follow_offset = 50;


	// New World Params 11/16/2023 
	// Motors hung at 19'2"
	bool use_nws_params = false;
	glm::vec3 nws_zone_drawing_pos = glm::vec3(0, -4000, 0);
	float nws_zone_drawing_height = 1500;
	float nws_zone_drawing_width = 3000;
	glm::vec3 nws_motion_line_pos_min = glm::vec3(-2000, -5250, 0);



	void on_zone_pos_changed(glm::vec3& val);
	void on_zone_width_changed(float& val);
	void on_zone_height_changed(float& val);

	void on_zone_drawing_pos_changed(glm::vec3& val);
	void on_zone_drawing_width_changed(float& val);
	void on_zone_drawing_height_changed(float& val);

	void on_osc_connect();

	void setup_sensors();
	ofxGizmo gizmo_sensor;
	void update_gizmos();
	void draw_zones();

	ofRectangle zone;
	ofRectangle zone_drawing;

	ofNode sensor;
	vector<ofNode*> skeleton;
	void draw_skeleton(vector<ofNode*> joints);
	void update_sensor_path();
	void draw_sensor_path();
	ofPolyline path_sensor;

	ofPolyline path_drawing;
	vector<ofPolyline*> drawing_paths;
	void update_drawing_path(ofPolyline* path, glm::vec3 pt);
	void update_path(ofPolyline* path, glm::vec3 pt);
	void draw_path(ofPolyline* path);

	enum k4abt_joint_names {
		K4ABT_JOINT_PELVIS,
		K4ABT_JOINT_SPINE_NAVEL,
		K4ABT_JOINT_SPINE_CHEST,
		K4ABT_JOINT_NECK,
		K4ABT_JOINT_CLAVICLE_LEFT,
		K4ABT_JOINT_SHOULDER_LEFT,
		K4ABT_JOINT_ELBOW_LEFT,
		K4ABT_JOINT_WRIST_LEFT,
		K4ABT_JOINT_HAND_LEFT,
		K4ABT_JOINT_HANDTIP_LEFT,
		K4ABT_JOINT_THUMB_LEFT,
		K4ABT_JOINT_CLAVICLE_RIGHT,
		K4ABT_JOINT_SHOULDER_RIGHT,
		K4ABT_JOINT_ELBOW_RIGHT,
		K4ABT_JOINT_WRIST_RIGHT,
		K4ABT_JOINT_HAND_RIGHT,
		K4ABT_JOINT_HANDTIP_RIGHT,
		K4ABT_JOINT_THUMB_RIGHT,
		K4ABT_JOINT_HIP_LEFT,
		K4ABT_JOINT_KNEE_LEFT,
		K4ABT_JOINT_ANKLE_LEFT,
		K4ABT_JOINT_FOOT_LEFT,
		K4ABT_JOINT_HIP_RIGHT,
		K4ABT_JOINT_KNEE_RIGHT,
		K4ABT_JOINT_ANKLE_RIGHT,
		K4ABT_JOINT_FOOT_RIGHT,
		K4ABT_JOINT_HEAD,
		K4ABT_JOINT_NOSE,
		K4ABT_JOINT_EYE_LEFT,
		K4ABT_JOINT_EAR_LEFT,
		K4ABT_JOINT_EYE_RIGHT,
		K4ABT_JOINT_EAR_RIGHT
	};

	/*string k4abt_joint_names[32] = {
		"K4ABT_JOINT_PELVIS",
		"K4ABT_JOINT_SPINE_NAVEL",
		"K4ABT_JOINT_SPINE_CHEST",
		"K4ABT_JOINT_NECK",
		"K4ABT_JOINT_CLAVICLE_LEFT",
		"K4ABT_JOINT_SHOULDER_LEFT",
		"K4ABT_JOINT_ELBOW_LEFT",
		"K4ABT_JOINT_WRIST_LEFT",
		"K4ABT_JOINT_HAND_LEFT",
		"K4ABT_JOINT_HANDTIP_LEFT",
		"K4ABT_JOINT_THUMB_LEFT",
		"K4ABT_JOINT_CLAVICLE_RIGHT",
		"K4ABT_JOINT_SHOULDER_RIGHT",
		"K4ABT_JOINT_ELBOW_RIGHT",
		"K4ABT_JOINT_WRIST_RIGHT",
		"K4ABT_JOINT_HAND_RIGHT",
		"K4ABT_JOINT_HANDTIP_RIGHT",
		"K4ABT_JOINT_THUMB_RIGHT",
		"K4ABT_JOINT_HIP_LEFT",
		"K4ABT_JOINT_KNEE_LEFT",
		"K4ABT_JOINT_ANKLE_LEFT",
		"K4ABT_JOINT_FOOT_LEFT",
		"K4ABT_JOINT_HIP_RIGHT",
		"K4ABT_JOINT_KNEE_RIGHT",
		"K4ABT_JOINT_ANKLE_RIGHT",
		"K4ABT_JOINT_FOOT_RIGHT",
		"K4ABT_JOINT_HEAD",
		"K4ABT_JOINT_NOSE",
		"K4ABT_JOINT_EYE_LEFT",
		"K4ABT_JOINT_EAR_LEFT",
		"K4ABT_JOINT_EYE_RIGHT",
		"K4ABT_JOINT_EAR_RIGHT"
	};*/
};
