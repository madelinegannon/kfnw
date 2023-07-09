#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOsc.h"
#include "ofxGizmo.h"

#include "controllers/robot/RobotController.h"
#include "controllers/motion/MotionController.h"

#define DEBUG


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

	RobotController* robots;
	MotionController* motion;
	ofNode origin;
	ofEasyCam cam;


	void setup_camera();
	void key_pressed_camera(int key);
	void disable_camera(bool val);
	//void key_pressed_gizmo(int key);

	void on_set_camera_view(glm::vec3 position, glm::vec3 target, float distance=2250);
	void on_print_camera_view();

	glm::vec3 camera_top = glm::vec3(800, -1000, 2400);
	glm::vec3 camera_side = glm::vec3(0, 0, -10000);
	glm::vec3 camera_front = glm::vec3(4000, 2000, -2000);
	glm::vec3 camera_perspective = glm::vec3(5000, -3000, -5000);

	glm::vec3 camera_target = glm::vec3(800, -1000, 0);

	ofColor background_inner = ofColor(238);
	ofColor background_outer = ofColor(118);
};
