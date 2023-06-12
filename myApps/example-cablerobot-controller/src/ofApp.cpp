#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	ofLogToConsole();

	int count = 2;
	int offset_x = 450; // mm
	vector<glm::vec3> positions;
	for (int i = 0; i < count; i++) {
		positions.push_back(glm::vec3(offset_x * i, 0, 0));
	}
	robots = new RobotController(positions);

}

//--------------------------------------------------------------
void ofApp::update() {

}

void ofApp::draw()
{
	robots->draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	// pass the key pressed to the robot controller
	robots->key_pressed(key);


	switch (key)
	{
	case 'f':
	case 'F': {
		ofToggleFullscreen();
		break;
	}
	default:
		break;
	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
	//reset_osc_controller();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
