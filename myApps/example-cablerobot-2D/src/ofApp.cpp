#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	ofLogToConsole();
	ofSetCircleResolution(60);

	setup_camera();

	int count = 2;
	int offset = 914.4; // mm
	vector<glm::vec3> positions;
	for (int i = 0; i < count; i++) {
		positions.push_back(glm::vec3(offset * i, 0, 0));
	}

	// set the world coordinate system of the robots (flip to match screen coord axes)
	origin.rotateAroundDeg(180, glm::vec3(1, 0, 0), glm::vec3(0, 0, 0));
	origin.setGlobalPosition(500, 0, 0);
	robots = new RobotController(positions, &origin);

	

}

//--------------------------------------------------------------
void ofApp::update() {

	// disable tha camera if we are interacting with a gizmo
	disable_camera(robots->disable_camera());
}

void ofApp::draw()
{
	ofBackgroundGradient(background_inner, background_outer);
	cam.begin();

	// draw the axis and planes
	ofPushStyle();
	//ofDrawGridPlane(1000, 10, true);
	ofSetLineWidth(3);
	ofSetColor(ofColor::blue, 200);
	ofLine(glm::vec3(0, 0, 0), glm::vec3(0, 0, 10000));
	ofSetColor(ofColor::blue, 80);
	ofLine(glm::vec3(0, 0, 0), glm::vec3(0, 0, -10000));
	ofSetColor(ofColor::green, 200);
	ofLine(glm::vec3(0, 0, 0), glm::vec3(0, 10000, 0));
	ofSetColor(ofColor::green, 80);
	ofLine(glm::vec3(0, 0, 0), glm::vec3(0, -10000, 0));
	ofSetColor(ofColor::red, 200);
	ofLine(glm::vec3(0, 0, 0), glm::vec3(10000, 0, 0));
	ofSetColor(ofColor::red, 80);
	ofLine(glm::vec3(0, 0, 0), glm::vec3(-10000, 0, 0));
	ofPopStyle();


	// draw the robots
	for (auto gizmo : robots->get_gizmos())
		gizmo->draw(cam);
	robots->draw();

	
	cam.end();

	// draw 2D
	robots->draw_gui();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	key_pressed_camera(key);

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
	robots->windowResized(w, h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::setup_camera()
{
	cam.setUpAxis(glm::vec3(0, 0, 1));
	cam.setPosition(camera_top);
	cam.setTarget(camera_target);
	cam.setDistance(5000);
	cam.setFarClip(30000);
	cam.setNearClip(.0001);
}

void ofApp::key_pressed_camera(int key)
{
	switch (key)
	{
	case 'v':
		on_print_camera_view();
		break;
	case '1':
		// TOP
		on_set_camera_view(camera_top, camera_target, 3000);
		break;
	case '2': 
		// SIDE
		on_set_camera_view(camera_side, camera_target);
		break;
	case '3':
		// FRONT
		on_set_camera_view(camera_front, camera_target);
		break;
	case '4':
		// PERSPECTIVE
		on_set_camera_view(camera_perspective, camera_target);
		break;
	default:
		break;
	}
}

//void ofApp::key_pressed_gizmo(int key)
//{
//	switch (key)
//	{
//	case 'e':
//	case 'E':
//		gizmo_origin.setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_ROTATE);
//		break;
//	case 'w':
//	case 'W':
//		gizmo_origin.setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_MOVE);
//		break;
//	case '0':
//		// reset to the transform
//		gizmo_origin.setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_MOVE);
//		gizmo_origin.setMatrix(glm::mat4()); 
//		break;
//	default:
//		break;
//	}
//}

void ofApp::disable_camera(bool val)
{
	if (val) {
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
	}
	else {
		if (!cam.getMouseInputEnabled()) cam.enableMouseInput();
	}
}

void ofApp::on_set_camera_view(glm::vec3 position, glm::vec3 target, float distance)
{
	cam.setPosition(position);
	cam.setTarget(target);	
	cam.setDistance(distance);
}

void ofApp::on_print_camera_view()
{
	cout << "CAMERA VALUES:" << endl;
	cout << "\tpos:\t" << cam.getGlobalPosition() << endl;
	cout << "\ttarget:\t" << cam.getTarget().getGlobalPosition() << endl;
	cout << "\tdist:\t" << cam.getDistance() << endl;
	cout << "\tup axis:\t" << cam.getUpAxis() << endl;
	cout << "\tup dir:\t" << cam.getUpDir() << endl;
	cout << "\tnear clip:\t" << cam.getNearClip() << endl;
	cout << "\tfar clip:\t" << cam.getFarClip() << endl << endl;
}
