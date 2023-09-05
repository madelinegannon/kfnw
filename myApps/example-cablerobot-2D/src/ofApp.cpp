#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	ofLogToConsole();
	ofSetCircleResolution(60);

	setup_gui();

	setup_camera();

	// set the postions of each motor
	int offset = 2500; // mm
	int offset_z = -140; //mm
	int num_cable_bots = 4;
	int num_motors = num_cable_bots * 2;
	int count = num_motors / num_cable_bots;
	vector<glm::vec3> positions;
	for (int j = 0; j < num_cable_bots; j++) {
		for (int i = 0; i < count; i++) {
			positions.push_back(glm::vec3(offset * i, 0, offset_z * j));
		}
	}

	// set the world coordinate system of the robots (flip to match screen coord axes)
	origin.rotateAroundDeg(180, glm::vec3(1, 0, 0), glm::vec3(0, 0, 0));
	origin.setGlobalPosition( -1 * (positions[0].x + positions[1].x) / 2.0, 0, 0);
	robots = new RobotController(positions, &origin);
	motion = new MotionController(positions, &origin, offset_z);


	on_set_camera_view(camera_top, camera_target, 2250);
	ofSetFrameRate(60);

	// Nest the robot panel under the OSC panel
	robots->panel.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 5);
}

//--------------------------------------------------------------
void ofApp::update() {

	if (osc_status.get() == "CONNECTED") {
		check_for_messages();
	}

	motion->update();

	if (motion->play.get())
		robots->set_targets(motion->get_targets());

	// disable tha camera if we are interacting with a gizmo
	// @NOTE 8/18/2023: this is doing it by itself for some reason
	//disable_camera(robots->disable_camera());
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

	// draw the motion controller
	motion->draw();

	// draw the robots
	robots->draw();
	// draw all the gizmos
	for (auto gizmo : robots->get_gizmos())
		gizmo->draw(cam);

	
	cam.end();

	// draw 2D
	panel.draw();
	robots->draw_gui();
	motion->draw_gui();

	ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate()), ofGetWidth() - 100, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	key_pressed_camera(key);

	// pass the key pressed to the motion & robot controller
	motion->keyPressed(key);
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
	if (robots)
		robots->windowResized(w, h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::setup_gui()
{
	panel.setup();
	panel.setName("CableBot_Server");
	panel.setPosition(10, 10);
	panel.setWidthElements(250);

	params.setName("OSC_Receiver");
	params.add(osc_port_listening.set("Listening_Port", 55555));
	params.add(osc_connect.set("CONNECT"));
	params.add(osc_status.set("Status", "DISCONNECTED"));

	panel.add(params);

	osc_connect.addListener(this, &ofApp::on_osc_connect);
}

void ofApp::on_osc_connect()
{
	if (osc_status.get() == "DISCONNECTED") {
		ofxOscReceiverSettings settings;
		settings.port = osc_port_listening.get();
		osc_receiver.setup(settings);
		osc_status.set("CONNECTED");
	}
	else {
		osc_receiver.stop();
		osc_status.set("DISCONNECTED");
	}	
}

void ofApp::check_for_messages()
{
	while (osc_receiver.hasWaitingMessages()) {
		// get the next message
		ofxOscMessage m;
		osc_receiver.getNextMessage(m);

		float bounds_x_min = -300;
		float bounds_x_max = 300;
		float bounds_y_min = -650;
		float bounds_y_max = -1250;

		// We received a normalized XY target in range {[0,1], [0,1]}
		if (m.getAddress() == "/tgt_norm") {
			int i = m.getArgAsInt(0);
			float x = m.getArgAsFloat(1);
			float y = m.getArgAsFloat(2);

			x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);
			
			//cout << "x: " << x << ", y: " << y << endl;

			robots->set_target(0, x, y);
		}
		else if (m.getAddress() == "/values/label18/gamepad/stick_left_x") {
			float x = stof(m.getArgAsString(0));
			x = ofMap(x, -1, 1, bounds_x_min, bounds_x_max);
			robots->set_target_x(0, x);
		}
		else if (m.getAddress() == "/values/label20/gamepad/stick_left_y") {
			float y = stof(m.getArgAsString(0));
			y = ofMap(y, -1, 1, bounds_y_min, bounds_y_max);
			robots->set_target_y(0, y);
		}
		// We received an absolute XY target in range {[0,0], [bounds.min,bounds.max]}
		else if (m.getAddress() == "/tgt_abs") {
			//int i = m.getArgAsInt(0);
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			// just check the first set of robots
			//if (i == 0) {
			robots->set_target(0, x, y);
			//}
		}
		else {
			// unrecognized message: display on the bottom of the screen
			string msgString;
			msgString = m.getAddress();
			msgString += ":";
			for (size_t i = 0; i < m.getNumArgs(); i++) {

				// get the argument type
				msgString += " ";
				msgString += m.getArgTypeName(i);
				msgString += ":";

				// display the argument - make sure we get the right type
				if (m.getArgType(i) == OFXOSC_TYPE_INT32) {
					msgString += ofToString(m.getArgAsInt32(i));
				}
				else if (m.getArgType(i) == OFXOSC_TYPE_FLOAT) {
					msgString += ofToString(m.getArgAsFloat(i));
				}
				else if (m.getArgType(i) == OFXOSC_TYPE_STRING) {
					msgString += m.getArgAsString(i);
				}
				else {
					msgString += "unhandled argument type " + m.getArgTypeName(i);
				}
			}
			cout << msgString << endl;
		}

	}
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
		on_set_camera_view(camera_top, camera_target, 2250);
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
