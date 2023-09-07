#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	ofLogToConsole();
	ofSetCircleResolution(60);

	setup_gui();
	setup_comms();
	setup_sensors();

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
	origin.setGlobalPosition(-1 * (positions[0].x + positions[1].x) / 2.0, 0, 0);
	robots = new RobotController(positions, &origin);
	motion = new MotionController(positions, &origin, offset_z);


	on_set_camera_view(camera_top, camera_target, 2250);
	ofSetFrameRate(60);

	// Nest the robot panel under the OSC panel
	robots->panel.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 5);
}

//--------------------------------------------------------------
void ofApp::update() {
	update_gizmos();
	update_sensor_path();
	//check_for_messages(&osc_receiver_skeleton);

	skeleton = sensor_comms.get_data();

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

	// draw the floor plane
	ofSetColor(200, 180);
	ofPushMatrix();
	ofRotateX(90);
	ofTranslate(0, 2800 / 2 + -140 * 3, 3350);
	ofDrawPlane(4000, 2800);
	ofPopMatrix();

	// draw the zones
	draw_zones();
	draw_sensor_path();

	gizmo_sensor.draw(cam);

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

	draw_skeleton(skeleton);

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
	key_pressed_gizmo(key);
}


void ofApp::key_pressed_gizmo(int key)
{
	switch (key)
	{
	case 'e':
	case 'E':
		gizmo_sensor.setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_ROTATE);
		break;
	case 'w':
	case 'W':

		gizmo_sensor.setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_MOVE);

		break;
	case 'r':
	case 'R':

		gizmo_sensor.setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_SCALE);
		break;

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

	params_zones.setName("Zone_Params");
	params_zones.add(zone_pos.set("Position", glm::vec3(0, 650, 3350), glm::vec3(-3000, -3000, 0), glm::vec3(3000, 3000, 3350)));
	params_zones.add(zone_width.set("Width", 3000, 0, 4000));
	params_zones.add(zone_height.set("Height", 1000, 0, 3000));


	zone.setFromCenter(zone_pos.get(), zone_width.get(), zone_height.get());


	zone_pos.addListener(this, &ofApp::on_zone_pos_changed);
	zone_width.addListener(this, &ofApp::on_zone_width_changed);
	zone_height.addListener(this, &ofApp::on_zone_height_changed);

	panel.add(params);
	panel.add(params_zones);

	osc_connect.addListener(this, &ofApp::on_osc_connect);
}

void ofApp::on_zone_pos_changed(glm::vec3& val)
{
	if (val.z != zone.getPosition().z) {

	}
	zone.setFromCenter(val, zone_width.get(), zone_height.get());
}

void ofApp::on_zone_width_changed(float& val)
{
	zone.setFromCenter(zone_pos.get(), val, zone_height.get());
}

void ofApp::on_zone_height_changed(float& val)
{
	zone.setFromCenter(zone_pos.get(), zone_width.get(), val);
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

void ofApp::setup_sensors()
{
	sensor.setGlobalPosition(0, -3350, -140 * 3);
	sensor.setGlobalOrientation(glm::quat(0.0419488, 0.00833346, 0.194672, 0.979936));	// Calibrated 09/07/2023 @ Oolite
	gizmo_sensor.setNode(sensor);
	gizmo_sensor.setDisplayScale(.33);
	for (int i = 0; i < 32; i++) {
		skeleton.push_back(new ofNode());
		skeleton.back()->setParent(sensor);
	}
	sensor_comms.setup(skeleton, port_skeleton);
}

void ofApp::update_gizmos()
{
	// update the sensor node to match the gizmo
	if (gizmo_sensor.isInteracting()) {
		cout << "[[ " << gizmo_sensor.getTranslation() << " ], [ " << gizmo_sensor.getRotation() << " ]]" << endl;
		sensor.setGlobalPosition(gizmo_sensor.getTranslation());
		sensor.setGlobalOrientation(gizmo_sensor.getRotation());
	}
}

void ofApp::draw_zones()
{
	ofPushStyle();
	ofPushMatrix();
	//ofTranslate(zone.getPosition());
	ofNoFill();
	ofSetLineWidth(3);
	auto pelvis = skeleton[K4ABT_JOINT_PELVIS]->getGlobalPosition();
	if (zone.inside(pelvis.x, pelvis.z)) {
		ofSetColor(ofColor::cyan);
	}
	else {
		ofSetColor(60);
	}
	ofRotateX(90);
	ofTranslate(zone.getPosition());
	ofTranslate(0, 0, zone_pos.get().z);
	ofDrawRectangle(0, 0, zone.getWidth(), zone.getHeight());
	ofPopMatrix();
	ofPopStyle();
}

void ofApp::draw_skeleton(vector<ofNode*> joints)
{
	joints = sensor_comms.get_data();
	ofPushStyle();
	int K4ABT_JOINT_COUNT = 32;
	ofNoFill();
	ofSetColor(250);
	// Draw joints 
	for (int i = 0; i < K4ABT_JOINT_COUNT; ++i)
	{
		ofDrawBox(joints[i]->getGlobalPosition(), 15);
	}

	// Draw bones
	ofSetColor(ofColor::magenta);
	// Spine
	ofDrawLine(joints[K4ABT_JOINT_PELVIS]->getGlobalPosition(), joints[K4ABT_JOINT_SPINE_NAVEL]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_SPINE_CHEST]->getGlobalPosition(), joints[K4ABT_JOINT_SPINE_NAVEL]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_SPINE_CHEST]->getGlobalPosition(), joints[K4ABT_JOINT_NECK]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_HEAD]->getGlobalPosition(), joints[K4ABT_JOINT_NECK]->getGlobalPosition());

	// Head
	ofDrawLine(joints[K4ABT_JOINT_HEAD]->getGlobalPosition(), joints[K4ABT_JOINT_NOSE]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_EYE_LEFT]->getGlobalPosition(), joints[K4ABT_JOINT_NOSE]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_EYE_LEFT]->getGlobalPosition(), joints[K4ABT_JOINT_EYE_RIGHT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_EYE_RIGHT]->getGlobalPosition(), joints[K4ABT_JOINT_EAR_RIGHT]->getGlobalPosition());

	// Left Leg
	ofDrawLine(joints[K4ABT_JOINT_PELVIS]->getGlobalPosition(), joints[K4ABT_JOINT_HIP_LEFT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_KNEE_LEFT]->getGlobalPosition(), joints[K4ABT_JOINT_HIP_LEFT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_KNEE_LEFT]->getGlobalPosition(), joints[K4ABT_JOINT_ANKLE_LEFT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_FOOT_LEFT]->getGlobalPosition(), joints[K4ABT_JOINT_ANKLE_LEFT]->getGlobalPosition());

	// Right Leg
	ofDrawLine(joints[K4ABT_JOINT_PELVIS]->getGlobalPosition(), joints[K4ABT_JOINT_HIP_RIGHT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_KNEE_RIGHT]->getGlobalPosition(), joints[K4ABT_JOINT_HIP_RIGHT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_KNEE_RIGHT]->getGlobalPosition(), joints[K4ABT_JOINT_ANKLE_RIGHT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_FOOT_RIGHT]->getGlobalPosition(), joints[K4ABT_JOINT_ANKLE_RIGHT]->getGlobalPosition());

	// Left Arm
	ofDrawLine(joints[K4ABT_JOINT_NECK]->getGlobalPosition(), joints[K4ABT_JOINT_CLAVICLE_LEFT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_SHOULDER_LEFT]->getGlobalPosition(), joints[K4ABT_JOINT_CLAVICLE_LEFT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_SHOULDER_LEFT]->getGlobalPosition(), joints[K4ABT_JOINT_ELBOW_LEFT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_WRIST_LEFT]->getGlobalPosition(), joints[K4ABT_JOINT_ELBOW_LEFT]->getGlobalPosition());

	// Right Arm
	ofDrawLine(joints[K4ABT_JOINT_NECK]->getGlobalPosition(), joints[K4ABT_JOINT_CLAVICLE_RIGHT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_SHOULDER_RIGHT]->getGlobalPosition(), joints[K4ABT_JOINT_CLAVICLE_RIGHT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_SHOULDER_RIGHT]->getGlobalPosition(), joints[K4ABT_JOINT_ELBOW_RIGHT]->getGlobalPosition());
	ofDrawLine(joints[K4ABT_JOINT_WRIST_RIGHT]->getGlobalPosition(), joints[K4ABT_JOINT_ELBOW_RIGHT]->getGlobalPosition());
	ofPopStyle();
}

void ofApp::update_sensor_path()
{
	auto pt = skeleton[K4ABT_JOINT_PELVIS]->getGlobalPosition();
	if (zone.inside(pt.x, pt.z)) {
		auto hand_right = skeleton[K4ABT_JOINT_WRIST_RIGHT]->getGlobalPosition();
		auto hand_left = skeleton[K4ABT_JOINT_WRIST_LEFT]->getGlobalPosition();
		auto chest = skeleton[K4ABT_JOINT_SPINE_CHEST]->getGlobalPosition();

		if (hand_right.y > chest.y) {
			if (path_sensor.getVertices().size() == 0)
				path_sensor.addVertex(hand_right);
			else {
				// filter out small differences
				float dist_thresh = 10;
				float dist_sq = glm::distance2(path_sensor.getVertices().back(), hand_right);
				if (dist_sq > dist_thresh * dist_thresh) {
					path_sensor.addVertex(hand_right);
				}
			}
		}
		// remove that oldest point if we're over a certain size
		if (path_sensor.getVertices().size() > 100)
			path_sensor.removeVertex(0);
	}
	// clear the path if we stepped outside the zone
	else {
		if (path_sensor.getVertices().size() > 0)
			path_sensor.clear();
	}
}

void ofApp::draw_sensor_path()
{
	ofPushStyle();
	ofNoFill();
	ofSetColor(ofColor::cyan, 128);
	path_sensor.draw();
	ofPopStyle();
}

void ofApp::setup_comms()
{
	//ofxOscReceiverSettings settings;
	//settings.port = port_skeleton;
	//osc_receiver_skeleton.setup(settings);

}

void ofApp::check_for_messages(ofxOscReceiver* receiver)
{
	while (receiver->hasWaitingMessages()) {
		// get the next message
		ofxOscMessage m;
		receiver->getNextMessage(m);
		if (m.getAddress() == "/body") {
			for (int i = 0; i < 32 * 4; i += 4) {
				int id = m.getArgAsInt(i);
				float x = m.getArgAsFloat(i + 1);
				float y = m.getArgAsFloat(i + 2);
				float z = m.getArgAsFloat(i + 3);
				skeleton[id]->setPosition(x, y, z);
				//cout << id << ": {" << x << ", " << y << ", " << z << "}" << endl;
			}
			cout << endl;
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
