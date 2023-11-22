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
	int offset = 6100;// 2750; // mm
	int offset_z = -140; //mm
	int num_cable_bots = 4;
	int num_motors = num_cable_bots * 2;
	int count = num_motors / num_cable_bots;
	vector<glm::vec3> positions;
	for (int j = 0; j < num_cable_bots; j++) {
		for (int i = 0; i < count; i++) {
			positions.push_back(glm::vec3(offset * i, 0, offset_z * j));
			//cout << "bases: " << ofToString(positions[i]);			
		}
		drawing_paths.push_back(new ofPolyline());
		//drawing_paths[i]->translate(glm::vec3(0, 0, offset_z * j));
	}

	// set the world coordinate system of the robots (flip to match screen coord axes)
	origin.rotateAroundDeg(180, glm::vec3(1, 0, 0), glm::vec3(0, 0, 0));
	origin.setGlobalPosition(-1 * (positions[0].x + positions[1].x) / 2.0, 0, 0);
	robots = new RobotController(positions, &origin);
	motion = new MotionController(positions, &origin, offset_z);

	//agents = new AgentController();
	//agents->setup(num_cable_bots);
	//vector<ofNode*> tgts;
	//for (int i = 0; i < num_cable_bots; i++) {
	//	ofNode node;
	//	node.setGlobalPosition(0, -2500, 0);
	//	tgts.push_back(&node);
	//}
	//agents->set_targets(tgts);


	cam.setGlobalPosition(2436.1, -1399.14, 5507.93);
	cam.setGlobalOrientation(glm::quat(0.923639, -0.0197251, 0.382754, 0.00137394));


	ofSetFrameRate(60);

	// Nest the robot panel under the OSC panel
	robots->panel.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 250);

	//motion->panel.add(agents->params);

	if (use_nws_params) {
		motion->pos.set(nws_zone_drawing_pos);
		motion->motion_pos.setMin(nws_motion_line_pos_min);
		motion->motion_pos.set(nws_zone_drawing_pos);
	}
}

//--------------------------------------------------------------
void ofApp::update() {
	update_gizmos();
	update_sensor_path();
	//update_drawing_path(&path_drawing, sensor_comms.get_incoming_pt());
	//check_for_messages(&osc_receiver_skeleton);

	skeleton = sensor_comms.get_data();

	if (osc_status.get() == "CONNECTED") {
		check_for_messages();
	}

	// handle the master drawing all robots should follow first
	if (path_drawing.getVertices().size()) {// > zone_drawing_length.get()) {
		update_path(&path_drawing, path_drawing.getVertices().back());
	}
	// handle geometric and individual movements second
	else {
		//agents->set_targets(motion->get_targets());
		//agents->update();
		////agents->update(robots->get_targets()); // <-- NOT WORKING

		//if (motion->play.get()) {
		//	//auto agent_targets = agents->get_trail_targets();
		//	//if (agent_targets.size() > 0) {
		//	//	robots->set_targets(agent_targets);
		//	//}
		//	//else {
		//	//	robots->set_targets(motion->get_targets());
		//	//}
		//	robots->set_targets(motion->get_targets());
		//}
		////else {

		////	// send the robot targets to the sensor path
		////	if (path_sensor.getVertices().size() > 0) {
		////		vector<glm::vec3*> tgts;
		////		auto pt = path_sensor.getVertices().back();
		////		float offset_z = -140;
		////		tgts.push_back(new glm::vec3(pt.x, pt.y, 0));
		////		tgts.push_back(new glm::vec3(pt.x, pt.y, offset_z * 1));
		////		tgts.push_back(new glm::vec3(pt.x, pt.y, offset_z * 2));
		////		tgts.push_back(new glm::vec3(pt.x, pt.y, offset_z * 3));
		////		robots->set_targets(tgts);
		////	}
		////}

		// Testing moving drawing path into motion controller
		motion->update();
		if (motion->motion_drawing_follow) {
			motion->update_targets(robots->get_actual_positions());
			//if (robots->get_targets().size() == motion->get_targets().size()) {
			//	motion->update_targets(robots->get_actual_positions());
			//	//robots->set_targets(motion->get_targets());
			//}
			//else {
			//	//ofLogError(__FUNCTION__) << "Robots and MotionTargets do not match! There are " << robots->get_targets().size() << " robots and " << motion->get_targets().size() << " motion targets.";
			//}
		}
		robots->set_targets(motion->get_targets());
		//else if (!motion->motion_drawing_follow) {
		//	robots->set_targets(motion->get_targets());
		//}

	// disable tha camera if we are interacting with a gizmo
	// @NOTE 8/18/2023: this is doing it by itself for some reason
	//disable_camera(robots->disable_camera());
	}
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
	draw_path(&path_drawing);

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

	// draw the agents
	//agents->draw();

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
	case 'c':
		cout << "Camera pos: " << ofToString(cam.getGlobalPosition()) << ", orient: " << ofToString(cam.getGlobalOrientation()) << endl;

		break;
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
	params_zone_sensor.setName("Zone_Sensor");
	params_zone_sensor.add(zone_pos.set("Position", glm::vec3(0, 650, 3350), glm::vec3(-3000, -3000, 0), glm::vec3(3000, 3000, 3350)));
	params_zone_sensor.add(zone_width.set("Width", 3000, 0, 4000));
	params_zone_sensor.add(zone_height.set("Height", 1000, 0, 3000));

	params_zone_drawing.setName("Zone_Drawing");
	params_zone_drawing.add(zone_drawing_pos.set("Position", glm::vec3(0, -2750, 0), glm::vec3(-3000, -5000, 0), glm::vec3(3000, 0000, 0)));
	params_zone_drawing.add(zone_drawing_width.set("Width", 3000, 0, 5000));
	params_zone_drawing.add(zone_drawing_height.set("Height", 2500, 0, 6000));
	params_zone_drawing.add(zone_drawing_follow.set("Follow_the_Leader", true));
	params_zone_drawing.add(zone_drawing_length.set("Length", 20, 2, 100));
	params_zone_drawing.add(zone_drawing_follow_offset.set("Follow_Offset", 50, 0, 1000));
	params_zone_drawing.add(zone_drawing_accuracy.set("Accuracy", 50, 10, 100));

	//params_zones.add(params_zone_sensor);
	params_zones.add(params_zone_drawing);

	zone.setFromCenter(zone_pos.get(), zone_width.get(), zone_height.get());
	zone_pos.addListener(this, &ofApp::on_zone_pos_changed);
	zone_width.addListener(this, &ofApp::on_zone_width_changed);
	zone_height.addListener(this, &ofApp::on_zone_height_changed);

	zone_drawing.setFromCenter(zone_drawing_pos.get(), zone_drawing_width.get(), zone_drawing_height.get());
	zone_drawing_pos.addListener(this, &ofApp::on_zone_drawing_pos_changed);
	zone_drawing_width.addListener(this, &ofApp::on_zone_drawing_width_changed);
	zone_drawing_height.addListener(this, &ofApp::on_zone_drawing_height_changed);

	if (use_nws_params) {
		zone_drawing_pos.set(nws_zone_drawing_pos);
		zone_drawing_width.set(nws_zone_drawing_width);
		zone_drawing_height.set(nws_zone_drawing_height);
		zone_pos.set(glm::vec3(zone_pos.get().x, zone_drawing_pos.get().y - zone_drawing_height.get() / 2, zone_pos.get().z));
	}

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

void ofApp::on_zone_drawing_pos_changed(glm::vec3& val)
{
	zone_drawing.setFromCenter(val, zone_drawing_width.get(), zone_drawing_height.get());
}

void ofApp::on_zone_drawing_width_changed(float& val)
{
	zone_drawing.setFromCenter(zone_drawing_pos.get(), val, zone_drawing_height.get());
}

void ofApp::on_zone_drawing_height_changed(float& val)
{
	zone_drawing.setFromCenter(zone_drawing_pos.get(), zone_drawing_width.get(), val);
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

	ofSetColor(60);
	ofDrawRectangle(zone_drawing);

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
		if (path_sensor.getVertices().size() > 50)
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

void ofApp::update_drawing_path(ofPolyline* path, glm::vec3 pt)
{
	// map incoming point to drawing zone
	float bounds_x_min = zone_drawing.getTopLeft().x;
	float bounds_x_max = zone_drawing.getBottomRight().x;
	float bounds_y_min = zone_drawing.getBottomRight().y;
	float bounds_y_max = zone_drawing.getTopLeft().y;

	float x = ofMap(pt.x, 0, 1, bounds_x_min, bounds_x_max);
	float y = ofMap(pt.y, 0, 1, bounds_y_min, bounds_y_max);

	update_path(path, glm::vec3(x, y, 0));
}

void ofApp::update_path(ofPolyline* path, glm::vec3 pt)
{
	if (path->getVertices().size() == 0) {
		path->addVertex(pt);
	}
	else {
		// filter out small moves
		float dist_thresh = 20;
		float dist_sq = glm::distance2(path->getVertices().back(), pt);
		if (dist_sq > dist_thresh * dist_thresh) {
			path->addVertex(pt);

		}
	}
	// cap the length of the path
	if (path->getVertices().size() > motion->motion_drawing_length_max.get()) {
		path->removeVertex(0);
	}

	// move the robots
	if (path->getVertices().size() > 2) {	// POLYLINE MUST HAVE AT LEAST 3 POINTS, OTHERWISE SENDS TO (0,0,0)

		float dist_thresh = motion->motion_drawing_accuracy.get();// zone_drawing_accuracy.get();

		auto pt_0 = path->getPointAtPercent(0.0);
		auto pt_1 = robots->get_target(0);// path_drawing.getPointAtPercent(0.33);
		auto pt_2 = robots->get_target(1);// path_drawing.getPointAtPercent(0.66);
		auto pt_3 = robots->get_target(2);// path_drawing.getPointAtPercent(1.0);
		/*auto pt_1 = path_drawing.getPointAtPercent(0.33);
		auto pt_2 = path_drawing.getPointAtPercent(0.66);
		auto pt_3 = path_drawing.getPointAtPercent(1.0);*/

		//cout << "robots->get_target(0): " << robots->get_target(0) << ", pt_0: " << pt_0 << endl;
		auto mid_pt = robots->get_target(3);
		float dist_sq = glm::distance2(pt_0, glm::vec3(mid_pt.x, mid_pt.y, 0));
		if (dist_sq < dist_thresh * dist_thresh) {
			path->removeVertex(0);
			pt_0 = path_drawing.getPointAtPercent(0.0);



			//pt_1 = robots->get_target(0);// path_drawing.getPointAtPercent(0.33);
			//pt_2 = robots->get_target(1);// path_drawing.getPointAtPercent(0.66);
			//pt_3 = robots->get_target(2);// path_drawing.getPointAtPercent(1.0);
		}

		robots->set_target(3, pt_0.x, pt_0.y);
		// follow the leader
		if (zone_drawing_follow.get()) {
			float offset = zone_drawing_follow_offset.get(); //300;

			// send them to the same point
			if (offset < 50) {
				robots->set_target(2, pt_0.x, pt_0.y);
				robots->set_target(1, pt_0.x, pt_0.y);
				robots->set_target(0, pt_0.x, pt_0.y);
			}
			// follow the leader with the given offset
			else {
				auto heading = robots->get_target(3) - pt_0;
				auto p = glm::normalize(heading) * offset;
				p += robots->get_target(3);
				robots->set_target(2, p.x, p.y);

				heading = robots->get_target(2) - p;
				p = glm::normalize(heading) * offset;
				p += robots->get_target(2);
				robots->set_target(1, p.x, p.y);

				heading = robots->get_target(1) - p;
				p = glm::normalize(heading) * offset;
				p += robots->get_target(1);
				robots->set_target(0, p.x, p.y);
			}
		}

	}
}

void ofApp::draw_path(ofPolyline* path)
{
	ofPushStyle();
	ofNoFill();
	ofSetLineWidth(2);
	ofSetColor(ofColor::magenta);
	path->draw();
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

		float bounds_x_min = zone_drawing.getTopLeft().x;
		float bounds_x_max = zone_drawing.getBottomRight().x;
		float bounds_y_min = zone_drawing.getBottomRight().y;
		float bounds_y_max = zone_drawing.getTopLeft().y;

		if (m.getAddress() == "/stop") {
			// stop all the cablebots (same as pressing SPACEBAR)
			robots->pause();
		}
		else if (m.getAddress() == "/move") {
			// turn on move_vel for all the cablebots
			robots->move_vel_all(m.getArgAsBool(0));
		}
		// We received a normalized XY target in range {[0,1], [0,1]}
		// Move all the Robots
		else if (m.getAddress() == "/drawing/tgt_norm") {
			//int i = m.getArgAsInt(0);
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			//cout << "x: " << x << ", y: " << y << endl;
			update_path(&path_drawing, glm::vec3(x, y, 0));	// does "follow the leader"

			//motion->add_to_path(0, glm::vec3(x, y, 0));
			//motion->add_to_path(1, glm::vec3(x, y, 0));
			//motion->add_to_path(2, glm::vec3(x, y, 0));
			//motion->add_to_path(3, glm::vec3(x, y, 0));

		}
		// Add to Robot 0 Path
		else if (m.getAddress() == "/drawing/0/tgt_norm") {
			//cout << "move robot 0" << endl;
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			motion->add_to_path(0, glm::vec3(x, y, 0));
			//update_path(&path_drawing, glm::vec3(x, y, 0));
		}
		// Add to Robot 1 Path
		else if (m.getAddress() == "/drawing/1/tgt_norm") {
			//cout << "move robot 1" << endl;
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			motion->add_to_path(1, glm::vec3(x, y, 0));

			//update_path(&path_drawing, glm::vec3(x, y, 0));
		}
		// Add to Robot 2 Path
		else if (m.getAddress() == "/drawing/2/tgt_norm") {
			//cout << "move robot 2" << endl;
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			motion->add_to_path(2, glm::vec3(x, y, 0));

			//update_path(&path_drawing, glm::vec3(x, y, 0));
		}
		// Add to Robot 3 Path
		else if (m.getAddress() == "/drawing/3/tgt_norm") {
			//cout << "move robot 3" << endl;
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			motion->add_to_path(3, glm::vec3(x, y, 0));

			//update_path(&path_drawing, glm::vec3(x, y, 0));
		}
		else if (m.getAddress() == "/drawing/clear") {
			path_drawing.clear();
			for (auto path : drawing_paths)
				path->clear();
			motion->clear_paths();
		}
		else if (m.getAddress() == "/drawing/enable_follow") {
			zone_drawing_follow.set(m.getArgAsBool(0));
			motion->motion_drawing_follow.set(m.getArgAsBool(0));
		}
		else if (m.getAddress() == "/drawing/accuracy") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_drawing_accuracy.getMin(), motion->motion_drawing_accuracy.getMax());
			zone_drawing_accuracy.set(val);
			motion->motion_drawing_accuracy.set(val);
		}
		else if (m.getAddress() == "/drawing/num_pts") {
			//float val =; // normalized value between 0 and 1
			int val = int(ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_drawing_length_max.getMin(), motion->motion_drawing_length_max.getMax(), true));
			zone_drawing_length.set(val);
			motion->motion_drawing_length_max.set(val);
		}
		else if (m.getAddress() == "/drawing/follow_offset") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_drawing_offset.getMin(), motion->motion_drawing_offset.getMax());
			zone_drawing_follow_offset.set(val);
			motion->motion_drawing_offset.set(val);
		}
		else if (m.getAddress() == "/line/enable_follow") {
			bool val = m.getArgAsBool(0);
			if (val) {
				// clear any drawing paths
				path_drawing.clear();
				for (auto path : drawing_paths)
					path->clear();
				motion->clear_paths();
			}
			motion->motion_line_follow.set(val);
		}
		else if (m.getAddress() == "/line/length") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_line_length.getMin(), motion->motion_line_length.getMax());
			motion->motion_line_length.set(val);
		}
		else if (m.getAddress() == "/line/reset") {
			motion->on_motion_reset();
		}
		else if (m.getAddress() == "/line/theta") {
			float val = ofMap(m.getArgAsFloat(0), -1, 1, motion->motion_theta.getMin(), motion->motion_theta.getMax());
			motion->motion_theta.set(val);
		}
		else if (m.getAddress() == "/line/spin") {
			motion->motion_spin_enable.set(m.getArgAsBool(0));
		}
		else if (m.getAddress() == "/line/spin_speed") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_spin_speed.getMin(), motion->motion_spin_speed.getMax());
			motion->motion_spin_speed.set(val);
		}
		else if (m.getAddress() == "/line/position") {
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			motion->motion_pos.set(glm::vec3(x, y, 0));
			//motion->centroid.setGlobalPosition(motion->motion_pos);
		}
		else if (m.getAddress() == "/line/start") {
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			motion->motion_line.getVertices()[0].x = x;
			motion->motion_line.getVertices()[0].y = y;
			//motion->calculate_theta(motion->motion_line);
			motion->centroid.setGlobalPosition((motion->motion_line.getVertices()[0] + motion->motion_line.getVertices()[1]) / 2);

			motion->motion_pos_prev = motion->centroid.getGlobalPosition();
			motion->motion_pos.set(motion->centroid.getGlobalPosition());

			//motion->motion_pos.set(motion->motion_line.getCentroid2D());	// <-- not working
		}
		else if (m.getAddress() == "/line/end") {
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			motion->motion_line.getVertices()[1].x = x;
			motion->motion_line.getVertices()[1].y = y;
			//motion->calculate_theta(motion->motion_line);

			motion->centroid.setGlobalPosition((motion->motion_line.getVertices()[0] + motion->motion_line.getVertices()[1]) / 2);
			motion->motion_pos_prev = motion->centroid.getGlobalPosition();
			motion->motion_pos.set(motion->centroid.getGlobalPosition());

			//motion->motion_pos.set(motion->motion_line.getCentroid2D());	// <-- not working
		}
		else if (m.getAddress() == "/circle/enable_follow") {
			bool val = m.getArgAsBool(0);
			if (val) {
				// clear any drawing paths
				path_drawing.clear();
				for (auto path : drawing_paths)
					path->clear();
				motion->clear_paths();
			}
			motion->motion_circle_follow.set(val);
		}
		else if (m.getAddress() == "/circle/radius") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_circle_radius.getMin(), motion->motion_circle_radius.getMax());
			motion->motion_circle_radius.set(val);
		}
		else if (m.getAddress() == "/circle/theta") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_theta.getMin(), motion->motion_theta.getMax());
			motion->motion_theta.set(val);
		}
		else if (m.getAddress() == "/circle/position") {
			float x = m.getArgAsFloat(0);
			float y = m.getArgAsFloat(1);

			x = ofMap(x, 0, 1, zone_drawing.getMinX(), zone_drawing.getMaxX());
			y = ofMap(y, 0, 1, zone_drawing.getMaxY(), zone_drawing.getMinY());

			//x = ofMap(x, 0, 1, bounds_x_min, bounds_x_max);
			//y = ofMap(y, 0, 1, bounds_y_min, bounds_y_max);

			motion->motion_pos.set(glm::vec3(x, y, 0));
		}
		else if (m.getAddress() == "/circle/spin") {
			motion->motion_spin_enable.set(m.getArgAsBool(0));
		}
		else if (m.getAddress() == "/circle/spin_speed") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_spin_speed.getMin(), motion->motion_spin_speed.getMax());
			motion->motion_spin_speed.set(val);
		}
		else if (m.getAddress() == "/circle/reset") {
			motion->on_motion_reset();
		}
		else if (m.getAddress() == "/circle/arc_angle") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->motion_circle_angle_start.getMin(), motion->motion_circle_angle_start.getMax());
			float theta = 90 + val / 2.0;
			motion->motion_circle_angle_end.set(theta);
			theta = 90 - val / 2.0;
			motion->motion_circle_angle_start.set(theta);
		}
		else if (m.getAddress() == "/enable_sine") {
			motion->enable_sine_wave.set(m.getArgAsBool(0));
		}
		else if (m.getAddress() == "/sine_speed") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->sine_wave_speed.getMin(), motion->sine_wave_speed.getMax());
			motion->sine_wave_speed.set(val);
		}
		else if (m.getAddress() == "/sine_amp") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->sine_wave_amplitude.getMin(), motion->sine_wave_amplitude.getMax());
			motion->sine_wave_amplitude.set(val);
		}
		else if (m.getAddress() == "/enable_pendulum") {
			motion->enable_pendulum.set(m.getArgAsBool(0));
		}
		else if (m.getAddress() == "/pendulum_speed") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->pendulum_speed.getMin(), motion->pendulum_speed.getMax());
			motion->pendulum_speed.set(val);
		}
		else if (m.getAddress() == "/pendulum_offset") {
			float val = ofMap(m.getArgAsFloat(0), 0, 1, motion->pendulum_offset.getMin(), motion->pendulum_offset.getMax());
			motion->pendulum_offset.set(val);
		}

		// We received an absolute XY target in range {[0,0], [bounds.min,bounds.max]}
		//else if (m.getAddress() == "/tgt_abs") {
		//	//int i = m.getArgAsInt(0);
		//	float x = m.getArgAsFloat(0);
		//	float y = m.getArgAsFloat(1);

		//	// just check the first set of robots
		//	//if (i == 0) {
		//	robots->set_target(0, x, y);
		//	//}
		//}
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
		//Camera pos: -1152.07, -1986.03, 4307.33, orient: 1, 0, 0, 0
		cam.setGlobalPosition(-1152.07, -1986.03, 4307.33);// -1010.88, -1967.36, 3923.83);
		cam.setGlobalOrientation(glm::quat(1, 0, 0, 0));
		//on_set_camera_view(camera_front, camera_target);
		break;
	case '4':
		// PERSPECTIVE		
		cam.setGlobalPosition(2436.1, -1399.14, 5507.93);
		cam.setGlobalOrientation(glm::quat(0.923639, -0.0197251, 0.382754, 0.00137394));
		//on_set_camera_view(camera_perspective, camera_target);
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
