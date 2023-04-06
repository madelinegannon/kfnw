#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	ofLogToConsole();

	// create the initial curve
	crv_width = ofGetWidth();
	int step = crv_width / crv_res;
	for (int i = 0; i <= crv_width; i += step) {
		curve.addVertex(i, ofGetHeight() / 2, 0);
	}
	curve.setClosed(false);

	setup_gui();

	if (!initialize()) {
		close();
		exit();	// <-- not killing the program for some reason
	}

	// Add the Axis GUIs to the scene
	int x = panel.getPosition().x + panel.getWidth() + 15;
	int y = panel.getPosition().y;
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->panel.setPosition(x, y);
		x += axes[i]->panel.getWidth() + 5;
	}
}

//--------------------------------------------------------------
void ofApp::update() {
	for (int i = 0; i < axes.size(); i++) {

		// move the motor based on the sine curve
		if (use_sine_curve && play) {
			auto curr_pos_cnt = axes[i]->GetPosition();
			auto origin = axes[i]->position_world.get();
			auto projected = get_projected_point(curve, origin.x);
			auto dist = glm::distance(toGlm(origin), projected);
			axes[i]->move_target_mm.set(dist);
			//axes[i]->PosnMove(axes[i]->mm_to_count(dist), true);
			//axes[i]->WaitForMove(1000);
			// add a movement command if there's space in the buffer
			auto pos_cnt = axes[i]->mm_to_count(dist);
			axes[i]->Get()->Status.RT.Refresh();
			if (axes[i]->Get()->Status.RT.Value().cpm.MoveBufAvail) {
				// calculate velocity between current and desired position
				// assume time is 1/60.0 for now

				auto origin = toGlm(axes[i]->position_world.get());
				auto projected = get_projected_point(curve, origin.x);

				// Get ACTUAL position
				auto actual_pos_mm = axes[i]->count_to_mm(axes[i]->GetPosition());
				glm::vec3 actual_ee = glm::vec3(origin.x, origin.y + actual_pos_mm, 0);

				// Check that we are in bounds (mm)
				float min_bounds = -10;
				float max_bounds = 800;
				// If we're in bounds, send the velocity move
				if (actual_pos_mm >= min_bounds && actual_pos_mm <= max_bounds) {
					// Get velocity to DESIRED position		
					auto actual_dist = glm::distance(actual_ee, projected);
					float dt = vel_time_step.get();
					auto vel = actual_dist / dt;
					// clamp the velocity in case the distance jumps
					if (vel > vel_max.get())
						cout << "CLAMPING VELOCITY " << ofToString(vel) << endl;
					vel = min(vel, vel_max.get());
					if (projected.y - actual_ee.y < 0)
						vel *= -1;
					// Do a velocity move to send the ee towards the desired position
					axes[i]->Get()->Motion.MoveVelStart(vel);
				}
				// Ohterwise, stop the motor
				else {
					cout << "out of bounds! Position is: " << actual_pos_mm << ". STOPPING MOTION." << endl;
					//axes[i]->Get()->Motion.MoveVelStart(0);
					axes[i]->Get()->Motion.NodeStop(STOP_TYPE_RAMP);
				}

				// This is a trapezoidal move ... it won't run smoothly
				//axes[i]->Get()->Motion.MovePosnStart(pos_cnt, true, false);
			}
		}
		axes[i]->update();
	}

	// Update sine curve
	if (use_sine_curve) {
		update_sine_curve();
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(60);

	// Draw Curve (2D)
	ofPushStyle();
	ofNoFill();
	ofSetColor(ofColor::magenta, 120);
	ofSetLineWidth(5);
	curve.draw();
	ofFill();
	ofSetColor(ofColor::white, 120);
	ofSetLineWidth(3);
	for (auto axis : axes) {
		auto origin = axis->position_world.get();
		auto projected = get_projected_point(curve, origin.x);
		auto dist = glm::distance(toGlm(origin), projected);
		ofSetColor(ofColor::white, 120);
		ofDrawCircle(origin.x, origin.y, 20);
		ofDrawLine(origin, projected);
		ofSetColor(ofColor::aquamarine);
		ofDrawCircle(projected.x, projected.y, 5);
		ofDrawBitmapString(ofToString(dist) + " mm", origin.x + 10, origin.y + dist / 2.0);

		// Show ACTUAL position
		auto actual_pos_mm = axis->count_to_mm(axis->GetPosition());
		glm::vec3 actual_ee = glm::vec3(origin.x, origin.y + actual_pos_mm, 0);
		ofSetColor(ofColor::red);
		ofDrawCircle(actual_ee.x, actual_ee.y, 5);

		// Show velocity to DESIRED position		
		ofSetColor(ofColor::lightCoral);
		auto actual_dist = glm::distance(actual_ee, projected);
		auto dt = vel_time_step.get();
		auto vel = actual_dist / dt;
		vel = min(vel, vel_max.get());
		if (projected.y - actual_ee.y < 0)
			vel *= -1;
		ofDrawBitmapString(ofToString(vel) + " RPM", origin.x + 10, origin.y + dist / 2.0 + 20);
	}
	ofPopStyle();

	if (showGUI) {
		panel.draw();
		panel_curve.draw();
		for (int i = 0; i < axes.size(); i++) {
			axes[i]->panel.draw();
		}
	}
}

/**
 * @brief Get the closest point on the curve to a given x-value.
 *
 * @param (ofPolyline)  crv: curve to query
 * @param (float)  x: x coordinate
 * @return (glm::vec3)  point on the curve with the closest x-value
 */
glm::vec3 ofApp::get_projected_point(ofPolyline crv, float x)
{
	int i = 0;
	float min_dist = FLT_MAX;
	int min_index = 0;
	float search_window = 5.0;
	for (auto pt : crv.getVertices()) {
		// use a search window to account for varing resolutions of line
		if (pt.x >= x - search_window && pt.x <= x + search_window) {
			auto dist = abs(x - pt.x);
			if (dist < min_dist) {
				min_index = i;
				min_dist = dist;
			}
		}
		i++;
	}
	return crv.getVertices()[min_index];
}


/**
 * Animates the sine curve.
 */
void ofApp::update_sine_curve()
{
	float dx = (TWO_PI / crv_period) * (crv_width / crv_res);
	crv_theta += crv_speed;
	float x = crv_theta;
	for (int i = 0; i < curve.getVertices().size(); i++) {
		curve.getVertices()[i].y = sin(x) * crv_amp + ofGetHeight() / 2;
		x += dx;
	}
}

//--------------------------------------------------------------
/**
 * @brief Create the System Manger, then setup and open the port.
 * Logs how many nodes it finds on the port.
 *
 * @return (bool) True if the system was initialized properly.
 *				  False if there was a problem accessing the port.
 *
 */
bool ofApp::initialize() {

	// Create the System Manager 
	myMgr = SysManager::Instance();

	int portCount = 0;
	vector<string> comHubPorts;
	try {
		SysManager::FindComHubPorts(comHubPorts);
		ofLogNotice("ofApp::initialize") << "Found " << ofToString(comHubPorts.size()) << " SC Hubs";

		// Define the first SC Hub port (port 0) to be associated with COM portnum (as seen in windows device manager)
		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());

			// @TODO: OK because we just have one port for now
			comPortNumber.set(comHubPorts[portCount].c_str());
		}
		numPorts = portCount;

		if (portCount > 0) {
			// Open the port
			myMgr->PortsOpen(portCount);

			for (size_t i = 0; i < portCount; i++) {
				IPort& myPort = myMgr->Ports(i);
				ofLogNotice("ofApp::initialize") << "\tSTATUS: Port " << myPort.NetNumber() << ", state=" << myPort.OpenState() << ", nodes=" << myPort.NodeCount();

				// The the default vel, accel values for each node
				for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
					axes.push_back(new Axis(*myMgr, &myPort.Nodes(iNode), i, iNode));
				}

			}

			// @TODO: Update the GUI: OK because we just have one port for now
			IPort& myPort = myMgr->Ports(0);
			systemStatus.set("Connected");
			numHubs.set(ofToString(numPorts));
			numNodes.set(ofToString(myPort.NodeCount()));
		}
		else {
			ofLogWarning("ofApp::initialize") << "Unable to locate SC hub port";

			// Update the GUI
			systemStatus.set("No SC Hub Ports Found");
			numHubs.set("0");

			return false;
		}
		return true;
	}
	catch (mnErr& theErr)
	{
		ofLogError("ofApp::initialize") << "Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port";
		ofLogError("ofApp::initialize") << "Caught error: addr=" << ofToString(theErr.TheAddr) << ", err=" << ofToString(theErr.ErrorCode) << ", msg=" << ofToString(theErr.ErrorMsg);

		// Update the GUI
		systemStatus.set("Error: " + ofToString(theErr.ErrorMsg));
		numHubs.set("0");
		return false;
	}
}


/**
 * @brief Close all operations down and close the ports.
 *
 * This function closes the serial ports in the system and release all resources related to them.
 *
 */
void ofApp::close() {
	if (myMgr != nullptr) {
		ofLogNotice() << "Closing HUB Ports...";
		myMgr->PortsClose();
	}
}


/**
 * @brief Create a GUI for the System Manager.
 *
 */
void ofApp::setup_gui()
{
	mode_color_eStop = ofColor(250, 0, 0, 100);
	mode_color_normal = ofColor(60, 120);
	mode_color_disabled = ofColor(0, 200);

	int gui_width = 250;

	// System Manager Parameters
	params_system.setName("System_Manager");
	params_system.add(systemStatus.set("Status:", ""));
	params_system.add(numHubs.set("Number_of_Hubs:", ""));

	params_motion_control.setName("Motion_Controls");
	params_motion_control.add(play.set("Play", false));
	params_motion_control.add(pause.set("Pause", true));
	params_motion_control.add(vel_time_step.set("Vel_dt", 2., 0.05, 1.));
	params_motion_control.add(vel_max.set("VEL_MAX", 300, 0, 500));


	// SC Hub Parameters
	params_hub.setName("Hub_0");
	params_hub.add(comPortNumber.set("COM_Port:", ""));
	params_hub.add(numNodes.set("Number_of_Nodes:", ""));
	params_hub.add(enable_all.set("Enable_All", false));
	params_hub.add(eStopAll.set("E_STOP_ALL", false));
	params_motion.setName("Motion_Parameters");
	params_motion.add(vel_all.set("Velocity_ALL", 300, 0, 600));
	params_motion.add(accel_all.set("Acceleration_ALL", 2000, 0, 4000));
	params_macros.setName("Macros");
	params_macros.add(move_trigger_all.set("Trigger_Move", false));
	params_macros.add(move_target_all.set("Move_Target", 0, -50000, 50000));
	params_macros.add(move_absolute_pos_all.set("Move_Absolute", false));
	params_macros.add(move_zero_all.set("Move_to_Zero", false));

	params_hub.add(params_motion);
	params_hub.add(params_macros);

	play.addListener(this, &ofApp::on_play);
	pause.addListener(this, &ofApp::on_pause);
	enable_all.addListener(this, &ofApp::on_enable_all);
	eStopAll.addListener(this, &ofApp::on_eStop_all);
	vel_all.addListener(this, &ofApp::on_vel_all);
	accel_all.addListener(this, &ofApp::on_accel_all);
	move_trigger_all.addListener(this, &ofApp::on_move_trigger_all);
	move_target_all.addListener(this, &ofApp::on_move_target_all);
	move_absolute_pos_all.addListener(this, &ofApp::on_move_absolute_pos_all);
	move_zero_all.addListener(this, &ofApp::on_move_zero_all);

	panel.setup("Clearpath_Controller");
	panel.setWidthElements(gui_width);
	panel.setPosition(15, 15);
	panel.add(params_system);
	panel.add(params_motion_control);
	panel.add(params_hub);

	params_curve.setName("Sine_Params");
	params_curve.add(crv_period.set("Period", 500, 10, 1000));
	params_curve.add(crv_amp.set("Amplitude", 0, 0, 1000));
	params_curve.add(crv_speed.set("Speed", 0.02, 0.001, .25));

	panel_curve.setup("Curve_Controller");
	panel_curve.add(use_sine_curve.set("Use_Sine_Crv", true));
	panel_curve.add(params_curve);
	panel_curve.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 15);

}

void ofApp::on_play(bool& val)
{
	if (val) {
		pause.set(false);
	}
	else {
		pause.set(true);
	}
}

void ofApp::on_pause(bool& val)
{
	if (val) {
		play.set(false);
		for (auto axis : axes) {
			axis->Get()->Motion.NodeStop(STOP_TYPE_RAMP);
		}
	}
}

/**
 * @brief Enables or Disables all motors.
 *
 * @param (bool)  val
 */
void ofApp::on_enable_all(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		if (val) {
			if (eStopAll)
				eStopAll.set(false);
			panel.setBorderColor(mode_color_normal);
		}
		else {
			panel.setBorderColor(mode_color_disabled);
		}
		axes[i]->enable = val;
	}
}

/**
 * @brief Triggers or Clears an E-Stop for all motors.
 *
 * @param (bool)  val
 */
void ofApp::on_eStop_all(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		if (val) {
			// disable & uncheck enable_all in the GUI
			if (enable_all)
				enable_all.set(false);
			panel.setBorderColor(mode_color_eStop);
		}
		else {
			panel.setBorderColor(mode_color_normal);
		}

		axes[i]->eStopAll = val;
		axes[i]->eStop.set(val);
	}
}

void ofApp::on_vel_all(float& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->vel = val;
	}
}

void ofApp::on_accel_all(float& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->accel = val;
	}
}

void ofApp::on_move_trigger_all(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->move_trigger = val;
	}
	move_trigger_all.set(false);
}

void ofApp::on_move_absolute_pos_all(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->move_absolute_pos = val;
	}
}

void ofApp::on_move_target_all(float& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->move_target_cnts = val;
	}
}

void ofApp::on_move_zero_all(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->move_zero = val;
	}
	move_zero_all.set(false);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	switch (key)
	{
	case 'h':
	case 'H':
		showGUI.set(!showGUI);
		break;
	case 'f':
	case 'F': {
		ofToggleFullscreen();

		// resize / reposition the curve
		crv_width = ofGetWidth();
		float step = crv_width / (crv_res * 1.0);
		for (int i = 0; i < curve.getVertices().size(); i++)
			curve[i].x = i * step;

		break;
	}
	case ' ':
		for (int i = 0; i < axes.size(); i++) {
			axes[i]->PrintStats();
		}
		break;
	case 'p':
	case 'P':
		// software enable / disable sending move commands to motors
		play.set(!play.get());
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

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
