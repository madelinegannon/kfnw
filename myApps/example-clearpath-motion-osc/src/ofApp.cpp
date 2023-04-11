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

	setup_osc_controller();
}

//--------------------------------------------------------------
void ofApp::update() {

	if (use_osc_controller)
		update_osc_controller();

	else if (use_sine_curve)
		update_sine_curve();


	for (int i = 0; i < axes.size(); i++) {

		actual_dist_mm = axes[i]->count_to_mm(axes[i]->GetPosition());

		if (use_sine_curve) {
			origin = toGlm(axes[i]->position_world.get());
			actual_pos = glm::vec3(origin.x, origin.y + actual_dist_mm, 0);
			desired_pos = get_projected_point(curve, origin.x);
			desired_dist_mm = glm::distance(origin, desired_pos);
		}
		else if (use_osc_controller) {
			origin = axes[i]->spool.home.getGlobalPosition();
			actual_pos = glm::vec3(origin.x, origin.y + actual_dist_mm, 0);
			desired_pos = axes[i]->spool.pos.getGlobalPosition();
			desired_dist_mm = glm::distance(origin, desired_pos);
		}


		// Calculate the desired velocity
		// Get velocity to DESIRED position		
		auto actual_to_desired_dist = glm::distance(actual_pos, desired_pos);
		//float pid_actual_to_desired_dist = axes[i]->pid.update(actual_to_desired_dist);
		//cout << "pid_actual_to_desired_dist: " << pid_actual_to_desired_dist << endl;
		float dt = vel_time_step.get();
		desired_vel = actual_to_desired_dist / dt;
		auto curr_vel = axes[i]->Get()->Motion.VelMeasured.Value();
		//axes[i]->pid.setSetpoint(curr_vel);
		//pid_vel = axes[i]->pid.update(desired_vel);
		//desired_vel = pid_actual_to_desired_dist / dt;
		// clamp the velocity in case the distance jumps
		if (desired_vel > vel_max.get())
			cout << "CLAMPING VELOCITY {" << ofToString(desired_vel) << "} to " << vel_max.get() << endl;
		desired_vel = min(desired_vel, vel_max.get());
		if (desired_pos.y - actual_pos.y < 0) {
			desired_vel *= -1;
		}
		//else {
		//	pid_vel *= -1;
		//}
		//cout << "pid_vel: " << pid_vel << endl;
		//desired_vel = pid_vel;


		// move the motor based on the sine curve
		if ((use_sine_curve || use_osc_controller) && play) {
			// Update the gui with the origin_to_desired distance
			axes[i]->move_target_mm.set(desired_dist_mm);

			// add a movement command if there's space in the buffer
			axes[i]->Get()->Status.RT.Refresh();
			if (axes[i]->Get()->Status.RT.Value().cpm.MoveBufAvail) {
				float min_bounds = -50;
				float max_bounds = 1100;
				// If we're in bounds, send the velocity move
				if (actual_dist_mm >= min_bounds && actual_dist_mm <= max_bounds) {
					// Do a velocity move to send the ee towards the desired position
					axes[i]->Get()->Motion.MoveVelStart(desired_vel);
				}
				// Ohterwise, stop the motor
				else {
					cout << "Out of bounds! Position is: " << actual_dist_mm << ". STOPPING MOTION." << endl;
					axes[i]->Get()->Motion.NodeStop(STOP_TYPE_RAMP);
				}

				// Can't use this trapezoidal move ... it does't work smoothly for continuous movement (great for PTP)
				//axes[i]->Get()->Motion.MovePosnStart(pos_cnt, true, false);
			}
		}
		axes[i]->update();
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(60);

	if (use_sine_curve) {
		draw_sine_curve();
	}
	else if (use_osc_controller) {
		draw_osc_controller();
	}

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

void ofApp::draw_sine_curve()
{
	ofPushStyle();
	ofNoFill();
	ofSetColor(ofColor::magenta, 120);
	ofSetLineWidth(5);
	curve.draw();
	ofFill();
	ofSetColor(ofColor::white, 120);
	ofSetLineWidth(3);
	for (auto axis : axes) {

		// Show ACTUAL position
		ofSetColor(ofColor::red);
		auto dist_mm = axis->count_to_mm(axis->GetPosition());
		auto home = toGlm(axis->position_world.get());
		auto pos = glm::vec3(home.x, home.y + dist_mm, 0);
		auto _desired_pos = get_projected_point(curve, home.x);
		auto _desired_dist_mm = glm::distance(home, _desired_pos);

		auto offset = home + ((_desired_pos - home) / 2);
		ofSetColor(ofColor::white, 120);
		ofDrawCircle(home.x, home.y, 20);
		ofDrawLine(home, _desired_pos);
		ofSetColor(ofColor::aquamarine);
		ofDrawCircle(_desired_pos.x, _desired_pos.y, 5);
		ofDrawBitmapString(ofToString(_desired_dist_mm) + " mm", offset.x + 10, offset.y);

		// Show ACTUAL position
		ofSetColor(ofColor::red);
		ofDrawCircle(pos.x, pos.y, 5);
		//ofDrawCircle(actual_pos.x, actual_pos.y, 5);

		// Show velocity to DESIRED position		
		//ofSetColor(ofColor::lightCoral);
		//ofDrawBitmapString(ofToString(desired_vel) + " RPM", offset.x + 10, offset.y + 20);

		// Show PID velocity
		//ofSetColor(ofColor::greenYellow);
		//ofDrawBitmapString(ofToString(pid_vel) + " RPM", offset.x + 10, offset.y + 40);
	}
	ofPopStyle();
}

void ofApp::setup_osc_controller()
{
	// setup OSC connection
	receiver.setup(port);

	start.set(ofGetWidth() / 4, ofGetHeight() / 4, 0);
	end.set(3 * ofGetWidth() / 4, ofGetHeight() / 4, 0);
	float dist = start.distance(end);
	float step = (axes.size() > 1) ? dist / (axes.size() - 1) : 0;
	for (int i = 0; i < axes.size(); i++) {
		float x = start.x + i * step;
		float y = start.y;
		axes[i]->spool.setup(ofVec3f(x, y, 0), 12, 1000, 50);
		//Spool spool;
		//spool.setup(ofVec3f(x, y, 0), 12, 1000, 50);
		//spools.push_back(spool);
	}
}

void ofApp::update_osc_controller()
{
	checkForOSCMessage();
}

void ofApp::draw_osc_controller()
{
	ofPushStyle();

	// draw the start and end path
	ofDrawEllipse(start, 15, 15);
	ofDrawEllipse(end, 15, 15);
	ofSetLineWidth(3);
	ofSetColor(80);
	ofDrawLine(start, end);

	for (int i = 0; i < axes.size(); i++) {
		axes[i]->spool.draw();

		// Show ACTUAL position
		ofSetColor(ofColor::red);
		auto dist_mm = axes[i]->count_to_mm(axes[i]->GetPosition());
		auto home = axes[i]->spool.home.getGlobalPosition();
		auto pos = glm::vec3(home.x, home.y + dist_mm, 0);
		ofDrawCircle(pos.x, pos.y, 5);
	}

	ofPopStyle();
}

void ofApp::reset_osc_controller()
{
	start.set(ofGetWidth() / 4, ofGetHeight() / 4, 0);
	end.set(3 * ofGetWidth() / 4, ofGetHeight() / 4, 0);
	float dist = start.distance(end);
	float step = (axes.size() > 1) ? dist / (axes.size() - 1) : 0;
	for (int i = 0; i < axes.size(); i++) {
		float x = start.x + i * step;
		float y = start.y;
		axes[i]->spool.home.setGlobalPosition(x, y, 0);
	}
}

void ofApp::checkForOSCMessage()
{

	// check for waiting messages
	while (receiver.hasWaitingMessages()) {

		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage(m);

		// check for mouse moved message
		if (m.getAddress().find("microfreak") != std::string::npos) {
			std::cout << "MICROFREAK!!!!! " << '\n';
			string msgString = "" + m.getAddress() + "\n";
			if (m.getAddress().find("envelope_decay") != std::string::npos) {
			}
			else if (m.getAddress().find("filter_cutoff") != std::string::npos) {
			}
			else if (m.getAddress().find("filter_resonance") != std::string::npos) {
			}
			else if (m.getAddress().find("cycling_env_rise") != std::string::npos) {
			}
			else if (m.getAddress().find("cycling_env_fall") != std::string::npos) {
			}
			else if (m.getAddress().find("cycling_env_hold") != std::string::npos) {
			}
			else if (m.getAddress().find("cycling_env_amount") != std::string::npos) {
			}
			else if (m.getAddress().find("envelope_attack") != std::string::npos) {
			}
			else if (m.getAddress().find("envelope_decay") != std::string::npos) {
			}
			else if (m.getAddress().find("envelope_sustain") != std::string::npos) {
			}
			else if (m.getAddress().find("envelope_amount") != std::string::npos) {
			}
			else if (m.getAddress().find("lfo_rate_sync") != std::string::npos) {
			}
			else if (m.getAddress().find("lfo_rate_free") != std::string::npos) {
			}
			else if (m.getAddress().find("arp_rate_sync") != std::string::npos) {
			}
			else if (m.getAddress().find("arp_rate_free") != std::string::npos) {
			}
			else if (m.getAddress().find("oscillator_type") != std::string::npos) {
			}
			else if (m.getAddress().find("oscillator_wave") != std::string::npos) {
			}
			else if (m.getAddress().find("oscillator_timbre") != std::string::npos) {
			}
			else if (m.getAddress().find("oscillator_shape") != std::string::npos) {
			}
			else if (m.getAddress().find("pitchbend") != std::string::npos) {
			}
			else if (m.getAddress().find("glide") != std::string::npos) {
			}

			// The rest are keys
			else {

				for (size_t i = 0; i < m.getNumArgs(); i++) {

					// get the last number in the address
					auto start = m.getAddress().find_last_of('/') + 1;

					auto end = m.getAddress().length() - 1;
					if (start != std::string::npos) {
						cout << m.getAddress().substr(start, end) << endl;
						auto key_code = stoi(m.getAddress().substr(start, end));
						cout << "keycode: " << key_code << ", (key_code % 12): " << (key_code % 12) << endl;
						int interval = key_code % 12;
						switch (interval)
						{
						case 0:
							cout << "\tC!" << endl;

							break;
						case 1:
							cout << "\tC#!" << endl;
							break;
						case 2:
							cout << "\tD!" << endl;
							break;
						case 3:
							cout << "\tE#!" << endl;
							break;
						case 4:
							cout << "\tE!" << endl;
							break;
						case 5:
							cout << "\tF!" << endl;
							break;
						case 6:
							cout << "\tF#!" << endl;
							break;
						case 7:
							cout << "\tG!" << endl;
							break;
						case 8:
							cout << "\tG#!" << endl;
							break;
						case 9:
							cout << "\tA!" << endl;
							break;
						case 10:
							cout << "\tBb!" << endl;
							break;
						case 11:
							cout << "\tB!" << endl;
							break;
						default:
							break;
						}
						if (key_code == 48) interval = 0;
						else if (key_code == 60) interval = 12;

						// chose which motor to move ... key_codes in range 48 - 72
						// hard coded for 2 motors right now :'( one octave per motor
						int index = 0;
						if (axes.size() > 1) {
							int key_code_start = 48;
							int key_code_end = 72;
							int num_steps = key_code_end - key_code_start;

							if (key_code < key_code_start + num_steps / axes.size()) {
								index = 0;
							}
							else {
								index = 1;
							}
						}

						auto target_pos = interval;
						if (m.getArgType(i) == OFXOSC_TYPE_FLOAT) {
							float pressure = m.getArgAsFloat(i);		// value between 0-1
							pressure = ofMap(pressure, 0, 1, 0, axes[0]->spool.interval_dist);

							axes[index]->spool.pressure_offset = pressure;
						}
						axes[index]->spool.set_position(interval);
					}

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
					msgString += "\n";
				}
			}
			//cout << msgString << endl;
		}
		// unrecognized message
		else {
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
	mode_color_playing = ofColor(ofColor::blueSteel);

	int gui_width = 250;

	// System Manager Parameters
	params_system.setName("System_Manager");
	params_system.add(systemStatus.set("Status:", ""));
	params_system.add(numHubs.set("Number_of_Hubs:", ""));

	params_motion_control.setName("Motion_Controls");
	params_motion_control.add(play.set("Play", false));
	params_motion_control.add(pause.set("Pause", true));
	params_motion_control.add(vel_time_step.set("Vel_dt", 1., 0.05, 1.));
	params_motion_control.add(vel_max.set("VEL_MAX", 300, 0, 500));


	// SC Hub Parameters
	params_hub.setName("Hub_0");
	params_hub.add(comPortNumber.set("COM_Port:", ""));
	params_hub.add(numNodes.set("Number_of_Nodes:", ""));
	params_hub.add(enable_all.set("Enable_All", false));
	params_hub.add(eStopAll.set("E_STOP_ALL", false));
	params_motion.setName("Motion_Parameters");
	params_motion.add(vel_all.set("Velocity_ALL", 300, 0, 600));
	params_motion.add(accel_all.set("Acceleration_ALL", 500, 0, 4000));
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

	panel_curve.setup("Motion_Controller");
	panel_curve.add(use_sine_curve.set("Use_Sine_Crv", false));
	panel_curve.add(use_osc_controller.set("Use_OSC_Controller", true));
	panel_curve.add(params_curve);
	panel_curve.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 15);

	use_sine_curve.addListener(this, &ofApp::on_use_sine_curve);
	use_osc_controller.addListener(this, &ofApp::on_use_osc_controller);

}

void ofApp::on_play(bool& val)
{
	if (val) {
		panel.setBorderColor(mode_color_playing);
		pause.set(false);
	}
	else {
		panel.setBorderColor(mode_color_normal);
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

void ofApp::on_use_sine_curve(bool& val)
{
	if (val) {
		use_osc_controller.set(false);
	}
	else {
	}
}

void ofApp::on_use_osc_controller(bool& val)
{
	if (val) {
		use_sine_curve.set(false);
	}
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
		for (int i = 0; i < axes.size(); i++) {
			axes[i]->pid.resetIntegral();
			axes[i]->pid.resetActivityCounter();
		}
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
	reset_osc_controller();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
