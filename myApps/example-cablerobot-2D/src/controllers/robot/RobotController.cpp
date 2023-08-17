#include "RobotController.h"

RobotController::RobotController(int count, float offset_x, float offset_y, float offset_z)
{
}

RobotController::RobotController(vector<glm::vec3> bases, ofNode* _origin)
{
	this->bases = bases;
	this->origin = _origin;
	
	//load_settings();

	gizmo_origin.setNode(*origin);
	gizmo_origin.setDisplayScale(.33);

	// add the origin gizmo to the gizmos list
	gizmos.push_back(&gizmo_origin);

	// create an end_effector in the middle of the robots
	//ee = ofNode();
	//if (system_config == Configuration::TWO_D) {
	//	ee.setParent(*origin);
	//	glm::vec3 pos;
	//	for (auto base : bases)
	//		pos += base;
	//	pos /= bases.size();
	//	pos.y -= 1000;
	//	ee.setGlobalPosition(pos + origin->getGlobalPosition());
	//}
	


	startThread();
}

/**
 * @brief Saves system settings to a file.
 * 
 * @param (string)  filename: file saved to local /bin/data folder (must end in .xml). Defaults to "settings.xml"
 */
void RobotController::save_settings(string filename)
{
	ofxXmlSettings config;
	config.addTag("config");
	config.pushTag("config");
	config.addValue("timestamp", ofGetTimestampString());
	config.addValue("run_offline", run_offline);
	config.addValue("system_config", system_config);
	config.addValue("auto_home", auto_home);
	config.addValue("load_robots_from_file", load_robots_from_file);

	config.addTag("origin");
	config.pushTag("origin");
	config.addValue("X", origin->getGlobalPosition().x);
	config.addValue("Y", origin->getGlobalPosition().y);
	config.addValue("Z", origin->getGlobalPosition().z);
	config.addValue("QX", origin->getGlobalOrientation().x);
	config.addValue("QY", origin->getGlobalOrientation().y);
	config.addValue("QZ", origin->getGlobalOrientation().z);
	config.addValue("QW", origin->getGlobalOrientation().w);
	config.popTag();

	config.popTag();

	config.saveFile(filename);

	for (auto robot : robots)
		robot->save_config_to_file();
}

/**
 * @brief Loads system settings from a file.
 *
 * @param (string)  filename: file must be in local /bin/data folder (must end in .xml). Defaults to "settings.xml"
 */
void RobotController::load_settings(string filename)
{
	ofxXmlSettings config;
	if (filename == "")
		filename = "settings.xml";

	if (config.loadFile(filename)) {

		run_offline = config.getValue("config:run_offline", 0);
		system_config = Configuration(config.getValue("config:system_config", 0));
		auto_home = config.getValue("config:auto_home", 0);
		load_robots_from_file = config.getValue("config:load_robots_from_file", 0);

		float x = config.getValue("config:origin:X", 0);
		float y = config.getValue("config:origin:Y", 0);
		float z = config.getValue("config:origin:Z", 0);
		float qx = config.getValue("config:origin:QX", 0);
		float qy = config.getValue("config:origin:QY", 0);
		float qz = config.getValue("config:origin:QZ", 0);
		float qw = config.getValue("config:origin:QW", 0);
		origin->setGlobalPosition(x, y, z);
		origin->setGlobalOrientation(glm::quat(qw, qx, qy, qz));
	}
	else {
		ofLogWarning("CableRobot::load_settings") << "No settings file found at: /bin/data/" << filename;
	}
}

/**
 * @brief Create the System Manger, then setup and open the port.
 * Logs how many nodes it finds on the port.
 *
 * @return (bool) True if the system was initialized properly.
 *				  False if there was a problem accessing the port.
 *
 */
bool RobotController::initialize()
{
	// Create the CPM System Manager
	myMgr = SysManager::Instance();

	// Find & Open SC-Hub Ports
	// Create one CableRobot per detected motor
	try
	{
		// Find how many SC-Hub Ports are connected
		int portCount = 0;
		vector<string> comHubPorts;
		SysManager::FindComHubPorts(comHubPorts);
		ofLogNotice("RobotController::initialize") << "Found " << ofToString(comHubPorts.size()) << " SC Hubs";

		// Define the first SC Hub port (port 0) to be associated with COM portnum (as seen in windows device manager)
		string com_port_names;
		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());

			// update the gui
			com_port_names += comHubPorts[portCount].c_str();
			if (portCount != comHubPorts.size() - 1) {
				com_port_names += " , ";
			}
		}
		// update the gui
		num_com_hubs.set(ofToString(portCount));
		com_ports.set(com_port_names);

		// Create a new CableRobot for each Motor detected on each port
		if (portCount > 0) {
			// Open the port
			myMgr->PortsOpen(portCount);

			// For each motor on the port, create a new CableRobot
			for (size_t i = 0; i < portCount; i++) {
				IPort& myPort = myMgr->Ports(i);
				ofLogNotice("RobotController::initialize") << "\tSTATUS: Port " << myPort.NetNumber() << ", state=" << myPort.OpenState() << ", nodes=" << myPort.NodeCount();
				
				// Create each cable robot and set its world position
				for (size_t j = 0; j < myPort.NodeCount(); j++) {
					// Store each individual robot, no matter the configuration
					robots.push_back(new CableRobot(*myMgr, &myPort.Nodes(j), this->load_robots_from_file));
					if (system_config == Configuration::ONE_D) {
						// configure for 1D application
						robots.back()->configure(origin, bases[j]);
						gizmos.push_back(robots.back()->get_gizmo());
					}
					else if (system_config == Configuration::TWO_D) {
						// Make a 2D robot
						if (j % 2 != 0) {
							robots_2D.push_back(new CableRobot2D(robots[j - 1], robots[j], origin, bases[j - 1], bases[j], j / 2));
							gizmos.push_back(robots_2D.back()->get_gizmo());
						}
					}
				}

			}
			// update the gui
			num_robots.set(ofToString(robots.size()));
			sync_index.setMax(robots.size() - 1);
		}
		else {
			ofLogWarning("RobotController::initialize") << "Unable to locate any SC hub ports.\n\tCheck that ClearView is closed and no other Clearpath applications are running.";
			return false;
		}
	}
	catch (mnErr& theErr)
	{
		ofLogError("RobotController::initialize") << "Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port";
		ofLogError("RobotController::initialize") << "Caught error: addr=" << ofToString(theErr.TheAddr) << ", err=" << ofToString(theErr.ErrorCode) << ", msg=" << ofToString(theErr.ErrorMsg);

		return false;
	}
	return true;
}

void RobotController::update()
{
	// update the gizmos
	update_gizmos();

	if (system_config == Configuration::ONE_D) {
		for (int i = 0; i < robots.size(); i++) {
			robots[i]->update();
		}
	}
	else if (system_config == Configuration::TWO_D) {
		for (int i = 0; i < robots_2D.size(); i++) {
			robots_2D[i]->update();
		}
	}
	
}

void RobotController::draw()
{
	if (system_config == Configuration::ONE_D) {
		for (int i = 0; i < robots.size(); i++) {
			robots[i]->draw();
		}
	}
	else if (system_config == Configuration::TWO_D) {
		for (int i = 0; i < robots_2D.size(); i++) {
			robots_2D[i]->draw();
		}
	}
}

void RobotController::shutdown()
{
	if (myMgr != nullptr) {
		ofLogNotice() << "Closing HUB Ports...";
		myMgr->PortsClose();
	}
}

void RobotController::windowResized(int w, int h)
{
	for (auto gizmo : gizmos)
		gizmo->setViewDimensions(w, h);
}

void RobotController::threadedFunction()
{
	
	while (isThreadRunning()) {
		if (!is_initialized) {
			
			// run once
			if (!is_gui_setup)
				setup_gui();

			// Initialize the CPM System Manager
			// Create one CableRobot per detected motor
			if (initialize()) {
				// update each robot's gui
				if (system_config == Configuration::ONE_D) {
					int x = panel.getPosition().x;
					int y = panel.getPosition().y;
					int w = panel.getWidth();
					int padding = 5;
					for (int i = 0; i < robots.size(); i++) {
						robots[i]->panel.setPosition(x + w + padding, y);
						x = robots[i]->panel.getPosition().x;
						w = robots[i]->panel.getWidth();
						robots[i]->panel.setParent(&panel);
					}
				}
				else if (system_config == Configuration::TWO_D) {
					for (int i = 0; i < robots_2D.size(); i++) {
						robots_2D[i]->update_gui(&panel);

					}

					robots_2D[0]->plot.name = "Bot 1 RPM: Motor 1 (RED), Motor 2 (BLUE)";
					robots_2D[1]->plot.name = "Bot 2 RPM: Motor 3 (RED), Motor 4 (BLUE)";
				}
				// check if system is ready to move (all motors are homed)
				check_for_system_ready();
				is_initialized = true;
			}
			// add a delay before trying to initialize again
			else {
				float timer = ofGetElapsedTimeMillis();
				float delay = 5;
				ofLogWarning("RobotController::threadedFunction") << "initialize() FAILED. Check that the motors are powered and connected to PC.\n\tRETRYING " << delay << " SECONDS.\n";
				while (ofGetElapsedTimeMillis() < timer + (delay * 1000))
				{
					// ... hold the thread for 5 seconds
				}
			}
		}

		//if (state == ControllerState::PLAY)
		update();
	}
}
/**
 * @brief Checks if all motors have been homed.
 * Updates the Controller State and updates the status in the GUI.
 */
void RobotController::check_for_system_ready()
{
	// check if motors are already enabled
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->check_for_system_ready();
		//robots[i]->enable.set(robots[i]->is_enabled());
	}

	// check if motors are homed and ready
	bool is_ready = true;
	for (int i = 0; i < robots.size(); i++) {
		if (robots[i]->status.get() == "ENABLED" || robots[i]->status.get() == "DISABLED") {
		}
		else {
			is_ready = false;
		}
	}
	state = is_ready ? ControllerState::READY : ControllerState::NOT_READY;
	// update the gui with the READY status
	status.set(state_names[state]);
}

void RobotController::setup_gui()
{
	mode_color_eStop = ofColor(250, 0, 0, 100);
	mode_color_normal = ofColor(60, 120);
	mode_color_disabled = ofColor(0, 200);
	mode_color_playing = ofColor(ofColor::blueSteel);
	
	int gui_width = 250;

	panel.setup("System_Controller");
	panel.setWidthElements(gui_width);
	panel.setPosition(10, 15);
	panel.add(status.set("Status", state_names[0]));
	panel.add(check_status.set("Check_Status"));
	panel.add(save_settings_files.set("Save_Settings"));
	
	params_info.setName("System_Info");
	params_info.add(com_ports.set("COM_Ports", ""));
	params_info.add(num_com_hubs.set("Num_Hubs", ""));
	params_info.add(num_robots.set("Num_Motors", ""));

	//params_sync.setName("Synchronize_Params");
	//params_sync.add(sync_index.set("Synchronize_Index", 0, 0, 0));
	//params_sync.add(is_synchronized.set("Synchronize", false));
	//params_sync.add(ee_offset.set("EE_Offset", 700, 0, 1000));
	

	check_status.addListener(this, &RobotController::check_for_system_ready);
	save_settings_files.addListener(this, &RobotController::on_save_settings);
	//is_synchronized.addListener(this, &RobotController::on_synchronize);
	//ee_offset.addListener(this, &RobotController::on_ee_offset_changed);

	panel.add(params_info);
	//panel.add(params_sync);

	// Minimize less important parameters
	panel.getGroup("System_Info").minimize();

	is_gui_setup = true;
}

void RobotController::draw_gui()
{
	if (showGUI) {
		panel.draw();
		if (system_config == Configuration::ONE_D) {
			for (int i = 0; i < robots.size(); i++) {
				robots[i]->panel.draw();
			}
		}
		else if (system_config == Configuration::TWO_D) {
			for (int i = 0; i < robots_2D.size(); i++) {
				int padding = 20;
				int x = panel.getPosition().x + (i * panel.getWidth()) + (i * padding);
				int y = panel.getPosition().y + panel.getHeight() + padding;
				robots_2D[i]->panel.setPosition(x, y);
				robots_2D[i]->draw_gui();

				ofPushMatrix();
				//for (auto& plot : robots_2D[i]->plots_rpm) {
					ofTranslate(ofGetWidth() - 550, i * 150 + 60);
				//	plot.draw();
				//}
					robots_2D[i]->plot.draw();
				ofPopMatrix();
			}
		}			
	}
}


void RobotController::play()
{
	state = ControllerState::PLAY;
}

void RobotController::pause()
{
	state = ControllerState::PAUSE;
	if (system_config == Configuration::ONE_D) {
		for (int i = 0; i < robots.size(); i++) {
			robots[i]->stop();
		}
	}
	else if (system_config == Configuration::TWO_D) {
		for (int i = 0; i < robots_2D.size(); i++) {
			robots_2D[i]->stop();
		}
	}
}

void RobotController::set_e_stop(bool val)
{
	state = ControllerState::E_STOP;
	if (system_config == Configuration::ONE_D) {
		for (int i = 0; i < robots.size(); i++) {
			robots[i]->set_e_stop(val);
		}
	}
	else if (system_config == Configuration::TWO_D) {
		for (int i = 0; i < robots_2D.size(); i++) {
			robots_2D[i]->set_e_stop(val);
		}
	}
}

void RobotController::key_pressed(int key)
{
	switch (key)
	{
		case '?':
			debugging = !debugging;
			break;
		case 'h':
		case 'H':
			showGUI = !showGUI;
			break;
		case 's':
		case 'S':
			ofLogNotice() << "Saving Configuration Files.";
			save_settings();
			break;
	default:
		break;
	}

	key_pressed_gizmo(key);

	if (system_config == Configuration::ONE_D) {
		for (int i = 0; i < robots.size(); i++)
			robots[i]->key_pressed(key);
	}
	else if (system_config == Configuration::TWO_D) {
		for (int i = 0; i < robots_2D.size(); i++)
			robots_2D[i]->key_pressed(key);
	}	
}

void RobotController::key_pressed_gizmo(int key)
{
	switch (key)
	{
	case 'e':
	case 'E':
		for (auto gizmo : gizmos)
			gizmo->setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_ROTATE);
		break;
	case 'w':
	case 'W':
		for (auto gizmo : gizmos) {
			gizmo->setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_MOVE);
		}
		break;
	case 'r':
	case 'R':
		for (auto gizmo : gizmos)
			gizmo->setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_SCALE);
		break;
	case '0':
		// reset the transform of the origin node
		gizmo_origin.setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_MOVE);
		gizmo_origin.setMatrix(glm::mat4());
		//for (auto gizmo : gizmos) {
		//	gizmo->setType(ofxGizmo::ofxGizmoType::OFX_GIZMO_MOVE);
		//	gizmo->setMatrix(glm::mat4());
		//}
		break;
	default:
		break;
	}
}

bool RobotController::disable_camera()
{
	bool val = false;
	for (auto gizmo : gizmos)
		if (gizmo->isInteracting())
			val = true;
	return val;
}

void RobotController::set_origin(glm::vec3 pos, glm::quat orient)
{
	origin->setGlobalPosition(pos);
	origin->setGlobalOrientation(orient);
}

void RobotController::set_targets(vector<glm::vec3*> targets)
{
	if (system_config == Configuration::TWO_D) {
		for (int i = 0; i < robots_2D.size(); i++) {
			ofNode node;
			node.setPosition(*targets[i]);
			robots_2D[i]->get_gizmo()->setNode(node);
		}
	}
}

void RobotController::update_gizmos()
{
	// update the origin node to match the gizmo
	origin->setGlobalPosition(gizmos[0]->getTranslation());
	origin->setGlobalOrientation(gizmos[0]->getRotation());

	// override ee gizmo transforms if we're moving the origin gizmo
	bool val = gizmos[0]->isInteracting();
	if (system_config == Configuration::ONE_D) {
		for (int i = 0; i < robots.size(); i++) {
			robots[i]->override_gizmo = val;
			robots[i]->update_gizmo();
		}
	}
	else if (system_config == Configuration::TWO_D) {
		for (int i = 0; i < robots_2D.size(); i++) {
			robots_2D[i]->override_gizmo = val;
			robots_2D[i]->update_gizmo();
		}
	}
}

//void RobotController::set_ee(glm::vec3 pos, glm::quat orient)
//{
//	// move the ee gizmo if we're moving the origin gizmo
//	if (gizmos[0]->isInteracting()) {
//		gizmo_ee_0.setNode(*ee);
//	}
//	// otherwise update the ee based on the ee gizmo 
//	else {
//		ee->setGlobalPosition(pos);
//		ee->setGlobalOrientation(orient);
//	}
//}

/**
 * @brief Makes all the robots match the motion parameters and position of the `sync_index` robot.
 * 
 * @param (bool)  val: sync or unsync
 */
void RobotController::on_synchronize(bool& val)
{
	//if (val) {
	//	// Make all the robot match the `sync_index` robot
	//	//		if not homed, do nothing and print warning
	//	//		if not enabled, enable
	//	//		clear motion all acceptions
	//	//		set all velocity/accel limits
	//	//		if not in the same position, move to position of `sync_index` robot
	//	int index = sync_index.get();
	//	vector<float> motion_params = robots[index]->get_motion_parameters();
	//	float vel = motion_params[0];
	//	float accel = motion_params[1];
	//	float bounds_min = motion_params[2];
	//	float bounds_max = motion_params[3];
	//	vector<float> jogging_params = robots[index]->get_jogging_parameters();
	//	float jog_vel = jogging_params[0];
	//	float jog_accel = jogging_params[1];
	//	float jog_dist = jogging_params[2];
	//	float pos = robots[index]->get_position_actual();

	//	for (int i = 0; i < robots.size(); i++) {
	//		if (i != index) {
	//			if (robots[i]->is_homed()) {
	//				// Sync velocity/accel/bound limits
	//				robots[i]->set_motion_parameters(vel, accel, bounds_min, bounds_max);
	//				// Sync all jogging values
	//				robots[i]->set_jogging_parameters(jog_vel, jog_accel, jog_dist);
	//				// Move to the same position
	//				robots[i]->move_to.set(pos);
	//				robots[i]->btn_move_to.trigger();

	//				// Collapse the GUI of all the other robots
	//				robots[i]->panel.minimize();
	//			}
	//			else {
	//				ofLogWarning("RobotController::on_synchronize") << "Cannot sync to Motor " << robots[i]->get_id() << " because it is not HOMED. Skipping until HOMED.";
	//			}				
	//		}
	//	}
	//}
	//else {
	//	for (int i = 0; i < robots.size(); i++) {
	//		if (i != sync_index.get()) {
	//			// Reopen the GUI of all the other robots
	//			robots[i]->panel.maximize();
	//		}
	//	}
	//}
}

void RobotController::on_save_settings()
{
	save_settings();
}

//void RobotController::on_ee_offset_changed(float& val)
//{
//	float offset =  val;
//	robots[0]->get_target()->setPosition(-offset, 0, 0);
//	robots[1]->get_target()->setPosition(offset, 0, 0);
//	//for (int i = 0; i < robots.size(); i++) {
//	//	if (i == 0)
//	//		offset *= -1;
//	//	auto pos = ee.getPosition();
//	//	cout << "Robot " << i << " ee position: " << ofToString(robots[i]->get_target()->getPosition()) << ", and offset: " << offset << endl;
//	//	robots[i]->get_target()->setPosition(offset, 0, 0);
//	//}
//}
