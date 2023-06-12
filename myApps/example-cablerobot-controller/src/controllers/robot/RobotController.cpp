#include "RobotController.h"

RobotController::RobotController(int count, float offset_x, float offset_y, float offset_z)
{
}

RobotController::RobotController(vector<glm::vec3> positions_base)
{
	this->positions_base = positions_base;
	
	startThread();
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
				
				// Create each cable robot, set its world position
				for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
					robots.push_back(new CableRobot(*myMgr, &myPort.Nodes(iNode)));
					auto pos = positions_base[iNode];
					float diameter = 100;
					float length = 30;
					int turns = 30;
					Groove dir;
					//if (iNode % 2 == 0)
						dir = Groove::LEFT_HANDED;
					//else
					//	dir = Groove::RIGHT_HANDED;
					robots.back()->configure(pos, dir, diameter, length, turns);
#ifdef AUTO_HOME
					// if the motor is not homed,
					if (!robots.back()->is_homed()) {
						// automatically run the homing routine
						cout << "Motor " << iNode << " is not home ... running homing routine now:" << endl;
						robots.back()->run_homing_routine();
					}
#endif // AUTO_HOME
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
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->update();
	}
}

void RobotController::draw()
{
	if (showGUI) {
		panel.draw();
		for (int i = 0; i < robots.size(); i++) {
			robots[i]->draw();
			robots[i]->panel.draw();
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
					// hold the thread for 5 seconds
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
	
	params_info.setName("System_Info");
	params_info.add(com_ports.set("COM_Ports", ""));
	params_info.add(num_com_hubs.set("Num_Hubs", ""));
	params_info.add(num_robots.set("Num_Motors", ""));

	params_sync.setName("Synchronize_Params");
	params_sync.add(sync_index.set("Synchronize_Index", 0, 0, 0));
	params_sync.add(is_synchronized.set("Synchronize", false));
	

	check_status.addListener(this, &RobotController::check_for_system_ready);
	is_synchronized.addListener(this, &RobotController::on_synchronize);

	panel.add(params_info);
	panel.add(params_sync);

	// Minimize less important parameters
	panel.getGroup("System_Info").minimize();

	is_gui_setup = true;
}

void RobotController::draw_gui()
{
}

void RobotController::set_position(int index, float position)
{
}

void RobotController::set_positions(vector<float> positions)
{
}

float RobotController::get_position(int index)
{
	return 0.0f;
}

vector<float> RobotController::get_positions()
{
	return vector<float>();
}

string RobotController::get_status(int index)
{
	return string();
}

void RobotController::play()
{
	state = ControllerState::PLAY;
}

void RobotController::pause()
{
	state = ControllerState::PAUSE;
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->stop();
	}
}

void RobotController::set_e_stop(bool val)
{
	state = ControllerState::E_STOP;
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->set_e_stop(val);
	}
}

void RobotController::key_pressed(int key)
{
	for (int i = 0; i < robots.size(); i++)
		robots[i]->key_pressed(key);
	
	
	switch (key)
	{
		case 'h':
		case 'H':
			showGUI = !showGUI;
			break;
	default:
		break;
	}
}

/**
 * @brief Makes all the robots match the motion parameters and position of the `sync_index` robot.
 * 
 * @param (bool)  val: sync or unsync
 */
void RobotController::on_synchronize(bool& val)
{
	if (val) {
		// Make all the robot match the `sync_index` robot
		//		if not homed, do nothing and print warning
		//		if not enabled, enable
		//		clear motion all acceptions
		//		set all velocity/accel limits
		//		if not in the same position, move to position of `sync_index` robot
		int index = sync_index.get();
		vector<float> motion_params = robots[index]->get_motion_parameters();
		float vel = motion_params[0];
		float accel = motion_params[1];
		float bounds_min = motion_params[2];
		float bounds_max = motion_params[3];
		vector<float> jogging_params = robots[index]->get_jogging_parameters();
		float jog_vel = jogging_params[0];
		float jog_accel = jogging_params[1];
		float jog_dist = jogging_params[2];
		float pos = robots[index]->get_position_actual();

		for (int i = 0; i < robots.size(); i++) {
			if (i != index) {
				if (robots[i]->is_homed()) {
					// Sync velocity/accel/bound limits
					robots[i]->set_motion_parameters(vel, accel, bounds_min, bounds_max);
					// Sync all jogging values
					robots[i]->set_jogging_parameters(jog_vel, jog_accel, jog_dist);
					// Move to the same position
					robots[i]->move_to.set(pos);
					robots[i]->btn_move_to.trigger();

					// Collapse the GUI of all the other robots
					robots[i]->panel.minimize();
				}
				else {
					ofLogWarning("RobotController::on_synchronize") << "Cannot sync to Motor " << robots[i]->get_id() << " because it is not HOMED. Skipping until HOMED.";
				}				
			}
		}
	}
	else {
		for (int i = 0; i < robots.size(); i++) {
			if (i != sync_index.get()) {
				// Reopen the GUI of all the other robots
				robots[i]->panel.maximize();
			}
		}
	}
}
