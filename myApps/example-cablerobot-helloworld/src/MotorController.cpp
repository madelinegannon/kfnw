#include "MotorController.h"

MotorController::MotorController()
{
	// Create the System Manager 
	myMgr = SysManager::Instance();
	if (!initialize()) {
		close();
	}
	else {
		startThread();
	}
}

MotorController::~MotorController()
{
}


void MotorController::update(bool use_sine)
{
	for (int i = 0; i < axes.size(); i++) {

		auto actual_dist_mm = axes[i]->count_to_mm(axes[i]->GetPosition());
		glm::vec3 origin, actual_pos, desired_pos;
		float desired_dist_mm;
		//if (use_sine) {
		origin = toGlm(axes[i]->position_world.get());
		actual_pos = glm::vec3(origin.x, origin.y + actual_dist_mm, 0);
		//desired_pos = get_projected_point(curve, origin.x);
	//	//desired_dist_mm = glm::distance(origin, desired_pos);
	//}
	//else {
	//origin = axes[i]->spool.home.getGlobalPosition();
	//actual_pos = glm::vec3(origin.x, origin.y + actual_dist_mm, 0);
	//desired_pos = axes[i]->spool.pos.getGlobalPosition();
	//desired_dist_mm = glm::distance(origin, desired_pos);
	//}

		desired_dist_mm = target_positions[i];
		desired_pos = glm::vec3(origin.x, origin.y + desired_dist_mm, 0);


		// Calculate the desired velocity
		// Get velocity to DESIRED position		
		auto actual_to_desired_dist = glm::distance(actual_pos, desired_pos);
		//float pid_actual_to_desired_dist = axes[i]->pid.update(actual_to_desired_dist);
		//cout << "pid_actual_to_desired_dist: " << pid_actual_to_desired_dist << endl;
		float dt = 1.0;
		auto desired_vel = actual_to_desired_dist / dt;
		auto curr_vel = axes[i]->Get()->Motion.VelMeasured.Value();
		//axes[i]->pid.setSetpoint(curr_vel);
		//pid_vel = axes[i]->pid.update(desired_vel);
		//desired_vel = pid_actual_to_desired_dist / dt;
		 
		
		// clamp the velocity in case the distance jumps
		//if (desired_vel > 300)
		//	cout << "CLAMPING VELOCITY {" << ofToString(desired_vel) << "} to " << 300 << endl;

		desired_vel = MIN(desired_vel, 300);
		if (desired_pos.y - actual_pos.y < 0) {
			desired_vel *= -1;
		}

		//cout << "\t" << ofToString(i) << ": DESIRED DIST = " << desired_dist_mm << endl;
		//cout << "\t" << ofToString(i) << ": DESIRED VEL = " << desired_vel << endl << endl;

		//else {
		//	pid_vel *= -1;
		//}
		//cout << "pid_vel: " << pid_vel << endl;
		//desired_vel = pid_vel;


		// move the motor based on the sine curve
		//if ((use_sine_curve || use_osc_controller) && play) {
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
				if (!play) {
					axes[i]->Get()->Motion.MoveVelStart(0);
				}
				else {
					axes[i]->Get()->Motion.MoveVelStart(desired_vel);
				}
			}
			// Ohterwise, stop the motor
			else {
				cout << "Out of bounds! Position is: " << actual_dist_mm << ". STOPPING MOTION." << endl;
				axes[i]->Get()->Motion.NodeStop(STOP_TYPE_RAMP);
			}

			// Can't use this trapezoidal move ... it does't work smoothly for continuous movement (great for PTP)
			//axes[i]->Get()->Motion.MovePosnStart(pos_cnt, true, false);
		}
		axes[i]->update();
	}
	//}

}

void MotorController::threadedFunction()
{

	while (isThreadRunning()) {
		if (play)
			update(false);
	}

}

//ofxPanel MotorController::get_gui(int index)
//{
//	return axes[index]->panel;
//}


void MotorController::pause()
{
	play = false;
	for (int i = 0; i < axes.size(); i++) {
		cout << "pausing " << i << endl;
		axes[i]->Get()->Motion.NodeStop(STOP_TYPE_RAMP);
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
bool MotorController::initialize() {



	int portCount = 0;
	vector<string> comHubPorts;
	try {
		SysManager::FindComHubPorts(comHubPorts);
		ofLogNotice("ofApp::initialize") << "Found " << ofToString(comHubPorts.size()) << " SC Hubs";

		// Define the first SC Hub port (port 0) to be associated with COM portnum (as seen in windows device manager)
		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());

			//// @TODO: OK because we just have one port for now
			//comPortNumber.set(comHubPorts[portCount].c_str());
		}
		//numPorts = portCount;

		if (portCount > 0) {
			// Open the port
			myMgr->PortsOpen(portCount);

			for (size_t i = 0; i < portCount; i++) {
				IPort& myPort = myMgr->Ports(i);
				ofLogNotice("ofApp::initialize") << "\tSTATUS: Port " << myPort.NetNumber() << ", state=" << myPort.OpenState() << ", nodes=" << myPort.NodeCount();

				// The the default vel, accel values for each node
				for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
					axes.push_back(new Axis(*myMgr, &myPort.Nodes(iNode), i, iNode));
					target_positions.push_back(0.0);
				}

			}

			// @TODO: Update the GUI: OK because we just have one port for now
			IPort& myPort = myMgr->Ports(0);
			//systemStatus.set("Connected");
			//numHubs.set(ofToString(numPorts));
			//numNodes.set(ofToString(myPort.NodeCount()));
		}
		else {
			ofLogWarning("ofApp::initialize") << "Unable to locate SC hub port";

			// Update the GUI
			//systemStatus.set("No SC Hub Ports Found");
			//numHubs.set("0");

			return false;
		}
		return true;
	}
	catch (mnErr& theErr)
	{
		ofLogError("ofApp::initialize") << "Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port";
		ofLogError("ofApp::initialize") << "Caught error: addr=" << ofToString(theErr.TheAddr) << ", err=" << ofToString(theErr.ErrorCode) << ", msg=" << ofToString(theErr.ErrorMsg);

		// Update the GUI
		//systemStatus.set("Error: " + ofToString(theErr.ErrorMsg));
		//numHubs.set("0");
		return false;
	}
}


/**
 * @brief Close all operations down and close the ports.
 *
 * This function closes the serial ports in the system and release all resources related to them.
 *
 */
void MotorController::close() {
	if (myMgr != nullptr) {
		ofLogNotice() << "Closing HUB Ports...";
		myMgr->PortsClose();
	}
}