#include <stdio.h>
#include "Axis.h"

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#endif


/**
 * @brief Creates the Axis object and its GUI.
 * 
 * @param ()  SysMgr: 
 * @param ()  node: 
 * @param ()  iPort: 
 * @param ()  iNode: 
 */
Axis::Axis(SysManager& SysMgr, INode* node, int iPort, int iNode) :
	m_node(node),
	m_moveCount(0),
	m_positionWrapCount(0),
	m_quitting(false),
	m_sysMgr(SysMgr) {

//	// OPTIONAL - Save the config file before starting
//	// This would typically be performed right after tuning each axis.
//	// Chosing a file name that contains the address and serial number
//	// can ensure that the system does not get miswired.
//	// Using the UserID as a filename key can allow swapping motors with 
//	// minimal setup required. This requires that no invalid characters 
//	// are present in the UserID.
//#if defined(_WIN32) || defined(_WIN64)
//	char tempPath[MAX_CONFIG_PATH_LENGTH];
//	GetTempPathA(MAX_CONFIG_PATH_LENGTH, tempPath);
//	snprintf(m_configFile, sizeof(m_configFile), "%s\\config-%02d-%d.mtr", tempPath,
//		m_node->Info.Ex.Addr(), m_node->Info.SerialNumber.Value());
//#else
//	snprintf(m_configFile, sizeof(m_configFile), "/tmp/config-%02d-%d.mtr",
//		m_node->Info.Ex.Addr(), m_node->Info.SerialNumber.Value());
//#endif
//
//	if (m_node->Setup.AccessLevelIsFull())
//		m_node->Setup.ConfigSave(m_configFile);

	this->iPort = iPort;
	this->iNode = iNode;
	setup_gui();
	InitMotionParams();

};

/**
 * @brief Destructor for the axis state.
 * Disable the node and restore the saved config file.
 */
Axis::~Axis() {
	// Print out the move statistics one last time
	PrintStats();

	// If we don't have full access, there's nothing to do here
	if (!m_node->Setup.AccessLevelIsFull()) {
		return;
	}

	// Disable the node and wait for it to disable
	m_node->EnableReq(false);

	// Poll the status register to confirm the node's disable
	time_t timeout;
	timeout = time(NULL) + 3;
	m_node->Status.RT.Refresh();
	while (m_node->Status.RT.Value().cpm.Enabled) {
		if (time(NULL) > timeout) {
			printf("Error: Timed out waiting for disable\n");
			return;
		}
		m_node->Status.RT.Refresh();
	};

	// Restore the original config file
	m_node->Setup.ConfigLoad(m_configFile);
}

/**
 * @brief Clears & flushes alerts, enable the node and  wait within a timeout limit for it to enable.
 * 
 */
void Axis::Enable() {
	if (m_node != NULL) {
		// Clear alerts and node stops
		m_node->Status.AlertsClear();
		m_node->Motion.NodeStop(STOP_TYPE_ABRUPT);
		m_node->Motion.NodeStop(STOP_TYPE_CLR_ALL);

		// Enable the node
		m_node->EnableReq(true);

		// If the node is not currently ready, wait for it to get there
		time_t timeout;
		timeout = time(NULL) + 3;
		// Basic mode - Poll until disabled
		while (!m_node->Status.IsReady()) {
			if (time(NULL) > timeout) {
				ofLogWarning("Axis::Enable()") << "Error: Timed out waiting for enable";
				return;
			}
		}
	}
}

/**
 * @brief Disable the node and wait within a timeout limit for it to disable.
 *
 */
void Axis::Disable()
{
	// Disable the node and wait for it to disable
	m_node->EnableReq(false);

	// Poll the status register to confirm the node's disable
	time_t timeout;
	timeout = time(NULL) + 3;
	m_node->Status.RT.Refresh();
	while (m_node->Status.RT.Value().cpm.Enabled) {
		if (time(NULL) > timeout) {
			printf("Error: Timed out waiting for disable\n");
			return;
		}
		m_node->Status.RT.Refresh();
	};
}

/**
 * @brief  This function clears the MotionLock, E-Stop, Controlled, Disabled, etc. latching conditions.
 * This allows normal operations to continue, unless the motor is shutdown state.
 */
void Axis::clearMotionStops() {
	// Update the registers
	m_node->Status.RT.Refresh();
	m_node->Status.Alerts.Refresh();

	if (m_node->Status.Alerts.Value().cpm.Common.EStopped) {
		ofLogNotice("Axis::clearMotionStop") << "(Port " << iPort << ", Node " << ofToString(m_node->Info.Ex.Addr()) << ") is e-stopped: Clearing E-Stop.";
		m_node->Motion.NodeStopClear();
	}
	else {
		ofLogNotice("Axis::clearMotionStop") << "(Port " << iPort << ", Node " << ofToString(m_node->Info.Ex.Addr()) << ") is NOT E-Stopped.";
	}
}

/**
 * @brief Stop the executing motion for a given.
 * Flush any pending motion commands and ramp the node's speed to zero using the E-Stop Deceleration Rate.
 *
 * Future motion is blocked until a directed NodeStopClear is sent.
 *
 * WARNING: Motor Shaft will freely move on E-Stop.
 *
 */
void Axis::triggerEStop() {
	ofLogNotice("Axis::triggerEStop") << "Triggering E-Stop at (Port " << iPort << ", Node " << ofToString(m_node->Info.Ex.Addr()) << ").";
	m_node->Motion.NodeStop(STOP_TYPE_ESTOP_RAMP);
}

/**
 * @brief Initialize vel, accel, position tracking, jerk and global torque limits.
 * 
 */
void Axis::InitMotionParams() {
	// Set the user units to RPM and RPM/s
	m_node->VelUnit(INode::RPM);
	m_node->AccUnit(INode::RPM_PER_SEC);

	// Set the motion's limits
	m_node->Motion.VelLimit = vel.get();
	m_node->Motion.AccLimit = accel.get();
	m_node->Motion.JrkLimit = 3;
	m_node->Limits.PosnTrackingLimit.Value(m_node->Info.PositioningResolution.Value() / 4);
	m_node->Limits.TrqGlobal.Value(100);

	// Set the dwell used by the motor when applicable
	m_node->Motion.DwellMs = 5;
}


/**
 * @brief Issues a position move.
 * 
 * @param (int32_t)  targetPosn: target motor position.
 * @param (bool)  isAbsolute: target is abosulte or relative to current postion.
 * @param (bool)  addDwell: whether to delay the next move being issued by DwellMs after this move profile completes.
 */
void Axis::PosnMove(int32_t targetPosn, bool isAbsolute, bool addDwell) {
	// Save the move's calculated time for timeout handling
	m_moveTimeoutMs = (int32_t)m_node->Motion.MovePosnDurationMsec(targetPosn,
		isAbsolute);
	m_moveTimeoutMs += 20;

	// Check for number-space wrapping. With successive moves, the position may
	// pass the 32-bit number cap (+ or -). The check below looks for that and
	// updates our counter to be used with GetPosition
	if (!isAbsolute) {
		if (GetPosition(false) + targetPosn > INT32_MAX) {
			m_positionWrapCount++;
		}
		else if (GetPosition(false) + targetPosn < INT32_MIN) {
			m_positionWrapCount--;
		}
	}

	// Issue the move to the node
	m_node->Motion.MovePosnStart(targetPosn, isAbsolute, addDwell);
}

/**
 * @brief Issues a velocity move.
 * 
 * @param (double)  targetVel: 
 * @param (uint32_t)  duration: 
 */
void Axis::VelMove(double targetVel, uint32_t duration) {
	// Issue the move to the node
	m_node->Motion.MoveVelStart(targetVel);

	// Move at the set velocity for half a second
	m_sysMgr.Delay(duration);

	// Issue a node stop, forcing any movement (like a 
	// velocity move) to stop. There are several 
	// different types of node stops
	m_node->Motion.NodeStop(STOP_TYPE_ABRUPT);
	//m_node->Motion.NodeStop(STOP_TYPE_RAMP);

	// Configure the timeout for the node stop to settle
	m_moveTimeoutMs = 100;
}

/**
 * @brief Wait for attention that move has completed.
 * 
 * @param (int32_t)  timeoutMs: 
 * @param (bool)  secondCall: 
 * @return (bool)  True if the Move Successfully Finished
 */
bool Axis::WaitForMove(int32_t timeoutMs, bool secondCall) {
	if (m_node == NULL) {
		return false;
	}

	// Wait for the proper calculated move time
	time_t timeout;
	timeout = time(NULL) + timeoutMs;

	m_node->Status.RT.Refresh();
	// Basic mode - Poll until move done
	while (!m_node->Status.RT.Value().cpm.MoveDone) {
		m_node->Status.RT.Refresh();
		if (time(NULL) > timeout) {
			ofLogWarning("Axis::WaitForMove") << "ERROR: "<< ofToString(m_node->Info.Ex.Addr()) << " Timed out during move.";
			return false;
		}

		// Catch specific errors that can occur during a move
		if (m_node->Status.RT.Value().cpm.Disabled) {
			ofLogWarning("Axis::WaitForMove") << "ERROR: "<< ofToString(m_node->Info.Ex.Addr()) << " disabled during move.";
			return false;
		}
		if (m_node->Status.RT.Value().cpm.NotReady) {
			ofLogWarning("Axis::WaitForMove") << "ERROR: " << ofToString(m_node->Info.Ex.Addr()) << " went NotReady during move.";
			return false;
		}
	}

	return m_node->Status.RT.Value().cpm.MoveDone;
}



/**
 * @brief Example of how to check and handle Status Alerts for nodes.
 *
 */
void Axis::CheckForAlerts()
{
	// Refresh our relevant registers
	m_node->Status.Alerts.Refresh();
	m_node->Status.RT.Refresh();

	int id = m_node->Info.Ex.Addr();

	// Check for alerts
	if (m_node->Status.RT.Value().cpm.AlertPresent) {
		char alertList[256];
		m_node->Status.Alerts.Value().StateStr(alertList, 256);

		ofLogNotice("ofApp::checkForAlerts") << "Node" << id << " has alerts!Alert: \n\n" << alertList << endl;

		if (m_node->Status.Alerts.Value().isInAlert()) {

			// Examples of checking & handling for specific alerts
			// E-Stop
			if (m_node->Status.Alerts.Value().cpm.Common.EStopped) {
				//ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(id) << " is e-stopped: Clearing E-Stop";
				//m_node->Motion.NodeStopClear();
			}
			// Tracking Error
			else if (m_node->Status.Alerts.Value().cpm.TrackingShutdown) {
				//ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(id) << " exceeded Tracking error limit";
			}
			// Non-Serious Non-E-stop Errors
			else {
				//m_node->Status.AlertsClear();
			}
		}
	}
	else {
		ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(id) << " has no alerts!";
	}

}

/**
 * @brief Setup the hardware brake control on the node's port. Set's the indicated brake's 
 * mode to the passed-in value.
 * 
 * @param (size_t)  brakeNum: 
 * @param (BrakeControls)  brakeMode: 
 */
void Axis::SetBrakeControl(size_t brakeNum, BrakeControls brakeMode)
{
	m_node->Port.BrakeControl.BrakeSetting(brakeNum, brakeMode);
}


/**
 * @brief Return the node's commanded position scaled properly to wraps of the number space.
 * 
 * @param (bool)  includeWraps: 
 * @return (int64_t)  
 */
int64_t Axis::GetPosition(bool includeWraps)
{
	// Create a variable to return and refresh the position
	long long scaledPosn;
	m_node->Motion.PosnCommanded.Refresh();

	// If there have been no number wraps, just return the position
	scaledPosn = int64_t(m_node->Motion.PosnCommanded.Value());

	if (includeWraps) {
		scaledPosn += int64_t(m_positionWrapCount) << 32;
	}

	return scaledPosn;
}

/**
 * @brief Create a GUI for the Axis.
 *
 */
void Axis::setup_gui()
{
	mode_color_eStop = ofColor(250, 0, 0, 100);
	mode_color_normal = ofColor(60, 120);
	mode_color_disabled = ofColor(0, 200);

	int gui_width = 250;
	params_motion.setName("Motion_Parameters");
	params_motion.add(vel.set("Velocity", 700, 0, 4000));
	params_motion.add(accel.set("Acceleration", 100000, 0, 500000));
	//params_motion.add(decel.set("Deceleration", 0, 0, 0));
	params_macros.setName("Macros");
	params_macros.add(move_trigger.set("Trigger_Move", false));
	params_macros.add(move_target.set("Move_Target", 0, -50000, 50000));
	params_macros.add(move_absolute_pos.set("Move_Absolute", false));
	params_macros.add(move_zero.set("Move_to_Zero", false));

	enable.addListener(this, &Axis::on_enable);
	eStop.addListener(this, &Axis::on_eStop);
	vel.addListener(this, &Axis::on_vel_changed);
	accel.addListener(this, &Axis::on_accel_changed);
	move_trigger.addListener(this, &Axis::on_move_trigger);
	move_zero.addListener(this, &Axis::on_move_zero);

	panel.setup("Node_" + this->iNode);
	panel.setWidthElements(gui_width);
	panel.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 5);
	panel.add(status.set("Status:", "Disabled"));
	panel.add(enable.set("Enable", false));
	panel.add(eStop.set("E_STOP", false));
	panel.add(params_motion);
	panel.add(params_macros);
}



/**
 * @brief Enables or Disables the axis.
 *
 * @param (bool)  val
 */
void Axis::on_enable(bool& val)
{
	// Don't update if there is a FULL SYSTEM E-STOP
	if (eStopAll) {
		// Override user input
		enable.set(false);
	}
	else {
		if (val) {
			if (eStop)
				eStop.set(false);
			Enable();
			panel.setBorderColor(mode_color_normal);
			status.set("Enabled");
		}
		else {
			Disable();
			panel.setBorderColor(mode_color_disabled);
			status.set("Disabled");
		}
	}
}

/**
 * @brief Triggers or Clears an E-Stop for the motor.
 *
 * @param (bool)  val
 */
void Axis::on_eStop(bool& val)
{
	if (val) {
		if (enable)
			enable.set(false); // disable & uncheck enable in the GUI
		triggerEStop();
		panel.setBorderColor(mode_color_eStop);
		status.set("E-Stop");
	}
	else {
		// Don't update if there is a FULL SYSTEM E-STOP
		if (eStopAll) {
			// Override user input
			eStop.set(true);
		}
		else {
			clearMotionStops();
			panel.setBorderColor(mode_color_normal);
			if (!enable)
				enable.set(false);		// disable motor after E-Stop recovery
		}
	}
}

/**
 * @brief Callback to update velocity limit.
 * 
 * @param (float)  val: 
 */
void Axis::on_vel_changed(float& val)
{
	m_node->Motion.VelLimit = val;
}

/**
 * @brief Callback to update acceleration limit.
 * 
 * @param (float)  val: 
 */
void Axis::on_accel_changed(float& val)
{
	m_node->Motion.AccLimit = val;
}

/**
 * @brief Callback to trigger a move to the target postion.
 * 
 * @param (bool)  val: 
 */
void Axis::on_move_trigger(bool& val)
{
	if (val && !eStop.get()) {

		//int MOVE_DISTANCE_CNTS = resolution * revs;
		int MOVE_DISTANCE_CNTS = move_target.get();

		// NOTE: The program will crash if you send a move if the motor is disabled.
		// If the motor is enabled, run the move and print the duration
		if (m_node->Status.RT.Value().cpm.Enabled) {
			printf("Moving Node \t%zi \n", 0);
			m_node->Motion.MovePosnStart(MOVE_DISTANCE_CNTS, move_absolute_pos.get());			//Execute 10000 encoder count move 
			printf("\t%f estimated time for Node 0.\n", m_node->Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTS));
		}
		else {
			ofLogNotice("Axis::on_move_trigger") << "Axis " << iNode << " cannot move. It is disabled.";
		}

		move_trigger.set(false);
	}
}

/**
 * @brief Callback to trigger a move to absolute zero position.
 * 
 * @param (bool)  val: 
 */
void Axis::on_move_zero(bool& val)
{
	if (val) {
		move_target.set(0);
		move_absolute_pos.set(true);
		move_trigger.set(true);
		move_zero.set(false);
	}
}
