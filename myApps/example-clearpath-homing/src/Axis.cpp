#include <stdio.h>
#include "Axis.h"

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#endif


/**
 * @brief Creates the Axis object and its GUI.
 *
 * @param (SysManager&)  SysMgr:
 * @param (INode*)  node:
 * @param (int)  iPort:
 * @param (int)  iNode:
 */
Axis::Axis(SysManager& SysMgr, INode* node, int iPort, int iNode) :
	m_node(node),
	m_moveCount(0),
	m_positionWrapCount(0),
	m_quitting(false),
	m_sysMgr(SysMgr) {

	if (ofGetLogLevel() == OF_LOG_NOTICE) {
		printf("   Node[%d]: type=%d\n", iNode, m_node->Info.NodeType());
		printf("            userID: %s\n", m_node->Info.UserID.Value());
		printf("        FW version: %s\n", m_node->Info.FirmwareVersion.Value());
		printf("          Serial #: %d\n", m_node->Info.SerialNumber.Value());
		printf("             Model: %s\n\n", m_node->Info.Model.Value());
	}

	/*
		// OPTIONAL - Save the config file before starting
		// This would typically be performed right after tuning each axis.
		// Chosing a file name that contains the address and serial number
		// can ensure that the system does not get miswired.
		// Using the UserID as a filename key can allow swapping motors with
		// minimal setup required. This requires that no invalid characters
		// are present in the UserID.
	#if defined(_WIN32) || defined(_WIN64)
		char tempPath[MAX_CONFIG_PATH_LENGTH];
		GetTempPathA(MAX_CONFIG_PATH_LENGTH, tempPath);
		snprintf(m_configFile, sizeof(m_configFile), "%s\\config-%02d-%d.mtr", tempPath,
			m_node->Info.Ex.Addr(), m_node->Info.SerialNumber.Value());
	#else
		snprintf(m_configFile, sizeof(m_configFile), "/tmp/config-%02d-%d.mtr",
			m_node->Info.Ex.Addr(), m_node->Info.SerialNumber.Value());
	#endif
	if (m_node->Setup.AccessLevelIsFull())
		m_node->Setup.ConfigSave(m_configFile);
	*/

	this->iPort = iPort;
	this->iNode = iNode;
	//CheckMotorStatus();	// <-- called inside setup_gui() now
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

void Axis::update() {
	if (m_node != NULL) {
		auto curr_position = GetPosition();
		position_cnts.set(ofToString(curr_position));
		position_mm.set(ofToString(count_to_mm(curr_position)));
		auto val = mm_to_count(count_to_mm(curr_position));
		if (val != 0)
			cout << "checking mm_to_count: " << ofToString(val) << endl;
	}
}

/**
 * @brief Converts from motor position (count) to linear distance (mm).
 *
 * @param (int)  val: motor position (in step counts)
 * @param (float)  diameter: diameter of cable drum
 * @return (float)  linear distance (in mm)
 */
float Axis::count_to_mm(int val, float diameter)
{
	float step_resolution = 6400.0;
	float circumference = PI * diameter;
	float mm_per_step = circumference / step_resolution;	// 0.01963495408 @diameter = 40
	return val * mm_per_step;
}

/**
 * @brief Converts from linear distance (mm) to motor position.
 *
 * @param (int)  val:  linear distance (in mm)
 * @param (float)  diameter: diameter of cable drum
 * @return (float) motor position (in step counts)
 */
int Axis::mm_to_count(float val, float diameter)
{
	float step_resolution = 6400.0;
	float circumference = PI * diameter;
	float mm_per_step = circumference / step_resolution;	// 0.01963495408 @diameter = 40
	return val / mm_per_step;
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
		status.set("E-STOPPED");
		ofLogNotice("Axis::clearMotionStop") << "(Port " << iPort << ", Node " << ofToString(m_node->Info.Ex.Addr()) << ") is e-stopped: Clearing E-Stop.";
		m_node->Motion.NodeStopClear();
		status.set("");
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
			ofLogWarning("Axis::WaitForMove") << "ERROR: " << ofToString(m_node->Info.Ex.Addr()) << " Timed out during move.";
			return false;
		}

		// Catch specific errors that can occur during a move
		if (m_node->Status.RT.Value().cpm.Disabled) {
			ofLogWarning("Axis::WaitForMove") << "ERROR: " << ofToString(m_node->Info.Ex.Addr()) << " disabled during move.";
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
 * @brief Checks the motor state (enabled, estopped, homed) and updates the GUI.
 *
 */
void Axis::CheckMotorStatus()
{
	if (m_node != NULL) {
		// Refresh our relevant registers
		m_node->Status.Alerts.Refresh();
		m_node->Status.RT.Refresh();

		// Check for E-STOP first, it updates enable
		bool eStoped;
		if (m_node->Status.Alerts.Value().cpm.Common.EStopped) {
			eStop.set(true);
		}
		else {
			eStop.set(false);
		}

		// check for enable
		bool enabled = m_node->EnableReq();
		enable.set(enabled);
		status.set(enabled ? "Enabled" : "Disabled");

		// check for IsHomed
		CheckHomingStatus();
	}

}

/**
 * @brief Checks whether or not Homing parameters have been set up in ClearView for a motor.
 * Updates the GUI with a "HOMED" or "NOT HOMED" status.
 * 
 */
void Axis::CheckHomingStatus()
{
	if (m_node != NULL) {
		if (m_node->Motion.Homing.HomingValid())
		{
			if (m_node->Motion.Homing.WasHomed())
			{
				auto curr_position = m_node->Motion.PosnMeasured.Value();
				position_cnts.set(ofToString(curr_position));
				position_mm.set(ofToString(count_to_mm(curr_position)));
				ofLogNotice("Axis::CheckHomingStatus") << "Node " << this->iNode << " is already homed. Current Pos: " << curr_position << endl;
				homing_status.set("HOMED");
			}
			else
			{
				homing_status.set("NOT HOMED");
				ofLogNotice("Axis::CheckHomingStatus") << "Node " << this->iNode << " HAS NOT BEEN homed." << endl;
			}
		}
		else {
			ofLogWarning("Axis::CheckHomingStatus") << "Node " << this->iNode << " cannot be homed. It needs to first setup homing parameters through ClearView." << endl;
		}
	}
}


/**
 * @brief Example of how to check and handle Status Alerts for nodes.
 *
 */
void Axis::CheckForAlerts()
{
	if (m_node != NULL) {
		// Refresh our relevant registers
		m_node->Status.Alerts.Refresh();
		m_node->Status.RT.Refresh();

		int id = m_node->Info.Ex.Addr();

		// Check for alerts
		if (m_node->Status.RT.Value().cpm.AlertPresent) {
			char alertList[256];
			m_node->Status.Alerts.Value().StateStr(alertList, 256);

			ofLogNotice("Axis::checkForAlerts") << "Node" << id << " has alerts!Alert: \n\n" << alertList << endl;

			if (m_node->Status.Alerts.Value().isInAlert()) {

				// Examples of checking & handling for specific alerts
				// E-Stop
				if (m_node->Status.Alerts.Value().cpm.Common.EStopped) {
					ofLogNotice("Axis::checkForAlerts") << "\t\tNode " << ofToString(id) << " is e-stopped: Clearing E-Stop";
					status.set("E-STOPPED");
					m_node->Motion.NodeStopClear();
					status.set("");
				}
				// Tracking Error
				else if (m_node->Status.Alerts.Value().cpm.TrackingShutdown) {
					//ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(id) << " exceeded Tracking error limit";
					status.set("TRACKING ERROR");
				}
				// Non-Serious Non-E-stop Errors
				else {
					ofLogNotice("Axis::checkForAlerts") << "\t\tNode " << ofToString(id) << " has non-serious errors. Clearing Now.";
					m_node->Status.AlertsClear();
				}
			}
		}
		else {
			ofLogNotice("Axis::checkForAlerts") << "\t\tNode " << ofToString(id) << " has no alerts!";
		}
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
 * @brief Moves the motor to the target position.
 * DOES NOT CHECK FOR TIMEOUT
 *
 * @param (int32_t)  targetPosn:
 * @param (bool)  isAbsolute:
 * @param (bool)  addDwell:
 */
void Axis::SetPosition(int32_t targetPosn, bool isAbsolute, bool addDwell)
{
	// Save the move's calculated time for timeout handling
	//m_moveTimeoutMs = (int32_t)m_node->Motion.MovePosnDurationMsec(targetPosn,
	//	isAbsolute);
	//m_moveTimeoutMs += 20;

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
 * @brief Create a GUI for the Axis.
 *
 */
void Axis::setup_gui()
{
	mode_color_eStop = ofColor(250, 0, 0, 100);
	mode_color_normal = ofColor(60, 120);
	mode_color_disabled = ofColor(0, 200);

	int gui_width = 250;

	params_homing.setName("Homing_Parameters");
	params_homing.add(homing_status.set("Homing_Status", "NOT HOMED"));
	//params_homing.add(rehome.set("GO_TO_HOME", false));
	params_homing.add(reset_home_position.set("Reset_Home_Position", false));


	params_motion.setName("Motion_Parameters");
	params_motion.add(vel.set("Velocity", 300, 0, 600));
	params_motion.add(accel.set("Acceleration", 2000, 0, 4000));

	params_macros.setName("Macros");
	params_macros.add(move_trigger.set("Trigger_Move", false));
	params_macros.add(move_target_cnts.set("Move_Target(cnts)", 0, -64000, 64000));
	params_macros.add(move_target_mm.set("Move_Target(mm)", 0, 0, 1900));		// 3770mm is theoretical spool capacity @diameter = 40mm
	params_macros.add(move_absolute_pos.set("Move_Absolute", true));
	params_macros.add(move_zero.set("Move_to_Zero", false));


	panel.setup("Node_" + ofToString(this->iNode));
	panel.setWidthElements(gui_width);
	panel.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 5);
	panel.add(status.set("Status:", "Disabled"));
	enable.setName("Enable");
	eStop.setName("E_STOP");
	position_cnts.setName("Position(cnts)");
	position_mm.setName("Position(mm)*");
	CheckMotorStatus();		// updates enable, eStop, homing, position_cnts, position_mm
	panel.add(enable);
	panel.add(eStop);
	panel.add(position_cnts);
	panel.add(position_mm);
	panel.add(params_homing);
	panel.add(params_motion);
	panel.add(params_macros);

	enable.addListener(this, &Axis::on_enable);
	eStop.addListener(this, &Axis::on_eStop);

	vel.addListener(this, &Axis::on_vel_changed);
	accel.addListener(this, &Axis::on_accel_changed);
	move_trigger.addListener(this, &Axis::on_move_trigger);
	move_zero.addListener(this, &Axis::on_move_zero);

	reset_home_position.addListener(this, &Axis::on_reset_home_position);
	move_target_mm.addListener(this, &Axis::on_move_target_mm);
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
		status.set("E-STOPPED");
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

void Axis::on_rehome(bool& val)
{
	// @TODO
}

/**
 * @brief Uses the current motor position as the new HOME position (posn=0).
 * 
 * @param (bool)  val: 
 */
void Axis::on_reset_home_position(bool& val)
{
	if (m_node != NULL) {
		if (val) {
			// Refresh our current measured position
			m_node->Motion.PosnMeasured.Refresh();
			auto curr_pos = m_node->Motion.PosnMeasured.Value();

			ofLogNotice("Axis::on_reset_home_position") << "Resetting Home Position for Node " << iNode << ". Adjusting by " << ofToString(-1 * curr_pos) << " counts.";

			// Now the node is no longer considered "homed", and soft limits are turned off
			m_node->Motion.AddToPosition(-1 * curr_pos);

			// Reset the Node's "sense of home" soft limits (unchanged) are now active again
			m_node->Motion.Homing.SignalComplete();

			// Refresh our current measured position
			m_node->Motion.PosnMeasured.Refresh();
			curr_pos = m_node->Motion.PosnMeasured.Value();
			position_cnts.set(ofToString(curr_pos));
			ofLogNotice("Axis::on_reset_home_position") << "\tNumberspace changed. Current Position: " << curr_pos << endl;

			// turn off the reset home button in the gui
			reset_home_position.set(false);
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
		auto MOVE_DISTANCE_CNTS = move_target_cnts.get();
		cout << "Axis::on_move_trigger: move_target_cnts val" << MOVE_DISTANCE_CNTS << endl;

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
		move_target_mm.set(0);
		move_absolute_pos.set(true);
		move_trigger.set(true);
		move_zero.set(false);
	}
}

/**
 * @brief Callback to convert mm to motor counts.
 * 
 * @param ()  val: linear distance (in mm)
 */
void Axis::on_move_target_mm(float& val)
{
	move_target_cnts.set(mm_to_count(val));
}
