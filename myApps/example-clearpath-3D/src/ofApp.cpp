#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	ofLogToConsole();

	setup_gui();

	if (!initialize()) {
		close();
		exit();	// <-- not killing the program for some reason
	}

	int x = panel.getPosition().x;
	int y = panel.getPosition().y + panel.getHeight() + 5;
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->panel.setPosition(x, y);
		y += axes[i]->panel.getHeight() + 5;
	}
}

//--------------------------------------------------------------
void ofApp::update() {

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(60);

	if (showGUI) {
		panel.draw();
		//panel_node.draw();
		for (int i = 0; i < axes.size(); i++) {
			axes[i]->panel.draw();
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
				for (int iNode = 0; iNode < myPort.NodeCount(); iNode++) {

					axes.push_back(new Axis(*myMgr, &myPort.Nodes(iNode), i, iNode));

					//setup_node(i, iNode);
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

void ofApp::setup_node(int iPort, int iNode)
{
	IPort& myPort = myMgr->Ports(iPort);
	INode& node = myPort.Nodes(iNode);
	node.AccUnit(INode::RPM_PER_SEC);				// Set the units for Acceleration to RPM/SEC
	node.VelUnit(INode::RPM);						// Set the units for Velocity to RPM
	node.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		// Set Acceleration Limit (RPM/Sec)
	node.Motion.VelLimit = VEL_LIM_RPM;				// Set Velocity Limit (RPM)
}

string ofApp::node_toString(int iPort, int iNode)
{
	string ss = "null ";
	if (myMgr != nullptr) {
		IPort& myPort = myMgr->Ports(iPort);
		INode& node = myPort.Nodes(iNode);
		double accLim = node.Motion.AccLimit;
		double velLim = node.Motion.VelLimit;
		node.Motion.PosnMeasured.Refresh();
		double pos = node.Motion.PosnMeasured.Value();

		ss = "Node {" + ofToString(iPort) + ":" + ofToString(iNode) + "}\n";
		ss += "\t VelLimit: " + ofToString(velLim) + "\n\t AccLimit: " + ofToString(accLim) + "\n";;
		ss += "\t CurrPos: " + ofToString(pos);
	}

	return ss;
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
 * @brief Example of how to check for Status Alerts for nodes.
 *
 */
void ofApp::checkForAlerts() {

	for (int iPort = 0; iPort < numPorts; iPort++) {
		// Get a reference to the port, to make accessing it easier
		IPort& myPort = myMgr->Ports(iPort);

		char alertList[256];

		ofLogNotice("ofApp::checkForAlerts") << "Checking for Alerts:";

		for (int iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			// Get a reference to the node, to make accessing it easier
			INode& node = myPort.Nodes(iNode);

			// make sure our registers are up to date
			node.Status.RT.Refresh();
			node.Status.Alerts.Refresh();


			ofLogNotice("ofApp::checkForAlerts") << "\tChecking Node " << iNode << " for Alerts: ";

			// Check the status register's "AlertPresent" bit
			// The bit is set true if there are alerts in the alert register
			if (!node.Status.RT.Value().cpm.AlertPresent) {
				ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << " has no alerts!";
			}
			//Check to see if the node experienced torque saturation
			else if (node.Status.HadTorqueSaturation()) {
				ofLogWarning("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << " has experienced torque saturation since last checking";
			}
			// get an alert register reference, check the alert register directly for alerts
			else if (node.Status.Alerts.Value().isInAlert()) {
				// get a copy of the alert register bits and a text description of all bits set
				node.Status.Alerts.Value().StateStr(alertList, 256);
				ofLogWarning("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << " has alerts! Alerts:\n" << alertList << "\n\n";

				// Example:
				// Access and Clear specific alerts using the method below:
				if (node.Status.Alerts.Value().cpm.Common.EStopped) {
					ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << " is e - stopped: Clearing E - Stop";
					node.Motion.NodeStopClear();
				}
				if (node.Status.Alerts.Value().cpm.TrackingShutdown) {
					ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << " exceeded Tracking error limit";
				}

				// Example:
				// Check for more Alerts and Clear Alerts
				node.Status.Alerts.Refresh();
				if (node.Status.Alerts.Value().isInAlert()) {
					node.Status.Alerts.Value().StateStr(alertList, 256);
					ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << "has non - estop alerts : \n" << alertList << "\n\n";
					ofLogNotice("ofApp::checkForAlerts") << "\t\tClearing non-serious alerts";
					node.Status.AlertsClear();

					// Are there still alerts?
					node.Status.Alerts.Refresh();
					if (node.Status.Alerts.Value().isInAlert()) {
						node.Status.Alerts.Value().StateStr(alertList, 256);
						ofLogError("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << " has serious, non-clearing alerts:\n" << alertList << "\n\n";
					}
					else {
						ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << ": all alerts have been cleared";
					}
				}
				else {
					ofLogNotice("ofApp::checkForAlerts") << "\t\tNode " << ofToString(node.Info.Ex.Addr()) << ": all alerts have been cleared";
				}
			}
		}
	}
}

/**
 * @brief This function clears the MotionLock, E-Stop, Controlled, Disabled, etc. latching conditions from all nodes.
 * This allows normal operations to continue, unless the motor is shutdown state.
 *
 */
void ofApp::clearMotionStops()
{
	for (int iPort = 0; iPort < numPorts; iPort++) {
		int nodeCount = myMgr->Ports(iPort).NodeCount();
		for (int iNode = 0; iNode < nodeCount; iNode++) {
			clearMotionStop(iPort, iNode);
		}
	}
}

/**
 * @brief  This function clears the MotionLock, E-Stop, Controlled, Disabled, etc. latching conditions from a given node.
 * This allows normal operations to continue, unless the motor is shutdown state.
 *
 * @param[in] (int)  iPort: index of SC-HUB
 * @param[in] (int)  iNode: index of node on the port
 *
 */
void ofApp::clearMotionStop(int iPort, int iNode)
{
	if (myMgr == nullptr) {
		return;
	}
	else if (iPort < 0 || iPort >= numPorts) {
		ofLogWarning("ofApp::clearMotionStop") << "The Port Index {" << iPort << "} is outside the NodeCount range of {" << numPorts << "}.";
		return;
	}
	else if (iNode < 0 || iNode >= myMgr->Ports(iPort).NodeCount()) {
		ofLogWarning("ofApp::clearMotionStop") << "The Node Index {" << iNode << "} is outside the NodeCount range of {" << myMgr->Ports(iPort).NodeCount() << "}.";
		return;
	}

	IPort& myPort = myMgr->Ports(iPort);
	INode& node = myPort.Nodes(iNode);

	// Update the registers
	node.Status.RT.Refresh();
	node.Status.Alerts.Refresh();

	if (node.Status.Alerts.Value().cpm.Common.EStopped) {
		ofLogNotice("ofApp::clearMotionStop") << "(Port " << iPort << ", Node " << ofToString(node.Info.Ex.Addr()) << ") is e - stopped: Clearing E - Stop.";
		node.Motion.NodeStopClear();
	}
	else {
		ofLogNotice("ofApp::clearMotionStop") << "(Port " << iPort << ", Node " << ofToString(node.Info.Ex.Addr()) << ") is NOT E-Stopped.";
	}

}

/**
 * @brief Stop the executing motion for all nodes.
 * Flush any pending motion commands and ramp the node's speed to zero using the E-Stop Deceleration Rate.
 *
 * WARNING: Motor Shaft will freely move on E-Stop.
 *
 * Future motion is blocked until a directed NodeStopClear is sent.
 */
void ofApp::triggerEStops()
{
	for (int iPort = 0; iPort < numPorts; iPort++) {
		int nodeCount = myMgr->Ports(iPort).NodeCount();
		for (int iNode = 0; iNode < nodeCount; iNode++) {
			triggerEStop(iPort, iNode);
		}
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
 * @param (int)  iPort: index of SC-HUB
 * @param (int)  iNode: index of node on the port
 */
void ofApp::triggerEStop(int iPort, int iNode)
{
	if (iPort < numPorts) {
		IPort& myPort = myMgr->Ports(iPort);
		if (iNode < myPort.NodeCount()) {
			INode& node = myPort.Nodes(iNode);
			ofLogNotice("ofApp::triggerEStop") << "Triggering E-Stop at (Port " << iPort << ", Node " << ofToString(node.Info.Ex.Addr()) << ").";
			node.Motion.NodeStop(STOP_TYPE_ESTOP_RAMP);
		}
		else {
			ofLogWarning("ofApp::triggerEStop") << "The Node Index {" << iNode << "} is outside the NodeCount range of {" << myPort.NodeCount() << "}.";
		}
	}
	else {
		ofLogWarning("ofApp::triggerEStop") << "The Port Index {" << iPort << "} is outside the NodeCount range of {" << numPorts << "}.";
	}
}

void ofApp::enableMotors(bool val)
{
	for (int iPort = 0; iPort < numPorts; iPort++) {
		int nodeCount = myMgr->Ports(iPort).NodeCount();
		for (int iNode = 0; iNode < nodeCount; iNode++) {
			enableMotor(val, iPort, iNode);
		}
	}
}

void ofApp::enableMotor(bool val, int iPort, int iNode)
{
	// Check that the port and node indices are within range
	if (myMgr == nullptr) {
		return;
	}
	else if (iPort < 0 || iPort >= numPorts) {
		ofLogWarning("ofApp::enableMotor") << "The Port Index {" << iPort << "} is outside the NodeCount range of {" << numPorts << "}.";
		return;
	}
	else if (iNode < 0 || iNode >= myMgr->Ports(iPort).NodeCount()) {
		ofLogWarning("ofApp::enableMotor") << "The Node Index {" << iNode << "} is outside the NodeCount range of {" << myMgr->Ports(iPort).NodeCount() << "}.";
		return;
	}

	IPort& myPort = myMgr->Ports(iPort);
	INode& node = myPort.Nodes(iNode);
	if (val) {
		node.Status.AlertsClear();			// Clear Alerts on node 
		node.Motion.NodeStopClear();		// Clear Nodestops on Node  				
		node.EnableReq(true);				// Enable node 
	}
	else {
		node.EnableReq(false);				// Enable node 
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

	// SC Hub Parameters
	params_hub.setName("Hub_0");
	params_hub.add(comPortNumber.set("COM_Port:", ""));
	params_hub.add(numNodes.set("Number_of_Nodes:", ""));
	params_hub.add(enableAll.set("Enable_All", false));
	params_hub.add(eStopAll.set("E_STOP_ALL", false));
	params_motion.setName("Motion_Parameters");
	params_motion.add(velAll.set("Velocity_ALL", 700, 0, 4000));
	params_motion.add(accelAll.set("Acceleration_ALL", 100000, 0, 500000));
	params_macros.setName("Macros");
	params_macros.add(move_triggerAll.set("Trigger_Move", false));
	params_macros.add(move_targetAll.set("Move_Target", 0, -50000, 50000));
	params_macros.add(move_absolute_posAll.set("Move_Absolute", false));
	params_macros.add(move_zeroAll.set("Move_to_Zero", false));

	params_hub.add(params_motion);
	params_hub.add(params_macros);


	enableAll.addListener(this, &ofApp::on_enableAll);
	eStopAll.addListener(this, &ofApp::on_eStopAll);
	velAll.addListener(this, &ofApp::on_velAll);
	accelAll.addListener(this, &ofApp::on_accelAll);
	move_triggerAll.addListener(this, &ofApp::on_move_triggerAll);
	move_targetAll.addListener(this, &ofApp::on_move_targetAll);
	move_absolute_posAll.addListener(this, &ofApp::on_move_absolute_posAll);
	move_zeroAll.addListener(this, &ofApp::on_move_zeroAll);

	panel.setup("Clearpath_Controller");
	panel.setWidthElements(gui_width);
	panel.setPosition(15, 15);
	panel.add(showGUI.set("Show_GUI", true));
	panel.add(params_system);
	panel.add(params_hub);

}

/**
 * @brief Enables or Disables all motors.
 *
 * @param (bool)  val
 */
void ofApp::on_enableAll(bool& val)
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
void ofApp::on_eStopAll(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		if (val) {
			// disable & uncheck enableAll in the GUI
			if (enableAll)
				enableAll.set(false); 
			panel.setBorderColor(mode_color_eStop);
		}
		else {
			panel.setBorderColor(mode_color_normal);
		}

		axes[i]->eStopAll = val;
		axes[i]->eStop.set(val);
	}
}

void ofApp::on_velAll(float& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->vel = val;
	}
}

void ofApp::on_accelAll(float& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->accel = val;
	}
}

void ofApp::on_move_triggerAll(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->move_trigger = val;
	}
}

void ofApp::on_move_absolute_posAll(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->move_absolute_pos = val;
	}
}

void ofApp::on_move_targetAll(float& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->move_target = val;
	}
}

void ofApp::on_move_zeroAll(bool& val)
{
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->move_zero = val;
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	switch (key)
	{
	case 'h':
	case 'H':
		showGUI.set(!showGUI);
	case ' ':
		cout << node_toString(0, 0) << endl;
		cout << node_toString(0, 1) << endl;
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
