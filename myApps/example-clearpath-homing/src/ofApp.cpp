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
		axes[i]->update();
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(60);

	if (showGUI) {
		panel.draw();
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
	panel.add(params_hub);

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
	case 'F':
		ofToggleFullscreen();
		break;
	case ' ':
		for (int i = 0; i < axes.size(); i++) {
			axes[i]->PrintStats();
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

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
