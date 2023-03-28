#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	ofLogToConsole();

	setup_gui();
	setup_gui_camera();

	if (!initialize()) {
		close();
		exit();	// <-- not killing the program for some reason
	}


	// Add the Axis GUIs to the scene
	int x = panel.getPosition().x;
	int y = panel.getPosition().y + panel.getHeight() + 5;
	for (int i = 0; i < axes.size(); i++) {
		axes[i]->panel.setPosition(x, y);
		y += axes[i]->panel.getHeight() + 5;
	}

	// Move the camera to a perspective angle
	show_perspective.set(true);
}

//--------------------------------------------------------------
void ofApp::update() {

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackgroundGradient(background_inner, background_outer, OF_GRADIENT_CIRCULAR);

	ofEnableDepthTest();
	cam.begin();

	ofDrawAxis(1500);

	cam.end();
	ofDisableDepthTest();

	if (show_gui) {
		panel.draw();
		for (int i = 0; i < axes.size(); i++) {
			axes[i]->panel.draw();
		}
		panel_camera.draw();
		ofDrawBitmapStringHighlight("FPS: " + ofToString(ofGetFrameRate()), ofGetWidth() - 100, 10);
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

				// Setup each Node
				for (int iNode = 0; iNode < myPort.NodeCount(); iNode++) {
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
 * @brief Create a GUI for 3D Navigation.
 *
 */
void ofApp::setup_gui_camera() {

	int gui_width = 250;

	params_camera.setName("3D_Navigation");
	params_camera.add(show_gui.set("Show_GUI", true));
	params_camera.add(show_top.set("TOP", true));
	params_camera.add(show_front.set("FRONT", false));
	params_camera.add(show_side.set("SIDE", false));
	params_camera.add(show_perspective.set("PERSP", false));

	show_top.addListener(this, &ofApp::on_show_top);
	show_front.addListener(this, &ofApp::on_show_front);
	show_side.addListener(this, &ofApp::on_show_side);
	show_perspective.addListener(this, &ofApp::on_show_perspective);

	panel_camera.setup(params_camera);
	panel_camera.setWidthElements(gui_width);
	panel_camera.setPosition(ofGetWidth() - gui_width - 5, 15);

	ofSetCircleResolution(60);

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
	params_motion.add(vel_all.set("Velocity_ALL", 700, 0, 4000));
	params_motion.add(accel_all.set("Acceleration_ALL", 100000, 0, 500000));
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
		axes[i]->move_target = val;
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
void ofApp::setup_camera() {
	cam.setFarClip(9999999);
	cam.setDistance(5000);
	ofNode tgt;
	tgt.setGlobalPosition(0, 400, 1000);
	tgt.setGlobalOrientation(ofQuaternion(0, 0, 0, 1));
	cam.setTarget(tgt);
	cam.lookAt(ofVec3f(0, 0, -1), ofVec3f(1, 0, 0));
}


//--------------------------------------------------------------
void ofApp::on_show_top(bool& val)
{
	if (val) {

		int x = 0;
		int y = 400;
		int z = 3000;

		ofVec3f pos = ofVec3f(x, y, z);
		ofVec3f tgt = ofVec3f(pos.x, pos.y, 0);
		cam.setGlobalPosition(pos);
		cam.setTarget(tgt);
		cam.lookAt(tgt, ofVec3f(1, 0, 0));

		show_front = false;
		show_side = false;
		show_perspective = false;
	}
}

//--------------------------------------------------------------
void ofApp::on_show_front(bool& val)
{
	if (val) {

		int x = 3000;
		int y = 400;
		int z = 600;

		ofVec3f pos = ofVec3f(x, y, z);
		ofVec3f tgt = ofVec3f(0, pos.y, pos.z);
		cam.setGlobalPosition(pos);
		cam.setTarget(tgt);
		cam.lookAt(tgt, ofVec3f(0, 0, 1));

		show_top = false;
		show_side = false;
		show_perspective = false;
	}
}

//--------------------------------------------------------------
void ofApp::on_show_side(bool& val)
{
	if (val) {

		int x = 900;
		int y = -2000;
		int z = 600;

		ofVec3f pos = ofVec3f(x, y, z);
		ofVec3f tgt = ofVec3f(pos.x, 0, pos.z);
		cam.setGlobalPosition(pos);
		cam.setTarget(tgt);
		cam.lookAt(tgt, ofVec3f(0, 0, 1));

		show_top = false;
		show_front = false;
		show_perspective = false;
	}
}

void ofApp::on_show_perspective(bool& val)
{
	if (val) {

		int x = 4000;
		int y = -2000;
		int z = 2000;

		ofVec3f pos = ofVec3f(x, y, z);
		ofVec3f tgt = ofVec3f(0, 400, 0);
		cam.setGlobalPosition(pos);
		cam.setTarget(tgt);
		cam.lookAt(tgt, ofVec3f(0, 0, 1));
		cam.setGlobalPosition(pos);

		show_top = false;
		show_front = false;
		show_side = false;
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	switch (key)
	{
	case 'f':
	case 'F':
		ofToggleFullscreen();
		panel_camera.setPosition(ofGetWidth() - 250 - 10, 15);
		break;
	case 'h':
	case 'H':
		show_gui.set(!show_gui);
		break;
	case ' ':
		for (int i = 0; i < axes.size(); i++) {
			axes[i]->PrintStats();
		}
		break;
	case '1': 
		show_top.set(true);
		break;
	case '2': 
		show_front.set(true);
		break;
	case '3': 
		show_side.set(true);
		break;
	case '4': 
		show_perspective.set(true);
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
