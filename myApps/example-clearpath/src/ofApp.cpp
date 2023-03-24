#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	myMgr = SysManager::Instance();							//Create System Manager myMgr
	cout << "Hello World, I am SysManager" << endl;

	int portCount = 0;
	vector<string> comHubPorts;
	try {
		SysManager::FindComHubPorts(comHubPorts);
		cout << "Found # SC Hub: " << ofToString(comHubPorts.size()) << endl;

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {

			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}
	}
	catch (mnErr& theErr)	//This catch statement will intercept any error from the Class library
	{
		cout << "Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port" << endl;
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

	}
	exit();
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
