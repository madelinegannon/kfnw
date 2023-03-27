#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "pubSysCls.h"	
#include "Axis.h"

using namespace sFnd;

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		SysManager* myMgr;
		vector<Axis*> axes;
		int numPorts;
		bool initialize();
		void setup_node(int iPort, int iNode);
		string node_toString(int iPort, int iNode);
		void close();
		void checkForAlerts();
		void clearMotionStops();
		void clearMotionStop(int iPort, int iNode);
		void triggerEStops();
		void triggerEStop(int iPort, int iNode);
		void enableMotors(bool val);
		void enableMotor(bool val, int iPort, int iNode);

		const int ACC_LIM_RPM_PER_SEC = 100000;
		const int VEL_LIM_RPM = 700;

		// GUI
		void setup_gui();
		ofxPanel panel;
		ofParameter<bool> showGUI;
		ofParameterGroup params_system;
		ofParameter<string> systemStatus;
		ofParameter<string> numHubs;
		//ofParameter<int> numPorts;
		ofParameterGroup params_hub;
		ofParameter<bool> enableAll;
		ofParameter<bool> eStopAll;
		ofParameterGroup params_motion;
		ofParameter<float> velAll;	
		ofParameter<float> accelAll; 
		ofParameterGroup params_macros;
		ofParameter<bool> move_triggerAll;
		ofParameter<float> move_targetAll;
		ofParameter<bool> move_absolute_posAll;
		ofParameter<bool> move_zeroAll;
		ofParameter<bool> homeAll;   // <-- @TODO
		ofParameter<string> comPortNumber;
		ofParameter<string> numNodes;

		
		// GUI Listeners
		void on_enableAll(bool& val);
		void on_eStopAll(bool& val);
		void on_velAll(float& val);
		void on_accelAll(float& val);

		void on_move_triggerAll(bool& val);
		void on_move_zeroAll(bool& val);
		void on_move_absolute_posAll(bool& val);
		void on_move_targetAll(float& val);

		ofColor mode_color_disabled;
		ofColor mode_color_eStop;
		ofColor mode_color_normal;


};
