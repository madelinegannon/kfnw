#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "pubSysCls.h"	

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
		ofParameter<bool> enableAll;
		ofParameter<bool> eStopAll;
		ofParameter<float> velAll;	 // <-- @TODO
		ofParameter<float> accelAll; // <-- @TODO
		ofParameter<bool> homeAll;   // <-- @TODO
		//ofParameter<int> numPorts;
		ofParameterGroup params_hub;
		ofParameter<string> comPortNumber;
		ofParameter<string> numNodes;
		ofParameterGroup params_node;
		ofParameter<string> nodeStatus;
		ofParameter<bool> nodeEStop;
		
		// GUI Listeners
		void on_enableAll(bool& val);
		void on_eStopAll(bool& val);

		ofColor mode_color_disabled;
		ofColor mode_color_eStop;
		ofColor mode_color_normal;

		ofxPanel panel_node;
		ofParameterGroup params_limits;
		ofParameter<string> status;
		ofParameter<bool> enable;
		ofParameter<bool> eStop;
		ofParameterGroup params_motion;
		ofParameter<float> vel;
		ofParameter<float> accel;
		ofParameter<float> decel;
		ofParameterGroup params_macros;
		ofParameter<bool> move_trigger;
		ofParameter<float> move_target;
		ofParameter<bool> move_absolute_pos;
		ofParameter<bool> move_zero;



		// GUI Listeners
		void on_enable(bool& val);
		void on_eStop(bool& val);
		void on_vel_changed(float& val);
		void on_accel_changed(float& val);
		void on_decel_changed(float& val);

		void on_move_trigger(bool& val);
		void on_move_zero(bool& val);

};
