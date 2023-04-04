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
		void close();

		//const int ACC_LIM_RPM_PER_SEC = 100000;
		//const int VEL_LIM_RPM = 700;

		// GUI
		void setup_gui();
		ofxPanel panel;
		ofParameter<bool> showGUI;
		ofParameterGroup params_system;
		ofParameter<string> systemStatus;
		ofParameter<string> numHubs;

		ofParameterGroup params_hub;
		ofParameter<string> comPortNumber;
		ofParameter<string> numNodes;
		ofParameter<bool> enable_all;
		ofParameter<bool> eStopAll;

		ofParameterGroup params_motion;
		ofParameter<float> vel_all;
		ofParameter<float> accel_all;

		ofParameterGroup params_macros;
		ofParameter<bool> move_trigger_all;
		ofParameter<float> move_target_all;
		ofParameter<bool> move_absolute_pos_all;
		ofParameter<bool> move_zero_all;
		ofParameter<bool> home_all;   // <-- @TODO


		// GUI Listeners
		void on_enable_all(bool& val);
		void on_eStop_all(bool& val);
		void on_vel_all(float& val);
		void on_accel_all(float& val);

		void on_move_trigger_all(bool& val);
		void on_move_absolute_pos_all(bool& val);
		void on_move_target_all(float& val);
		void on_move_zero_all(bool& val);


		ofColor mode_color_disabled;
		ofColor mode_color_eStop;
		ofColor mode_color_normal;

};
