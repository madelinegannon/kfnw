#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOsc.h"
#include "pubSysCls.h"	
#include "Axis.h"

using namespace sFnd;

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	// Sine Curve
	ofPolyline curve;
	float crv_width;
	int crv_res = 1000;
	glm::vec3 get_projected_point(ofPolyline crv, float x);
	void update_sine_curve();
	void draw_sine_curve();

	// OSC Controller
	void setup_osc_controller();
	void update_osc_controller();
	void draw_osc_controller();
	void reset_osc_controller();
	ofVec3f start, end;

	int port = 12345;
	ofxOscReceiver receiver;
	void checkForOSCMessage();

	// CPM Parameters
	SysManager* myMgr;
	int numPorts;
	bool initialize();
	void close();

	vector<Axis*> axes;
	glm::vec3 origin;
	glm::vec3 desired_pos;
	glm::vec3 actual_pos;	
	float actual_dist_mm;
	float desired_dist_mm;
	float desired_vel;
	float pid_vel = 0;



	// GUI
	void setup_gui();
	ofxPanel panel;
	ofParameter<bool> showGUI;
	ofParameterGroup params_system;
	ofParameter<string> systemStatus;
	ofParameter<string> numHubs;

	ofParameterGroup params_motion_control;
	ofParameter<bool> play;
	ofParameter<bool> pause;
	ofParameter<float> vel_time_step;
	ofParameter<float> vel_max;

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

	ofxPanel panel_curve;
	ofParameterGroup params_curve;
	ofParameter<bool> use_sine_curve;
	ofParameter<bool> use_osc_controller;
	ofParameter<float> crv_amp = 120.0;
	ofParameter<float> crv_speed = 0.1;
	ofParameter<float> crv_period = 500;
	ofParameter<float> crv_theta = 0;


	// GUI Listeners
	void on_play(bool& val);
	void on_pause(bool& val);

	void on_enable_all(bool& val);
	void on_eStop_all(bool& val);
	void on_vel_all(float& val);
	void on_accel_all(float& val);

	void on_move_trigger_all(bool& val);
	void on_move_absolute_pos_all(bool& val);
	void on_move_target_all(float& val);
	void on_move_zero_all(bool& val);

	void on_use_sine_curve(bool& val);
	void on_use_osc_controller(bool& val);


	ofColor mode_color_disabled;
	ofColor mode_color_eStop;
	ofColor mode_color_normal;
	ofColor mode_color_playing;



	struct Spool
	{
		ofNode home;
		ofNode start;
		ofNode end;
		ofNode pos;
		float dist;
		float max_dist;
		int interval_count = 2;
		int interval_index = 0;
		float interval_dist = 0;
		float start_offset;
		float end_offset;
		float pressure_offset = 0;
		vector<ofNode> intervals;

		void setup(ofVec3f home_pos, int num_intervals, float end_offset, float start_offset = 0) {
			this->start_offset = start_offset;
			this->end_offset = end_offset;

			home.setGlobalPosition(home_pos);
			start.setParent(home);
			end.setParent(start);
			pos.setParent(start);

			start.setPosition(0, start_offset, 0);
			end.setPosition(0, end_offset, 0);
			pos.setPosition(0, end_offset, 0);

			interval_dist = (end_offset - start_offset) / num_intervals;

			interval_count = num_intervals;
			for (int i = interval_count; i >= 0; i--) {			// DO ONE EXTRA INTERVAL SO WE HAVE A FULL SCALE (13 positions)
				float t = float(i) / float(interval_count);
				ofNode n;
				auto p = toGlm(toOf(end.getGlobalPosition()).getInterpolated(toOf(start.getGlobalPosition()), t));
				float offset = glm::distance(start.getGlobalPosition(), p);
				n.setParent(start);
				n.setPosition(0, offset, 0);
				intervals.push_back(n);
			}

			stringstream ss;
			ss << "home position (GLOBAL): " << ofToString(home.getGlobalPosition()) << endl;
			ss << "START position (GLOBAL): " << ofToString(start.getGlobalPosition());
			ss << "\tSTART position (LOCAL): " << ofToString(start.getPosition()) << endl;
			ss << "POS position (GLOBAL): " << ofToString(pos.getGlobalPosition());
			ss << "\tPOS position (LOCAL): " << ofToString(pos.getPosition()) << endl;
			ss << "END position (GLOBAL): " << ofToString(end.getGlobalPosition());
			ss << "\tEND position (LOCAL): " << ofToString(end.getPosition()) << endl;
			cout << ss.str() << endl << endl;
		}

		void draw() {
			ofPushStyle();
			ofSetColor(ofColor::white, 80);

			auto home_pos = home.getGlobalPosition();
			auto start_pos = start.getGlobalPosition();
			auto end_pos = end.getGlobalPosition();

			// weird bug that's resetting the global position ... force it to re-correct 
			if (start.getGlobalPosition() == start.getPosition()) {
				start.clearParent();
				end.clearParent();
				pos.clearParent();
				start.setParent(home);
				end.setParent(start);
				pos.setParent(start);

				start.setPosition(0, start_offset, 0);
				end.setPosition(0, end_offset, 0);
				int i = 0;
				for (auto& interval : intervals) {
					interval.clearParent();
					interval.setParent(start);
					float t = float(i) / float(interval_count);
					auto p = toGlm(toOf(end.getGlobalPosition()).getInterpolated(toOf(start.getGlobalPosition()), t));
					float offset = glm::distance(start.getGlobalPosition(), p);
					interval.setPosition(0, offset, 0);
					i++;
				}

				stringstream ss;
				ss << "START position (GLOBAL): " << ofToString(start.getGlobalPosition());
				ss << "\tSTART position (LOCAL): " << ofToString(start.getPosition()) << endl;
				ss << "POS position (GLOBAL): " << ofToString(pos.getGlobalPosition());
				ss << "\tPOS position (LOCAL): " << ofToString(pos.getPosition()) << endl;
				ss << "END position (GLOBAL): " << ofToString(end.getGlobalPosition());
				ss << "\tEND position (LOCAL): " << ofToString(end.getPosition()) << endl;
				cout << ss.str() << endl << endl;
			}


			// Draw line from home to end
			ofSetLineWidth(5);
			ofDrawLine(home.getGlobalPosition(), end_pos);

			// Draw home position
			ofDrawCircle(home_pos.x, home_pos.y, 20);

			// Draw start  & end positions
			ofSetColor(ofColor::white, 200);
			ofDrawCircle(start_pos.x, start_pos.y, 10);
			ofDrawCircle(end_pos.x, end_pos.y, 10);

			// Draw intervals
			ofSetColor(ofColor::black, 120);
			int i = 0;
			for (auto interval : intervals) {
				ofDrawCircle(interval.getGlobalPosition().x, interval.getGlobalPosition().y, 5);
				ofDrawBitmapString(ofToString(i), interval.getGlobalPosition().x - 25, interval.getGlobalPosition().y + 5);
				i++;
			}

			// Show current position
			ofSetColor(ofColor::orange, 120);
			ofDrawLine(start_pos, pos.getGlobalPosition());
			ofSetColor(ofColor::orange);
			ofDrawCircle(pos.getGlobalPosition().x, pos.getGlobalPosition().y, 5);

			ofSetColor(ofColor::black, 120);
			auto mid_pt = (start_pos + pos.getGlobalPosition()) / 2;
			auto dist = glm::distance(start_pos, pos.getGlobalPosition());
			ofDrawBitmapString(ofToString(dist) + " mm", pos.getGlobalPosition().x + 10, pos.getGlobalPosition().y + 4);

			dist = glm::distance(home_pos, pos.getGlobalPosition());
			ofSetColor(ofColor::black, 80);
			ofDrawBitmapString(ofToString(dist) + " mm (actual)", pos.getGlobalPosition().x + 10, pos.getGlobalPosition().y + 20);

			ofPopStyle();
		}

		/**
		 * @brief Moves the pos to a given interval.
		 *
		 * @param ()  index:
		 */
		void set_position(int interval_index) {
			auto p = intervals[interval_index].getPosition();
			p.y -= pressure_offset * 2;
			pos.setPosition(p);
			
			// reset the pressure offset
			pressure_offset = 0;
		}
	};
	vector<Spool> spools;

};
