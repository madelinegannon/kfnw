/*****************************************************************//**
 * @file   Axis.h
 * @brief
 *
 * @author mad
 * @date   March 2023
 *********************************************************************/
#ifndef __AXIS_H__
#define __AXIS_H__


#include <time.h>
#include "pubSysCls.h"
#if defined(_WIN32) || defined(_WIN64)
#include "windows.h"
#endif

#include "ofMain.h"
#include "ofxGui.h"
#include "PID.h"

 // Use the Teknic library's namespace
using namespace sFnd;


/**
 * This class interfaces with the ClearPath SC SDK.
 */
class Axis {
private:
	INode* m_node;				// The ClearPath-SC for this axis
	SysManager& m_sysMgr;

	// Filename for the stored config file
#define MAX_CONFIG_PATH_LENGTH 256
#define MAX_CONFIG_FILENAME_LENGTH 384
	char m_configFile[MAX_CONFIG_FILENAME_LENGTH];

	// State machine information
	enum StateEnum {
		STATE_IDLE,
		STATE_SEND_MOVE,
		STATE_WAIT_FOR_MOVE_DONE,
		STATE_EXITED
	};
	StateEnum m_state;
	StateEnum m_lastState;

	// Move information
	uint32_t m_moveCount;
	int32_t m_positionWrapCount;
	int32_t m_moveTimeoutMs;

	bool m_quitting;			// Axis quitting

	// Enable the node and get it ready to go
	void Enable();

	// Diable the node and get it ready to go
	void Disable();

	// Initialize accLim, velLim, etc
	void InitMotionParams();


	void VelMove(double targetVel, uint32_t duration = 500);

	// HW Brake Setup
	void SetBrakeControl(size_t brakeNum, BrakeControls brakeMode);



public:
	// Constructor/Destructor
	Axis(SysManager& SysMgr, INode* node, int iPort = 0, int iNode = 0);
	~Axis();

	// Return a reference to our node
	INode* Get() {
		return(m_node);
	};

	PID pid;
	void setup_pid();



	// Wait for attention that move has completed
	bool WaitForMove(::int32_t timeoutMs, bool secondCall = false);

	// Initiate a move
	void PosnMove(int32_t targetPosn,
		bool isAbsolute = false,
		bool addDwell = false);

	// Return scaled position
	int64_t GetPosition(bool includeWraps = true);
	void SetPosition(int32_t targetPosn,
		bool isAbsolute = false,
		bool addDwell = false);

	int64_t GetHomingPosition();
	void SetHomingPosition(int32_t homingPosn);


	// Reset sequencer to "idle" state
	void ResetToIdle() {
		if (m_state != STATE_IDLE)
			m_state = m_lastState = STATE_IDLE;
	};

	// Time to quit
	void Quit() {
		m_quitting = true;
	}


	void setup();
	void update();
	void draw();

	ofParameter<float> spool_diamter = 39.35;

	float count_to_mm(int val, float diameter = 39.35);
	int mm_to_count(float val, float diameter = 39.35);

	int iPort;
	int iNode;

	void InitHomingRoutine();

	// Check Motor Statuses
	void CheckMotorStatus();
	void CheckHomingStatus();

	// Alert handlers
	void CheckForAlerts();

	void clearMotionStops();
	void triggerEStop();

	// GUI
	void setup_gui();
	bool eStopAll = false;

	ofColor mode_color_disabled;
	ofColor mode_color_eStop;
	ofColor mode_color_normal;

	ofxPanel panel;
	ofParameterGroup params_limits;
	ofParameter<string> status;
	ofParameter<bool> enable;
	ofParameter<bool> eStop;
	ofParameter<string> position_cnts;
	ofParameter<string> position_mm;
	ofParameter<ofVec3f> position_world;

	ofParameterGroup params_homing;
	ofParameter<string> homing_status;
	ofParameter<bool> rehome;
	ofParameter<bool> reset_home_position;

	ofParameterGroup params_motion;
	ofParameter<float> vel;
	ofParameter<float> accel;
	ofParameter<float> decel;


	//float kp = 1.0;
	//float ki = 0.005;
	//float kd = 0.005;
	//float time_sampling = (1 / 60.);
	//float cuttoff;
	//float max_output;
	ofParameterGroup params_pid;
	ofParameter<float> kp;
	ofParameter<float> ki;
	ofParameter<float> kd;
	ofParameter<float> time_sampling;
	ofParameter<float> cuttoff;
	ofParameter<float> max_output;

	ofParameterGroup params_macros;
	ofParameter<bool> move_trigger;
	ofParameter<int> move_target_cnts;
	ofParameter<float> move_target_mm;
	ofParameter<bool> move_absolute_pos;
	ofParameter<bool> move_zero;

	// GUI Listeners
	void on_enable(bool& val);
	void on_eStop(bool& val);
	void on_rehome(bool& val);
	void on_reset_home_position(bool& val);

	void on_vel_changed(float& val);
	void on_accel_changed(float& val);

	void on_move_trigger(bool& val);
	void on_move_zero(bool& val);

	void on_move_target_mm(float& val);


	void on_kp(float& val);
	void on_ki(float& val);
	void on_kd(float& val);
	void on_cutoff(float& val);
	void on_max_output(float& val);



	// Print the current stats (number of moves performed and
	// current commanded position)
	void PrintStats(bool onInit = false) {
		// Refresh the measures position register
		m_node->Motion.PosnCommanded.Refresh();
		if (onInit) {
			printf("  [%2d]:\t\t**at startup**\t\t%jd\n",
				m_node->Info.Ex.Addr(),
				(intmax_t)GetPosition());
		}
		else {
			printf("  [%2d]:\t\t%8d\t\t%jd\n",
				m_node->Info.Ex.Addr(), m_moveCount,
				(intmax_t)GetPosition());
		}
	}



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
				auto p = toGlm(toOf(start.getGlobalPosition()).getInterpolated(toOf(end.getGlobalPosition()), t));
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
			ofSetColor(ofColor::white, 120);
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

			ofSetColor(ofColor::white, 120);
			auto mid_pt = (start_pos + pos.getGlobalPosition()) / 2;
			auto dist = glm::distance(start_pos, pos.getGlobalPosition());
			ofDrawBitmapString(ofToString(dist) + " mm", pos.getGlobalPosition().x + 10, pos.getGlobalPosition().y + 4);

			dist = glm::distance(home_pos, pos.getGlobalPosition());
			ofSetColor(ofColor::white, 80);
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
	Spool spool;

};

#endif
