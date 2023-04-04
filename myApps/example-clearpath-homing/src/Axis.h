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

// Use the Teknic library's namespace
using namespace sFnd;


/**
 * This class interfaces with the ClearPath SC SDK.
 */
class Axis {
private:
	INode *m_node;				// The ClearPath-SC for this axis
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

	// Initiate a move
	void PosnMove(int32_t targetPosn,
		bool isAbsolute = false,
		bool addDwell = false);

	void VelMove(double targetVel, uint32_t duration=500);

	// Wait for attention that move has completed
	bool WaitForMove(::int32_t timeoutMs, bool secondCall = false);


	// HW Brake Setup
	void SetBrakeControl(size_t brakeNum, BrakeControls brakeMode);


public:
	// Constructor/Destructor
	Axis(SysManager& SysMgr, INode *node, int iPort=0, int iNode=0);
	~Axis();

	void setup();
	void update();
	void draw();

	int iPort;
	int iNode;

	void InitHomingRoutine();

	// Check Motor Statuses
	void CheckMotorStatus();
	void CheckHomingStatus();

	
	// Alert handlers
	void CheckForAlerts();

	// Return a reference to our node
	INode* Get() {
		return(m_node);
	};

	// Print the current stats (number of moves performed and
	// current commanded position)
	void PrintStats(bool onInit = false) {
		// Refresh the measures position register
		m_node->Motion.PosnCommanded.Refresh();
		if (onInit) {
			printf("  [%2d]:\t\t**at startup**\t\t%jd\n",
				m_node->Info.Ex.Addr(),
				(intmax_t) GetPosition());
		}
		else {
			printf("  [%2d]:\t\t%8d\t\t%jd\n",
				m_node->Info.Ex.Addr(), m_moveCount,
				(intmax_t) GetPosition());
		}
	}

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

	ofParameterGroup params_homing;
	ofParameter<string> homing_status;
	ofParameter<bool> rehome;
	ofParameter<bool> reset_home_position;

	ofParameterGroup params_motion;
	ofParameter<float> vel;
	ofParameter<float> accel;
	ofParameter<float> decel;
	
	ofParameterGroup params_macros;
	ofParameter<bool> move_trigger;
	ofParameter<int> move_target;
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

};

#endif
