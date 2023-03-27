/****************************************************************************
 * NAME:
 *		Axis
 *
 * DESCRIPTION:
 *		This class provides an interface to the ClearPath SC node 
 *		in our example program
 *
 ****************************************************************************/

#ifndef __AXIS_H__
#define __AXIS_H__

 /****************************************************************************
 * !NAME!																    *
 * 	Axis.h headers
 *																		!0! */
#include <time.h>
#include "pubSysCls.h"
#if defined(_WIN32) || defined(_WIN64)
#include "windows.h"
#endif

#include "ofMain.h"
#include "ofxGui.h"

// Use the Teknic library's namespace
using namespace sFnd;

/* 																	   !end!*
****************************************************************************/

/****************************************************************************
* !NAME!																	*
* 	Axis.h constants
* 																		!0!	*/
//#define RAS_CODE			3 
//#define TRQ_PCT				100
//#define ACC_LIM_RPM_PER_SEC	8000
//#define VEL_LIM_RPM			2000
//#define TRACKING_LIM		(m_node->Info.PositioningResolution.Value() / 4)
/* !end!																	*
****************************************************************************/

//******************************************************************************
// NAME																		   *
// 	Axis class
//
// DESCRIPTION
//	This is the interface to the ClearPath SC. 
//
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

	// Edit the current motion parameters
	void ChangeMotionParams(double newVelLimit,
		double newAccelLimit,
		uint32_t newJrkLimit,
		uint32_t newTrackingLimit,
		int32_t newTrqGlobal);

	// Initiate a move
	void PosnMove(int32_t targetPosn,
		bool isAbsolute = false,
		bool addDwell = false);
	void VelMove(double targetVel);

	// Wait for attention that move has completed
	bool WaitForMove(::int32_t timeoutMs, bool secondCall = false);

	// Alert handlers
	void CheckForAlerts();

	// HW Brake Setup
	void SetBrakeControl(size_t brakeNum, BrakeControls brakeMode);


public:
	// Constructor/Destructor
	Axis(SysManager&, INode *node, int iPort, int iNode);
	~Axis();

	int iPort;
	int iNode;

	// The state machine
	void AxisMain();

	// Return a reference to our node
	INode* MyNode() {
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
//																			   *
//******************************************************************************

#endif
