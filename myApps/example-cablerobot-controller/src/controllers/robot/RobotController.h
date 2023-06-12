#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "pubSysCls.h"
#include "CableRobot.h"

//#define AUTO_HOME     // Uncomment if you want the cable robots to home on startup



class RobotController :
    public ofThread
{
private:
    SysManager* myMgr;
    vector<CableRobot*> robots;
    vector<glm::vec3> positions_base;
    vector<float> positions_target;
    vector<float> positions_actual;
    
    bool is_initialized = false;
    bool initialize();
    void update();

    void check_for_system_ready();
    bool is_gui_setup = false;
    void setup_gui();
    void draw_gui();

    enum ControllerState {
        NOT_READY = 0,
        READY,
        PLAY,
        PAUSE,
        HOMING,
        E_STOP,
        SHUTDOWN
    };
    ControllerState state = ControllerState::NOT_READY;
    string state_names[7] = { "NOT_READY", "READY", "PLAY", "PAUSE", "HOMING", "E_STOP", "SHUTDOWN"};

    bool showGUI = true;

public:
    RobotController() = default;
    RobotController(int count, float offset_x=0, float offset_y=0, float offset_z=0);
    RobotController(vector<glm::vec3> positions_base);

    void draw();
    void shutdown();

    void threadedFunction();

    void set_position(int index, float position);
    void set_positions(vector<float> positions);

    float get_position(int index);
    vector<float> get_positions();

    string get_status(int index);

    void play();
    void pause();
    void set_e_stop(bool val);

    void key_pressed(int key);

    ofxPanel panel;
    ofParameter<string> status;
    ofParameter<void> check_status;

    ofParameterGroup params_info;
    ofParameter<string> com_ports;
    ofParameter<string> num_com_hubs;
    ofParameter<string> num_robots;

    ofParameterGroup params_sync;
    ofParameter<int> sync_index;
    ofParameter<bool> is_synchronized;

    // GUI Listeners
    void on_synchronize(bool& val);


    ofColor mode_color_disabled;
    ofColor mode_color_eStop;
    ofColor mode_color_normal;
    ofColor mode_color_playing;
};