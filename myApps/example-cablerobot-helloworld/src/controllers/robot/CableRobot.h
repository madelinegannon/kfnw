#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "CableDrum.h"
#include "EndEffector.h"
#include "MotorControllers.h"

#include "pubSysCls.h"

using namespace sFnd;

class CableRobot
{
private:
    // Motion Parameters
    float velocity_max = 300;   // RPM
    float accel_max = 800;      // RPM_PER_SEC
    float position_min = 0;     // absolute position in mm
    float position_max = 2000;  // absolute position in mm
    float position_shutdown = 2140;  // absolute position in mm

    bool is_in_bounds_absolute(float target_pos_absolute);
    bool is_in_bounds_relative(float target_pos_relative);


    ofNode position_base;
    ofNode position_target;
    ofNode position_actual;

    MotorControllers* motor_controller;
    CableDrum drum;
    EndEffector ee;

    void check_for_system_ready();
    void setup_gui();

    float mm_per_count;
    float count_to_mm(int val, bool use_abs=false);
    int mm_to_count(float val, bool use_abs = false);

    enum RobotState {
        NOT_HOMED,
        HOMING,
        ENABLED,
        DISABLED,
        E_STOP
    };
    RobotState state = RobotState::NOT_HOMED;
    string state_names[5] = { "NOT_HOMED", "HOMING", "ENABLED", "DISABLED", "E_STOP"};

    enum MoveType {
        POS,
        VEL
    };
    MoveType move_type = MoveType::POS;
public:
    CableRobot();
    CableRobot(SysManager& SysMgr, INode* node);
    CableRobot(glm::vec3 base);

    void update();
    void draw();
    void shutdown();

    void configure(glm::vec3 position,
        Groove direction,
        float diameter_drum, 
        float length, 
        int turns
    );
    void set_motion_parameters(float velocity_max, float accel_max, float position_min, float position_max);
    void set_velocity_max(float velocity_max);
    void set_accel_max(float accel_max);
    void set_position_min(float position_min);
    void set_position_max(float position_max);

    bool is_enabled();
    bool is_homed();
    bool run_homing_routine(int timeout=20);

    void move_position(float target_pos, bool absolute = true);
    float compute_desired_velocity(float target_pos);
    void move_velocity(float target_vel);

    void stop();
    void set_e_stop(bool val);
    void set_enabled(bool val);

    void jog_down(bool override = false);

    // GUI and Listeners
    void on_enable(bool& val);
    void on_e_stop(bool& val);
    void on_run_homing();
    void on_run_shutdown();
    void on_jog_up();
    void on_jog_down();
    
    ofxPanel panel;
    ofParameterGroup params_control;
    ofParameter<string> status;
    ofParameter<bool> enable;
    ofParameter<bool> e_stop;
    ofParameter<void> btn_run_homing;
    ofParameter<void> btn_run_shutdown;

    ofParameterGroup params_position;
    ofParameter<string> position_mm;
    //ofParameter<string> position_cnts;
    ofParameterGroup params_bounds;
    ofParameter<float> bounds_min;
    ofParameter<float> bounds_max;


    //ofParameter<ofVec3f> position_world;

    ofParameterGroup params_jog;
    ofParameter<float> jog_vel;
    ofParameter<float> jog_accel;
    ofParameter<float> jog_dist;
    ofParameter<void> btn_jog_up;
    ofParameter<void> btn_jog_down;

    ofParameterGroup params_homing;
    ofParameter<string> homing_status;
    ofParameter<bool> rehome;
    ofParameter<bool> reset_home_position;

    ofColor mode_color_enabled;
    ofColor mode_color_disabled;
    ofColor mode_color_not_homed;
    ofColor mode_color_eStop;
};