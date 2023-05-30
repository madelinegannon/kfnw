#include "CableRobot.h"

CableRobot::CableRobot(SysManager& SysMgr, INode* node)
{
	motor_controller = new MotorControllers(SysMgr, node);
	setup_gui();
	check_for_system_ready();
}

CableRobot::CableRobot(glm::vec3 base)
{
}

void CableRobot::configure(glm::vec3 position, Groove direction, float diameter_drum, float length, int turns)
{
	position_base.setGlobalPosition(position);
	drum.initialize(direction, diameter_drum, length, turns);

	// define the linear length per motor step
	float step_resolution = motor_controller->get_motor()->get_resolution();
	mm_per_count = drum.circumference / step_resolution;
}

void CableRobot::set_accel_max(float accel_max)
{
}

void CableRobot::update()
{
	if (move_type == MoveType::VEL) {

	}

	
}

void CableRobot::draw()
{
	// update gui
	position_mm.set(ofToString(count_to_mm(motor_controller->get_motor()->get_position(), true)));
}

void CableRobot::shutdown()
{
}

bool CableRobot::is_in_bounds_absolute(float target_pos_absolute)
{
	return target_pos_absolute >= bounds_min.get() && target_pos_absolute <= bounds_max.get();
}

bool CableRobot::is_in_bounds_relative(float target_pos_relative)
{
	int curr_pos = count_to_mm(motor_controller->get_motor()->get_position());
	// Move relative is UP
	if (target_pos_relative < 0) {
		return curr_pos - target_pos_relative >= bounds_min.get();
	}
	// Move relative is DOWN
	else {
		return curr_pos + target_pos_relative <= bounds_max.get();
	}
}

/**
 * @brief Checks if the robot is homed, updates the state,
 * and updates the GUI. 
 */
void CableRobot::check_for_system_ready()
{
	bool is_ready = is_homed();
	if (!is_ready) {
		state = RobotState::NOT_HOMED;
		status.set(state_names[state]);
		panel.setBorderColor(mode_color_not_homed);
	}
	else {
		bool _is_enabled = is_enabled();
		state = _is_enabled ? RobotState::ENABLED : RobotState::DISABLED;
		status.set(state_names[state]);
		auto color = _is_enabled ? mode_color_enabled : mode_color_disabled;
		panel.setBorderColor(color);
		
		// check that the gui matched the current state
		if (enable.get() != _is_enabled)
			enable.set(_is_enabled);
	}

}

void CableRobot::setup_gui()
{
	mode_color_eStop = ofColor(250, 0, 0, 100);
	mode_color_not_homed = ofColor::orangeRed;
	mode_color_enabled = ofColor(200, 120);
	mode_color_disabled = ofColor(0, 200);

	int gui_width = 250;

	panel.setup("Motor_" + ofToString(motor_controller->get_motor_id()));
	panel.setWidthElements(gui_width);
	panel.setPosition(10, 15);
	panel.add(status.set("Status", state_names[0]));

	params_control.setName("Control");
	params_control.add(enable.set("Enable", false));
	params_control.add(e_stop.set("E_Stop", false));
	params_control.add(btn_run_homing.set("Run_Homing"));
	params_control.add(btn_run_shutdown.set("Run_Shutdown"));

	params_position.setName("Position");
	params_position.add(position_mm.set("Position_(mm)", ""));
	params_bounds.setName("Bounds");
	params_bounds.add(bounds_min.set("Bounds_Min", 0, 0, 2000));
	params_bounds.add(bounds_max.set("Bounds_Max", 2000, 0, 2000));

	params_jog.setName("Jogging");
	params_jog.add(jog_vel.set("Jog_Vel", 30, 0, 200));			// RPM
	params_jog.add(jog_accel.set("Jog_Accel", 200, 20, 800));	// RPM_PER_SEC
	params_jog.add(jog_dist.set("Jog_Dist", 50, 0, 1000));		// MM
	params_jog.add(btn_jog_up.set("Jog_Up"));
	params_jog.add(btn_jog_down.set("Jog_Down"));

	
	// bind GUI listeners
	e_stop.addListener(this, &CableRobot::on_e_stop);
	enable.addListener(this, &CableRobot::on_enable);
	btn_run_homing.addListener(this, &CableRobot::on_run_homing);
	btn_run_shutdown.addListener(this, &CableRobot::on_run_shutdown);
	btn_jog_up.addListener(this, &CableRobot::on_jog_up);
	btn_jog_down.addListener(this, &CableRobot::on_jog_down);

	panel.add(params_control);
	panel.add(params_position);
	panel.add(params_bounds);
	panel.add(params_jog);

	// Minimize less important parameters
	panel.getGroup("Bounds").minimize();
}

/**
 * @brief Converts from motor position (count) to linear distance (mm).
 *
 * @param (int)  val: motor position (in step counts)
 * @return (float)  linear distance (in mm)
 */
float CableRobot::count_to_mm(int val, bool use_abs)
{
	if(use_abs)
		return abs(val) * mm_per_count;
	return val * mm_per_count;
}

/**
 * @brief Converts from linear distance (mm) to motor position.
 *
 * @param (float)  val:  linear distance (in mm)
 * @return (int) motor position (in step counts)
 */
int CableRobot::mm_to_count(float val, bool use_abs)
{
	if (use_abs)
		return abs(val) / mm_per_count;
	return val / mm_per_count;
}

bool CableRobot::is_enabled()
{
	bool val = motor_controller->get_motor()->is_enabled();

	if (is_homed()) {
		if (val)
			panel.setBorderColor(mode_color_enabled);
		else
			panel.setBorderColor(mode_color_disabled);
	}
	return val;
}

bool CableRobot::is_homed()
{
	bool val = motor_controller->get_motor()->is_homed();
	if (!val) {
		panel.setBorderColor(mode_color_not_homed);
		state = RobotState::NOT_HOMED;
		status.set(state_names[state]);
	}
	return val;
}

/**
 * @brief Runs the robot's homing routine (retracts until it feels a hard stop).
 * 
 * @param (int)  timeout: homing routine timeout (in seconds)
 * @return (bool)  
 */
bool CableRobot::run_homing_routine(int timeout)
{
	return motor_controller->get_motor()->run_homing_routine(timeout);
}
/**
 * @brief Issues a position move command. This is a trapezoidal move,
 * meaning that the motor with accelerate, then decelerate and stop at 
 * the target position.
 * 
 * NOTE: Use only for Point-to-Point / Waypoint moves. Use move_velocity for
 * streaming smooth trajectories.
 *
 * @param (int)  target_pos: target position in mm.
 * @param (bool)  absolute: target is abosulte or relative to current postion. 
 */
void CableRobot::move_position(float target_pos, bool absolute)
{
	// convert from mm to motor counts
	int count = mm_to_count(target_pos);
	// send move command
	if (absolute) {
		if (is_in_bounds_absolute(target_pos))
			motor_controller->get_motor()->move_position(count, true);
	}
	else {
		if (is_in_bounds_relative(target_pos))
			motor_controller->get_motor()->move_position(count, false);
	}
}

float CableRobot::compute_desired_velocity(float target_pos)
{
	return 0.0f;
}

void CableRobot::move_velocity(float target_vel)
{
	// clamp velocity to abs(velocity_max)
	// check that there is space in the motor's move buffer
	// send vel command
	motor_controller->get_motor()->move_velocity(target_vel);
}

void CableRobot::stop()
{
	motor_controller->get_motor()->stop();
}

void CableRobot::set_e_stop(bool val)
{
	motor_controller->get_motor()->set_e_stop(val);
}

void CableRobot::set_enabled(bool val)
{
	motor_controller->get_motor()->set_enabled(val);
}

void CableRobot::on_enable(bool& val)
{
	set_enabled(val);
	if (is_homed()) {
		if (val) {
			panel.setBorderColor(mode_color_enabled);
			state = RobotState::ENABLED;
		}
		else {
			panel.setBorderColor(mode_color_disabled);
			state = RobotState::DISABLED;
		}
		status.set(state_names[state]);
	}
}

/**
 * @brief E Stop triggered / cleared through GUI.
 * 
 * @param (bool)  val: triggers or clears an E Stop.
 */
void CableRobot::on_e_stop(bool& val)
{
	set_e_stop(val);
	if (val) {
		panel.setBorderColor(mode_color_eStop);
		state = RobotState::E_STOP;
	}
	else {
		bool _is_enabled = is_enabled();
		state = _is_enabled ? RobotState::ENABLED : RobotState::DISABLED;
		status.set(state_names[state]);
		auto color = _is_enabled ? mode_color_enabled : mode_color_disabled;
		panel.setBorderColor(color);
	}
	status.set(state_names[state]);
}

void CableRobot::on_run_homing()
{
	state = RobotState::HOMING;
	status.set(state_names[state]);

	// if we successfully homed, update the state and gui color
	if (run_homing_routine(60)) {
		bool _is_enabled = is_enabled();
		state = _is_enabled ? RobotState::ENABLED : RobotState::DISABLED;
		status.set(state_names[state]);
		auto color = _is_enabled ? mode_color_enabled : mode_color_disabled;
		panel.setBorderColor(color);
	}
	else {
		state = RobotState::NOT_HOMED;
		status.set(state_names[state]);
		panel.setBorderColor(mode_color_not_homed);
	}
}

/**
 * @brief The Shutdown Routine moves to bounds_max, 
 * moves slowly to the shutdown_position, 
 * and then disables the motor.
 * 
 */
void CableRobot::on_run_shutdown()
{
	ofLogNotice("CableRobot::on_run_shutdown") << "Shutting Down Motor " << motor_controller->get_motor_id();
	jog_vel.set(50);
	jog_accel.set(200);
	jog_dist.set(bounds_max.get());
	jog_down();

	auto diff = position_shutdown - bounds_max.get();
	jog_vel.set(10);
	jog_accel.set(50);
	jog_dist.set(diff);
	jog_down(true);

	bool shutting_down = true;
	while (shutting_down) {
		auto pos = count_to_mm(motor_controller->get_motor()->get_position(), true);
		if (position_shutdown - pos < 1) {
			shutting_down = false;
		}
	}
	// wait a brief moment before disabling the motor
	float start_time = ofGetElapsedTimeMillis();
	float delay = 1500;
	while (ofGetElapsedTimeMillis() < start_time + delay)
	{
	}
	// disbale the motor and update the GUI
	enable.set(shutting_down);
}

/**
 * @brief Jogs the robot up relative to its current postion.
 * Compensates for sign based on RIGHT- or LEFT-HANDED cable drum.
 * Checks if it is within bounds before sending the move.
 * 
 */
void CableRobot::on_jog_up()
{
	if (!is_enabled()) {
		ofLogWarning("CableRobot::on_jog_up") << "Enable Motor " << motor_controller->get_motor_id() << " before trying to jog.";
	}
	else {
		// convert from distance to counts
		int pos = mm_to_count(jog_dist);
		// check the sign to see if up is + or - for jogging up
		if (drum.direction == Groove::RIGHT_HANDED) {
			pos *= -1;
		}
		else if (drum.direction == Groove::LEFT_HANDED) {
		}
		else {
			ofLogWarning("CableRobot::on_jog_up") << "The CableDrum direction for Motor " << motor_controller->get_motor_id() << " is not configured.It must be configured before moving.";
		}
		// Check that we are not jogging out of bounds
		bool send_move = true;
		//int curr_pos = motor_controller->get_motor()->get_position();
		//if (curr_pos + abs(pos) > mm_to_count(bounds_min, true)) {
		//}
		int curr_pos = abs(motor_controller->get_motor()->get_position());
		if (curr_pos - abs(pos) < mm_to_count(bounds_min.get())) {
			pos = mm_to_count(bounds_min.get());
			pos = (drum.direction == Groove::LEFT_HANDED) ? pos * -1 : pos * 1;
			ofLogWarning("CableRobot::on_jog_up") << "Moving CableRobot " << motor_controller->get_motor_id() << " to bounds_min: " << count_to_mm(pos);
			motor_controller->get_motor()->move_position(pos, true);
			send_move = false;
		}
		if (send_move) {
			float curr_vel_limit = motor_controller->get_motor()->get_velocity();
			float curr_accel = motor_controller->get_motor()->get_acceleration();
			// use the jogging vel & accel
			motor_controller->get_motor()->set_velocity(jog_vel.get());
			motor_controller->get_motor()->set_acceleration(jog_accel.get());
			// send the move
			motor_controller->get_motor()->move_position(pos, false);
			// switch back to previous vel & accel
			motor_controller->get_motor()->set_velocity(curr_vel_limit);
			motor_controller->get_motor()->set_acceleration(curr_accel);
		}
	}
}

/**
 * @brief Jogs the robot up relative to its current postions.
 * Compensates for sign based on RIGHT- or LEFT-HANDED cable drum.
 * Checks if it is within bounds before sending the move. Use the @param override to jog past the bounds_max.
 *
 * @param (bool)  override: if true, overrides checking for bounds_max (false by default)
 */
void CableRobot::jog_down(bool override) {
	if (!is_enabled()) {
		ofLogWarning("CableRobot::on_jog_down") << "Enable Motor " << motor_controller->get_motor_id() << " before trying to jog.";
	}
	else {
		// convert from distance to counts
		int pos = mm_to_count(jog_dist);
		// check the drum groove direction to see if jogging is + or - for down
		if (drum.direction == Groove::RIGHT_HANDED) {
		}
		else if (drum.direction == Groove::LEFT_HANDED) {
			pos *= -1;
		}
		else {
			ofLogWarning("CableRobot::on_jog_down") << "The CableDrum direction for Motor " << motor_controller->get_motor_id() << " is not configured.It must be configured before moving.";
		}
		// Check that we are not jogging out of bounds
		bool send_move = true;
		if (!override) {
			int curr_pos = abs(motor_controller->get_motor()->get_position());
			if (curr_pos + abs(pos) > mm_to_count(bounds_max.get())) {
				pos = mm_to_count(bounds_max.get());
				pos = (drum.direction == Groove::LEFT_HANDED) ? pos * -1 : pos * 1;
				ofLogWarning("CableRobot::on_jog_down") << "Moving CableRobot " << motor_controller->get_motor_id() << " to bounds_max: " << count_to_mm(pos);
				motor_controller->get_motor()->move_position(pos, true);
				send_move = false;
			}
		}
		// If we're in bounds, send the jog move
		if (send_move) {
			float curr_vel_limit = motor_controller->get_motor()->get_velocity();
			float curr_accel = motor_controller->get_motor()->get_acceleration();
			// use the jogging vel & accel
			motor_controller->get_motor()->set_velocity(jog_vel.get());
			motor_controller->get_motor()->set_acceleration(jog_accel.get());
			// send the move
			cout << "Moving Down: " << pos << endl;
			motor_controller->get_motor()->move_position(pos, false);
			// switch back to previous vel & accel
			motor_controller->get_motor()->set_velocity(curr_vel_limit);
			motor_controller->get_motor()->set_acceleration(curr_accel);
		}
	}
}

/**
 * @brief GUI Callback to Jog Down.
 */
void CableRobot::on_jog_down()
{
	jog_down();
}
