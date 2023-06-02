#include "CableRobot.h"

CableRobot::CableRobot(SysManager& SysMgr, INode* node)
{
	motor_controller = new MotorControllers(SysMgr, node);
	setup_gui();
	motor_controller->get_motor()->set_motion_params(vel_limit.get(), accel_limit.get());
	// referesh the gui
	vel_limit.set(vel_limit.get());
	accel_limit.set(accel_limit.get());

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

float CableRobot::get_position_target()
{
	return count_to_mm(motor_controller->get_motor()->get_position(), true);
}

float CableRobot::get_position_actual()
{
	return count_to_mm(motor_controller->get_motor()->get_position(false), true);
}

/**
 * @brief Returns the main motion parameters for the robot.
 * 
 * @return (vector<float>)  params: { vel_limit, accel_limit, bounds_min, bounds_max }
 */
vector<float> CableRobot::get_motion_parameters()
{
	vector<float> params;
	params.push_back(vel_limit.get());
	params.push_back(accel_limit.get());
	params.push_back(bounds_min.get());
	params.push_back(bounds_max.get());
	return params;
}

/**
 * @brief Returns the main motion parameters for the robot.
 *
 * @return (vector<float>)  params: { jogging_vel_limit, jogging_accel_limit, jogging_distance }
 */
vector<float> CableRobot::get_jogging_parameters()
{
	vector<float> params;
	params.push_back(jog_vel.get());
	params.push_back(jog_accel.get());
	params.push_back(jog_dist.get());
	return params;
}

/**
 * @brief Sets the main motion parameters for the robot.
 * 
 * @param (float)  velocity_max: velocity limit (RPM)
 * @param (float)  accel_max: acceleration limit (RPM/s)
 * @param (float)  position_min: minimum position bounds
 * @param (float)  position_max: maximum position bounds
 */
void CableRobot::set_motion_parameters(float velocity_max, float accel_max, float position_min, float position_max)
{
	vel_limit.set(velocity_max);
	accel_limit.set(accel_max);
	bounds_min.set(position_min);
	bounds_max.set(position_max);
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
	info_position_mm.set(ofToString(get_position_actual()));
	info_position_cnt.set(ofToString(motor_controller->get_motor()->get_position(false)));	// for debugging
}

bool CableRobot::shutdown(int timeout)
{
	ofLogNotice("CableRobot::on_run_shutdown") << "Shutting Down CableRobot " << motor_controller->get_motor_id();
	jog_vel.set(50);
	jog_accel.set(200);
	jog_dist.set(bounds_max.get());
	jog_down();

	auto diff = position_shutdown - bounds_max.get();
	jog_vel.set(10);
	jog_accel.set(50);
	jog_dist.set(diff);
	jog_down(true);

	float timer = ofGetElapsedTimeMillis();
	bool shutting_down = true;
	while (shutting_down) {
		// check if we've timed out
		if (ofGetElapsedTimeMillis() > timer + (timeout * 1000)) {
			ofLogWarning("CableRobot::shutdown") << "CableRobot " << motor_controller->get_motor_id() << " did not finish shutting down. Timed out after " << timeout << " seconds.";
			return false;
		}
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
	return true;
}

void CableRobot::key_pressed(int key)
{
	switch (key)
	{
	// minimize all gui groups, expect control
	case '-':
		panel.minimizeAll();
		panel.getGroup("Control").maximize();
		break;
	// maximize all gui groups
	case '+':
		panel.maximizeAll();
		break;
	default:
		break;
	}
}

int CableRobot::get_id()
{
	return motor_controller->get_motor_id();
}

/**
 * @brief Returns whether the target position is in bounds based on absolute coordinates.
 * 
 * @param (float)  target_pos_absolute: target position in absolute coordinates.
 * @return (bool)  
 */
bool CableRobot::is_in_bounds_absolute(float target_pos_absolute)
{
	return target_pos_absolute >= bounds_min.get() && target_pos_absolute <= bounds_max.get();
}

/**
 * @brief Returns whether the target position would be in bounds based on a relative move.
 * 
 * @param (float)  target_pos_relative: (-) for relative move UP and (+) for relative move DOWN
 * @return (bool)  
 */
bool CableRobot::is_in_bounds_relative(float target_pos_relative)
{
	int curr_pos = count_to_mm(motor_controller->get_motor()->get_position(), true);
	// (-) relative move is UP
	if (target_pos_relative < 0) {
		return curr_pos - abs(target_pos_relative) >= bounds_min.get();
	}
	// (+) relative move is DOWN
	else {
		return curr_pos + abs(target_pos_relative) <= bounds_max.get();
	}
}

/**
 * @brief Returns whether the target position would be in bounds.
 * Use (-) for relative move UP and (+) for relative move DOWN.
 *
 * @param (float)  target_pos: target position in absolute or relative coordinates.
 * @param (bool) is_aboslute: target_pos is in absolute or relative coordinates
 * @return (bool)
 */
bool CableRobot::is_in_bounds(float target_pos, bool is_absolute)
{
	if (is_absolute) {
		return target_pos >= bounds_min.get() && target_pos <= bounds_max.get();
	}
	else {
		int curr_pos = count_to_mm(motor_controller->get_motor()->get_position(), true);
		// (-) relative move is UP
		if (target_pos < 0) {
			return curr_pos - abs(target_pos) >= bounds_min.get();
		}
		// (+) relative move is DOWN
		else {
			return curr_pos + abs(target_pos) <= bounds_max.get();
		}
	}
}

/**
 * @brief Returns the sign for moving the motor based on the cable drum's groove direction.
 *  
 * A homed, left-handed drum operates in negative rotation space.
 * @return (int)  -/+ 1 depending on configuration
 */
int CableRobot::get_rotation_direction()
{
	return drum.direction == Groove::LEFT_HANDED ? -1 : 1;
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

	panel.setup("Motor_" + ofToString(get_id()));
	panel.setWidthElements(gui_width);
	panel.setPosition(10, 15);
	panel.add(status.set("Status", state_names[0]));

	params_control.setName("Control");
	params_control.add(enable.set("Enable", false));
	params_control.add(e_stop.set("E_Stop", false));
	params_control.add(btn_run_homing.set("Run_Homing"));
	params_control.add(btn_run_shutdown.set("Run_Shutdown"));

	params_info.setName("Info");
	params_info.add(info_position_mm.set("Position_(mm)", ""));
	params_info.add(info_position_cnt.set("Position_(cnt)", ""));
	string vel_lim = ofToString(motor_controller->get_motor()->get_velocity());
	string accel_lim = ofToString(motor_controller->get_motor()->get_acceleration());
	params_info.add(info_vel_limit.set("Vel_Limit_(RPM)", vel_lim));
	params_info.add(info_accel_limit.set("Accel_Limit_(RPM/s)", accel_lim));

	params_limits.setName("Limits");
	params_limits.add(vel_limit.set("Vel_Limit_(RPM)", 30, 0, 300));
	params_limits.add(accel_limit.set("Accel_Limit_(RPM/s)", 200, 0, 800));
	params_limits.add(bounds_min.set("Bounds_Min", 0, 0, 2000));
	params_limits.add(bounds_max.set("Bounds_Max", 2000, 0, 2000));

	params_jog.setName("Jogging");
	params_jog.add(jog_vel.set("Jog_Vel", 30, 0, 200));			// RPM
	params_jog.add(jog_accel.set("Jog_Accel", 200, 20, 800));	// RPM_PER_SEC
	params_jog.add(jog_dist.set("Jog_Dist", 50, 0, 1000));		// MM
	params_jog.add(btn_jog_up.set("Jog_Up"));
	params_jog.add(btn_jog_down.set("Jog_Down"));

	params_move_to.setName("Move_To");
	int val = (bounds_min.get() + bounds_max.get()) / 2;
	params_move_to.add(move_to.set("Move_to_Pos", val, bounds_min.get(), bounds_max.get()));
	params_move_to.add(btn_move_to.set("Send_Move"));

	
	// bind GUI listeners
	e_stop.addListener(this, &CableRobot::on_e_stop);
	enable.addListener(this, &CableRobot::on_enable);
	btn_run_homing.addListener(this, &CableRobot::on_run_homing);
	btn_run_shutdown.addListener(this, &CableRobot::on_run_shutdown);
	btn_move_to.addListener(this, &CableRobot::on_move_to);
	btn_jog_up.addListener(this, &CableRobot::on_jog_up);
	btn_jog_down.addListener(this, &CableRobot::on_jog_down);
	vel_limit.addListener(this, &CableRobot::on_vel_limit_changed);
	accel_limit.addListener(this, &CableRobot::on_accel_limit_changed);
	bounds_max.addListener(this, &CableRobot::on_bounds_changed);
	bounds_min.addListener(this, &CableRobot::on_bounds_changed);

	panel.add(params_control);
	panel.add(params_info);
	panel.add(params_limits);
	panel.add(params_jog);
	panel.add(params_move_to);

	// Minimize less important parameters
	//panel.getGroup("Limits").minimize();
}

/**
 * @brief Converts from motor position (count) to linear distance (mm).
 *
 * @param (int)  val: motor position (in step counts)
 * @param (bool) use_unsigned: returns abs(val) if true. False by default.
 * 
 * @return (float)  linear distance (in mm)
 */
float CableRobot::count_to_mm(int val, bool use_unsigned)
{
	if(use_unsigned)
		return abs(val) * mm_per_count;
	return val * mm_per_count;
}

/**
 * @brief Converts from linear distance (mm) to motor position.
 *
 * @param (float)  val:  linear distance (in mm)
 * @param (bool) use_unsigned: returns abs(val) if true. False by default.
 * 
 * @return (int) motor position (in step counts)
 */
int CableRobot::mm_to_count(float val, bool use_unsigned)
{
	if (use_unsigned)
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
 * @brief Issues a position move command: converts from mm to motor counts.
 * 
 * This is a trapezoidal move, meaning that the motor with accelerate, 
 * then decelerate and stop at the target position.
 * 
 * NOTE: Use only for Point-to-Point / Waypoint moves. Use move_velocity for
 * streaming smooth trajectories.
 *
 * @param (int)  target_pos: target position (mm).
 * @param (bool)  is_absolute: target is abosulte or relative to current postion. TRUE by default.
 */
void CableRobot::move_position(float target_pos, bool is_absolute)
{
	// send move command
	if (is_absolute) {
		// convert from mm to motor counts and flip sign based on cable drum groove direction
		int count = mm_to_count(abs(target_pos)) * get_rotation_direction();
		if (is_homed()) {
			if (is_in_bounds(target_pos, true)) {
				motor_controller->get_motor()->move_position(count, true);
			}
			else
				ofLogWarning("CableRobot::move_position") << "Move not sent: The target move would have been out of bounds.";
		}
		else {
			ofLogWarning("CableRobot::move_position") << "Move not sent: You must HOME the robot before you move to an absolute position.";
		}
	}
	else {
		 ofLogWarning("CableRobot::move_position") << "Move not sent: Relative move not implemented yet.";
		// @TODO
		//if (is_in_bounds_relative(target_pos))
		//	motor_controller->get_motor()->move_position(count, false);
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
	int timeout = 20;
	shutdown(timeout);
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
		// check if the target jogging distance is in bounds
		// (-) jog move is UP
		bool in_bounds = is_in_bounds(jog_dist.get() * -1, false);	// multiply (jog_dist.get() * -1) to signal jog UP direction
		if (in_bounds) {
			// save the current vel and accel values
			float curr_vel_limit = motor_controller->get_motor()->get_velocity();
			float curr_accel = motor_controller->get_motor()->get_acceleration();
			// switch to the jogging vel and accel values
			motor_controller->get_motor()->set_velocity(jog_vel.get());
			motor_controller->get_motor()->set_acceleration(jog_accel.get());
			// send the move
			float curr_pos_mm = count_to_mm(motor_controller->get_motor()->get_position(), true);
			float pos = curr_pos_mm - jog_dist.get();
			move_position(pos, true);
			// switch back to previous vel and accel values
			motor_controller->get_motor()->set_velocity(curr_vel_limit);
			motor_controller->get_motor()->set_acceleration(curr_accel);
		}
		else {
			ofLogWarning("CableRobot::on_jog_up") << "Moving CableRobot " << motor_controller->get_motor_id() << " to bounds_min: " << bounds_min.get();
			move_position(bounds_min.get(), true); 
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
		// check if the target jogging distance is in bounds
		// (+) jog move is DOWN
		bool in_bounds = is_in_bounds(jog_dist.get(), false);
		float curr_pos_mm = count_to_mm(motor_controller->get_motor()->get_position(), true);
		float pos = curr_pos_mm + jog_dist.get();
		if (!override) {
			if (in_bounds) {
				// save the current vel and accel values
				float curr_vel_limit = motor_controller->get_motor()->get_velocity();
				float curr_accel = motor_controller->get_motor()->get_acceleration();
				// switch to the jogging vel and accel values
				motor_controller->get_motor()->set_velocity(jog_vel.get());
				motor_controller->get_motor()->set_acceleration(jog_accel.get());
				// send the move
				move_position(pos, true);
				// switch back to previous vel and accel values
				motor_controller->get_motor()->set_velocity(curr_vel_limit);
				motor_controller->get_motor()->set_acceleration(curr_accel);
			}
			else {
				ofLogWarning("CableRobot::on_jog_down") << "Moving CableRobot " << motor_controller->get_motor_id() << " to bounds_max: " << bounds_max.get();
				move_position(bounds_max.get(), true);
			}
		}
		// override bounds_max for shutdown routine; send the move directly to the motor
		else {
			int count = mm_to_count(jog_dist.get()) * get_rotation_direction();	
			motor_controller->get_motor()->move_position(count, false);
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

/**
 * @brief GUI Callback to Move To absolute position.
 */
void CableRobot::on_move_to()
{
	move_position(move_to.get(), true);
}

void CableRobot::on_bounds_changed(float& val)
{
	move_to.setMin(bounds_min.get());
	move_to.setMax(bounds_max.get());
}

/**
 * @brief Updates the desired velocity limit.
 * 
 * @param (float)  val: desired velocity limit (RPM)
 */
void CableRobot::on_vel_limit_changed(float& val)
{
	motor_controller->get_motor()->set_velocity(val);	
	// update the gui
	info_vel_limit.set(ofToString(val));
}

/**
 * @brief Updates the desired acceleration limit.
 * 
 * @param (float)  val: desired acceleration limit (RPM/s)
 */
void CableRobot::on_accel_limit_changed(float& val)
{
	motor_controller->get_motor()->set_acceleration(val);
	// update the gui
	info_accel_limit.set(ofToString(val));
}
