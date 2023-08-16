#include "CableRobot.h"

CableRobot::CableRobot(SysManager& SysMgr, INode* node, bool load_config_file)
{
	motor_controller = new MotorController(SysMgr, node);
	setup_gui();
	motor_controller->get_motor()->set_motion_params(vel_limit.get(), accel_limit.get());

	check_for_system_ready();

	this->load_config_file = load_config_file;
	if (load_config_file) {
		load_config_from_file();
	}
}

CableRobot::CableRobot(glm::vec3 base)
{
}

void CableRobot::configure(ofNode* _origin, ofNode* _ee, glm::vec3 base, Groove direction, float diameter_drum, float length, int turns)
{
	this->origin = _origin;
	this->ee = _ee;
	this->base.setParent(*origin);
	this->base.setPosition(base);
	drum.initialize(direction, diameter_drum, length, turns);

	// HARD CODED
	if (motor_controller->get_motor_id() == 0) {
		int x = drum.get_tangent().x + 5;
		int y = drum.get_tangent().y - 80;
		drum.set_tangent(glm::vec3(x, y, 0));
	}
	else {
		int x = drum.get_tangent().x - 10;
		int y = drum.get_tangent().y - 90;
		drum.set_tangent(glm::vec3(x, y, 0));
	}

	// define the linear length per motor step
	float step_resolution = motor_controller->get_motor()->get_resolution();
	mm_per_count = drum.circumference / step_resolution;

	// setup the kinematic chain
	tangent.setParent(this->base);
	tangent.setPosition(drum.get_tangent());
	target.setPosition(0, 0, 0);
	actual.setParent(tangent);
	actual.setPosition(0, 0, 0);

	// if the external ee node is empty, create a new one that is internal and add a gizmo
	if (ee->getGlobalPosition() == glm::vec3() && ee->getGlobalOrientation() == glm::quat()) {
		ee = new ofNode();
		ee->setParent(tangent);
		ee->setPosition(0, -1 * (bounds_max.get() - bounds_min.get()) / 2, 0);
		target.setParent(*ee);

		gizmo_ee.setNode(*ee);
		gizmo_ee.setDisplayScale(.5);
		gizmo_ee.setTranslationAxisMask(IGizmo::AXIS_Y);
		gizmo_ee.setRotationAxisMask(IGizmo::AXIS_Z);
	}
	// otherwise, parent the target to the external ee
	else {
		// set the target position to the actual position
		float dist_actual = get_position_actual();
		actual.setPosition(0, -1 * dist_actual, 0);
		target.setParent(*ee);
		float x = this->base.getPosition().x + tangent.getPosition().x - ee->getPosition().x;
		target.setPosition(x, 0, ee->getPosition().z);
	}

	// set the trajectory starting point
	trajectory.reset(target.getGlobalPosition());
	trajectory.desired_pos.set(target.getGlobalPosition());

	if (load_config_file) {
		load_config_from_file();
	}
}

void CableRobot::configure(ofNode* _origin, glm::vec3 base, ofNode* _ee)
{
	this->origin = _origin;
	this->ee = _ee;
	this->base.setParent(*origin);
	this->base.setPosition(base);
	drum.initialize(drum.direction, drum.diameter_drum, drum.length, drum.turns);

	// HARD CODED
	if (motor_controller->get_motor_id() == 0) {
		int x = drum.get_tangent().x + 5;
		int y = drum.get_tangent().y - 80;
		drum.set_tangent(glm::vec3(x, y, 0));
	}
	else {
		int x = drum.get_tangent().x - 10;
		int y = drum.get_tangent().y - 90;
		drum.set_tangent(glm::vec3(x, y, 0));
	}

	// define the linear length per motor step
	float step_resolution = motor_controller->get_motor()->get_resolution();
	mm_per_count = drum.circumference / step_resolution;

	// setup the kinematic chain
	tangent.setParent(this->base);
	tangent.setPosition(drum.get_tangent());
	target.setPosition(0, 0, 0);
	actual.setParent(tangent);
	actual.setPosition(0, 0, 0);

	// if the external ee node is empty, create a new one that is internal and add a gizmo
	if (ee->getGlobalPosition() == glm::vec3() && ee->getGlobalOrientation() == glm::quat()) {
		ee = new ofNode();
		ee->setParent(tangent);
		ee->setPosition(0, -1 * (bounds_max.get() - bounds_min.get()) / 2, tangent.getGlobalPosition().z);
		target.setParent(*ee);

		gizmo_ee.setNode(*ee);
		gizmo_ee.setDisplayScale(.5);
		gizmo_ee.setTranslationAxisMask(IGizmo::AXIS_Y);
		gizmo_ee.setRotationAxisMask(IGizmo::AXIS_Z);
	}
	// otherwise, parent the target to the external ee and offset by ee_offset value
	else {
		target.setParent(*ee);
		target.setGlobalPosition(ee->getGlobalPosition());
		if (motor_controller->get_motor_id() % 2 == 0)		
			target.setPosition(-40, 0, 0);				// <-- ee_offset value hard coded offset at the moment : (
		else
			target.setPosition(40, 0, 0);


		gizmo_ee.setNode(*ee);
		gizmo_ee.setDisplayScale(.5);
		gizmo_ee.setRotationAxisMask(IGizmo::AXIS_Z);
	}
	//trajectory.reset();
	trajectory.curr_pos.set(target.getGlobalPosition());
	trajectory.desired_pos.set(target.getGlobalPosition());

	if (load_config_file) {
		load_config_from_file();
	}
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

void CableRobot::set_jogging_parameters(float jogging_vel, float jogging_accel, float jogging_dist)
{
	jog_vel.set(jogging_vel);
	jog_accel.set(jogging_accel);
	jog_dist.set(jogging_dist);
}

void CableRobot::set_velocity_limit(float velocity_max)
{
	vel_limit.set(velocity_max);
}

void CableRobot::set_accel_limit(float accel_max)
{
	accel_limit.set(accel_max);
}

/**
 * @brief Set minimum and maximum travel bounds .
 *
 * @param (float)  min: minimum bounds (in mm)
 * @param (float)  max: maximum bounds (in mm)
 */
void CableRobot::set_bounds(float min, float max)
{
	bounds_min.set(min);
	bounds_max.set(max);
}

bool CableRobot::is_torque_in_limits()
{
	auto torque_measured = motor_controller->get_motor()->get()->Motion.TrqMeasured.Value();
	//info_torque_actual.set(ofToString(torque_measured));
	if (torque_measured > torque_max.get() || torque_measured < torque_min.get()) {
		ofLogWarning(" CableRobot::is_torque_in_limits()") << "\tRobot " << ofToString(motor_controller->get_motor_id()) << " is OUT OF TORQUE RANGE with value of " << ofToString(torque_measured) << endl;
		return false;
	}
	return true;
}

void CableRobot::load_config_from_file(string filename)
{
	//motor_controller->get_motor()->get()->Status.RT.Refresh();
	//while (!motor_controller->get_motor()->get()->Status.IsReady() && !is_setup) {
	//	motor_controller->get_motor()->get()->Status.RT.Refresh();
	//	cout << "waiting..." << endl;
	//}

	int serial_number = int(motor_controller->get_motor()->get()->Info.SerialNumber.Value());
	if (filename == "")
		filename = "robot_config_" + ofToString(serial_number) + ".xml";
	ofLogNotice("CableRobot::load_config_from_file") << "Loading config file: " << filename;
	try
	{
		bool val = config.loadFile(filename);
		if (val) {
			// Check that the serial numbers match
			int sn = config.getValue("config:serial_number", 0);
			if (sn != serial_number) {
				ofLogWarning("CableRobot::load_config_from_file") << "Cannot load config file: Wrong serial number. Loaded " << ofToString(sn) << ", but should be " << ofToString(serial_number);
				return;
			}

			auto_home = config.getValue("config:auto_home", 0);

			ofLogNotice("CableRobot::load_config_from_file") << "\tSettings: " ;
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tSerial Number: " << sn;
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tAuto Homing: " << (auto_home ? "TRUE" : "FALSE");

			// kinematics
			float x = config.getValue("config:kinematics:base:X", 0);
			float y = config.getValue("config:kinematics:base:Y", 0);
			float z = config.getValue("config:kinematics:base:Z", 0);
			base.setPosition(x, y, z);

			x = config.getValue("config:kinematics:tangent:X", 0);
			y = config.getValue("config:kinematics:tangent:Y", 0);
			z = config.getValue("config:kinematics:tangent:Z", 0);
			tangent.setPosition(x, y, z);

			x = config.getValue("config:kinematics:target:X", 0);
			y = config.getValue("config:kinematics:target:Y", 0);
			z = config.getValue("config:kinematics:target:Z", 0);
			target.setPosition(x, y, z);

			ofLogNotice("CableRobot::load_config_from_file") << "\Kinematics: ";
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tBase Local Pos: " << ofToString(base.getPosition());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tTangent Local Pos: " << ofToString(tangent.getPosition());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tTarget Local Pos: " << ofToString(target.getPosition());

			// limits
			vel_limit.set(config.getValue("config:limits:vel_limit", 0));
			accel_limit.set(config.getValue("config:limits:accel_limit", 0));
			bounds_min.set(config.getValue("config:limits:bounds_min", 0));
			bounds_max.set(config.getValue("config:limits:bounds_max", 0));
			position_shutdown = config.getValue("config:limits:bounds_shutdown", 0);
			torque_min.set(config.getValue("config:limits:torque_min", 0));
			torque_max.set(config.getValue("config:limits:torque_max", 0));

			ofLogNotice("CableRobot::load_config_from_file") << "\tLimits: ";
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tVel Limit: " << ofToString(vel_limit.get());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tAccel Limit: " << ofToString(accel_limit.get());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tBounds Min: " << ofToString(bounds_min.get());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tBounds Max: " << ofToString(bounds_max.get());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tBounds Shutdown: " << ofToString(position_shutdown);
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tTorque Min: " << ofToString(torque_min.get());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tTorque Max: " << ofToString(torque_max.get());

			// jogging
			jog_vel.set(config.getValue("config:jogging:jog_vel", 0));
			jog_accel.set(config.getValue("config:jogging:jog_accel", 0));

			ofLogNotice("CableRobot::load_config_from_file") << "\tJogging: ";
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tJog Vel: " << ofToString(jog_vel.get());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tJog Accel: " << ofToString(jog_accel.get());

			// cable drum
			drum.direction = Groove(config.getValue("config:cable_drum:direction", 0));
			drum.set_diameter(config.getValue("config:cable_drum:diameter_drum", 0));
			x = config.getValue("config:cable_drum:tangent:X", 0);
			y = config.getValue("config:kinematics:tangent:Y", 0);
			z = config.getValue("config:kinematics:tangent:Z", 0);
			drum.set_tangent(glm::vec3(x, y, z));

			ofLogNotice("CableRobot::load_config_from_file") << "\tCable Drum: ";
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tDirection: " << (drum.direction == Groove::LEFT_HANDED ? "LEFT_HANDED" : "RIGHT_HANDED");
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tDrum Diameter: " << ofToString(drum.get_diameter());
			ofLogNotice("CableRobot::load_config_from_file") << "\t\tDrum Tangent: " << ofToString(drum.get_tangent()) << endl;

		}
		else {
			ofLogWarning("CableRobot::load_config_from_file") << "No config file found at: /bin/data/" << filename;
		}
	}
	catch (const std::exception& e)
	{
		ofLogError("CableRobot::load_config_from_file") << e.what();
	}
	
	
}

bool CableRobot::save_config_to_file(string filename)
{
	ofxXmlSettings config;
	int serial_number = int(motor_controller->get_motor()->get()->Info.SerialNumber.Value());

	// robot
	config.addTag("config");
	config.pushTag("config");

	config.addValue("timestamp", ofGetTimestampString());
	config.addValue("serial_number", serial_number);
	config.addValue("auto_home", auto_home);

	// kinematics
	config.addTag("kinematics");
	config.pushTag("kinematics");

	config.addTag("base");
	config.pushTag("base");	// store the local coordinates
	config.addValue("X", base.getPosition().x);	
	config.addValue("Y", base.getPosition().y);
	config.addValue("Z", base.getPosition().z);
	config.popTag();

	config.addTag("tangent");
	config.pushTag("tangent");	// store the local coordinates
	config.addValue("X", tangent.getPosition().x);
	config.addValue("Y", tangent.getPosition().y);
	config.addValue("Z", tangent.getPosition().z);
	config.popTag();

	config.addTag("target");
	config.pushTag("target");	// store the local coordinates
	config.addValue("X", target.getPosition().x);
	config.addValue("Y", target.getPosition().y);
	config.addValue("Z", target.getPosition().z);
	config.popTag();

	config.popTag();

	// limits
	config.addTag("limits");
	config.pushTag("limits");
	config.addValue("vel_limit", vel_limit.get());
	config.addValue("accel_limit", accel_limit.get());
	config.addValue("bounds_min", bounds_min.get());
	config.addValue("bounds_max", bounds_max.get());
	config.addValue("bounds_shutdown", position_shutdown);
	config.addValue("torque_min", torque_min.get());
	config.addValue("torque_max", torque_max.get());
	config.popTag();

	// jogging
	config.addTag("jogging");
	config.pushTag("jogging");
	config.addValue("jog_vel", jog_vel.get());
	config.addValue("jog_accel", jog_accel.get());
	config.popTag();

	// cable drum
	config.addTag("cable_drum");
	config.pushTag("cable_drum");
	config.addValue("direction", drum.direction);
	config.addValue("diameter_drum", drum.get_diameter());
	config.addValue("diameter_cable", 0.3048);
	config.addValue("length", 30);
	config.addValue("turns", 40);
	config.addTag("tangent");
	config.pushTag("tangent");
	config.addValue("X", drum.get_tangent().x);
	config.addValue("Y", drum.get_tangent().y);
	config.addValue("Z", drum.get_tangent().z);
	config.popTag();
	config.popTag();

	config.popTag();

	if (filename == "")
		filename = "robot_config_" + ofToString(serial_number) + ".xml";
	
	return config.saveFile(filename);
}

void CableRobot::update()
{
	// check if we should be homing
	if (state == RobotState::HOMING) {
		// if we successfully homed, update the state and gui color
		if (run_homing_routine(60)) {
			bool _is_enabled = is_enabled();
			state = _is_enabled ? RobotState::ENABLED : RobotState::DISABLED;
			status.set(state_names[state]);
			auto color = _is_enabled ? mode_color_enabled : mode_color_disabled;
			panel.setBorderColor(color);
		}
		else {
			//state = RobotState::NOT_HOMED;
			//status.set(state_names[state]);
			//panel.setBorderColor(mode_color_not_homed);
		}
	}

	if (move_type == MoveType::VEL) {

		//update_trajectory();
		
		if (state != RobotState::E_STOP) {
			if (is_in_bounds(abs(actual.getPosition().y), true)){	
				//move_velocity_rpm(trajectory.get_rpm());		// MAD EDIT 8/14/2023		
			}
			else {
				stop();
				stop();
				ofLogWarning("CableRobot::update") << "Stopping CableRobot " << motor_controller->get_motor_id() << ": trying to send past home position." << endl;
			}
		}
	}
	else {
		//ofLogNotice("CableRobot::configure") << motor_controller->get_motor_id() << " target global position: " << ofToString(target.getGlobalPosition());
		//ofLogNotice("CableRobot::configure") << motor_controller->get_motor_id() << " ee global position: " << ofToString(ee->getGlobalPosition());
		//ofLogNotice("CableRobot::configure") << motor_controller->get_motor_id() << " gizmo_ee global position: " << ofToString(gizmo_ee.getTranslation()) << endl;
	}

	// update the actual positions
	float dist_actual = get_position_actual();
	actual.setPosition(0, -1 * dist_actual, 0);

	// update the gui
	//info_velocity_actual.set(ofToString(motor_controller->get_motor()->get()->Motion.VelMeasured.Value()));
}

void CableRobot::draw()
{
	ofPushStyle();

	ofSetLineWidth(2);
	ofSetColor(255);
	ofNoFill();

	// draw base and tangent nodes
	ofDrawEllipse(base.getGlobalPosition(), drum.get_diameter(), drum.get_diameter());
	ofDrawLine(base.getGlobalPosition(), tangent.getGlobalPosition());
	base.draw();
	tangent.draw();


	// draw current ee

	// draw ghosted line between tangent and target & ee and target
	// show RED if the target is out of bounds
	if (-1 * ee->getPosition().y > bounds_min.get() && 
		-1 * ee->getPosition().y < bounds_max.get()) {
		ofSetLineWidth(1);
		ofSetColor(120);
	}
	else {
		ofSetLineWidth(5);
		ofSetColor(ofColor::red, 30);
	}
	ofDrawLine(tangent.getGlobalPosition(), target.getGlobalPosition());
	ofDrawLine(ee->getGlobalPosition(), target.getGlobalPosition());
	

	// draw the target
	ofFill();
	ofSetColor(ofColor::red, 200);
	ofDrawEllipse(target.getGlobalPosition(), 50, 50);
	


	if (debugging) {
		// draw the 1D trajectory and actual position
		trajectory.draw();
		draw_cable(tangent.getGlobalPosition(), actual.getGlobalPosition());

		// draw distance to target
		ofSetColor(60);
		string msg = ofToString(glm::distance(tangent.getGlobalPosition(), target.getGlobalPosition())) + " (mm)";
		auto pt = target.getGlobalPosition();
		int offset = 50;
		if (motor_controller->get_motor_id() % 2 == 0) {
			offset *= -5.5;
		}
		ofDrawBitmapString(msg, pt.x + offset, pt.y - 10, pt.z);

		// draw distance to actual
		msg = ofToString(glm::distance(tangent.getGlobalPosition(), actual.getGlobalPosition())) + " (mm)";
		pt = (tangent.getGlobalPosition() + actual.getGlobalPosition()) / 2.0;
		ofDrawBitmapString(msg, pt.x + offset, pt.y - 10, pt.z);
	}

	// draw the trajectory and actual position in (simulated) world coordinates
	if (trajectory_world_coords.getVertices().size() > 0) {
		ofSetLineWidth(5);
		ofSetColor(255, 0, 255, 100);
		trajectory_world_coords.draw();

		float dist = actual.getPosition().y;
		glm::vec3 heading = glm::normalize(tangent.getGlobalPosition() - trajectory_world_coords.getVertices()[0]) * dist ;
		actual_world_pos = tangent.getGlobalPosition() + heading;
		draw_cable(tangent.getGlobalPosition(), actual_world_pos);
	}

	//// draw distance to actual in (simulated) world coordinates
	//msg = ofToString(glm::distance(tangent.getGlobalPosition(), actual.getGlobalPosition())) + " (mm)";
	//pt = (tangent.getGlobalPosition() + actual.getGlobalPosition()) / 2.0;
	//ofDrawBitmapString(msg, pt.x + offset, pt.y - 10, pt.z);

	ofPopStyle();


	// update gui
	//info_position_mm.set(ofToString(get_position_actual()));
	//info_position_cnt.set(ofToString(motor_controller->get_motor()->get_position(false)));	// for debugging
}

void CableRobot::update_gizmo()
{
	if (override_gizmo) {
		gizmo_ee.setNode(*ee);
	}
	else {
		ee->setGlobalPosition(gizmo_ee.getTranslation());
		ee->setGlobalOrientation(gizmo_ee.getRotation());
		
		if (gizmo_ee.isInteracting()) {
			// update the gui
			update_move_to();
		}
	}	


}

void CableRobot::update_move_to() {
	float dist = glm::distance(tangent.getGlobalPosition(), target.getGlobalPosition());
	// don't go past the min bounds
	if (dist < bounds_min.get())
		dist = bounds_min.get();
	move_to.set(dist);
}

bool CableRobot::shutdown(int timeout)
{
	// turn off any velocity moves
	move_to_vel.set(false);
	move_type = MoveType::POS;

	ofLogNotice("CableRobot::on_run_shutdown") << "Shutting Down CableRobot " << motor_controller->get_motor_id();
	vel_limit.set(20);
	accel_limit.set(200);

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
	// trigger stop with SPACEBAR
	case ' ':
		stop();
		stop();
		break;
	// minimize all gui groups, expect control
	case '-':
		panel.minimizeAll();
		panel.getGroup("Control").maximize();
		break;
	// maximize all gui groups
	case '+':
		panel.maximizeAll();
		break;
	case '?':
		debugging = !debugging;
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
 * @brief Returns whether the target position is in bounds.
 * Use (-) for relative move UP and (+) for relative move DOWN.
 *
 * @param (float)  target_pos: target position in absolute or relative coordinates (in mm).
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
 * @brief Returns the sign for moving the motor based on the cable drum's groove direction. LEFT_HANDED = -1, RIGHT_HANDED = 1.
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
	RobotState _state;
	ofColor color;

	// check for ESTOP first
	if (is_estopped())
	{
		_state = RobotState::E_STOP;
		color = mode_color_estopped;
		e_stop.set(true);
	}
	else if (!is_homed()) {

		_state = RobotState::NOT_HOMED;
		color = mode_color_not_homed;
	}
	else {
		bool _is_enabled = is_enabled();
		_state = _is_enabled ? RobotState::ENABLED : RobotState::DISABLED;
		color = _is_enabled ? mode_color_enabled : mode_color_disabled;

		// check that the gui matched the current state
		if (enable.get() != _is_enabled)
			enable.set(_is_enabled);
	}
	state = _state;
	status.set(state_names[state]);
	panel.setBorderColor(color);
}

/**
 * @brief Returns whether the motor is ready for a movement command.
 * Motor is ready when there's no E-Stop, it's Homed, and it's Enabled.
 *
 * @return (bool)
 */
bool CableRobot::is_ready()
{
	return is_homed() && is_enabled();
}

void CableRobot::update_trajectory()
{
	////////////////////////////
	// MAD EDIT 08/10/2023 
	bool remove_target = false;
	if (trajectory.get_num_targets() == 0){
		remove_target = false;
	}
	else {
		float dist_sq = trajectory.curr_pos.distanceSquared(trajectory.path.getVertices()[0]);
		if (dist_sq <= trajectory.look_ahead_radius.get() * trajectory.look_ahead_radius.get() && trajectory.get_num_targets() >= 2) {
			move_done = true;
		}
	}
	////////////////////////////

	// check if we need to remove a target from the world trajectory
	if (trajectory.get_num_targets() < trajectory_world_coords.getVertices().size() &&
		trajectory_world_coords.getVertices().size() > 0) {
		trajectory_world_coords.removeVertex(0);
	}

	//// compensate for x-axis tilt
	//float diff = planar_compensation_x.get() - prev_planar_compensation_x;
	//glm::vec3 x_tilt_compensation = glm::vec3(0, -diff, 0);
	//target.setGlobalPosition(target.getGlobalPosition() + x_tilt_compensation);

	// get the distance to the target and the distance to the trajectory's last target
	float dist = glm::distance(tangent.getGlobalPosition(), target.getGlobalPosition());

	// check if we need to add a target to the trajectory
	bool add_target = false;
	if (trajectory.get_num_targets() == 0)
		add_target = true;
	else {
		// add the target to the trajectory path, but don't add small moves
		float dist_sq_to_last_target = glm::distance2(tangent.getGlobalPosition(), trajectory.get_last_target());
		float dist_diff_sq = abs(dist*dist - dist_sq_to_last_target);
		float threshold = 10;
		if (dist_diff_sq > threshold * threshold) {
			//	add_target = true;
		}
	}

	if (add_target){
		// get the correct sign for the distance target
		tangent.getGlobalPosition().y > target.getGlobalPosition().y ? dist *= -1 : dist *= 1;
		// convert the distance into a 1D target for the trajectory
		glm::vec3 trajectory_target = glm::vec3(tangent.getGlobalPosition());
		// compensate for lateral direction
		trajectory_target.y += dist;
		// add to the trajectory
		trajectory.add_target(trajectory_target);
		// save the trajectory targets as 3D world coordinates, too
		trajectory_world_coords.addVertex(target.getGlobalPosition());
	}
	
	// update the trajectory's position / vel / heading / rpm
	trajectory.update(actual.getGlobalPosition());
}

void CableRobot::setup_gui()
{
	mode_color_estopped = ofColor(250, 0, 0, 100);
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
	params_info.add(info_velocity_target.set("Vel_Target_(RPM)", "0"));
	params_info.add(info_velocity_actual.set("Vel_Actual_(RPM)", "0"));
	params_info.add(info_torque_actual.set("Torque_Actual_(%)", "0"));

	params_limits.setName("Limits");
	params_limits.add(vel_limit.set("Vel_Limit_(RPM)", 100, 0, 300));
	params_limits.add(accel_limit.set("Accel_Limit_(RPM/s)", 800, 0, 1000));
	params_limits.add(bounds_min.set("Bounds_Min", 100, 0, 3000));
	params_limits.add(bounds_max.set("Bounds_Max", 3000, 0, 3000));
	params_limits.add(torque_min.set("Torque_Min", -5, -5, 10));
	params_limits.add(torque_max.set("Torque_Max", 40, 0, 100));

	params_jog.setName("Jogging");
	params_jog.add(jog_vel.set("Jog_Vel", 30, 0, 200));			// RPM
	params_jog.add(jog_accel.set("Jog_Accel", 200, 20, 800));	// RPM_PER_SEC
	params_jog.add(jog_dist.set("Jog_Dist", 50, 0, 1000));		// MM
	params_jog.add(btn_jog_up.set("Jog_Up"));
	params_jog.add(btn_jog_down.set("Jog_Down"));

	params_move.setName("Move");
	int val = (bounds_min.get() + bounds_max.get()) / 2;
	params_move.add(move_to.set("Move_To", val, bounds_min.get(), bounds_max.get()));
	params_move.add(move_to_pos.set("Move_Pos"));
	params_move.add(move_to_vel.set("Move_Vel", false));
	
	// bind GUI listeners
	e_stop.addListener(this, &CableRobot::on_e_stop);
	enable.addListener(this, &CableRobot::on_enable);
	btn_run_homing.addListener(this, &CableRobot::on_run_homing);
	btn_run_shutdown.addListener(this, &CableRobot::on_run_shutdown);
	move_to_pos.addListener(this, &CableRobot::on_move_to_pos);
	move_to.addListener(this, &CableRobot::on_move_to_changed);
	move_to_vel.addListener(this, &CableRobot::on_move_to_vel);
	btn_jog_up.addListener(this, &CableRobot::on_jog_up);
	btn_jog_down.addListener(this, &CableRobot::on_jog_down);
	vel_limit.addListener(this, &CableRobot::on_vel_limit_changed);
	accel_limit.addListener(this, &CableRobot::on_accel_limit_changed);
	bounds_max.addListener(this, &CableRobot::on_bounds_changed);
	bounds_min.addListener(this, &CableRobot::on_bounds_changed);

	vel_limit.set(100);
	accel_limit.set(800);
	trajectory.max_vel.set(vel_limit.get() / 2.0);

	panel.add(params_control);
	panel.add(params_info);
	panel.add(params_limits);
	panel.add(params_jog);
	panel.add(params_move);
	panel.add(trajectory.params);

	// Minimize less important parameters
	panel.getGroup("Info").minimize();
	panel.getGroup("Limits").minimize();
	panel.getGroup("Jogging").minimize();
	panel.getGroup("Info").minimize();
	panel.getGroup("Trajectory").minimize();

	is_setup = true;
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

bool CableRobot::is_moving()
{
	return motor_controller->get_motor()->is_moving();
}

bool CableRobot::is_estopped()
{
	bool val = motor_controller->get_motor()->is_estopped();
	if (val) {
		panel.setBorderColor(mode_color_estopped);
		state = RobotState::E_STOP;
		status.set(state_names[state]);
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
	state = RobotState::HOMING;
	status.set(state_names[state]);
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
	if (!is_estopped() && is_enabled() && is_homed()) {
		// send move command
		if (is_absolute) {
			// convert from mm to motor counts and flip sign based on cable drum groove direction
			if (is_in_bounds(target_pos, true)) {
				glm::vec3 pos = glm::vec3(0, -1 * target_pos, 0);
				ee->setPosition(pos);
				//target.setPosition(glm::vec3(0, -1 * target_pos, 0));
				int count = mm_to_count(abs(target_pos)) * get_rotation_direction();
				motor_controller->get_motor()->move_position(count, true);
			}
			else
				ofLogWarning("CableRobot::move_position") << "Move not sent: The target move would have been out of bounds.";
		}
		else {
			// @TODO
			ofLogWarning("CableRobot::move_position") << "Move not sent: Relative move not implemented yet.";
		}
	}
	else {
		string msg = "";
		if (is_estopped())
			msg = "Cannot move Robot " + ofToString(get_id()) + " while in an ESTOP state.";
		else if (!is_homed())
			msg = "Cannot move Robot " + ofToString(get_id()) + ". It must be homed first.";
		else
			msg = "Cannot move Robot " + ofToString(get_id()) + ". It must be enabled first.";
		ofLogWarning("CableRobot::move_velocity") << msg;
	}
}


void CableRobot::draw_cable(glm::vec3 start, glm::vec3 end)
{
	ofPushStyle();
	ofSetColor(255);
	ofSetLineWidth(2);
	ofDrawLine(start, end);
	ofFill();
	ofSetColor(ofColor::orange, 200);
	ofDrawEllipse(end, 40, 40);
	ofPopStyle();
}

void CableRobot::move_velocity_rpm(float rpm)
{
	if (!is_estopped() && is_enabled() && is_homed()) {
		//cout << "RPM from Trajectory: " << rpm << endl;

		// get whether we're moving up (1) or down (-1)
		if (trajectory.get_heading().y != 0)
			rpm *= trajectory.get_heading().y * -1;
		// convert for cable drum direction 
		rpm *= get_rotation_direction();
		// clamp to velocity limit
		//cout << "MOTOR " << get_id() << ", incoming RPM: " << rpm << endl;
		rpm = ofClamp(rpm, -vel_limit.get(), vel_limit.get());
		//cout << "\tfinal RPM: " << rpm << endl;
		
		// send velocity command to motor
		motor_controller->get_motor()->move_velocity(rpm);

		// update the gui
		//info_velocity_target.set(ofToString(rpm));
	}
	else {
		string msg = "";
		if (is_estopped())
			msg = "Cannot move Robot " + ofToString(get_id()) + " while in an ESTOP state.";
		else if (!is_homed())
			msg = "Cannot move Robot " + ofToString(get_id()) + ". It must be homed first.";
		else
			msg = "Cannot move Robot " + ofToString(get_id()) + ". It must be enabled first.";
		ofLogWarning("CableRobot::move_velocity") << msg;
	}
}

void CableRobot::remove_target(int index)
{
	trajectory.remove_target(index);
	move_done = false;
}

void CableRobot::stop()
{
	move_type = MoveType::POS;
	motor_controller->get_motor()->stop();
	move_done = true;

	// upadate the gui
	info_velocity_target.set("0");
}

void CableRobot::set_e_stop(bool val)
{
	if (val) {
		stop();
		stop();
	}
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
	ofColor color;
	if (val) {
		color = mode_color_estopped;
		state = RobotState::E_STOP;
	}
	else {
		bool _is_enabled = is_enabled();
		state = _is_enabled ? RobotState::ENABLED : RobotState::DISABLED;
		color = _is_enabled ? mode_color_enabled : mode_color_disabled;;
	}
	panel.setBorderColor(color);
	status.set(state_names[state]);
}

/**
 * @brief  Changes the state so the homing routing runs in update() thread.
 * 
 */
void CableRobot::on_run_homing()
{
	state = RobotState::HOMING;
	status.set(state_names[state]);
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
 *  @brief Sets the precision radius for when to move to the next point 
 *  in the trajectory. Smaller radius is more precise, but will move slower.
 * 
 * @param (float) val: radius of precision zone
 */
void CableRobot::set_zone(float val)
{
	trajectory.look_ahead_radius.set(val);
}

/**
 * @brief GUI Callback to Jog Down.
 */
void CableRobot::on_jog_down()
{
	jog_down();
}

void CableRobot::on_move_to_changed(float& val)
{
	glm::vec3 pos = glm::vec3(0, -1 * val, 0);
	ee->setPosition(pos);
	gizmo_ee.setNode(*ee);
}

/**
 * @brief GUI Callback to Move To absolute position.
 */
void CableRobot::on_move_to_pos()
{
	move_to_vel.set(false);
	move_type = MoveType::POS;
	float dist = glm::distance(tangent.getGlobalPosition(), target.getGlobalPosition());;
	move_position(dist, true);// move_to.get(), true);
}

void CableRobot::on_move_to_vel(bool &val)
{
	if (val)
		// velocity moves are handled in update()
		move_type = MoveType::VEL;
	else {
		stop();
		stop();
		trajectory.reset(target.getGlobalPosition());
	}
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
	// update the trajectory's max vel
	trajectory.max_vel.set(val / 2.0);
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
