#include "CableRobot2D.h"

CableRobot2D::CableRobot2D(CableRobot* top_left, CableRobot* top_right, ofNode* _origin, glm::vec3 base_top_left, glm::vec3 base_top_right, int id)
{
	robots.push_back(top_left);
	robots.push_back(top_right);


	this->base_top_left = base_top_left;
	this->base_top_right = base_top_right;

	this->origin = _origin;
	this->id = id;

	setup_gui();
	
	// Setup the end effector
	this->ee = new ofNode();
	this->ee->setParent(top_left->get_base());
	float h = robots[0]->bounds_max - robots[0]->bounds_min;
	float width = glm::distance(top_left->get_base().getGlobalPosition(), top_right->get_base().getGlobalPosition());
	glm::vec3 pos = glm::vec3((width / 2.0), - h / 2, top_left->get_base().getGlobalPosition().z);
	this->ee->setGlobalPosition(pos);

	// setup the end effector control gizmo
	gizmo_ee.setNode(*ee);
	gizmo_ee.setDisplayScale(.5);

	gizmo_ee.setRotationAxisMask(IGizmo::AXIS_Z);
	gizmo_ee.setScaleAxisMask(IGizmo::AXIS_X);

	// Configure the robots with one end effector
	robots[0]->configure(_origin, base_top_left, this->ee);
	robots[1]->configure(_origin, base_top_right, this->ee);

	// Setup the 2D bounds
	bounds.setHeight(-1 * h);
	float w = robots[1]->get_tangent().getGlobalPosition().x - robots[0]->get_tangent().getGlobalPosition().x;
	bounds.setWidth(w);
	pos = robots[0]->get_tangent().getGlobalPosition();
	pos.y -= robots[0]->bounds_min.get();
	bounds.setPosition(pos);

	// update the gui
	move_to.setMax(glm::vec2(bounds.getWidth(), -1 * bounds.getHeight()));
	move_to.set(glm::vec2(bounds.getWidth()/2, -1 * bounds.getHeight()/2));
	//zone.set(20);
	ee_offset.set(40);
	vel_limit.set(35);
	accel_limit.set(120);
	bounds_max.set(3000);

	this->ee->setGlobalPosition(bounds.getCenter().x, bounds.getCenter().y, top_left->get_base().getGlobalPosition().z);
	gizmo_ee.setNode(*ee);

	get_status();

	plot.name = "Bot 1 RPM: Motor 1 (RED), Motor 2 (BLUE)";
	plot.resolution = 125/2.0;
	plot.min = vel_limit.get() * -1;
	plot.max = vel_limit.get() * 1;
	plot.reset();
	plot.colors[0] = ofColor(ofColor::red);
	plot.colors[1] = ofColor::yellow;
	plot.colors[2] = ofColor(ofColor::blue);
	plot.colors[3] = ofColor::cyan;

}

void CableRobot2D::update()
{
	update_gizmo();
	

	//for (int i = 0; i < robots.size(); i++) {
	//	// monitor the torque and stop all motors if unexpected value
	//	if (robots[i]->is_torque_in_limits()) {

	//		//robots[i]->update();
	//	}
	//	else {
	//		if (robots[i]->is_moving()) {
	//			ofLogNotice("CableRobot2D::update()") << "TORQUE OUT OF RANGE: Stopping Robots " << endl;
	//			stop();
	//		}
	//	}
	//}

	if (move_to_vel) {
		//update_trajectories_2D();

		// Update each robot's velocity scalar so they arrive at the
		// target at the same time
		float scale_factor = 1.0;
		float dist_0 = robots[0]->actual_to_desired_distance;
		float dist_1 = robots[1]->actual_to_desired_distance;
		if (abs(dist_0) > abs(dist_1)) {
			if (dist_0 != 0) scale_factor = abs(dist_1 / dist_0);
			robots[1]->velocity_scalar = scale_factor;
		}
		else {
			if (dist_1 != 0) scale_factor = abs(dist_0 / dist_1);
			robots[0]->velocity_scalar = scale_factor;
		}

		// get the smoothed RPMs
		float rpm_0 = robots[0]->compute_velocity();
		float rpm_1 = robots[1]->compute_velocity();

		// record the raw and filtered rpm for visualization
		if (debugging) {
			plot_data[0] = robots[0]->plot_data_vel[0];
			plot_data[1] = robots[0]->plot_data_vel[1];
			plot_data[2] = robots[1]->plot_data_vel[0];
			plot_data[3] = robots[1]->plot_data_vel[1];
			plot.update(plot_data);
		}

		//robots[0]->move_velocity_rpm(rpm_0);
		//robots[1]->move_velocity_rpm(rpm_1);

		robots[0]->get_motor_controller()->get_motor()->move_velocity(rpm_0);
		robots[1]->get_motor_controller()->get_motor()->move_velocity(rpm_1);
	}
}

void CableRobot2D::draw_ee_path()
{
	//ofPushStyle();
	//ofSetLineWidth(5);
	//ofNoFill();
	//ofSetColor(250, 0, 250, 100);
	//// left cable bot
	//ofBeginShape();
	//for (int i = 0; i < ee_path.size(); i++) {
	//	ofVertex(ee_path[i][0]);
	//}
	//ofEndShape();
	//// right cable bot
	//ofBeginShape();
	//for (int i = 0; i < ee_path.size(); i++) {
	//	ofVertex(ee_path[i][2]);draw_ee_path
	//}
	//ofEndShape();

	//ofSetLineWidth(1);
	//ofFill();
	//for (int i = 0; i < ee_path.size(); i++) {		
	//	ofSetColor(250);
	//	ofLine(ee_path[i][0], ee_path[i][2]);
	//	
	//	ofSetColor(ofColor::orangeRed, 200);
	//	ofDrawEllipse(ee_path[i][0], 15, 15);
	//	ofDrawEllipse(ee_path[i][2], 15, 15);
	//	ofDrawEllipse(ee_path[i][1], 20, 20);
	//}

	//ofPopStyle();
}

void CableRobot2D::draw()
{
	// draw the bounds
	ofPushStyle();
	ofColor color;
	if (bounds.inside(gizmo_ee.getTranslation()))
		color = ofColor::yellow;
	else
		color = ofColor::red;
	ofSetColor(color, 10);
	ofDrawRectangle(bounds.getPosition(), bounds.width, bounds.height);
	
	draw_cables_2D();

	ofPopStyle();

	//for (int i = 0; i < robots.size(); i++) {
	//	robots[i]->draw();
	//}
}

void CableRobot2D::draw_gui() {
	panel.draw();
	
	if (debugging) {
		int y = panel.getPosition().y + panel.getHeight();
		for (int i = 0; i < robots.size(); i++) {
			robots[i]->panel.setPosition(panel.getPosition().x + 25, y);
			robots[i]->panel.draw();
			y += robots[i]->panel.getHeight();
		}
	}
}

void CableRobot2D::get_status() {

	string state = "";
	for (int i = 1; i < robots.size(); i++) {
		state += robots[i]->status.get() + ", ";
	}

	// check if any of the motors are in E_STOP
	if (state.find("E_STOP") != std::string::npos) {
		// if so, put all in E_STOP
		move_to_vel.set(false);
		e_stop.set(true);		
	}
	// then check if any are not homed
	else if (state.find("NOT_HOMED") != std::string::npos) {
		status.set("NOT_HOMED");
		panel.setBorderColor(mode_color_not_homed);
	}
	// then check if any are homing
	else if (state.find("HOMING") != std::string::npos) {
		status.set("HOMING");
		panel.setBorderColor(mode_color_not_homed);

		// check if we're done homing
		bool val = true;
		for (int i = 1; i < robots.size(); i++) {
			if (!robots[i]->is_homed())
				val = false;
		}
		if (val) {
			enable.set(true);
			status.set("ENABLED");
		}
	}
	// then check if any are disabled
	else if (state.find("DISABLED") != std::string::npos) {
		status.set("DISABLED");
		// don't trigger on_enable(false) in case any are not diabled
		panel.setBorderColor(mode_color_disabled);
	}
	else {
		// then check if all are enabled
		bool val = true;
		for (int i = 1; i < robots.size(); i++) {
			if (!robots[i]->is_enabled())
				val = false;
		}
		if (val) {
			enable.set(true);
		}
	}
}

void CableRobot2D::update_gizmo()
{
	if (override_gizmo) {
		gizmo_ee.setNode(*ee);
	}
	else if (gizmo_ee.getTranslation() != ee->getGlobalPosition() ||
		gizmo_ee.getRotation() != ee->getGlobalOrientation()) {
		ee->setGlobalPosition(gizmo_ee.getTranslation());
		ee->setGlobalOrientation(gizmo_ee.getRotation());
		// update the gui
		if (gizmo_ee.isInteracting()) {
			auto pos = ee->getPosition();
			move_to.set(glm::vec2(pos.x, -1 * pos.y));
		}
	}
	// update bounds if we've moved the origin gizmo
	if (bounds.getPosition() != robots[0]->get_tangent().getGlobalPosition())
		bounds.setPosition(robots[0]->get_tangent().getGlobalPosition());
}

void CableRobot2D::key_pressed(int key)
{
	switch (key)
	{
	case '?':
		debugging = !debugging;
		break;
	default:
		break;
	}
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->key_pressed(key);
	}
}

void CableRobot2D::update_trajectories_2D()
{
	float scale_factor = 1.0;
	float dist_0 = robots[0]->actual_to_desired_distance;
	float dist_1 = robots[1]->actual_to_desired_distance;

	if (abs(dist_0) > abs(dist_1)) {
		if (dist_0 != 0) scale_factor = abs(dist_1 / dist_0);
		robots[1]->velocity_scalar = scale_factor;
	}
	else {
		if (dist_1 != 0) scale_factor = abs(dist_0 / dist_1);
		robots[0]->velocity_scalar = scale_factor;
	}

	// get the smoothed velocities
	float rpm_0 = robots[0]->compute_velocity();
	float rpm_1 = robots[1]->compute_velocity();

	// record the raw and filtered rpm for visualization
	if (debugging) {
		plot_data[0] = robots[0]->plot_data_vel[0];
		plot_data[1] = robots[0]->plot_data_vel[1];
		plot_data[2] = robots[1]->plot_data_vel[0];
		plot_data[3] = robots[1]->plot_data_vel[1];
		plot.update(plot_data);
	}

	robots[0]->move_velocity_rpm(rpm_0);
	robots[1]->move_velocity_rpm(rpm_1);
}

void CableRobot2D::draw_cables_actual(glm::vec3 start_0, glm::vec3 end_0, float dist_0, glm::vec3 start_1, glm::vec3 end_1, float dist_1)
{
	auto heading_0 = glm::normalize(end_0 - start_0) * dist_0;
	auto heading_1 = glm::normalize(end_1 - start_1) * dist_1;
	end_0 = start_0 + heading_0;
	end_1 = start_1 + heading_1;

	ofSetLineWidth(2);
	ofSetColor(255);
	ofDrawLine(start_0, end_0);
	ofFill();
	ofSetColor(ofColor::orange, 200);
	ofDrawEllipse(end_0, 40, 40);

	ofSetColor(255);
	ofDrawLine(start_1, end_1);
	ofFill();
	ofSetColor(ofColor::orange, 200);
	ofDrawEllipse(end_1, 40, 40);
}


void CableRobot2D::draw_cables_2D() {
	//ofPushStyle();
	ofSetColor(120);
	ofSetLineWidth(1);

	if (robots.size() > 0) {

		float actual_0 = robots[0]->get_position_actual();
		float actual_1 = robots[1]->get_position_actual();

		glm::vec3 start_0 = robots[0]->get_tangent_ptr()->getGlobalPosition();
		glm::vec3 start_1 = robots[1]->get_tangent_ptr()->getGlobalPosition();
		glm::vec3 end_0 = robots[0]->get_target()->getGlobalPosition();
		glm::vec3 end_1 = robots[1]->get_target()->getGlobalPosition();

		ofDrawLine(start_0, end_0);
		ofDrawLine(end_0, end_1);
		ofDrawLine(start_1, end_1);

		ofSetColor(255, 60);
		ofDrawEllipse(end_0, zone.get() * 2, zone.get() * 2);
		ofDrawEllipse(end_1, zone.get() * 2, zone.get() * 2);

		draw_cables_actual(start_0, end_0, actual_0, start_1, end_1, actual_1);
	}	

	//ofPopStyle();
}

void CableRobot2D::setup_gui()
{
	mode_color_estopped		= robots[0]->mode_color_estopped;
	mode_color_not_homed	= robots[0]->mode_color_not_homed;
	mode_color_enabled		= robots[0]->mode_color_enabled;
	mode_color_disabled		= robots[0]->mode_color_disabled;

	int gui_width = 250;

	panel.setup("2D_Robot_" + ofToString(id));
	panel.setWidthElements(gui_width);
	panel.setPosition(10, 15);
	panel.add(status.set("Status", state_names[0]));

	params_control.setName("Control");
	params_control.add(enable.set("Enable", false));
	params_control.add(e_stop.set("E_Stop", false));
	params_control.add(btn_run_homing.set("Run_Homing"));
	params_control.add(btn_run_shutdown.set("Run_Shutdown"));

	params_limits.setName("Limits");
	params_limits.add(vel_limit.set("Vel_Limit_(RPM)", 30, 0, 300));
	params_limits.add(accel_limit.set("Accel_Limit_(RPM/s)", 100, 0, 1000));
	params_limits.add(bounds_min.set("Bounds_Min", 100, 0, 3000));
	params_limits.add(bounds_max.set("Bounds_Max", 4000, 0, 5000));
	params_limits.add(torque_min.set("Torque_Min", -5, -5, 10));
	params_limits.add(torque_max.set("Torque_Max", 40, 0, 100));

	params_kinematics.setName("Kinematics");
	params_kinematics.add(base_offset.set("Base_Offset", 2500, 0, 3000));
	params_kinematics.add(ee_offset.set("EE_Offset", 70, 0, 750));
	params_kinematics.add(x_offset_max.set("X_Offset_Max", 0, 0, 1000));

	params_motion.setName("Motion");
	params_motion.add(zone.set("Approach_Zone", 50, 0, 300));
	params_motion.add(kp.set("Proportional_Gains", 1, 0, 500));	// gains for Propotional Component
	params_motion.add(kd.set("Derivative_Gains", 15, 0, 100));     // gains for Derivitive Component
	params_motion.add(steering_scalar.set("Steering_Scalar", 1.25, 0, 5));

	params_move.setName("Move");
	params_move.add(move_to.set("Move_To", glm::vec2(0, 0), glm::vec2(0, 0), glm::vec2(750, 2000)));
	params_move.add(move_to_pos.set("Move_Pos"));
	params_move.add(move_to_vel.set("Move_Vel", false));
	params_move.add(params_motion);


	// bind GUI listeners
	e_stop.addListener(this, &CableRobot2D::on_e_stop);
	enable.addListener(this, &CableRobot2D::on_enable);
	btn_run_homing.addListener(this, &CableRobot2D::on_run_homing);
	btn_run_shutdown.addListener(this, &CableRobot2D::on_run_shutdown);

	vel_limit.addListener(this, &CableRobot2D::on_vel_limit_changed);
	accel_limit.addListener(this, &CableRobot2D::on_accel_limit_changed);
	bounds_min.addListener(this, &CableRobot2D::on_bounds_changed);
	bounds_max.addListener(this, &CableRobot2D::on_bounds_changed);
	torque_min.addListener(this, &CableRobot2D::on_torque_limits_changed);
	torque_max.addListener(this, &CableRobot2D::on_torque_limits_changed);

	base_offset.addListener(this, &CableRobot2D::on_base_offset_changed);
	ee_offset.addListener(this, &CableRobot2D::on_ee_offset_changed);
	x_offset_max.addListener(this, &CableRobot2D::on_x_offset_max_changed);

	move_to.addListener(this, &CableRobot2D::on_move_to_changed);
	move_to_pos.addListener(this, &CableRobot2D::on_move_to_pos);
	move_to_vel.addListener(this, &CableRobot2D::on_move_to_vel);
	zone.addListener(this, &CableRobot2D::on_zone_changed);

	kp.addListener(this, &CableRobot2D::on_gains_changed);
	kd.addListener(this, &CableRobot2D::on_gains_changed);
	steering_scalar.addListener(this, &CableRobot2D::on_gains_changed);

	panel.add(params_control);
	panel.add(params_limits);
	panel.add(params_kinematics);	
	panel.add(params_move);
	
	robots[0]->panel.setWidthElements(gui_width - 25);
	robots[0]->panel.setParent(&panel);
	robots[0]->panel.minimizeAll();
	robots[1]->panel.setParent(&robots[0]->panel);
	robots[1]->panel.setWidthElements(gui_width - 25);
	robots[1]->panel.minimizeAll();
}

void CableRobot2D::update_gui(ofxPanel* _panel) {
	// update each robot's gui
	int x = _panel->getPosition().x;
	int y = _panel->getPosition().y;
	int w = _panel->getWidth();
	int h = _panel->getHeight();
	int padding = 5;

	panel.setPosition(x + w + padding, y);
	x = panel.getPosition().x;
	h = panel.getHeight();
	y += h;

	robots[0]->panel.setPosition(x, y + padding);
	y += robots[0]->panel.getHeight();
	robots[1]->panel.setPosition(x, y + padding);
}

void CableRobot2D::on_enable(bool& val)
{
	if (val) {
		status.set("ENABLED");
		panel.setBorderColor(mode_color_enabled);
	}
	else {
		status.set("DISABLED");
		panel.setBorderColor(mode_color_disabled);
	}
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_enable(val);
	}
}

void CableRobot2D::on_e_stop(bool& val)
{
	stop();
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_e_stop(val);
	}
}

void CableRobot2D::on_run_homing()
{
	status.set("HOMING");
	panel.setBorderColor(mode_color_not_homed);
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_run_homing();
	}
	get_status();
}


void CableRobot2D::on_run_shutdown()
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_run_shutdown();
	}
	get_status();
}

void CableRobot2D::on_bounds_changed(float& val)
{
	// update the height of the bounds rectangle
	float h = robots[0]->bounds_max - robots[0]->bounds_min;
	bounds.setHeight(-1 * h);
	auto pos = robots[0]->get_tangent().getGlobalPosition();
	pos.y -= robots[0]->bounds_min.get();
	bounds.setPosition(pos);
	// relay change to robots
	for (int i=0; i<robots.size(); i++){
		robots[i]->bounds_min.set(bounds_min);
		robots[i]->bounds_max.set(bounds_max);
	}
	// update the gui
	move_to.setMax(glm::vec2(bounds.getWidth(), -1 * bounds.getHeight()));
}

void CableRobot2D::on_vel_limit_changed(float& val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->vel_limit.set(val);
	}
}

void CableRobot2D::on_ee_offset_changed(float& val)
{
	float offset =  val;
	robots[0]->get_target()->setPosition(-offset, 0, 0);
	robots[1]->get_target()->setPosition(offset, 0, 0);
}

void CableRobot2D::on_x_offset_max_changed(float& val)
{
}

/**
 * @brief Change the precision zone to the robot's trajectory.
 * 
 * @param (float)  val: radius of precision zone
 */
void CableRobot2D::on_zone_changed(float& val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->set_zone(val);
	}
}

void CableRobot2D::on_gains_changed(float& val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->velocity_controller.kd.set(kd);
		robots[i]->velocity_controller.kp.set(kp);
		robots[i]->velocity_controller.steering_scalar.set(steering_scalar);
	}
}

void CableRobot2D::on_move_to_pos()
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_move_to_pos();
	}
}

void CableRobot2D::on_move_to_vel(bool& val)
{
	for (int i = 0; i < robots.size(); i++)
		robots[i]->on_move_to_vel(val);
}


void CableRobot2D::on_base_offset_changed(float& val)
{
	auto pos = robots[1]->get_base().getPosition();
	pos.x = val;
	robots[1]->set_base_position(pos);

	// update 2D bounds
	bounds.setWidth(val);
}

void CableRobot2D::on_accel_limit_changed(float& val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->accel_limit.set(val);
	}
}

void CableRobot2D::on_torque_limits_changed(float& val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->torque_min.set(torque_min.get());
		robots[i]->torque_max.set(torque_max.get());
	}
}

void CableRobot2D::on_move_to_changed(glm::vec2& val)
{
	ee->setPosition(glm::vec3(val.x, -1 * val.y, base_top_left.z));
	gizmo_ee.setNode(*ee);
}

bool CableRobot2D::is_estopped()
{
	bool val = false;
	for (int i=0; i<robots.size(); i++){
		if (!robots[i]->is_estopped())
			val = true;
	}
	// If one motor is estopped, make sure ALL are estopped 
	if (val) {
		for (int i=0; i<robots.size(); i++){
			if (!robots[i]->is_estopped())
				robots[i]->set_e_stop(true);
		}
	}
	return val;
}

bool CableRobot2D::is_homed()
{
	bool val = true;
	for (int i=0; i<robots.size(); i++){
		if (!robots[i]->is_homed())
			val = false;
	}
	return val;
}

bool CableRobot2D::is_enabled()
{
	bool val = true;
	for (int i=0; i<robots.size(); i++){
		if (!robots[i]->is_enabled())
			val = false;
	}
	return val;
}

void CableRobot2D::stop()
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->stop();
	}
}

void CableRobot2D::set_e_stop(bool val)
{
	if (val) {
		status.set("E_STOP");
		panel.setBorderColor(mode_color_estopped);
		move_to_vel.set(false);
	}
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->e_stop.set(val);
	}
}

void CableRobot2D::set_enabled(bool val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->enable = &val;
	}
	string state = val ? "ENABLED" : "DISABLED";
	status.set(state);
}
