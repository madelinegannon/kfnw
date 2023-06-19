#include "CableRobot2D.h"

CableRobot2D::CableRobot2D(CableRobot* top_left, CableRobot* top_right, ofNode* _origin, int id)
{
	robots.push_back(top_left);
	robots.push_back(top_right);
	this->origin = _origin;
	this->id = id;

	setup_gui();

	// setup the 2D bounds
	float h = robots[0]->bounds_max - robots[0]->bounds_min;
	bounds.setHeight(-1 * h);
	float w = robots[1]->get_tangent().getGlobalPosition().x - robots[0]->get_tangent().getGlobalPosition().x;
	bounds.setWidth(w);
	auto pos = robots[0]->get_tangent().getGlobalPosition();
	pos.y -= robots[0]->bounds_min.get();
	bounds.setPosition(pos);

	// setup the end effector
	this->ee.setParent(*origin);
	this->ee.setGlobalPosition(bounds.getCenter());

	// setup the end effector control gizmo
	gizmo_ee.setNode(ee);
	gizmo_ee.setDisplayScale(.5);

	//gizmo_ee.setTranslationAxisMask(IGizmo::AXIS_Y);
	gizmo_ee.setRotationAxisMask(IGizmo::AXIS_Z);
	gizmo_ee.setScaleAxisMask(IGizmo::AXIS_X);
}

void CableRobot2D::update()
{
	update_gizmo();
	// update bounds
	bounds.setPosition(robots[0]->get_tangent().getGlobalPosition());

	for (int i = 0; i < robots.size(); i++) {
		robots[i]->update();
	}
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
	ofPopStyle();

	for (int i = 0; i < robots.size(); i++) {
		robots[i]->draw();
	}
}

void CableRobot2D::draw_gui() {
	panel.draw();
	
	if (debugging) {
		int y = panel.getPosition().y + panel.getHeight();
		for (int i = 0; i < robots.size(); i++) {
			robots[i]->panel.setPosition(panel.getPosition().x, y);
			robots[i]->panel.draw();
			y += robots[i]->panel.getHeight();
		}
	}
}

void CableRobot2D::update_gizmo()
{
	if (override_gizmo) {
		gizmo_ee.setNode(ee);
	}
	else {
		ee.setGlobalPosition(gizmo_ee.getTranslation());
		ee.setGlobalOrientation(gizmo_ee.getRotation());
	}
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
	params_limits.add(accel_limit.set("Accel_Limit_(RPM/s)", 200, 0, 1000));
	params_limits.add(bounds_min.set("Bounds_Min", 100, 0, 2000));
	params_limits.add(bounds_max.set("Bounds_Max", 2000, 0, 2000));

	params_kinematics.setName("Kinematics");
	params_kinematics.add(base_offset.set("Base_Offset", 1500, 0, 3000));
	params_kinematics.add(ee_offset.set("EE_Offset", 0, 0, 750));

	params_motion.setName("Motion");
	params_motion.add(accel_rate.set("Accel_Rate_(RPM/s)", 1, 0.01, 2.0));
	params_motion.add(decel_radius.set("Decel_Radius", 100, 0, 500));

	panel.add(params_control);
	panel.add(params_limits);
	panel.add(params_kinematics);
	panel.add(params_motion);
	
	robots[0]->panel.setParent(&panel);
	robots[0]->panel.minimizeAll();
	robots[1]->panel.setParent(&robots[0]->panel);
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
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_enable(val);
	}
}

void CableRobot2D::on_e_stop(bool& val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_e_stop(val);
	}
}

void CableRobot2D::on_run_homing()
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_run_homing();		// blocks until complete <-- move into update() for threaded?
	}
}


void CableRobot2D::on_run_shutdown()
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_run_homing();
	}
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
}

void CableRobot2D::on_vel_limit_changed(float& val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->on_vel_limit_changed(val);
	}
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
		robots[i]->on_accel_limit_changed(val);
	}
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
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->set_e_stop(val);
	}
}

void CableRobot2D::set_enabled(bool val)
{
	for (int i = 0; i < robots.size(); i++) {
		robots[i]->set_enabled(val);
	}
}
