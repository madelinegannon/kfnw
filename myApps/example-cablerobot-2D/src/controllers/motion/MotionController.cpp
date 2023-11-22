#include "MotionController.h"


//--------------------------------------------------------------
MotionController::MotionController()
{
	centroid.setGlobalPosition(500 + 1250, -2500, 0);
	setup_gui();

	glm::vec3 pos = centroid.getGlobalPosition();
	for (int i = 0; i < bases.size() / 2; i++) {
		ofNode n;
		n.setGlobalPosition(pos.x, pos.y, pos.z + (i * offset_z));
		//paths.push_back(create_line(n, 750));
		paths.push_back(create_polygon(n, radius.get(), resolution.get(), offset_theta.get()));
	}

	// add a reference to each path's target
	//for (auto& path : paths) {
	//	targets.push_back(&path.target);
	//}


	// setup default targets
	for (int i = 0; i < 4; i++) {
		targets.push_back(new glm::vec3(0, -2750, (i * offset_z)));
	}

	setup_motion_line();
	setup_motion_circle();
	setup_paths();
	//setup_agents();
	setup_eyes();
}

MotionController::MotionController(vector<glm::vec3> bases, ofNode* _origin, float offset_z)
{
	this->bases = bases;
	this->origin = _origin;
	this->offset_z = offset_z;

	float width = glm::distance(bases[0], bases[1]);
	auto pos = _origin->getGlobalPosition();
	pos.x += width / 2;
	pos.y -= 2500;
	centroid.setGlobalPosition(pos);

	setup_gui();

	for (int i = 0; i < bases.size() / 2; i++) {
		ofNode n;
		n.setGlobalPosition(pos.x, pos.y, pos.z + (i * offset_z));
		//paths.push_back(create_line(n, 750));
		paths.push_back(create_polygon(n, radius.get(), resolution.get(), offset_theta.get()));
	}

	//// add a reference to each path's target
	//for (auto& path : paths) {
	//	targets.push_back(&path.target);
	//}

	// setup default targets
	for (int i = 0; i < 4; i++) {
		targets.push_back(new glm::vec3(0, -2750, (i * offset_z)));
	}

	setup_motion_line();
	setup_motion_circle();
	setup_paths();
	//setup_agents();
	setup_eyes();
}

//--------------------------------------------------------------
MotionController::~MotionController()
{
}

//--------------------------------------------------------------
void MotionController::setup()
{
}

//--------------------------------------------------------------
void MotionController::update()
{

	if (motion_line_follow || motion_circle_follow) {

		if (motion_spin_enable) {
			motion_theta -= motion_spin_speed;
			//calculate_theta(motion_line);
			if (motion_theta < -180) {
				motion_theta = 180;
				motion_line_rotation = motion_theta;
			}
		}

		for (int i = 0; i < targets.size(); i++) {
			float t = (i * 1.0) / (targets.size() - 1);
			if (motion_line_follow) {

				targets[i]->x = motion_line.getPointAtPercent(t).x;
				targets[i]->y = motion_line.getPointAtPercent(t).y;

				if (enable_sine_wave) {
					sine_wave_counter += sine_wave_speed;
					float val = ofMap(sin(sine_wave_counter), -1, 1, -1 * sine_wave_amplitude / 2, sine_wave_amplitude / 2);
					targets[i]->y += val;
				}
			}
			else if (motion_circle_follow) {

				if (enable_pendulum) {
					pendulum_counter += pendulum_speed;
					float val = ofMap(sin(pendulum_counter + (i * pendulum_offset)), -1.1, 1.1, -160, -20);
					motion_theta.set(val);
				}

				targets[i]->x = motion_circle.getPointAtPercent(t).x;
				targets[i]->y = motion_circle.getPointAtPercent(t).y;
			}
		}
	}

	if (enable_eyes) {
		update_eyes();
	}

	//else if (motion_agents_follow) {
	//	// update the agent 
	//	float z_offset = -140;
	//	for (int i = 0; i < targets.size(); i++) {
	//		agents->set_target(i, &glm::vec3(motion_pos.get().x, motion_pos.get().y, i * z_offset));
	//	}
	//	agents->update();
	//	// add the agent's pos to the individual drawing path
	//	auto positions = agents->get_positions();
	//	for (int i = 0; i < paths_drawing.size(); i++) {
	//		add_to_path(i, glm::vec3(positions[i].x, positions[i].y, i * z_offset));
	//	}
	//}


	//for (int i = 0; i < paths.size(); i++) {
	//	// follow the can-can line
	//	if (motion_line_follow || motion_circle_follow) {

	//		if (motion_spin_enable) {
	//			motion_theta += motion_spin_speed;
	//			//calculate_theta(motion_line);
	//			if (motion_theta > 180) {
	//				motion_theta = -180;
	//				motion_line_rotation = motion_theta;
	//			}
	//		}

	//		float t = 0;
	//		if (paths.size() > 1)
	//			t = (i * 1.0) / (paths.size() - 1);
	//		if (motion_line_follow) {
	//			paths[i].target = motion_line.getPointAtPercent(t);
	//			targets[i] = &motion_line.getPointAtPercent(t);
	//		}
	//		else if (motion_circle_follow) {
	//			paths[i].target = motion_circle.getPointAtPercent(t);
	//			targets[i] = &motion_circle.getPointAtPercent(t);
	//		}
	//	}
	//	else {
	//		//// update the time-based targets
	//		//paths[i].target = paths[i].path.getPointAtPercent(evaluate_percent);
	//	}
	//}
	////}

	////if (motion_drawing_follow) {
	////	update_paths();
	////}

}

//--------------------------------------------------------------
void MotionController::draw()
{
	//for (int i = 0; i < paths.size(); i++) {
	//	paths[i].draw();
	//}

	if (motion_line_follow && motion_line.getVertices().size() > 1) {
		draw_motion_line();
	}
	else if (motion_circle_follow) {
		draw_motion_circle();
	}

	if (motion_drawing_follow) {
		//if (motion_agents_follow) {
		//	agents->draw();
		//}
		draw_paths();
	}
	//else if (motion_agents_follow) {
	//	agents->draw();
	//	draw_paths();
	//}
	if (enable_eyes) {
		draw_eyes();
	}
}

void MotionController::draw_gui()
{
	if (showGUI) {
		panel.draw();
	}
}

void MotionController::keyPressed(int key)
{
	switch (key) {
	case '?':
		debugging = !debugging;
		break;
	case 'h':
	case 'H':
		showGUI = !showGUI;
		break;
	default:
		break;
	}

}

vector<glm::vec3*> MotionController::get_targets()
{
	return targets;
}

void MotionController::rotate(float theta)
{
	float diff = theta - rotation_theta;
	rotation_theta += diff;
	for (int i = 0; i < paths.size(); i++) {
		auto offset = paths[i].centroid.getGlobalPosition();
		auto axis = glm::vec3(0, 0, 1);
		paths[i].path.translate(offset * -1);
		paths[i].path.rotateDeg(diff, axis);
		paths[i].path.translate(offset * 1);
	}
}

void MotionController::scale(float scalar)
{
	scalar = ofMap(scalar, 0, 100, 0, 1);
	float diff = scalar / (scalar_percent + 0.0001);
	scalar_percent = scalar;
	for (int i = 0; i < paths.size(); i++) {
		auto offset = paths[i].centroid.getGlobalPosition();
		paths[i].path.translate(offset * -1);
		paths[i].path.scale(diff, diff);
		paths[i].path.translate(offset * 1);
	}
}

void MotionController::setup_paths(int count)
{
	for (int i = 0; i < count; i++) {
		paths_drawing.push_back(new ofPolyline());
	}
}

void MotionController::update_paths()
{
	for (int i = 0; i < paths_drawing.size(); i++) {
		auto path = paths_drawing[i];
		// cap the length of the path
		if (path->getVertices().size() > motion_drawing_length_max.get()) {
			path->removeVertex(0);
		}
	}
}

void MotionController::draw_paths()
{
	ofColor color;
	for (int i = 0; i < paths_drawing.size(); i++) {

		// Set the color
		switch (i) {
		case 0:
			color = ofColor::floralWhite;
			break;
		case 1:
			color = ofColor::pink;
			break;
		case 2:
			color = ofColor::deepPink;
			break;
		case 3:
			color = ofColor::hotPink;
			break;
		default:
			color = ofColor::magenta;
		}

		ofPushStyle();

		// draw the path
		ofSetLineWidth(5);
		ofNoFill();
		ofSetColor(color, 180);
		paths_drawing[i]->draw();

		// draw the target
		ofFill();
		ofSetColor(ofColor::orange);
		if (paths_drawing[i]->getVertices().size() > 0)
			ofDrawEllipse(paths_drawing[i]->getVertices()[0], 30, 30);

		ofPopStyle();
	}
}

void MotionController::clear_paths()
{
	for (int i = 0; i < paths_drawing.size(); i++) {
		paths_drawing[i]->clear();
	}
}

//void MotionController::setup_agents()
//{
//	int num_cable_bots = 4;
//	agents = new AgentController();
//	agents->setup(num_cable_bots);
//	vector<ofNode*> tgts;
//
//	for (int i = 0; i < num_cable_bots; i++) {
//		ofNode node;
//		node.setGlobalPosition(*get_targets()[i]);
//		tgts.push_back(&node);
//	}
//	agents->set_targets(tgts);
//
//	panel.add(agents->params);
//}

void MotionController::add_to_path(int i, glm::vec3 pt)
{
	if (i < paths_drawing.size()) {
		auto path = paths_drawing[i];

		// add the first point
		if (path->getVertices().size() == 0) {
			path->addVertex(pt);
		}
		else {
			// filter out small moves
			float dist_thresh = 45;
			float dist_sq = glm::distance2(path->getVertices().back(), pt);
			if (dist_sq > dist_thresh * dist_thresh) {
				path->addVertex(pt);
			}
		}
	}
}

/**
 * @brief Checks if the actual 2D positions are close to the target positions.
 * If close, removes and then updates the new target positions.
 *
 * @param ()  actual: Estimated position of the robots
 */
void MotionController::update_targets(vector<glm::vec3> actual)
{
	for (int i = 0; i < paths_drawing.size(); i++) {
		auto path = paths_drawing[i];
		if (path->getVertices().size() > 2) {	// <--- POLYLINE MUST HAVE AT LEAST 3 POINTS, OTHERWISE SENDS TO (0,0,0)
			float dist_thresh = motion_drawing_accuracy.get();
			glm::vec2 pt_0 = glm::vec2(path->getVertices()[0].x, path->getVertices()[0].y);
			glm::vec2 pt_1 = glm::vec2(actual[i].x, actual[i].y);
			float dist_sq = glm::distance2(pt_0, pt_1);
			if (dist_sq < dist_thresh * dist_thresh) {
				path->removeVertex(0);
			}
			targets[i] = &path->getVertices()[0];
		}
	}
}

MotionController::MotionPath MotionController::create_polygon(ofNode centroid, float radius, float resolution, float offset_theta)
{
	MotionPath mp;
	mp.path_type = PATH_TYPE::POLYGON;
	mp.centroid = centroid;
	mp.radius = radius;
	mp.resolution = resolution;
	mp.offset_theta = offset_theta;
	mp.reset();
	return mp;
}

MotionController::MotionPath MotionController::create_line(ofNode centroid, float offset)
{
	MotionPath mp;
	mp.path_type = PATH_TYPE::LINE;
	mp.centroid = centroid;
	mp.offset = offset;
	mp.reset();
	return mp;
}

/**
 * @brief Rotates a Polyline about its centroid in the YZ Plane.
 *
 * @param ()  path: polyline to rotate
 * @param ()  theta: degrees to rotate
 */
void MotionController::rotateCircle(ofPolyline* path, float theta)
{
	if (path->getVertices().size() > 0) {
		glm::vec3 centroid = motion_pos.get();
		auto axis = glm::vec3(0, 0, 1);
		path->translate(centroid * -1);
		path->rotateDeg(theta, axis);
		path->translate(centroid * 1);
	}
}

/**
 * @brief Rotates a Polyline about its centroid in the YZ Plane.
 *
 * @param ()  path: polyline to rotate
 * @param ()  theta: degrees to rotate
 */
void MotionController::rotateLine(ofPolyline* path, float theta)
{
	if (path->getVertices().size() > 0) {
		glm::vec3 centroid = (motion_line.getVertices()[0] + motion_line.getVertices()[1]) / 2;
		auto axis = glm::vec3(0, 0, 1);
		path->translate(centroid * -1);
		path->rotateDeg(theta, axis);
		path->translate(centroid * 1);
	}
}

void MotionController::on_motion_pos_changed(glm::vec3& val)
{
	ofPolyline* path;
	if (motion_circle_follow) {
		path = &motion_circle;
	}
	else {
		path = &motion_line;
	}

	//glm::vec3 centroid = path->getCentroid2D();
	//for (int i = 0; i < path->getVertices().size(); i++) {
	//	centroid += path->getVertices()[i];
	//}
	//centroid /= path->getVertices().size();

	auto diff = val - motion_pos_prev;
	motion_pos_prev.x = val.x;
	motion_pos_prev.y = val.y;
	path->translate(diff);

	// update the eyeball
	center.setGlobalPosition(motion_line.getVertices()[0]);

	//glm::vec3 centroid;
	//for (int i = 0; i < motion_line.getVertices().size(); i++) {
	//	centroid += motion_line.getVertices()[i];
	//}
	//centroid /= motion_line.getVertices().size();

	//auto diff = val - centroid;
	//motion_line.translate(diff);
}

void MotionController::on_motion_line_length_changed(float& val)
{
	glm::vec3 centroid;
	for (int i = 0; i < motion_line.getVertices().size(); i++) {
		centroid += motion_line.getVertices()[i];
	}
	centroid /= motion_line.getVertices().size();

	auto heading = motion_line.getVertices()[0] - centroid;
	motion_line.getVertices()[0] = centroid + glm::normalize(heading) * (val / 2);
	motion_line.getVertices()[1] = centroid - glm::normalize(heading) * (val / 2);
}

void MotionController::on_enable_pendulum(bool& val)
{
	if (val) {
		enable_sine_wave = false;
		motion_circle_follow = true;
		pendulum_counter = 0;
		motion_circle_angle_start = 0;
		motion_circle_angle_end = 0;
	}
}

void MotionController::on_eye_spacing(float& val)
{
	left_eye.setPosition(-1 * val / 2, 0, 0);
	right_eye.setPosition(1 * val / 2, 0, 0);
}

void MotionController::setup_eyes()
{
	int spacing = 250;

	center.setGlobalPosition(motion_pos);

	left_eye.setParent(center);
	left_eye.setPosition(-1 * spacing, 0, 0);

	left_pupil.setParent(left_eye);

	right_eye.setParent(center);
	right_eye.setPosition(1 * spacing, 0, 0);

	right_pupil.setParent(right_eye);

	rigging_eyes.push_back(&center);
	rigging_eyes.push_back(&left_eye);
	rigging_eyes.push_back(&left_pupil);
	rigging_eyes.push_back(&right_eye);
	rigging_eyes.push_back(&right_pupil);
}

void MotionController::update_eyes()
{
	int z_offset = -140;
	// get the eyes to look at the line end_pt
	auto left = glm::normalize(motion_line.getVertices()[1] - left_eye.getGlobalPosition());
	left *= (eye_radius - pupil_radius);
	left_pupil.setPosition(left);

	auto right = glm::normalize(motion_line.getVertices()[1] - right_eye.getGlobalPosition());
	right *= (eye_radius - pupil_radius);
	right_pupil.setPosition(right);

	targets[0] = new glm::vec3(left_eye.getGlobalPosition().x, left_eye.getGlobalPosition().y, 0 * z_offset);
	targets[1] = new glm::vec3(left_pupil.getGlobalPosition().x, left_pupil.getGlobalPosition().y, 1 * z_offset);
	targets[2] = new glm::vec3(right_eye.getGlobalPosition().x, right_eye.getGlobalPosition().y, 2 * z_offset);
	targets[3] = new glm::vec3(right_pupil.getGlobalPosition().x, right_pupil.getGlobalPosition().y, 3 * z_offset);
}

void MotionController::draw_eyes() {
	ofPushStyle();
	rigging_eyes[0]->draw();	// draw the center points

	float diameter_eye = 250 * 2.0;
	float diameter_pupil = 125 * 2.0;

	ofFill();
	ofSetColor(ofColor::white);
	ofEllipse(rigging_eyes[1]->getGlobalPosition(), diameter_eye, diameter_eye);	// right eye
	ofEllipse(rigging_eyes[3]->getGlobalPosition(), diameter_eye, diameter_eye);	// left eye
	ofSetColor(ofColor::black);
	ofEllipse(rigging_eyes[2]->getGlobalPosition(), diameter_pupil, diameter_pupil);	// right pupil
	ofEllipse(rigging_eyes[4]->getGlobalPosition(), diameter_pupil, diameter_pupil);	// left pupil

	ofPopStyle();
}

void MotionController::on_motion_drawing_follow(bool& val)
{
	if (val) {
		motion_line_follow = false;
		motion_circle_follow = false;
	}
}

void MotionController::on_motion_line_follow(bool& val)
{
	if (val) {
		motion_drawing_follow = false;
		motion_circle_follow = false;
		//motion_agents_follow = false;
	}
}

void MotionController::on_motion_circle_follow(bool& val)
{
	if (val) {
		motion_drawing_follow = false;
		motion_line_follow = false;
		//motion_agents_follow = false;
	}
}

//void MotionController::on_motion_agents_follow(bool& val)
//{
//	if (val) {
//		//motion_drawing_follow = false;
//		//motion_line_follow = false;
//
//		// clear the drawing path lists
//		clear_paths();
//
//		// snap agent location to current positions
//		agents->set_targets(get_targets());
//		agents->update();
//	}
//	else {
//		// hide/minimize the gui
//	}
//}

void MotionController::on_motion_theta_changed(float& val)
{
	float diff = val - motion_line_rotation;
	motion_line_rotation += diff;
	//rotate(&motion_line, diff);

	ofPolyline* path;
	if (motion_circle_follow) {
		path = &motion_circle;
		rotateCircle(path, diff);
	}
	else {
		path = &motion_line;
		rotateLine(path, diff);
	}
	/*rotate(path, diff);*/
}

void MotionController::on_motion_reset()
{
	motion_pos.set(glm::vec3(0, -2750, 0));
	centroid.setGlobalPosition(motion_pos);
	motion_line_length.set(2000);

	// rebuild the line
	motion_theta.set(0);
	glm::vec3 start = centroid.getGlobalPosition();
	glm::vec3 end = centroid.getGlobalPosition();
	start.x -= motion_line_length.get() / 2;
	end.x += motion_line_length.get() / 2;
	motion_line.getVertices()[0] = start;
	motion_line.getVertices()[1] = end;

	motion_circle_radius.set(1000);
	motion_spin_enable.set(false);
	motion_spin_speed.set(0);
	// rebuild the circle
	float resolution = 3.0;
	float theta = 180.0 / resolution;
	for (int i = 0; i <= resolution; i++) {
		glm::vec3 pt = glm::rotateZ(glm::vec3(radius, 0, 0), ofDegToRad(theta * i));
		pt += motion_pos.get();
		motion_circle.getVertices()[i] = pt;
	}
}

/**
 * Update the motion_line_rotation based on.
 */
void MotionController::calculate_theta(ofPolyline path)
{
	glm::vec3 centroid;
	for (int i = 0; i < motion_line.getVertices().size(); i++) {
		centroid += motion_line.getVertices()[i];
	}
	centroid /= motion_line.getVertices().size();

	auto heading = motion_line.getVertices()[0] - centroid;
	heading = glm::normalize(heading);
	glm::vec3 x_axis = glm::vec3(1, 0, 0);

	float theta = glm::degrees(glm::acos(glm::dot(heading, x_axis)));
	motion_line_rotation = theta;
}

void MotionController::setup_motion_line()
{
	glm::vec3 start = centroid.getGlobalPosition();
	glm::vec3 end = centroid.getGlobalPosition();
	start.x -= motion_line_length.get() / 2;
	end.x += motion_line_length.get() / 2;

	motion_line.addVertex(start);
	motion_line.addVertex(end);
}

void MotionController::draw_motion_line()
{
	ofPushStyle();

	ofNoFill();
	ofSetLineWidth(5);
	ofSetColor(255, 100);
	motion_line.draw();

	ofFill();
	ofSetColor(ofColor::red);
	ofDrawEllipse(motion_line.getVertices()[0], 30, 30);
	ofSetColor(ofColor::cornflowerBlue);
	ofDrawEllipse(motion_line.getVertices()[1], 30, 30);

	ofSetColor(ofColor::orangeRed);
	auto p = (motion_line.getVertices()[0] + motion_line.getVertices()[1]) / 2;
	ofDrawEllipse(p, 30, 30);

	ofPopStyle();
}

void MotionController::on_motion_circle_radius(float& val)
{
	update_motion_circle(motion_circle_angle_start.get(), motion_circle_angle_end.get());
}

void MotionController::on_motion_circle_angle_start(float& val)
{
	update_motion_circle(motion_circle_angle_start.get(), motion_circle_angle_end.get());
}

void MotionController::update_motion_line()
{
}

void MotionController::on_motion_circle_angle_end(float& val)
{
	update_motion_circle(motion_circle_angle_start.get(), motion_circle_angle_end.get());
}


void MotionController::setup_motion_circle()
{
	float resolution = 3.0;
	float theta = 180.0 / resolution;
	for (int i = 0; i <= resolution; i++) {
		glm::vec3 pt = glm::rotateZ(glm::vec3(radius, 0, 0), ofDegToRad(theta * i));
		pt += motion_pos.get();
		motion_circle.addVertex(pt);
	}
}

void MotionController::update_motion_circle(float start_angle, float end_angle, float resolution)
{
	float theta = (end_angle - start_angle) / resolution;
	for (int i = 0; i <= resolution; i++) {
		glm::vec3 pt = glm::rotateZ(glm::vec3(motion_circle_radius.get(), 0, 0), ofDegToRad(theta * i + start_angle));
		pt += motion_pos.get();
		motion_circle.getVertices()[i].x = pt.x;
		motion_circle.getVertices()[i].y = pt.y;
	}
	rotateCircle(&motion_circle, motion_line_rotation);
}

void MotionController::draw_motion_circle()
{
	int last = motion_circle.getVertices().size() - 1;

	ofPushStyle();

	ofNoFill();
	ofSetLineWidth(5);
	ofSetColor(255, 100);
	motion_circle.draw();
	ofDrawLine(motion_circle.getVertices()[0], motion_pos.get());
	ofDrawLine(motion_pos.get(), motion_circle.getVertices()[last]);

	ofFill();
	ofSetColor(ofColor::red);
	ofDrawEllipse(motion_circle.getVertices()[0], 30, 30);
	ofSetColor(ofColor::cornflowerBlue);
	ofDrawEllipse(motion_circle.getVertices()[last], 30, 30);

	ofSetColor(ofColor::orangeRed);
	ofDrawEllipse(motion_pos.get(), 30, 30);

	ofPopStyle();
}

void MotionController::on_play(bool& val)
{
	timer_start = val ? ofGetElapsedTimef() : 0;
}

void MotionController::on_parameter_changed(float& val)
{
	auto pos = centroid.getGlobalPosition();
	for (int i = 0; i < paths.size(); i++) {
		paths[i].centroid.setGlobalPosition(pos.x, pos.y, pos.z + (i * offset_z));
		paths[i].radius = radius.get();
		paths[i].resolution = resolution.get();
		paths[i].offset_theta = i * offset_theta.get();
		paths[i].reset();
	}
}


void MotionController::on_pos_changed(glm::vec3& val)
{
	centroid.setGlobalPosition(val);
	float v = 0;
	on_parameter_changed(v);
}

void MotionController::on_reset()
{
	evaluate_percent = 0.0;
}

void MotionController::setup_gui()
{
	params.setName("Path_Params");
	params.add(pos.set("Position", centroid.getGlobalPosition(), glm::vec3(-2000, -5000, -2000), glm::vec3(2000, 0, 2000)));
	params.add(radius.set("Radius", 1000, 0, 2500));
	params.add(resolution.set("Resolution", 30, 3, 60));
	params.add(offset_theta.set("Offset_Theta", 0, -180, 180));
	params.add(speed.set("Speed", .0003, 0, .1));
	params.add(duration.set("Duration (Sec)", 15, 0.1, 30));
	params.add(play.set("Play", false));
	params.add(reset.set("Reset"));

	play.addListener(this, &MotionController::on_play);
	pos.addListener(this, &MotionController::on_pos_changed);
	reset.addListener(this, &MotionController::on_reset);
	radius.addListener(this, &MotionController::on_parameter_changed);
	offset_theta.addListener(this, &MotionController::on_parameter_changed);
	resolution.addListener(this, &MotionController::on_parameter_changed);

	params_motion.setName("Motion");
	params_motion.add(motion_drawing_follow.set("Enable_Drawing", false));
	params_motion.add(motion_line_follow.set("Enable_Line", true));
	params_motion.add(motion_circle_follow.set("Enable_Circle", false));
	params_motion.add(enable_sine_wave.set("Enable_Sine_Wave", false));
	params_motion.add(enable_pendulum.set("Enable_Pendulum", false));
	params_motion.add(enable_eyes.set("Enable_Eyes", false));
	params_motion.add(motion_pos.set("Position", centroid.getGlobalPosition(), glm::vec3(-2000, -5250, 0), glm::vec3(2000, 0, 0)));
	params_motion.add(motion_theta.set("Theta", 0, -180, 180));
	params_motion.add(motion_spin_enable.set("Enable_Spin", false));
	params_motion.add(motion_spin_speed.set("Spin_speed", 0, .01, .5));
	params_motion.add(motion_reset.set("Reset"));
	motion_pos_prev.x = motion_pos.get().x;
	motion_pos_prev.y = motion_pos.get().y;

	params_motion_drawing.setName("Drawing");
	params_motion_drawing.add(motion_drawing_offset.set("Follow_Offset", 50, 0, 1000));
	params_motion_drawing.add(motion_drawing_accuracy.set("Accuracy", 50, 10, 100));
	params_motion_drawing.add(motion_drawing_length_max.set("Length_Max", 250, 0, 500));

	params_motion_line.setName("Line");
	params_motion_line.add(motion_line_length.set("Length", 2000, 2, 3000));

	params_motion_circle.setName("Circle");
	params_motion_circle.add(motion_circle_radius.set("Radius", 1000, 1, 2000));
	params_motion_circle.add(motion_circle_angle_start.set("Start_Angle", 0, 0.0, 360));
	params_motion_circle.add(motion_circle_angle_end.set("End_Angle", 180, 0.0, 360));

	params_motion_sine.setName("Sine_Wave");
	params_motion_sine.add(sine_wave_speed.set("Speed", 0.005, 0.001, .02));
	params_motion_sine.add(sine_wave_amplitude.set("Amplitude", 50, 0, 1000));
	params_motion_sine.add(sine_wave_offset.set("Offset", 0, 0, 1));

	params_motion_pendulum.setName("Pendulum");
	params_motion_pendulum.add(pendulum_speed.set("Speed", 0.001, 0.0001, .01));
	params_motion_pendulum.add(pendulum_offset.set("Offset", 0, 0, 1));

	params_eyes.setName("Eyes");
	params_eyes.add(eye_spacing.set("Eye_Spacing", 250, 0, 1000));
	params_eyes.add(eye_radius.set("Eye_Radius", 250, 0, 300));
	//params_eyes.add(pupil_radius.set("Pupil_Radius", 125, 0, 500));

	motion_theta.addListener(this, &MotionController::on_motion_theta_changed);
	motion_pos.addListener(this, &MotionController::on_motion_pos_changed);
	motion_line_length.addListener(this, &MotionController::on_motion_line_length_changed);
	motion_reset.addListener(this, &MotionController::on_motion_reset);

	motion_drawing_follow.addListener(this, &MotionController::on_motion_drawing_follow);
	motion_line_follow.addListener(this, &MotionController::on_motion_line_follow);
	motion_circle_follow.addListener(this, &MotionController::on_motion_circle_follow);

	motion_circle_angle_start.addListener(this, &MotionController::on_motion_circle_angle_start);
	motion_circle_angle_end.addListener(this, &MotionController::on_motion_circle_angle_end);
	motion_circle_radius.addListener(this, &MotionController::on_motion_circle_radius);

	enable_pendulum.addListener(this, &MotionController::on_enable_pendulum);

	eye_spacing.addListener(this, &MotionController::on_eye_spacing);

	//motion_agents_follow.addListener(this, &MotionController::on_motion_agents_follow);

	params_motion.add(params_motion_line);
	params_motion.add(params_motion_circle);
	params_motion.add(params_motion_drawing);
	params_motion.add(params_motion_sine);
	params_motion.add(params_motion_pendulum);
	params_motion.add(params_eyes);

	panel.setup("Motion_Controller");
	panel.setWidthElements(250);
	panel.setPosition(550, 15);
	//panel.add(params);
	panel.add(params_motion);

}
