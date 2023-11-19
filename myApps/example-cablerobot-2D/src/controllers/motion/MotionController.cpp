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

	for (int i = 0; i < 4; i++) {
		targets.push_back(new glm::vec3(0,0, (i * offset_z)));
	}

	setup_motion_line();
	setup_motion_circle();
	setup_paths();
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

	for (int i = 0; i < 4; i++) {
		targets.push_back(new glm::vec3(0, 0, (i * offset_z)));
	}

	setup_motion_line();
	setup_motion_circle();
	setup_paths();
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
	//if (play.get()) {
		//// wrap around once we've hit 100%

		//if (paths[0].path_type == PATH_TYPE::POLYGON) {
		//	evaluate_percent += speed.get();
		//	if (evaluate_percent >= 1.0) evaluate_percent = evaluate_percent - 1.0;
		//}
		//else if (paths[0].path_type == PATH_TYPE::LINE) {
		//	float time_diff = ofGetElapsedTimef() - timer_start;
		//	if (time_diff <= duration.get()) {
		//		evaluate_percent = ofMap(time_diff, 0, duration.get(), 0, 1, true);
		//	}
		//}

	if (motion_line_follow || motion_circle_follow) {

		if (motion_spin_enable) {
			motion_theta += motion_spin_speed;
			//calculate_theta(motion_line);
			if (motion_theta > 180) {
				motion_theta = -180;
				motion_line_rotation = motion_theta;
			}
		}

		for (int i = 0; i < targets.size(); i++) {
			float t = (i * 1.0) / (targets.size() - 1);
			if (motion_line_follow) {
				targets[i]->x = motion_line.getPointAtPercent(t).x;
				targets[i]->y = motion_line.getPointAtPercent(t).y;
			}
			else if (motion_circle_follow) {
				//targets[i] = &motion_circle.getPointAtPercent(t);
				targets[i]->x = motion_circle.getPointAtPercent(t).x;
				targets[i]->y = motion_circle.getPointAtPercent(t).y;
			}
		}
	}

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
		draw_paths();
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
	}
}

void MotionController::on_motion_circle_follow(bool& val)
{
	if (val) {
		motion_drawing_follow = false;
		motion_line_follow = false;
	}
}

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

	params_motion.add(params_motion_drawing);
	params_motion.add(params_motion_line);
	params_motion.add(params_motion_circle);

	panel.setup("Motion_Controller");
	panel.setWidthElements(250);
	panel.setPosition(550, 15);
	//panel.add(params);
	panel.add(params_motion);

}
