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
	for (auto& path : paths) {
		targets.push_back(&path.target);
	}

	setup_motion_line();
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

	// add a reference to each path's target
	for (auto& path : paths) {
		targets.push_back(&path.target);
	}
	setup_motion_line();
}

//--------------------------------------------------------------
MotionController::~MotionController()
{
	setup_motion_line();
}

//--------------------------------------------------------------
void MotionController::setup()
{
}

//--------------------------------------------------------------
void MotionController::update()
{
	if (play.get()) {
		// wrap around once we've hit 100%

		if (paths[0].path_type == PATH_TYPE::POLYGON) {
			evaluate_percent += speed.get();
			if (evaluate_percent >= 1.0) evaluate_percent = evaluate_percent - 1.0;
		}
		else if (paths[0].path_type == PATH_TYPE::LINE) {
			float time_diff = ofGetElapsedTimef() - timer_start;
			if (time_diff <= duration.get()) {
				evaluate_percent = ofMap(time_diff, 0, duration.get(), 0, 1, true);
			}
		}

		for (int i = 0; i < paths.size(); i++) {
			// follow the can-can line
			if (motion_line_follow) {
				float t = 0;
				if (paths.size() > 1)
					t = (i * 1.0) / (paths.size() - 1);
				paths[i].target = motion_line.getPointAtPercent(t);
			}
			else {
				// update the time-based targets
				paths[i].target = paths[i].path.getPointAtPercent(evaluate_percent);
			}
		}
	}
}

//--------------------------------------------------------------
void MotionController::draw()
{
	for (int i = 0; i < paths.size(); i++) {
		paths[i].draw();
	}

	if (motion_line_follow && motion_line.getVertices().size() > 1) {
		draw_motion_line();
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
void MotionController::rotate(ofPolyline* path, float theta)
{
	if (path->getVertices().size() > 0) {
		glm::vec3 centroid;
		for (int i = 0; i < path->getVertices().size(); i++) {
			centroid += path->getVertices()[i];
		}
		centroid /= path->getVertices().size();

		auto axis = glm::vec3(0, 0, 1);
		for (int i = 0; i < path->getVertices().size(); i++) {
			path->translate(centroid * -1);
			path->rotateDeg(theta / 2, axis); // <-- working, but why /2 ??
			path->translate(centroid * 1);
		}
	}

}

void MotionController::on_motion_line_pos_changed(glm::vec3& val)
{
	glm::vec3 centroid;
	for (int i = 0; i < motion_line.getVertices().size(); i++) {
		centroid += motion_line.getVertices()[i];
	}
	centroid /= motion_line.getVertices().size();

	auto diff = val - centroid;
	motion_line.translate(diff);
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

void MotionController::on_motion_line_theta_changed(float& val)
{
	float diff = val - motion_line_rotation;
	motion_line_rotation += diff;
	rotate(&motion_line, diff);
}

void MotionController::on_motion_line_reset()
{
	motion_line_pos.set(centroid.getGlobalPosition());
	motion_line_theta.set(0);
	motion_line_length.set(2000);
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
	ofDrawEllipse(motion_line.getVertices()[1], 30, 30);

	ofSetColor(ofColor::orangeRed);
	auto p = (motion_line.getVertices()[0] + motion_line.getVertices()[1]) / 2;
	ofDrawEllipse(p, 30, 30);

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

	params_motion_line.setName("Motion_Line");
	params_motion_line.add(motion_line_follow.set("Enable_Line", true));
	params_motion_line.add(motion_line_reset.set("Reset_Line"));
	params_motion_line.add(motion_line_pos.set("Position", centroid.getGlobalPosition(), glm::vec3(-2000, -5250, 0), glm::vec3(2000, 0, 0)));
	params_motion_line.add(motion_line_length.set("Length", 2000, 2, 3000));
	params_motion_line.add(motion_line_theta.set("Theta", 0, -180, 180));

	motion_line_theta.addListener(this, &MotionController::on_motion_line_theta_changed);
	motion_line_pos.addListener(this, &MotionController::on_motion_line_pos_changed);
	motion_line_length.addListener(this, &MotionController::on_motion_line_length_changed);
	motion_line_reset.addListener(this, &MotionController::on_motion_line_reset);

	panel.setup("Motion_Controller");
	panel.setWidthElements(250);
	panel.setPosition(550, 15);
	panel.add(params);
	panel.add(params_motion_line);
}
