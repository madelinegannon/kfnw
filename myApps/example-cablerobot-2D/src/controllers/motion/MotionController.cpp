#include "MotionController.h"


//--------------------------------------------------------------
MotionController::MotionController()
{
    centroid.setGlobalPosition(500 + 1250, -1100, 0);
    setup_gui();
   
    glm::vec3 pos = centroid.getGlobalPosition();
    for (int i = 0; i < 2; i++) {
        ofNode n;
        n.setGlobalPosition(pos.x, pos.y, pos.z + (i * offset_z));
        paths.push_back(create_line(n, 750));
        //paths.push_back(create_polygon(n, radius.get(), resolution.get(), offset_theta.get()));
    }

    // add a reference to each path's target
    for (auto& path : paths) {
        targets.push_back(&path.target);
    }
}

MotionController::MotionController(vector<glm::vec3> bases, ofNode* _origin, float offset_z)
{
    this->bases = bases;
    this->origin = _origin;
    this->offset_z = offset_z;

    float width = glm::distance(bases[0], bases[1]);
    auto pos = _origin->getGlobalPosition();
    pos.x += width / 2;
    pos.y -= 1000;
    centroid.setGlobalPosition(pos);

    setup_gui();

    for (int i = 0; i < 2; i++) {
        ofNode n;
        n.setGlobalPosition(pos.x, pos.y, pos.z + (i * offset_z));
        paths.push_back(create_line(n, 750));
        //paths.push_back(create_polygon(n, radius.get(), resolution.get(), offset_theta.get()));
    }

    // add a reference to each path's target
    for (auto& path : paths) {
        targets.push_back(&path.target);
    }
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
    if (play.get()) {
        // wrap around once we've hit 100%
        
        if (paths[0].path_type == PATH_TYPE::POLYGON) {
            evaluate_percent += speed.get();
            if (evaluate_percent >= 1.0) evaluate_percent = evaluate_percent - 1.0;
        }
        if (paths[0].path_type == PATH_TYPE::LINE) {
            float time_diff = ofGetElapsedTimef() - timer_start;
            if (time_diff <= duration.get()) {
                evaluate_percent = ofMap(time_diff, 0, duration.get(), 0, 1, true);
            }
        }

        for (int i = 0; i < paths.size(); i++) {
            // update the target
            paths[i].target = paths[i].path.getPointAtPercent(evaluate_percent);
        }
    }
}

//--------------------------------------------------------------
void MotionController::draw()
{
    for (int i = 0; i < paths.size(); i++) {
        paths[i].draw();
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
    params.add(pos.set("Position", centroid.getGlobalPosition(), glm::vec3(-2000, -2000, -2000), glm::vec3(2000, 0, 2000)));
    params.add(radius.set("Radius", 300, 0, 500));
    params.add(resolution.set("Resolution", 30, 3, 60));
    params.add(offset_theta.set("Offset_Theta", 0, -180, 180));
    params.add(speed.set("Speed", .001, 0, .1));
    params.add(duration.set("Duration (Sec)", 15, 0.1, 30));
    params.add(play.set("Play", false));
    params.add(reset.set("Reset"));

    play.addListener(this, &MotionController::on_play);
    pos.addListener(this, &MotionController::on_pos_changed);
    reset.addListener(this, &MotionController::on_reset);
    radius.addListener(this, &MotionController::on_parameter_changed);
    offset_theta.addListener(this, &MotionController::on_parameter_changed);
    resolution.addListener(this, &MotionController::on_parameter_changed);
	
    panel.setup("Motion_Controller");
    panel.setWidthElements(250);
    panel.setPosition(550, 15);
    panel.add(params);
}
