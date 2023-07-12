#include "MotionController.h"


//--------------------------------------------------------------
MotionController::MotionController()
{
    centroid.setGlobalPosition(500 + 1250, -1350, 0);
    setup_gui();
   
    glm::vec3 pos = centroid.getGlobalPosition();
    for (int i = 0; i < 2; i++) {
        MotionPath mp;
        mp.centroid.setGlobalPosition(pos.x, pos.y, pos.z + (i * offset_z));
        mp.radius = radius.get();
        mp.resolution = resolution.get();
        mp.offset_theta = offset_theta.get();
        mp.reset();
        paths.push_back(mp);
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
        evaluate_percent += speed.get();
        if (evaluate_percent >= 1.0) evaluate_percent = evaluate_percent - 1.0;

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

void MotionController::keyPressed(int key)
{
    switch (key) {
       
        default:
            break;
    }
    
}

vector<glm::vec3*> MotionController::get_targets()
{
    return targets;
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
    params.add(radius.set("Radius", 250, 0, 500));
    params.add(resolution.set("Resolution", 30, 3, 60));
    params.add(offset_theta.set("Offset_Theta", 0, -180, 180));
    params.add(speed.set("Speed", .001, 0, .1));
    params.add(play.set("Play", false));
    params.add(reset.set("Reset"));

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
