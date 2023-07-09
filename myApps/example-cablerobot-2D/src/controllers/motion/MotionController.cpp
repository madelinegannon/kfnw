#include "MotionController.h"


//--------------------------------------------------------------
MotionController::MotionController()
{
    centroid.setGlobalPosition(850, -1350, 0);
    setup_gui();
    create_path(centroid.getGlobalPosition(), radius.get(), resolution.get());
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
    }
    // update the target
    target = path.getPointAtPercent(evaluate_percent);
}

//--------------------------------------------------------------
void MotionController::draw()
{
    ofPushStyle();

    centroid.draw();
    
    ofNoFill();
    ofSetLineWidth(5);
    ofSetColor(255, 100);
    path.draw();

    ofFill();
    ofSetColor(ofColor::red);
    ofDrawEllipse(target, 30, 30);

    ofPopStyle();
}

void MotionController::keyPressed(int key)
{
    switch (key) {
       
        default:
            break;
    }
    
}

void MotionController::create_path(glm::vec3 centroid, float radius, float resolution)
{
    path.clear();
    float theta = 360.0 / resolution;
    for (int i = 0; i <= resolution; i++) {
        glm::vec3 pt = glm::rotateZ(glm::vec3(radius, 0, 0), ofDegToRad(theta * i));
        pt += centroid;
        path.addVertex(pt);
    }

    target = path.getVertices()[0];
}

void MotionController::on_resolution_changed(float& val)
{
    create_path(centroid.getGlobalPosition(), radius.get(), resolution.get());
}


void MotionController::on_pos_changed(glm::vec3& val)
{
    centroid.setGlobalPosition(val);
    create_path(centroid.getGlobalPosition(), radius.get(), resolution.get());
}

void MotionController::on_radius_changed(float& val)
{
    create_path(centroid.getGlobalPosition(), radius.get(), resolution.get());
}

void MotionController::on_reset()
{
    evaluate_percent = 0.0;
}


void MotionController::setup_gui() 
{
	params.setName("Path_Params");
    params.add(pos.set("Position", centroid.getGlobalPosition(), glm::vec3(-2000, -2000, -2000), glm::vec3(2000, 0, 2000)));
    params.add(radius.set("Radius", 150, 0, 500));
    params.add(resolution.set("Resolution", 30, 3, 60));
    params.add(speed.set("Speed", .001, 0, .1));
    params.add(play.set("Play", false));
    params.add(reset.set("Reset"));

    pos.addListener(this, &MotionController::on_pos_changed);
    reset.addListener(this, &MotionController::on_reset);
    radius.addListener(this, &MotionController::on_radius_changed);
    resolution.addListener(this, &MotionController::on_resolution_changed);
	
    panel.setup("Motion_Controller");
    panel.setWidthElements(250);
    panel.setPosition(300, 15);
    panel.add(params);
}
