#include "AgentController.h"


//--------------------------------------------------------------
AgentController::AgentController(){}

//--------------------------------------------------------------
AgentController::~AgentController(){}

//--------------------------------------------------------------
void AgentController::setup(int num_agents)
{
	NUM_AGENTS = num_agents;
	for (int i=0; i<NUM_AGENTS; i++){
        agents.push_back(new Agent());
        agents[i]->setup(i);
        
        targets.push_back(ofNode());
        targets[i] = agents[i]->pose;
    }
	
    setup_gui();
}

//--------------------------------------------------------------
void AgentController::update()
{
    for (int i=0; i<NUM_AGENTS; i++){
        
        agents[i]->apply_forces(gravity);
        
        agents[i]->move(agents, targets[i]);
        
        // update the  agent params with agent 0
        if (match_all){
            agents[i]->set_params(agents[0]->radius, agents[0]->kp, agents[0]->kd, agents[0]->steering_scalar);
        }

        agents[i]->update();
    }    
}

void AgentController::update(vector<glm::vec3> curr_positions)
{
    // check if we need to remove a trail target
    for (int i = 0; i < curr_positions.size(); i++) {
        auto pt = agents[i]->trail.begin()->getGlobalPosition();
        glm::vec2 start = glm::vec2(pt.x, pt.y);
        glm::vec2 end = glm::vec2(curr_positions[i].x, curr_positions[i].y);
        auto dist_sq = glm::distance2(start, end);
        cout << i << ": pt { " << ofToString(start) << " }, curr_pos: { " << ofToString(end) << " }, dist_sq: " << dist_sq << endl;
        float dist_thresh = 250;
        if (dist_sq < dist_thresh * dist_thresh) {
            remove_trail_pt(i);
        }
    }
    update();
}

//--------------------------------------------------------------
void AgentController::draw()
{
    for (int i=0; i<NUM_AGENTS; i++){
        agents[i]->draw();
    }
}

//--------------------------------------------------------------
void AgentController::setup_gui()
{
    params.setName("Agent_Controller");
    params.add(gravity.set("Gravity", ofVec3f(0,0,0), ofVec3f(-20,-20,-20), ofVec3f()));
    params.add(match_all.set("Match_All_Params", false));
    
    for (int i=0; i<NUM_AGENTS; i++){        
        params.add(agents[i]->params);
    }
}

//--------------------------------------------------------------
void AgentController::set_targets(vector<ofMatrix4x4*> tgts)
{
    for (int i=0; i<NUM_AGENTS; i++){
        ofNode tgt;
        //    tgt.setGlobalOrientation(target->getRotate());
        tgt.setGlobalPosition(tgts[i]->getTranslation());
        targets[i] = tgt;
    }
}

//--------------------------------------------------------------
void AgentController::set_targets(vector<ofNode*> tgts)
{
    for (int i = 0; i < NUM_AGENTS; i++) {
        targets[i] = *tgts[i];
    }
}

//--------------------------------------------------------------
void AgentController::set_targets(vector<glm::vec3*> tgts)
{
    for (int i = 0; i < NUM_AGENTS; i++) {
        ofNode tgt;
        tgt.setGlobalPosition(*tgts[i]);
        targets[i] = tgt;

    }
}

void AgentController::set_target(int i, glm::vec3* tgt)
{
    if (i < NUM_AGENTS) {
        ofNode node;
        node.setGlobalPosition(*tgt);
        targets[i] = node;
    }
}

void AgentController::remove_trail_pt(int index)
{ 

    if (index < NUM_AGENTS && agents[index]->trail.size() > 0) {
        agents[index]->trail.erase(agents[index]->trail.begin());
    }
}

vector<glm::vec3> AgentController::get_trail_targets()
{
    vector<glm::vec3> targets;
    for (int i = 0; i < NUM_AGENTS; i++) {
        glm::vec3 pos;
        if (agents[i]->trail.size() > 0) {
            pos = agents[i]->trail.begin()->getGlobalPosition();
        }
        else {
            pos = agents[i]->pose.getGlobalPosition();
        }
        targets.push_back(pos);
    }
    return targets;
}

vector<glm::vec3> AgentController::get_positions()
{
    vector<glm::vec3> positions;
    for (int i = 0; i < agents.size(); i++) {
        positions.push_back(agents[i]->pose.getGlobalPosition());
    }
    return positions;
}

