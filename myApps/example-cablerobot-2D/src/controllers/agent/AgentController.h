#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "Agent.h"

class AgentController
{
public:
	AgentController();
	~AgentController();

	void setup(int num_agents);
	void update();
    void update(vector<glm::vec3> curr_positions);
	void draw();
    
    void set_targets(vector<ofMatrix4x4*> tgts);
    void set_targets(vector<ofNode*> tgts);
    void set_targets(vector<glm::vec3*> tgts);


    void remove_trail_pt(int index);
    vector<glm::vec3> get_trail_targets();

    ofParameterGroup params;

private:
    int NUM_AGENTS;
    
    vector<Agent*> agents;
    vector<ofNode> targets;

    void setup_gui();

    ofParameter<ofVec3f> gravity;
    ofParameter<bool> match_all;
};

