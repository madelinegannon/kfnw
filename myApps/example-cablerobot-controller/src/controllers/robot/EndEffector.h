#pragma once

#include "ofMain.h"

class EndEffector
{
private:
    ofNode node;
    ofMesh mesh;
public:
    EndEffector() = default;
    EndEffector(ofNode parent, glm::vec3 offset=glm::vec3(0,0,0));
    void update();
    void draw();
    void set_mesh(ofMesh mesh);
};