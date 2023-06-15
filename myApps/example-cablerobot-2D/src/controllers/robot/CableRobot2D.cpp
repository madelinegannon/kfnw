#include "CableRobot2D.h"

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
}
