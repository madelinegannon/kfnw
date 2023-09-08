#include "CommsSensor.h"

CommsSensor::CommsSensor()
{
}

void CommsSensor::setup(vector<ofNode*> data, int port)
{
	this->data = data;
	this->port = port;
	ofxOscReceiverSettings settings;
	settings.port = port;
	
	/* MAD EDIT 9/8/2023: TURNING OFF UNTIL WE GET A FASTER LAPTOP
	receiver.setup(settings);
	startThread();
	*/
}

void CommsSensor::threadedFunction()
{
	while (isThreadRunning()) {
		check_for_message();
	}
}

void CommsSensor::check_for_message()
{
	while (receiver.hasWaitingMessages()) {
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage(m);
		if (m.getAddress() == "/body") {
			for (int i = 0; i < K4A_JOINT_COUNT * 4; i += 4) {
				int id = m.getArgAsInt(i);
				float x = m.getArgAsFloat(i + 1);
				float y = m.getArgAsFloat(i + 2);
				float z = m.getArgAsFloat(i + 3);
				data[id]->setPosition(x, y, z);				
				//cout << id << ": {" << x << ", " << y << ", " << z << "}" << endl;
			}
		}
		else {
			// unrecognized message: display on the bottom of the screen
			string msgString;
			msgString = m.getAddress();
			msgString += ":";
			for (size_t i = 0; i < m.getNumArgs(); i++) {

				// get the argument type
				msgString += " ";
				msgString += m.getArgTypeName(i);
				msgString += ":";

				// display the argument - make sure we get the right type
				if (m.getArgType(i) == OFXOSC_TYPE_INT32) {
					msgString += ofToString(m.getArgAsInt32(i));
				}
				else if (m.getArgType(i) == OFXOSC_TYPE_FLOAT) {
					msgString += ofToString(m.getArgAsFloat(i));
				}
				else if (m.getArgType(i) == OFXOSC_TYPE_STRING) {
					msgString += m.getArgAsString(i);
				}
				else {
					msgString += "unhandled argument type " + m.getArgTypeName(i);
				}
			}
			cout << msgString << endl;
		}
	}
}
