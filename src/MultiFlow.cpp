/*
 * MultiFlow.cpp
 *
 *  Created on: May 16, 2019
 *      Author: angelo
 */

#include "MultiFlow.h"
#include "Generic.h"

MultiFlow::MultiFlow() {
	// TODO Auto-generated constructor stub

}

void MultiFlow::addSensor(Sensor *s) {
	sens_list.push_back(s);
}

void MultiFlow::addChargStationAndUAV(MyCoord c, UAV *u) {
	ChargingStation newcs;

	newcs.id = u->id;
	newcs.pos = c;
	newcs.u = u;

	cs_map[u->id] = newcs;
}

void MultiFlow::run(void) {

}


