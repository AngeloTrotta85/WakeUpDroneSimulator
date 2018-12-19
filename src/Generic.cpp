/*
 * Generic.cpp
 *
 *  Created on: Dec 19, 2018
 *      Author: angelo
 */

#include "Generic.h"

Generic::Generic() {
	// TODO Auto-generated constructor stub

}

double Generic::getTime2Travel(MyCoord start, MyCoord end) {
	double ris = 0;
	return ris;
}

double Generic::getEnergy2Travel(MyCoord start, MyCoord end) {
	double ris = 0;
	ris = start.distance(end) * 400;
	return ris;
}

double Generic::getTime2WakeRead(MyCoord uavCoord, MyCoord sensorCoord) {
	double ris = 0;
	return ris;
}

double Generic::getEnergy2WakeRead(MyCoord uavCoord, MyCoord sensorCoord) {
	double ris = 0;
	ris = 5000;
	return ris;
}
