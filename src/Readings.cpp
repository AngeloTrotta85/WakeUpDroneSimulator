/*
 * Readings.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "Readings.h"

Readings::Readings(Sensor *s, UAV *u, int timestamp, double val) {
	sensor = s;
	uav = u;
	value = val;
	read_time = timestamp;
}

