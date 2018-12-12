/*
 * Readings.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef READINGS_H_
#define READINGS_H_

#include "UAV.h"
#include "Sensor.h"

class Readings {
public:
	Readings(Sensor *s, UAV *u, int timestamp, double val);

public:
	Sensor *sensor;
	UAV *uav;
	int  read_time;
	double value;
};

#endif /* READINGS_H_ */
