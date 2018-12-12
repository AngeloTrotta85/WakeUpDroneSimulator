/*
 * Sensor.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "MyCoord.h"

class Readings;

class Sensor {
public:
	Sensor(MyCoord sensCoord, double re);
	Sensor(MyCoord sensCoord, double re, int id_new);

public:
	MyCoord coord;
	double residual_energy;
	std::list<Readings *> mySensorReadings;
	int id;
	static int idSensGen;
};

#endif /* SENSOR_H_ */
