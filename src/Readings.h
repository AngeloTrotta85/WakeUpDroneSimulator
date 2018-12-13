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
	static void generate_readings(std::list<Sensor *> &sl, std::list<UAV *> &ul, int maxTime);
	static void writeOnFileReadings(std::string fn, std::list<Sensor *> &sl, int t);
	static void importReadingsFromFile(std::string inputFileName, std::list<Sensor *> &sl, std::list<UAV *> &ul, int t);

public:
	Sensor *sensor;
	UAV *uav;
	int  read_time;
	double value;
};

#endif /* READINGS_H_ */
