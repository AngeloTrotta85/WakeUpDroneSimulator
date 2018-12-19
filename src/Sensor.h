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

	void update_energy(void);

public:
	static void generateRandomSensors(std::list<Sensor *> &pl, int ss, int ns);
	static void writeOnFileSensors(std::string fn, std::list<Sensor *> pointList);
	static void importSensorsFromFile(std::string inputFileName, std::list<Sensor *> &pl);

	static void printLogsSensors (std::list<Sensor *> &sl, int timeNow);

public:
	MyCoord coord;
	double residual_energy;
	std::list<Readings *> mySensorReadings;
	bool bookedReading;
	int id;
	static int idSensGen;
};

#endif /* SENSOR_H_ */
