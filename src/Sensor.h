/*
 * Sensor.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <map>
#include "MyCoord.h"

#define SENS_MIN_ID 1000
#define SENS_MAX_ID 9999

class Readings;

class Sensor {
public:
	Sensor(MyCoord sensCoord, double re);
	Sensor(MyCoord sensCoord, double re, int id_new);

	bool isBooked(void);

	static bool isSensorID(int id_check) {
		return ((id_check >= SENS_MIN_ID) && (id_check <= SENS_MAX_ID));
	}

public:
	static void generateRandomSensors(std::list<Sensor *> &pl, int ss, int ns);
	static void writeOnFileSensors(std::string fn, std::list<Sensor *> pointList);
	static void importSensorsFromFile(std::string inputFileName, std::list<Sensor *> &pl);

	static void printLogsSensors (std::list<Sensor *> &sl, int timeNow);

public:
	MyCoord coord;
	long double residual_energy;
	std::list<Readings *> mySensorReadings;
	std::map<int, bool> uavBookedReading;
	int id;
	static int idSensGen;
};

#endif /* SENSOR_H_ */
