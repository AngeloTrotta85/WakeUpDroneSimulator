/*
 * UAV.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef UAV_H_
#define UAV_H_

#include "MyCoord.h"

class Readings;

class UAV {
public:
	UAV(MyCoord recCoord, double re);
	UAV(MyCoord recCoord, double re, int id_new);

public:
	MyCoord recharge_coord;
	double residual_energy;
	std::list<Readings *> mySensorReadings;
	int id;
	static int idUAVGen;
};

#endif /* UAV_H_ */
