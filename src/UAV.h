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
	typedef enum uav_state {
		IDLE,
		RECHARGING,
		MOVING,
		WAKINGUP,
		READING
	} uav_state;

public:
	UAV(MyCoord recCoord, double re);
	UAV(MyCoord recCoord, double re, int id_new);

	void move(void);
	void update_energy(void);


public:
	static void generateRandomUAVs(std::list<UAV *> &pl, int ss, int nu);
	static void importUAVsFromFile(std::string inputFileName, std::list<UAV *> &pl);
	static void writeOnFileUAVs(std::string fn, std::list<UAV *> pointList);

public:
	MyCoord recharge_coord;
	double residual_energy;
	double max_energy;
	std::list<Readings *> mySensorReadings;
	uav_state state;

	int id;
	static int idUAVGen;
};

#endif /* UAV_H_ */
