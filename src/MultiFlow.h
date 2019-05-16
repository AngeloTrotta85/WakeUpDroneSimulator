/*
 * MultiFlow.h
 *
 *  Created on: May 16, 2019
 *      Author: angelo
 */

#ifndef MULTIFLOW_H_
#define MULTIFLOW_H_

#include <list>       // std::list
#include <map>       // std::map

#include "UAV.h"
#include "Sensor.h"
#include "MyCoord.h"

using namespace std;

class MultiFlow {
public:
	typedef struct ChargingStation{
		int id;
		UAV* u;
		MyCoord pos;
	} ChargingStation;

public:
	MultiFlow();

	void addSensor(Sensor *s);
	void addChargStationAndUAV(MyCoord c, UAV *u);

	void run(void);

private:
	map<int, ChargingStation> cs_map;
	list<Sensor *> sens_list;
};

#endif /* MULTIFLOW_H_ */
