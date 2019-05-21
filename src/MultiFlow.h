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
#include "Generic.h"

using namespace std;

class ChargingNode {
public:
	int id;
	UAV* u;
	MyCoord pos;
	int lastTimestamp;
};

class SensorNode {
public:
	typedef struct SensorRead{
		int readTime;
		UAV *uav;
	} SensorRead;
public:
	Sensor* sens;
	list<SensorRead> readings;
	int lastTimestamp;
};

class MultiFlow {
public:
	MultiFlow();

	void addSensor(Sensor *s);
	void addChargStationAndUAV(MyCoord c, UAV *u);

	void run(int end_time);

	double calculate_pWU(void);

	ChargingNode *getLeftMostUAV(int end_time);
	int updateSensorsEnergy(int starttime, int endtime);
	void calculateTSP(ChargingNode *leftmost);

	double calcPowEta(int t);
	double energy_loss_onArc(int tstart);

private:
	map<int, ChargingNode *> cs_map;
	list<SensorNode *> sens_list;

	int actSensorTimeStamp;
	int actUAVTimeStamp;

	double pWU;
};

#endif /* MULTIFLOW_H_ */
