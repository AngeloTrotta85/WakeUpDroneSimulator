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

	double getPDF_Eloc(MyCoord e);
	double getPDF_Erot(MyCoord r);

	double getPDF_Eloc_single(double e);
	double getPDF_Erot_single(double r);

	double calcProb_EReceived(double e);
	void calcProb_EReceivedTime_rec(double &acc, std::vector<double> &vect, int t, double e, double deltae);
	double calcProb_EReceivedTime(double e, double deltae, int t);
	double calculate_pWU(void);

	ChargingNode *getLeftMostUAV(int end_time);
	int updateSensorsEnergy(int starttime, int endtime);
	void calculateTSP(ChargingNode *leftmost);

	double calcPowEta(int t);
	double energy_loss_onArc(int tstart);

public:
	double calc_Beta(double d3D, double h, double d2D);
	double calc_Gain(double alpha, double gMAX, double alphaMAX);
	double calc_PathLoss(double d3D, double fMHz);
	double calc_Gamma(double x, double y, double rho_x, double rho_y);

private:
	map<int, ChargingNode *> cs_map;
	list<SensorNode *> sens_list;

	int actSensorTimeStamp;
	int actUAVTimeStamp;

	double pWU;
};

#endif /* MULTIFLOW_H_ */
