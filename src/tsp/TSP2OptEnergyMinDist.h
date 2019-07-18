/*
 * TSP2OptEnergy.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef TSP_TSP2OPTENERGYMINDIST_H_
#define TSP_TSP2OPTENERGYMINDIST_H_

#include "TSP.h"

#ifndef TSP_UAV_CODE
#define TSP_UAV_CODE 100000
#endif

class TSP2OptEnergyEdgeMinDist;

class TSP2OptEnergyMinDist: public TSP {
public:
	TSP2OptEnergyMinDist();

	//virtual void calculateTSP(std::vector<CoordCluster *> &cv, int time_now);
	virtual void calculateTSP(CoordCluster *cc, std::list<Sensor *> &sl, int time_now);

	void calculateCosts1Edge(TSP2OptEnergyEdgeMinDist *e, bool forceWakeUp, double &time, double &energy, bool log);
	void calculateCosts(std::list<TSP2OptEnergyEdgeMinDist *> edgesTSP, double &time, double &energy, bool log);

	void calculateTSP_subset(Sensor *uavDummySensor, std::list<Sensor *> &sList, std::list<TSP2OptEnergyEdgeMinDist *> &fCircuit);

public:
	static bool sortCosts (const std::pair<Sensor *, double> first, const std::pair<Sensor *, double> second) {
		return first.second < second.second;
	}
};

class TSP2OptEnergyEdgeMinDist {
public:
	static bool sortEdges (const TSP2OptEnergyEdgeMinDist *first, const TSP2OptEnergyEdgeMinDist *second) {
		return first->weight < second->weight;
	}
public:
	TSP2OptEnergyEdgeMinDist(Sensor *s1, Sensor *s2, double w) {
		first = s1;
		second = s2;
		weight = w;
		idTSP = -1;
	};

public:
	Sensor *first;
	Sensor *second;
	double weight;
	int idTSP;
};

#endif /* TSP_TSP2OPTENERGYMINDIST_H_ */
