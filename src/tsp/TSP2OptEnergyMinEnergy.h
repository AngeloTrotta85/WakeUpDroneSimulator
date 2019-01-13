/*
 * TSP2OptEnergy.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef TSP_TSP2OPTENERGYMINENERGY_H_
#define TSP_TSP2OPTENERGYMINENERGY_H_

#include "TSP.h"

//#ifndef TSP_UAV_CODE
#define TSP_UAV_CODE 100000
//#endif

class TSP2OptEnergyEdgeMinEnergy;

class TSP2OptEnergyMinEnergy: public TSP {
public:
	TSP2OptEnergyMinEnergy();

	//virtual void calculateTSP(std::vector<CoordCluster *> &cv, int time_now);
	virtual void calculateTSP(CoordCluster *cc, std::list<Sensor *> &sl, int time_now);

	void calculateCosts1Edge(TSP2OptEnergyEdgeMinEnergy *e, bool forceWakeUp, double &time, double &energy, bool log);
	void calculateCosts(std::list<TSP2OptEnergyEdgeMinEnergy *> edgesTSP, double &time, double &energy, bool log);

	void calculateTSP_subset(Sensor *uavDummySensor, std::list<Sensor *> &sList, std::list<TSP2OptEnergyEdgeMinEnergy *> &fCircuit);

public:
	static bool sortCosts (const std::pair<Sensor *, double> first, const std::pair<Sensor *, double> second) {
		return first.second < second.second;
	}
};

class TSP2OptEnergyEdgeMinEnergy {
public:
	static bool sortEdges (const TSP2OptEnergyEdgeMinEnergy *first, const TSP2OptEnergyEdgeMinEnergy *second) {
		return first->weight < second->weight;
	}
public:
	TSP2OptEnergyEdgeMinEnergy(Sensor *s1, Sensor *s2, double w) {
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

#endif /* TSP_TSP2OPTENERGYMINENERGY_H_ */
