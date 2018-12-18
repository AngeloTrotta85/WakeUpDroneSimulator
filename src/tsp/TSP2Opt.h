/*
 * TSP2Opt.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef TSP_TSP2OPT_H_
#define TSP_TSP2OPT_H_

#include "TSP.h"

#define TSP_UAV_CODE 100000

class TSP2Opt: public TSP {
public:
	TSP2Opt();

	//virtual void calculateTSP(std::vector<CoordCluster *> &cv, int time_now);
	virtual void calculateTSP(CoordCluster *cc, int time_now);


};

class TSP2OptEdge {
public:
	static bool sortEdges (const TSP2OptEdge *first, const TSP2OptEdge *second) {
		return first->weight < second->weight;
	}
public:
	TSP2OptEdge(Sensor *s1, Sensor *s2, double w) {
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

#endif /* TSP_TSP2OPT_H_ */
