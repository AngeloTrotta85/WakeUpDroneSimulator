/*
 * TSPNoFullRandom.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef TSP_TSPNOFULLRANDOM_H_
#define TSP_TSPNOFULLRANDOM_H_

#include "TSP.h"

class TSPNoFullRandom: public TSP {
public:
	TSPNoFullRandom();

	//virtual void calculateTSP(std::vector<CoordCluster *> &cv, int time_now);
	virtual void calculateTSP(CoordCluster *cc, int time_now);
};

#endif /* TSP_TSPNOFULLRANDOM_H_ */
