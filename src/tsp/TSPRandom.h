/*
 * TSPRandom.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef TSP_TSPRANDOM_H_
#define TSP_TSPRANDOM_H_

#include "TSP.h"

class TSPRandom: public TSP {
public:
	TSPRandom();

	//virtual void calculateTSP(std::vector<CoordCluster *> &cv, int time_now);
	virtual void calculateTSP(CoordCluster *cc, std::list<Sensor *> &sl, int time_now);
};

#endif /* TSP_TSPRANDOM_H_ */
