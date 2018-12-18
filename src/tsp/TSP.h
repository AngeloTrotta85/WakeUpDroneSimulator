/*
 * TSP.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef TSP_TSP_H_
#define TSP_TSP_H_

#include "../CoordCluster.h"

class TSP {
public:
	TSP() {};
	virtual ~TSP() {};

	//virtual void calculateTSP(std::vector<CoordCluster *> &cv, int time_now) {};
	virtual void calculateTSP(CoordCluster *cc, int time_now) {};
};


#endif /* TSP_TSP_H_ */
