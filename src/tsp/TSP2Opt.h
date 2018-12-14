/*
 * TSP2Opt.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef TSP_TSP2OPT_H_
#define TSP_TSP2OPT_H_

#include "TSP.h"

class TSP2Opt: public TSP {
public:
	TSP2Opt();

	virtual void calculateTSP(std::vector<CoordCluster *> &cv, int time_now);
};

#endif /* TSP_TSP2OPT_H_ */
