/*
 * Clustering.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef CLUSTERING_CLUSTERING_H_
#define CLUSTERING_CLUSTERING_H_

#include "../CoordCluster.h"

class Clustering {
public:
	Clustering() {};
	virtual ~Clustering() {};

	virtual void cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {};
};

#endif /* CLUSTERING_CLUSTERING_H_ */
