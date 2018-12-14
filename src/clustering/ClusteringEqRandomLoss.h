/*
 * ClusteringEqRandomLoss.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef CLUSTERING_CLUSTERINGEQRANDOMLOSS_H_
#define CLUSTERING_CLUSTERINGEQRANDOMLOSS_H_

#include "Clustering.h"

class ClusteringEqRandomLoss: public Clustering {
public:
	ClusteringEqRandomLoss();

	virtual void cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now);
};

#endif /* CLUSTERING_CLUSTERINGEQRANDOMLOSS_H_ */
