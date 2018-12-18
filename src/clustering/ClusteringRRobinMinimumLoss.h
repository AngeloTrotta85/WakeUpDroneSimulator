/*
 * ClusteringRRobinMinimumLoss.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef CLUSTERING_CLUSTERINGRROBINMINIMUMLOSS_H_
#define CLUSTERING_CLUSTERINGRROBINMINIMUMLOSS_H_

#include "Clustering.h"

class ClusteringRRobinMinimumLoss: public Clustering {
public:
	ClusteringRRobinMinimumLoss();

	virtual void cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now, int uav_id = -1);
};

#endif /* CLUSTERING_CLUSTERINGRROBINMINIMUMLOSS_H_ */
