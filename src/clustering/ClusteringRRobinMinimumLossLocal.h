/*
 * ClusteringRRobinMinimumLossLocal.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef CLUSTERING_CLUSTERINGRROBINMINIMUMLOSSLOCAL_H_
#define CLUSTERING_CLUSTERINGRROBINMINIMUMLOSSLOCAL_H_

#include "Clustering.h"

class ClusteringRRobinMinimumLossLocal: public Clustering {
public:
	ClusteringRRobinMinimumLossLocal();

	virtual void cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now, int uav_id = -1);
};

#endif /* CLUSTERING_CLUSTERINGRROBINMINIMUMLOSSLOCAL_H_ */
