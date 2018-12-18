/*
 * ClusteringKMeansRead.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef CLUSTERING_CLUSTERINGKMEANSREAD_H_
#define CLUSTERING_CLUSTERINGKMEANSREAD_H_

#include "Clustering.h"

class ClusteringKMeansRead: public Clustering {
public:
	ClusteringKMeansRead();

	virtual void cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now, int uav_id = -1);
};

#endif /* CLUSTERING_CLUSTERINGKMEANSREAD_H_ */
