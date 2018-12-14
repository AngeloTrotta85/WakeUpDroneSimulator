/*
 * ClusteringKMeans.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef CLUSTERING_CLUSTERINGKMEANS_H_
#define CLUSTERING_CLUSTERINGKMEANS_H_

#include "Clustering.h"

class ClusteringKMeans: public Clustering {
public:
	ClusteringKMeans();

	virtual void cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now);
};

#endif /* CLUSTERING_CLUSTERINGKMEANS_H_ */
