/*
 * ClusteringRandom.h
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#ifndef CLUSTERING_CLUSTERINGRANDOM_H_
#define CLUSTERING_CLUSTERINGRANDOM_H_

#include "Clustering.h"

class ClusteringRandom: public Clustering {
public:
	ClusteringRandom();

	virtual void cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now);

private:
	void equalizeLoss(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now);
	void randomizeClusters(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl);
};

#endif /* CLUSTERING_CLUSTERINGRANDOM_H_ */
