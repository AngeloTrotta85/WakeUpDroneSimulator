/*
 * Simulator.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "Simulator.h"

#include "clustering/ClusteringEqRandomLoss.h"
#include "clustering/ClusteringKMeans.h"
#include "clustering/ClusteringKMeansRead.h"
#include "clustering/ClusteringRandom.h"
#include "clustering/ClusteringRRobinMinimumLoss.h"
#include "clustering/ClusteringRRobinMinimumLossLocal.h"

Simulator::Simulator() {
	simulation_time = 0;
	end_time = -1;

	clust = nullptr;
}

void Simulator::init(int stime, int etime) {
	simulation_time = stime;
	end_time = etime;
}

void Simulator::setClusteringAlgo(std::string algotype_clustering) {
	if (!algotype_clustering.empty()) {
		if (algotype_clustering.compare("kmS") == 0) {
			clust = new ClusteringKMeans(); //simple k-means
		}
		else if (algotype_clustering.compare("kmR") == 0) {
			clust = new ClusteringKMeansRead();	//k-means with loss in the distance calculus
		}
		else if (algotype_clustering.compare("eqLoss") == 0) {
			clust = new ClusteringRandom();	// just random for now
		}
		else if (algotype_clustering.compare("eqRandLoss") == 0) {
			clust = new ClusteringEqRandomLoss();	// from each random sensor choose the closest cluster
		}
		else if (algotype_clustering.compare("rrMinLoss") == 0) {
			clust = new ClusteringRRobinMinimumLoss();	// round robin cluster, choose mis loss sensor
		}
		else if (algotype_clustering.compare("rrMinLossLocal") == 0) {
			clust = new ClusteringRRobinMinimumLossLocal();	// round robin cluster, choose min loss sensor
		}
		else {
			std::cerr << "Unknown algotype for clustering: \"" << algotype_clustering << "\". Using default simple k-means" << std::endl;
			clust = new ClusteringKMeans(); //simple k-means
		}
	}
	else {
		//default simple k-means
		std::cerr << "Undefined algotype for clustering. Using default simple k-means" << std::endl;
		clust = new ClusteringKMeans(); //simple k-means
	}
}

void Simulator::setTSPAlgo(std::string algotype_tsp) {

}

void Simulator::finish() {

}

void Simulator::run(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList) {
	while ((end_time < 0) || (simulation_time < end_time)) {

		++simulation_time;
	}
}

