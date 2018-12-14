/*
 * ClusteringKMeansRead.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "ClusteringKMeansRead.h"
#include "../Loss.h"

using namespace std;

ClusteringKMeansRead::ClusteringKMeansRead() {
	// TODO Auto-generated constructor stub

}

void ClusteringKMeansRead::cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	bool changed = true;
	int nIter = 200;

	while (changed && (nIter > 0)) {
		//while (nIter > 0) {
		// clear the clusters
		for (auto& cc : cv) {
			cc->clear();
		}

		// assign the sensors to the clusters
		for (auto& ss : sl) {
			CoordCluster *closestCL = nullptr;
			double minDist = std::numeric_limits<double>::max();

			for (auto& cc : cv) {
				double multiplier = 0.0;
				if (cc->pointsList.size() > 0) {
					//multiplier = (1.0 - calculate_loss_full(ss, time_now, cc->pointsList,  ALPHA, BETA, GAMMA));
					multiplier = Loss::getInstance().calculate_loss_full(ss, time_now, cc->pointsList);
				}
				double ddd = ss->coord.distance(*cc->clusterHead) * multiplier;

				if (ddd < minDist) {
					minDist = ddd;
					closestCL = cc;
				}
			}

			if (closestCL != nullptr) {
				closestCL->pointsList.push_back(ss);
			}
		}

		// update the cluster heads
		for (auto& cc : cv) {
			MyCoord newClusterCoord = MyCoord(0, 0);

			for (auto& ss : cc->pointsList) {
				newClusterCoord += ss->coord;
			}

			if (cc->pointsList.size() > 0) {
				newClusterCoord /= (double) cc->pointsList.size();
			}

			cc->clusterHead->x = newClusterCoord.x;
			cc->clusterHead->y = newClusterCoord.y;
		}

		// check if updated
		changed = false;
		for (auto& cc : cv) {
			if (!cc->checkNotChange()) {
				changed = true;
				break;
			}
		}
		nIter--;
	}
	cout << "k-means ended in " << 200 - nIter << " iterations" << endl;
}

