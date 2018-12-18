/*
 * ClusteringRandom.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "ClusteringRandom.h"
#include "../Loss.h"

using namespace std;

ClusteringRandom::ClusteringRandom() {
	// TODO Auto-generated constructor stub

}

void ClusteringRandom::cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now, int uav_id) {
	randomizeClusters(cv, sl);

	equalizeLoss(cv, sl, time_now);
}


void ClusteringRandom::randomizeClusters(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl) {
	unsigned int n4cluster = floor(double(sl.size()) / double(cv.size()));
	unsigned int remainingP = sl.size() - (cv.size() * n4cluster);

	std::vector<Sensor *> sl_bkp;
	for (auto& s : sl) {
		sl_bkp.push_back(s);
	}
	std::random_shuffle(sl_bkp.begin(), sl_bkp.end());

	auto it_s = sl_bkp.begin();

	for (auto& c : cv) {

		if (it_s == sl_bkp.end()) break;

		for (unsigned int j = 0; j < n4cluster; ++j) {
			c->pointsList.push_back(*it_s);
			it_s++;
		}

		if (it_s == sl_bkp.end()) break;

		if (remainingP > 0) {
			c->pointsList.push_back(*it_s);
			--remainingP;
			it_s++;
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
}

void ClusteringRandom::equalizeLoss(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	for (auto& c : cv) {
		double sumLosses = 0;
		double avgLoss = 0;
		double minLoss = std::numeric_limits<double>::max();
		double maxLoss = -1;

		for (auto& s : c->pointsList) {
			//double actLoss_only = calculate_loss_full(s, time_now, c->pointsList,  ALPHA, BETA, GAMMA);
			double actLoss_only = Loss::getInstance().calculate_loss_full(s, time_now, sl);
			double actLoss = s->coord.distance(c->clusterUAV->recharge_coord) * actLoss_only;

			sumLosses += actLoss;

			if (actLoss > maxLoss) {
				maxLoss = actLoss;
			}
			if (actLoss < minLoss) {
				minLoss = actLoss;
			}
		}
		if (c->pointsList.size() > 0) {
			avgLoss = sumLosses / ((double) c->pointsList.size());
		}

		cout << "Cluster " << c->clusterID << " has avLoss: " << avgLoss << "[" << minLoss << "; " << maxLoss << "]" << endl;
	}
}
