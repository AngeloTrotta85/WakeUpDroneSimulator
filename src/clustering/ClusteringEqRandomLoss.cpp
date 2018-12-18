/*
 * ClusteringEqRandomLoss.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "ClusteringEqRandomLoss.h"
#include "../Loss.h"

using namespace std;

ClusteringEqRandomLoss::ClusteringEqRandomLoss() {
	// TODO Auto-generated constructor stub

}

void ClusteringEqRandomLoss::cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now, int uav_id) {
	unsigned int n4cluster = floor(double(sl.size()) / double(cv.size()));
	unsigned int remainingP = sl.size() - (cv.size() * n4cluster);

	std::vector<Sensor *> sl_bkp;
	for (auto& s : sl) {
		sl_bkp.push_back(s);
	}
	std::random_shuffle(sl_bkp.begin(), sl_bkp.end());

	for (auto& s : sl_bkp) {
		double actLoss_only = Loss::getInstance().calculate_loss_full(s, time_now, sl);
		CoordCluster *closesrC = nullptr;
		double minLossC = std::numeric_limits<double>::max();

		for (auto& c : cv) {
			if ((c->pointsList.size() < n4cluster) || ((c->pointsList.size() == n4cluster) && (remainingP > 0))) {
				double actLoss = s->coord.distance(c->clusterUAV->recharge_coord) * actLoss_only;

				if (actLoss < minLossC) {
					actLoss = minLossC;
					closesrC = c;
				}
			}
		}
		if (closesrC != nullptr) {
			if (closesrC->pointsList.size() == n4cluster) {
				--remainingP;
			}
			closesrC->pointsList.push_back(s);
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
