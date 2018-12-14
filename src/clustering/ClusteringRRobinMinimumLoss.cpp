/*
 * ClusteringRRobinMinimumLoss.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "ClusteringRRobinMinimumLoss.h"
#include "../Loss.h"

using namespace std;

ClusteringRRobinMinimumLoss::ClusteringRRobinMinimumLoss() {
	// TODO Auto-generated constructor stub

}

void ClusteringRRobinMinimumLoss::cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	std::list<Sensor *> sl_bkp;
	for (auto& s : sl) {
		sl_bkp.push_back(s);
	}

	auto it_c = cv.begin();
	while (!sl_bkp.empty()) {
		std::list<Sensor *>::iterator closestSensor;
		double minLossC = std::numeric_limits<double>::max();

		for (auto it_s = sl_bkp.begin(); it_s != sl_bkp.end(); it_s++) {
			double actLoss_only = Loss::getInstance().calculate_loss_full(*it_s, time_now, sl);
			double actLoss = (*it_s)->coord.distance((*it_c)->clusterUAV->recharge_coord) * actLoss_only;

			if (actLoss < minLossC) {
				minLossC = actLoss;
				closestSensor = it_s;
			}
		}

		(*it_c)->pointsList.push_back(*closestSensor);
		sl_bkp.erase(closestSensor);

		it_c++;
		if (it_c == cv.end()){
			it_c = cv.begin();
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
