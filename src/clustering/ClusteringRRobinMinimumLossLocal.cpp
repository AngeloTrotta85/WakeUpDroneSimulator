/*
 * ClusteringRRobinMinimumLossLocal.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "ClusteringRRobinMinimumLossLocal.h"
#include "../Loss.h"

using namespace std;

ClusteringRRobinMinimumLossLocal::ClusteringRRobinMinimumLossLocal() {
	// TODO Auto-generated constructor stub

}

void ClusteringRRobinMinimumLossLocal::cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	std::list<Sensor *> sl_bkp;
	for (auto& s : sl) {
		sl_bkp.push_back(s);
	}
	std::vector<CoordCluster *> cv_bkp;
	for (auto& c : cv) {
		cv_bkp.push_back(c);
	}
	std::random_shuffle(cv_bkp.begin(), cv_bkp.end());

	auto it_c = cv_bkp.begin();
	while (!sl_bkp.empty()) {
		std::list<Sensor *>::iterator closestSensor; // = sl_bkp.begin();
		double minLossC = std::numeric_limits<double>::max();


		for (auto it_s = sl_bkp.begin(); it_s != sl_bkp.end(); it_s++) {
			double actLoss_only = Loss::getInstance().calculate_loss_full(*it_s, time_now, sl);
			double actLossLocal_only = Loss::getInstance().calculate_loss_full(*it_s, time_now, (*it_c)->pointsList);
			double actLoss_sum = Loss::algebraic_sum(actLoss_only, actLossLocal_only);
			double distance = (*it_s)->coord.distance((*it_c)->clusterUAV->recharge_coord);
			double actLoss = distance * actLoss_sum;

			//cout << "Losses ->"
			//		<< " actLoss_only: " << actLoss_only
			//		<< " actLossLocal_only: " << actLossLocal_only << "(pl_size: " << (*it_c)->pointsList.size() << ")"
			//		<< " actLoss_sum: " << actLoss_sum
			//		<< " actLoss: " << actLoss << endl << flush;

			if (actLoss < minLossC) {
				minLossC = actLoss;
				closestSensor = it_s;
			}
		}

		(*it_c)->pointsList.push_back(*closestSensor);
		sl_bkp.erase(closestSensor);

		it_c++;
		if (it_c == cv_bkp.end()){
			std::random_shuffle(cv_bkp.begin(), cv_bkp.end());
			it_c = cv_bkp.begin();
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
