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

void ClusteringRRobinMinimumLoss::cluster(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now, int uav_id) {
	std::vector<Sensor *> sl_bkp_v;
	std::list<Sensor *> sl_bkp;
	unsigned int round = 1;

	for (auto& s : sl) {
		bool s_found = false;
		for (auto& cc : cv) {
			for (auto& sl : cc->pointsTSP_listFinal) {
				if (sl->id == s->id) {
					s_found = true;
					break;
				}
			}
			if (s_found) break;
		}
		if (!s_found) {
			sl_bkp_v.push_back(s);
		}
	}

	for (auto& cc : cv) {
		if ((uav_id < 0) || (uav_id == cc->clusterUAV->id)) {
			cc->pointsList.clear();
		}
	}

	std::random_shuffle(sl_bkp_v.begin(), sl_bkp_v.end());
	for (auto& ss : sl_bkp_v) {
		sl_bkp.push_back(ss);
	}

	//cout << "Clustering after shuffle. sl_bkp.size: " << sl_bkp.size() << " and cv.size: " << cv.size() << endl << flush;

	auto it_c = cv.begin();
	while (!sl_bkp.empty()) {

		//cout << "Clustering while 1" << endl << flush;

		if (round > (*it_c)->pointsTSP_listFinal.size()) {
			std::list<Sensor *>::iterator closestSensor;
			double minLossC = std::numeric_limits<double>::max();

			//cout << "Clustering while 1_1" << endl << flush;

			for (auto it_s = sl_bkp.begin(); it_s != sl_bkp.end(); it_s++) {
				double actLoss_only;
				double actLoss;

				//cout << "Clustering while 1_1_1" << endl << flush;
				actLoss_only = Loss::getInstance().calculate_loss_full(*it_s, time_now, sl);
				//cout << "Clustering while 1_1_2" << endl << flush;
				actLoss = (*it_s)->coord.distance((*it_c)->clusterUAV->recharge_coord) * actLoss_only;
				//cout << "Clustering while 1_1_3" << endl << flush;
				if (actLoss < minLossC) {
					minLossC = actLoss;
					closestSensor = it_s;
				}
			}

			//cout << "Clustering while 1_2" << endl << flush;

			if ((uav_id < 0) || (uav_id == (*it_c)->clusterUAV->id)) {
				(*it_c)->pointsList.push_back(*closestSensor);
			}
			//cout << "Clustering while 1_3" << endl << flush;
			sl_bkp.erase(closestSensor);
			//cout << "Clustering while 1_4" << endl << flush;
		}

		//cout << "Clustering while 2" << endl << flush;

		it_c++;
		if (it_c == cv.end()){
			it_c = cv.begin();
			++round;
		}

		//cout << "Clustering while 3" << endl << flush;
	}

	//cout << "Clustering updating cluster heads" << endl << flush;

	// update the cluster heads
	for (auto& cc : cv) {
		if ((uav_id < 0) || (uav_id == cc->clusterUAV->id)) {
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
}
