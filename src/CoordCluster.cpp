/*
 * CoordCluster.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "CoordCluster.h"

CoordCluster::CoordCluster (UAV *uav, int cID){
	clusterID = cID;
	clusterHead = new MyCoord(uav->recharge_coord.x, uav->recharge_coord.y);
	clusterUAV = uav;
}


void CoordCluster::clear(void) {
	pointsList_bkp.clear();
	for (auto& pp : pointsList) {
		pointsList_bkp.push_back(pp);
	}
	pointsList.clear();
}

bool CoordCluster::checkNotChange(void) {
	bool ris = true;
	for (auto& pp : pointsList) {
		bool found = false;
		for (auto& pp_b : pointsList_bkp) {
			if (pp_b == pp) {
				found = true;
				break;
			}
		}
		if (found == false) {
			ris = false;
			break;
		}
	}
	return (ris && (pointsList.size() == pointsList_bkp.size()));
}

