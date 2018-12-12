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
	pointsList.clear();
}

