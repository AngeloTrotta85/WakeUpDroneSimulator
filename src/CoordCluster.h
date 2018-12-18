/*
 * CoordCluster.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef COORDCLUSTER_H_
#define COORDCLUSTER_H_

#include "UAV.h"
#include "Sensor.h"

class CoordCluster{
public:
	CoordCluster (UAV *uav, int cID);

	void clear(void);
	bool checkNotChange(void);

public:
	std::list<Sensor *> pointsList;
	std::list<Sensor *> pointsList_bkp;
	std::list<Sensor *> pointsTSP_listFinal;
	//std::list<Sensor *> pointsNoTSP_listFinal;
	MyCoord *clusterHead;
	UAV *clusterUAV;
	int clusterID;
};

#endif /* COORDCLUSTER_H_ */
