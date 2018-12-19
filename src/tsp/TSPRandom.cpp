/*
 * TSPRandom.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "TSPRandom.h"

TSPRandom::TSPRandom() {
	// TODO Auto-generated constructor stub

}

void TSPRandom::calculateTSP(CoordCluster * cc, std::list<Sensor *> &sl, int time_now) {
//void TSPRandom::calculateTSP(std::vector<CoordCluster *> &cv, int time_now) {
	//for (auto& c : cv) {
		cc->pointsTSP_listFinal.clear();
		//c->pointsNoTSP_listFinal.clear();

		for (auto& s : cc->pointsList) {
			cc->pointsTSP_listFinal.push_back(s);
		}
	//}
}
