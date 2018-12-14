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

void TSPRandom::calculateTSP(std::vector<CoordCluster *> &cv, int time_now) {
	for (auto& c : cv) {
		c->pointsTSP_listFinal.clear();
		c->pointsNoTSP_listFinal.clear();

		for (auto& s : c->pointsList) {
			c->pointsTSP_listFinal.push_back(s);
		}
	}
}
