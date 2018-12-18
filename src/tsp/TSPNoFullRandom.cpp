/*
 * TSPNoFullRandom.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "TSPNoFullRandom.h"

#include "../RandomGenerator.h"

TSPNoFullRandom::TSPNoFullRandom() {
	// TODO Auto-generated constructor stub

}

void TSPNoFullRandom::calculateTSP(CoordCluster *cc, int time_now) {
//void TSPNoFullRandom::calculateTSP(std::vector<CoordCluster *> &cv, int time_now) {
	//for (auto& c : cv) {
		cc->pointsTSP_listFinal.clear();
		//c->pointsNoTSP_listFinal.clear();

		int r_tsp = RandomGenerator::getInstance().getIntUniform(1.0, cc->pointsList.size() - 1);

		//cout << "noFullRandom_tsp: generating " << r_tsp

		for (auto& s : cc->pointsList) {
			if (r_tsp > 0) {
				cc->pointsTSP_listFinal.push_back(s);
			}
			else {
				//c->pointsNoTSP_listFinal.push_back(s);
			}
			--r_tsp;
		}
	//}
}
