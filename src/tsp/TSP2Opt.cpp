/*
 * TSP2Opt.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "TSP2Opt.h"

using namespace std;

TSP2Opt::TSP2Opt() {
	// TODO Auto-generated constructor stub

}

void TSP2Opt::calculateTSP(CoordCluster *cc, int time_now) {
//void TSP2Opt::calculateTSP(std::vector<CoordCluster *> &cv, int time_now) {
	Sensor *uavDummySensor = new Sensor(cc->clusterUAV->recharge_coord, 1, TSP_UAV_CODE);
	list<TSP2OptEdge *> edges;
	list<TSP2OptEdge *> finaledges;
	list<TSP2OptEdge *> finalcircuit;

	cc->pointsTSP_listFinal.clear();
	cout << endl << "Calculating TSP for UAV" << cc->clusterUAV->id << ". The sensors are:";
	for (auto& ss : cc->pointsList) {
		cout << " S" << ss->id;
	}
	cout << endl;

	auto it1 = cc->pointsList.begin();
	while (it1 != cc->pointsList.end()) {
		auto it2 = it1;
		it2++;
		while (it2 != cc->pointsList.end()) {
			double w = (*it1)->coord.distance((*it2)->coord);
			edges.push_back(new TSP2OptEdge(*it1, *it2, w));
			it2++;
		}
		it1++;
	}
	for (auto& s : cc->pointsList) {
		double w = s->coord.distance(uavDummySensor->coord);
		edges.push_back(new TSP2OptEdge(s, uavDummySensor, w));
	}

	edges.sort(TSP2OptEdge::sortEdges);

	cout << "Edges for this TSP:" << endl;
	for (auto& ee : edges) {
		cout << "\t" << "S" << ee->first->id << "-S" << ee->second->id << "[" << ee->weight << "]" << endl;
	}
	cout << endl;

	int idFinal = 0;
	for (auto& e : edges) {
		//check if the sensors in the edge are already caught
		int connected_first = 0;
		int connected_second = 0;
		int idFirst = -1;
		int idSecond = -2;

		for (auto& fe : finaledges) {
			if ((e->first->id == fe->first->id) || (e->first->id == fe->second->id)) {
				connected_first++;
				idFirst = fe->idTSP;
			}
			if ((e->second->id == fe->first->id) || (e->second->id == fe->second->id)) {
				connected_second++;
				idSecond = fe->idTSP;
			}
		}

		cout << "Checking S" << e->first->id << "-S" << e->second->id << "."
				<< " ConnFirst: " << connected_first << " ConnSecond: " << connected_second
				<< " idFirst: " << idFirst << " idSecond: " << idSecond
				<< endl;

		if ((idFirst != idSecond) && (connected_first < 2) && (connected_second < 2)) {
			if (idFirst < 0) {
				idFirst = idFinal++;
			}
			for (auto& fe : finaledges) {
				if (fe->idTSP == idSecond) {
					fe->idTSP = idFirst;
				}
			}
			e->idTSP = idFirst;
			finaledges.push_back(e);
		}

		/*int connected_first = 0;
		int connected_second = 0;
		for (auto& fe : finaledges) {
			if ((e->first->id == fe->first->id) || (e->first->id == fe->second->id)) {
				connected_first++;
			}
			if ((e->second->id == fe->first->id) || (e->second->id == fe->second->id)) {
				connected_second++;
			}
		}

		if ((connected_first < 2) && (connected_second < 2)) {
			finaledges.push_back(e);
		}*/
	}

	cout << "FINAL Edges for this TSP:" << endl;
	for (auto& ee : finaledges) {
		cout << "\t" << "S" << ee->first->id << "-S" << ee->second->id << "[" << ee->weight << "]" << endl;
	}
	cout << endl;

	// CLOSE THE CIRCUIT
	for (auto& e : edges) {
		int connected_first = 0;
		int connected_second = 0;
		for (auto& fe : finaledges) {
			if ((e->first->id == fe->first->id) || (e->first->id == fe->second->id)) {
				connected_first++;
			}
			if ((e->second->id == fe->first->id) || (e->second->id == fe->second->id)) {
				connected_second++;
			}
		}
		if ((connected_first == 1) && (connected_second == 1)) {
			e->idTSP = finaledges.front()->idTSP;
			finaledges.push_back(e);
		}
	}

	Sensor *nextS = uavDummySensor;
	int counter = 500;
	cout << "TSP final: UAV" << cc->clusterUAV->id;
	do {
		//for (auto& fe : finaledges) {
		for (auto it_fe = finaledges.begin(); it_fe != finaledges.end(); it_fe++) {
			if (((*it_fe)->first->id == nextS->id) || ((*it_fe)->second->id == nextS->id)) {
				if ((*it_fe)->first->id == nextS->id){
					finalcircuit.push_back(new TSP2OptEdge(nextS, (*it_fe)->second, (*it_fe)->weight));
					nextS = (*it_fe)->second;
				}
				else {
					finalcircuit.push_back(new TSP2OptEdge(nextS, (*it_fe)->first, (*it_fe)->weight));
					nextS = (*it_fe)->first;
				}

				if (nextS->id != uavDummySensor->id) {
					cc->pointsTSP_listFinal.push_back(nextS);
					cout << "-S" << nextS->id;
				}
				finaledges.erase(it_fe);
				break;
			}
		}
	} while ((nextS != uavDummySensor) && ((counter--) > 0));
	cout << "-UAV" << cc->clusterUAV->id << endl;

	if (counter == 0) {
		cerr << "ERRORE" << endl;
		exit(EXIT_FAILURE);
	}

	//2-OPT improvement
	cout << " FINAL CIRCUIT1: ";
	for (auto& fe : finalcircuit) {
		cout << " S" << fe->first->id << "-S" << fe->second->id;
	}
	cout << endl;

	for (auto it_fe1 = finalcircuit.begin(); it_fe1 != finalcircuit.end(); it_fe1++){
		auto fe1 = *it_fe1;
	//for (auto& fe1 : finalcircuit) {
		auto it_fe2 = it_fe1;
		it_fe2++;
		for (; it_fe2 != finalcircuit.end(); it_fe2++){
		//for (auto& fe2 : finalcircuit) {
			auto fe2 = *it_fe2;
			if (fe1->second->id != fe2->first->id){
				double sumActCost = fe1->first->coord.distance(fe1->second->coord) + fe2->first->coord.distance(fe2->second->coord);
				double sumSwitch = fe1->first->coord.distance(fe2->first->coord) + fe1->second->coord.distance(fe2->second->coord);
				if (sumSwitch < sumActCost) {

					auto it_feSwitch = it_fe1;
					it_feSwitch++;
					while (it_feSwitch)cdscasvde

					Sensor *tmp = fe1->second;
					fe1->second = fe2->first;
					fe1->weight = fe1->first->coord.distance(fe1->second->coord);

					fe1->first = tmp;
					fe2->weight = fe2->first->coord.distance(fe2->second->coord);
				}
			}
		}
	}

	cc->pointsTSP_listFinal.clear();
	cout << " FINAL CIRCUIT2: ";
	for (auto& fe : finalcircuit) {
		if (fe->second->id != TSP_UAV_CODE) {
			cc->pointsTSP_listFinal.push_back(fe->second);
		}
		cout << " S" << fe->first->id << "-S" << fe->second->id;
	}
	cout << endl;
}
