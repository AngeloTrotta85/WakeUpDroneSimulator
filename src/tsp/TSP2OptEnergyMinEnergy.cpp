/*
 * TSP2Opt.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: angelo
 */

#include "TSP2OptEnergyMinEnergy.h"
#include "../Loss.h"
#include "../Generic.h"

using namespace std;

TSP2OptEnergyMinEnergy::TSP2OptEnergyMinEnergy() {
	// TODO Auto-generated constructor stub

}

void TSP2OptEnergyMinEnergy::calculateCosts1Edge(TSP2OptEnergyEdgeMinEnergy *e, bool forceWakeUp, double &time, double &energy, bool log) {
	//time = 0;
	//energy = 0;
	double moveTime, moveEnergy, wakeTime, wakeEnergy;

	moveTime = moveEnergy = wakeTime = wakeEnergy = 0;

	moveTime = ceil(Generic::getInstance().getTime2Travel(e->first->coord, e->second->coord) / Generic::getInstance().timeSlot) + 1;
	moveEnergy = moveTime * Generic::getInstance().timeSlot * Generic::getInstance().singleMotorPowerUAV * 4.0;

	if (log) {
		cout << "edge S" << e->first->id << "-S" << e->second->id << " TT:" << moveTime << " TE:" << moveEnergy;
	}

	if ((e->second->id != TSP_UAV_CODE) || forceWakeUp) {	// Intermediate arc
		wakeTime = ceil(Generic::getInstance().getTime2WakeRead(e->first->coord, e->second->coord) / Generic::getInstance().timeSlot) + 1;
		wakeEnergy = wakeTime * Generic::getInstance().timeSlot * Generic::getInstance().singleMotorPowerUAV * 4.0;
		wakeEnergy += Generic::getInstance().getEnergy2WakeRead(e->first->coord, e->second->coord);

		if (log) {
			cout << " WT:" << wakeTime << " WE:" << wakeEnergy << endl;
		}
	}

	time = moveTime + wakeTime;
	energy = moveEnergy + wakeEnergy;
}

void TSP2OptEnergyMinEnergy::calculateCosts(list<TSP2OptEnergyEdgeMinEnergy *> edgesTSP, double &time, double &energy, bool log) {
	time = 0;
	energy = 0;

	for (auto& e : edgesTSP) {
		double actTime, actEnergy;

		calculateCosts1Edge(e, false, actTime, actEnergy, log);

		time += actTime;
		energy += actEnergy;
	}
}

void TSP2OptEnergyMinEnergy::calculateTSP(CoordCluster *cc, std::list<Sensor *> &sl, int time_now) {
	double t, e;
	Sensor *uavDummySensor = new Sensor(cc->clusterUAV->recharge_coord, 1, TSP_UAV_CODE);

	cc->pointsTSP_listFinal.clear();

	cout << "Start calculating TSP at time: " << time_now << endl;

	list< pair<Sensor *, double> > allSensors;
	for (auto& s : cc->pointsList) {
		double c = s->residual_energy;
		allSensors.push_back(make_pair(s, c));
	}
	allSensors.sort(sortCosts);


	list<Sensor *> chosenSensors;
	list<TSP2OptEnergyEdgeMinEnergy *> chosenCircuit;

	chosenSensors.push_back(uavDummySensor);

	auto it_as = allSensors.begin();
	while (it_as != allSensors.end()) {
		auto as = *it_as;


		// create a temporary list with the chosen sensors + one
		list<Sensor *> tmpSensors;
		for (auto& tmpS : chosenSensors) {
			tmpSensors.push_back(tmpS);
		}
		tmpSensors.push_back(as.first);

		list<TSP2OptEnergyEdgeMinEnergy *> tmpcircuit;
		calculateTSP_subset(uavDummySensor, tmpSensors, tmpcircuit);
		calculateCosts(tmpcircuit, t, e, false);
		if (e < cc->clusterUAV->residual_energy) {
			//as.first->uavBookedReading[cc->clusterUAV->id] = true;

			chosenSensors.push_back(as.first);
			for (auto& ccf : chosenCircuit) free(ccf);
			chosenCircuit.clear();
			for (auto& tmpC : tmpcircuit) {
				chosenCircuit.push_back(tmpC);
			}
		}
		else {
			for (auto& tc : tmpcircuit) free(tc);
			tmpcircuit.clear();
			it_as++;
		}
	}

	//calculateCosts(chosenCircuit, t, e, true);
	//cout << "Calculated TSP for UAV" << cc->clusterUAV->id << " having residual energy " << cc->clusterUAV->residual_energy
	//		<< "J. Cost: time=" << t<< "sec. Cost: energy=" << e << "J" << std::endl;

	cout << "Choosen circuit: ";
	cc->pointsTSP_listFinal.clear();
	for (auto& fe : chosenCircuit) {
		if (fe->second->id != TSP_UAV_CODE) {
			cc->pointsTSP_listFinal.push_back(fe->second);
			cout << "[S" << fe->second->id << " " << fe->second->coord << "]";
		}
	}
	cout << endl << endl;

	for (auto& ccf : chosenCircuit) free(ccf);
	chosenCircuit.clear();
	allSensors.clear();
	chosenSensors.clear();
}

void TSP2OptEnergyMinEnergy::calculateTSP_subset(Sensor *uavDummySensor , list<Sensor *> &sList, list<TSP2OptEnergyEdgeMinEnergy *> &fCircuit) {
	list<TSP2OptEnergyEdgeMinEnergy *> edges;
	list<TSP2OptEnergyEdgeMinEnergy *> finaledges;
	list<TSP2OptEnergyEdgeMinEnergy *> finalcircuit;
	//cout << endl << "Calculating TSP for UAV" << cc->clusterUAV->id << ". The sensors are:";
	//for (auto& ss : cc->pointsList) {
	//	cout << " S" << ss->id;
	//}
	//cout << endl;

	auto it1 = sList.begin();
	while (it1 != sList.end()) {
		auto it2 = it1;
		it2++;
		while (it2 != sList.end()) {
			double t;
			TSP2OptEnergyEdgeMinEnergy *ne = new TSP2OptEnergyEdgeMinEnergy(*it1, *it2, 0);

			calculateCosts1Edge(ne, true, t, ne->weight, false);
			edges.push_back(ne);

			it2++;
		}
		it1++;
	}

	edges.sort(TSP2OptEnergyEdgeMinEnergy::sortEdges);

	//cout << "Edges for this TSP:" << endl;
	//for (auto& ee : edges) {
	//	cout << "\t" << "S" << ee->first->id << "-S" << ee->second->id << "[" << ee->weight << "]" << endl;
	//}
	//cout << endl;

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

		/*cout << "Checking S" << e->first->id << "-S" << e->second->id << "."
				<< " ConnFirst: " << connected_first << " ConnSecond: " << connected_second
				<< " idFirst: " << idFirst << " idSecond: " << idSecond
				<< endl;*/

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
	}

	//cout << "FINAL Edges for this TSP:" << endl;
	//for (auto& ee : finaledges) {
	//	cout << "\t" << "S" << ee->first->id << "-S" << ee->second->id << "[" << ee->weight << "]" << endl;
	//}
	//cout << endl;

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
	//cout << "TSP final: UAV" << cc->clusterUAV->id;
	do {
		//for (auto& fe : finaledges) {
		for (auto it_fe = finaledges.begin(); it_fe != finaledges.end(); it_fe++) {
			if (((*it_fe)->first->id == nextS->id) || ((*it_fe)->second->id == nextS->id)) {
				if ((*it_fe)->first->id == nextS->id){
					finalcircuit.push_back(new TSP2OptEnergyEdgeMinEnergy(nextS, (*it_fe)->second, (*it_fe)->weight));
					nextS = (*it_fe)->second;
				}
				else {
					finalcircuit.push_back(new TSP2OptEnergyEdgeMinEnergy(nextS, (*it_fe)->first, (*it_fe)->weight));
					nextS = (*it_fe)->first;
				}

				//if (nextS->id != uavDummySensor->id) {
					//cout << "-S" << nextS->id;
				//}
				finaledges.erase(it_fe);
				break;
			}
		}
	} while ((nextS != uavDummySensor) && ((counter--) > 0));
	//cout << "-UAV" << cc->clusterUAV->id << endl;

	if (counter == 0) {
		cerr << "ERRORE" << endl;
		exit(EXIT_FAILURE);
	}

	//2-OPT improvement
	//cout << " FINAL CIRCUIT1: ";
	//for (auto& fe : finalcircuit) {
	//	cout << " S" << fe->first->id << "-S" << fe->second->id;
	//}
	//cout << endl;

	bool swapMade = false;
	int round = 50;
	do {
		swapMade = false;
		for (auto it_fe1 = finalcircuit.begin(); it_fe1 != finalcircuit.end(); it_fe1++){
			auto fe1 = *it_fe1;

			auto it_fe2 = it_fe1;
			if (it_fe2 != finalcircuit.end()) it_fe2++;
			if (it_fe2 != finalcircuit.end()) it_fe2++; //make 2-steps ahead

			for (; it_fe2 != finalcircuit.end(); it_fe2++){
				auto fe2 = *it_fe2;
				if (fe1->second->id != fe2->first->id){
					double sumActCost = fe1->first->coord.distance(fe1->second->coord) + fe2->first->coord.distance(fe2->second->coord);
					double sumSwitch = fe1->first->coord.distance(fe2->first->coord) + fe1->second->coord.distance(fe2->second->coord);
					if (sumSwitch < sumActCost) {
						list<TSP2OptEnergyEdgeMinEnergy *> tmpcircuit;
						Sensor *tmp;

						//cout << endl << "TEMP CIRCUIT BEFORE: ";
						//for (auto& fe : finalcircuit) cout << " S" << fe->first->id << "-S" << fe->second->id;
						//cout << endl;
						//cout << "Swapping S" << fe1->first->id << "-S" << fe1->second->id << " with S" << fe2->first->id << "-S" << fe2->second->id << endl;

						auto it_feSwitch = it_fe1;
						it_feSwitch++;
						while ((*it_feSwitch)->second->id != (*it_fe2)->second->id){
							tmp = (*it_feSwitch)->second;
							(*it_feSwitch)->second = (*it_feSwitch)->first;
							(*it_feSwitch)->first = tmp;

							it_feSwitch++;
						}

						tmp = fe1->second;
						fe1->second = fe2->first;
						fe1->weight = fe1->first->coord.distance(fe1->second->coord);

						fe2->first = tmp;
						fe2->weight = fe2->first->coord.distance(fe2->second->coord);

						auto it_b = it_fe1; it_b++;
						auto it_e = it_fe2; //it_e--;
						reverse(it_b,  it_e);

						//cout << "TEMP CIRCUIT AFTER : ";
						//for (auto& fe : finalcircuit) cout << " S" << fe->first->id << "-S" << fe->second->id;
						//cout << endl << endl;

						swapMade = true;
					}
				}

				if (swapMade) break;
			}
			if (swapMade) break;
		}

		//if (swapMade) cout << "SWAP-MADE" << endl;
		//else cout << "NO SWAP-MADE" << endl;

	} while (swapMade && ((round--) > 0));

	//cout << " FINAL CIRCUIT2: ";
	for (auto& fe : finalcircuit) {
		fCircuit.push_back(fe);
		//cout << " S" << fe->first->id << "-S" << fe->second->id;
	}
	//cout << endl;

	for (auto& ef : edges) free(ef);
	edges.clear();

	for (auto& ff : finaledges) free(ff);
	finaledges.clear();
}
