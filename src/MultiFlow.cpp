/*
 * MultiFlow.cpp
 *
 *  Created on: May 16, 2019
 *      Author: angelo
 */

#include "MultiFlow.h"
#include "Generic.h"
#include "RandomGenerator.h"
#include "Loss.h"

MultiFlow::MultiFlow(Algo_type at) {
	actSensorTimeStamp = 0;
	actUAVTimeStamp = 0;

	pWU = 0;
	mfAlgoType = at;
}

void MultiFlow::addSensor(Sensor *s) {
	SensorNode *newsn = new SensorNode();

	newsn->sens = s;
	newsn->lastTimestamp = 0;
	newsn->lastTimestamp_tslot = 0;
	newsn->accumulatedEnergy_uJ = 0;
	newsn->irradiatingTimeSlots = 0;
	newsn->startupTimeSlots = 0;
	newsn->commTimeSlots = 0;
	newsn->nCommAttempt = 0;
	newsn->irradiatingUAV = nullptr;

	sens_list.push_back(newsn);

	sens_map[s->id] = newsn;
}

void MultiFlow::addChargStationAndUAV(MyCoord c, UAV *u) {
	ChargingNode *newcs = new ChargingNode();

	newcs->id = u->id;
	newcs->pos = c;
	newcs->u = u;
	newcs->lastTimestamp = 0;
	newcs->lastTimestamp_tslot = 0;

	cs_map[u->id] = newcs;
}

void MultiFlow::addChargStationAndUAV_distributed(MyCoord c, UAV *u) {
	ChargingNode *newcs = new ChargingNode();

	newcs->id = u->id;
	newcs->pos = c;
	newcs->u = u;
	newcs->lastTimestamp = 0;
	newcs->lastTimestamp_tslot = 0;

	cs_map[u->id] = newcs;

	UavDistributed *newuav = new UavDistributed();
	newuav->cn = newcs;

	uav_list.push_back(newuav);
}

ChargingNode *MultiFlow::getLeftMostUAV(int end_time) {
	ChargingNode *ris = cs_map.begin()->second;
	int risTime = ris->lastTimestamp;

	//cerr << "Looking for the next UAV among ";
	//cerr << " U" << ris->u->id << "(" << ris->lastTimestamp << ")";

	for (auto& cs : cs_map) {
		//cerr << " U" << cs.second->u->id << "(" << cs.second->lastTimestamp << ")";

		if (cs.second->lastTimestamp < ris->lastTimestamp) {
			ris = cs.second;
			risTime = ris->lastTimestamp;
		}
	}
	if (risTime >= end_time) {
		//cerr << " - time best: " << risTime << " EXCEED time " << end_time;
		ris = nullptr;
	}
	//cerr << endl;
	return ris;
}

double MultiFlow::calcPowEta(int t) {
	return 0;
}

double MultiFlow::calcPowEtaSens(double e, double t) {

	long double selfDischargeRatio = (100.0 - Generic::getInstance().sensorBatterySelfDischarge) / 100.0;
	long double slotsPerMonth = (30.0 * 24.0 * 60.0 * 60.0) / Generic::getInstance().timeSlot;

	long double sensorSelfDischargePerSlot = powl(selfDischargeRatio, 1.0 / slotsPerMonth);

	return (e * (1.0 - sensorSelfDischargePerSlot));
}

double MultiFlow::energy_loss_onArc(int tstart) {
	double ris = 0;
	int tend = tstart + Generic::getInstance().tstartup + (Generic::getInstance().nr * Generic::getInstance().ttimeout);

	ris += calcPowEta(tend) * Generic::getInstance().timeSlot * (tend - tstart);

	double estartup = Generic::getInstance().pSstartup * Generic::getInstance().timeSlot * Generic::getInstance().tstartup;
	double ecomm = (Generic::getInstance().pSrx + ((Generic::getInstance().pStx - Generic::getInstance().pSrx) / Generic::getInstance().ttimeout))
			* Generic::getInstance().timeSlot * Generic::getInstance().nr * Generic::getInstance().ttimeout;
	ris += pWU * (estartup + ecomm);

	return ris;
}

double MultiFlow::sensor_energy_loss_read(double pwu) {
	double estartup = Generic::getInstance().pSstartup * Generic::getInstance().timeSlot * Generic::getInstance().tstartup;
	double ecomm = (Generic::getInstance().pSrx + ((Generic::getInstance().pStx - Generic::getInstance().pSrx) / Generic::getInstance().ttimeout))
						* Generic::getInstance().timeSlot * Generic::getInstance().nr * Generic::getInstance().ttimeout;

	return (pwu * (estartup + ecomm));
}

bool MultiFlow::updateSensorsEnergy(int starttime, int endtime) {
	bool ris = true;
	long double minEnergy = 100000;
	SensorNode *snMin = nullptr;

	for (auto& s : sens_list) {
		// update the energy with the self-discharge
		for (int i = (starttime+1); i <= endtime; i++) {
			//s->sens->residual_energy -= calcPowEtaSens(s->sens->residual_energy) * Generic::getInstance().timeSlot;
			s->sens->residual_energy -= calcPowEtaSens(s->sens->residual_energy, Generic::getInstance().timeSlot);
		}

		for (auto& l : s->readings) {
			if ((l.readTime > starttime) && (l.readTime <= endtime)) {
				if (s->lastTimestamp < l.readTime) {
					s->lastTimestamp = l.readTime;
				}

				// update the energy because of the reading
				double pwu = 1;	//TODO
				s->sens->residual_energy -= sensor_energy_loss_read(pwu);

				/*if (s->lastTimestamp == l.readTime) {
					s->lastTimestamp += Generic::getInstance().tstartup + (Generic::getInstance().nr * Generic::getInstance().ttimeout);
					s->sens->residual_energy -= energy_loss_onArc(l.readTime);
				}
				else {
					cerr << "Error in updateSensorsEnergy" << endl;
					exit(EXIT_FAILURE);
				}*/
			}
		}

		if (s->sens->residual_energy < minEnergy) {
			minEnergy = s->sens->residual_energy;
			snMin = s;
		}

		if (s->sens->residual_energy <= 0) {
			ris = false;
		}
	}

	if (snMin != nullptr) {
		cout << "SENSOR S" << snMin->sens->id << " has minimal energy of " << snMin->sens->residual_energy << endl;
	}

	/*cout << "SENSORS ";
	cout << "- Eta: " << calcPowEtaSens((*sens_list.begin())->sens->residual_energy, Generic::getInstance().timeSlot);
	cout << "- ReadLoss: " << sensor_energy_loss_read(1) << " - ";
	for (auto& s : sens_list) {
		cout << "S" << s->sens->id << "(" << s->sens->residual_energy << ") ";
	}
	cout << endl;*/

	return ris;
}

double MultiFlow::calcTimeToTravel(MyCoord p1, MyCoord p2) {
	double ttt = p1.distance(p2) / Generic::getInstance().maxVelocity;
	return (ceil(ttt / Generic::getInstance().timeSlot) * Generic::getInstance().timeSlot);
}
double MultiFlow::calcEnergyToTravel(MyCoord p1, MyCoord p2) {
	int ttt = calcTimeToTravel(p1, p2);
	return (ttt * Generic::getInstance().pUfly);
}

double MultiFlow::calcTimeToWuData(void) {
	double ris = (Generic::getInstance().tstartup + (Generic::getInstance().ttimeout * Generic::getInstance().nr));

	return (ceil(ris / Generic::getInstance().timeSlot) * Generic::getInstance().timeSlot);
}
double MultiFlow::calcEnergyToWuData(double pWU) {
	double ttwu = calcTimeToWuData();
	return ((ttwu * Generic::getInstance().pUfly)
			+ ( pWU * ( (	Generic::getInstance().pUstartup * Generic::getInstance().tstartup ) +
					(Generic::getInstance().ttimeout * Generic::getInstance().nr *
							(Generic::getInstance().pUrx + ((Generic::getInstance().pUtx - Generic::getInstance().pUrx) /
									ceil(Generic::getInstance().ttimeout / Generic::getInstance().timeSlot))) ) ) ) );
}

void MultiFlow::activateTSPandRecharge(ChargingNode *cnode, list<SensorNode *> &tsp) {
	double actTime = cnode->lastTimestamp;
	MyCoord uav_pos = cnode->pos;
	double uav_energy = cnode->u->max_energy;

	cerr << "Updating the UAV" << cnode->u->id
			<< " starting from time " << cnode->lastTimestamp << " at position " << uav_pos
			<< " with energy: " << cnode->u->max_energy
			<< endl;

	cerr << "TSP: C" << cnode->u->id << "(" << actTime << ") ";
	for (auto& s : tsp) {
		// calculate time to travel
		//double ttt = s->sens->coord.distance(uav_pos) / Generic::getInstance().maxVelocity;
		//uav_energy -= ttt * Generic::getInstance().pUfly;
		//actTime += ((int) (ceil(ttt / Generic::getInstance().timeSlot)));
		actTime += calcTimeToTravel(uav_pos, s->sens->coord);
		uav_energy -= calcEnergyToTravel(uav_pos, s->sens->coord);

		//move the UAV
		uav_pos = s->sens->coord;

		//calculate time to wake-up and gather the data
		//double ttwu = Generic::getInstance().tstartup + (Generic::getInstance().ttimeout * Generic::getInstance().nr);
		double pWU = 1; //TODO
		//uav_energy -= 	(ttwu * Generic::getInstance().timeSlot * Generic::getInstance().pUfly)
		//				+ ( pWU * ( (	Generic::getInstance().pUstartup * Generic::getInstance().timeSlot * Generic::getInstance().tstartup ) +
		//						(Generic::getInstance().timeSlot * Generic::getInstance().ttimeout * Generic::getInstance().nr *
		//								(Generic::getInstance().pUrx + ((Generic::getInstance().pUtx - Generic::getInstance().pUrx) / Generic::getInstance().ttimeout)) ) ) );
		//actTime += ttwu;
		actTime += calcTimeToWuData();
		uav_energy -= calcEnergyToWuData(pWU);

		//set the reading on the sensor
		//double pREAD = 1; //TODO
		//if (RandomGenerator::getInstance().getRealUniform(0, 1) <= pREAD) {
		SensorNode::SensorRead sr;
		sr.readTime = actTime;
		sr.uav = cnode->u;
		s->readings.push_back(sr);
		//}
		cerr << "S" << s->sens->id << "(" << actTime << ") ";

		if (actTime > s->lastTimestamp) {
			s->lastTimestamp = actTime;		// a cosa serve s->lastTimestamp?
		}
	}

	if (uav_pos != cnode->pos) {
		// calculate time to travel
		//double ttt = cnode->pos.distance(uav_pos) / Generic::getInstance().maxVelocity;
		//uav_energy -= ttt * Generic::getInstance().pUfly;
		//actTime += ((int) (ceil(ttt / Generic::getInstance().timeSlot)));

		actTime += calcTimeToTravel(uav_pos, cnode->pos);
		uav_energy -= calcEnergyToTravel(uav_pos, cnode->pos);

		//move the UAV
		uav_pos = cnode->pos;

		cerr << "C" << cnode->u->id << "(" << actTime << ") " << endl;
	}

	double tspTime = actTime;

	// calculate time to recharge
	double e2recharge = cnode->u->max_energy - max(uav_energy, 0.0);
	//actTime += ((int) (ceil(e2recharge / Generic::getInstance().rechargeStation_power)));
	actTime += e2recharge / Generic::getInstance().rechargeStation_power;

	cnode->lastTimestamp = actTime;

	cerr << "Updated the UAV" << cnode->u->id << " arriving at time " << tspTime
			<< " finishing recharging at time " << cnode->lastTimestamp
			<< " at position " << uav_pos
			<< " with residual energy " << uav_energy << " energy"
			<< ", i.e. consuming " << e2recharge << " energy"
			<< endl;
}

double MultiFlow::calcLossSensorOriginal(SensorNode *s_check, std::list<SensorNode *> &sList, int texp) {
	double ris = 0;

	for (auto& s : sList) {
		for (auto& r : s->readings) {
			if (r.readTime < texp) {
				double actLoss = Loss::getInstance().calculate_loss_distance(s->sens, s_check->sens)
						* Loss::getInstance().calculate_loss_time(r.readTime, texp);

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
	}

	return ris;
}

double MultiFlow::calcLossSensor(SensorNode *s_check, std::list<SensorNode *> &sList, int texp) {
	double ris = 0;

	std::list<Sensor *> ssList;
	for (auto& sn : sList){
		ssList.push_back(sn->sens);
	}

	for (auto& s : sList) {
		for (auto& r : s->readings) {
			if (r.readTime < texp) {
				double actLoss = Loss::algebraic_sum(Loss::getInstance().calculate_loss_distance(s->sens, s_check->sens)
						* Loss::getInstance().calculate_loss_time(r.readTime, texp),
						Loss::getInstance().calculate_loss_energy(s_check->sens, texp, ssList));

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
	}

	return ris;
}

double MultiFlow::calcLossSensorPresentFuture(SensorNode *s_check, std::list<SensorNode *> &sList, int texp) {
	double ris = 0;

	std::list<Sensor *> ssList;
	for (auto& sn : sList){
		ssList.push_back(sn->sens);
	}

	for (auto& s : sList) {
		for (auto& r : s->readings) {
			double actLoss = Loss::algebraic_sum(Loss::getInstance().calculate_loss_distance(s->sens, s_check->sens)
								* Loss::getInstance().calculate_loss_time(r.readTime, texp),
								Loss::getInstance().calculate_loss_energy(s_check->sens, texp, ssList));

			if (actLoss > ris) {
				ris = actLoss;
			}
		}
	}

	return ris;
}

SensorNode *MultiFlow::getMinLossSensor(list<SensorNode *> &sList, int texp) {
	SensorNode *ris = nullptr;
	double minLoss = std::numeric_limits<double>::max();
	//double minLoss = -1000;

	//cerr << "Calculating minLossSensor with Texp: " << texp << " - ";
	for (auto& s : sList) {
		//double actLoss = calcLossSensor(s, sens_list, texp);
		double actLoss = calcLossSensorPresentFuture(s, sens_list, texp);
		//cerr << "S" << s->sens->id << ":" << actLoss << " ";
		if (actLoss < minLoss) {
			minLoss = actLoss;
			ris = s;
		}
	}
	//cerr << " -> winner: S" << ris->sens->id << endl;

	return ris;
}

double MultiFlow::calculateCosts1Edge(TSP2MultiFlow *e) {
	if (	((e->first->sens->id == TSP_DUMMY_CODE_START_MF) && (e->second->sens->id == TSP_DUMMY_CODE_END_MF)) ||
			((e->second->sens->id == TSP_DUMMY_CODE_START_MF) && (e->first->sens->id == TSP_DUMMY_CODE_END_MF)) ){
		return (-1);
	}
	else {
		double ris = 0;
		double pwu = 1;	//TODO

		ris += calcEnergyToTravel(e->first->sens->coord, e->second->sens->coord);	//time to travel
		ris += calcEnergyToWuData(pwu);	//time to wake-up

		return ris;
	}
}

double MultiFlow::calculateCosts1Edge(SensorNode *s1, SensorNode *s2) {
	double ris = 0;
	double pwu = 1;	//TODO

	ris += calcEnergyToTravel(s1->sens->coord, s2->sens->coord);	//time to travel
	ris += calcEnergyToWuData(pwu);	//time to wake-up

	return ris;
}

void MultiFlow::calculateTSP_incremental(list<SensorNode *> &newTSP, list<SensorNode *> &actTSP,
		SensorNode *sj, ChargingNode *cnode, double &tsp_time, double &tsp_energy_cost) {
	double pwu = 1; //TODO

	if (actTSP.empty()) {
		newTSP.push_back(sj);
		tsp_time = calcTimeToTravel(cnode->pos, sj->sens->coord) + calcTimeToTravel(sj->sens->coord, cnode->pos);
		tsp_energy_cost = 	calcEnergyToTravel(cnode->pos, sj->sens->coord) +
				calcEnergyToWuData(pwu) +
				calcEnergyToTravel(sj->sens->coord, cnode->pos);
	}
	else {
		Sensor *uavDummySensor = new Sensor(cnode->pos, 1, TSP_UAV_CODE_MF);
		SensorNode uavDummySensorNode;
		list<SensorNode *> allSens;
		list<TSP2MultiFlow *> edges;
		list<TSP2MultiFlow *> finaledges;
		list<TSP2MultiFlow *> finalcircuit;

		uavDummySensorNode.sens = uavDummySensor;
		uavDummySensorNode.lastTimestamp = cnode->lastTimestamp;

		for (auto& s : actTSP) {
			allSens.push_back(s);
		}
		allSens.push_back(sj);
		allSens.push_back(&uavDummySensorNode);

		auto it1 = allSens.begin();
		while (it1 != allSens.end()) {
			auto it2 = it1;
			it2++;
			while (it2 != allSens.end()) {
				//double t;
				TSP2MultiFlow *ne = new TSP2MultiFlow(*it1, *it2, 0);

				ne->weight = calculateCosts1Edge(ne);
				edges.push_back(ne);

				it2++;
			}
			it1++;
		}
		edges.sort(TSP2MultiFlow::sortEdges);

		int idFinal = 0;
		for (auto& e : edges) {
			//check if the sensors in the edge are already caught
			int connected_first = 0;
			int connected_second = 0;
			int idFirst = -1;
			int idSecond = -2;

			for (auto& fe : finaledges) {
				if ((e->first->sens->id == fe->first->sens->id) || (e->first->sens->id == fe->second->sens->id)) {
					connected_first++;
					idFirst = fe->idTSP;
				}
				if ((e->second->sens->id == fe->first->sens->id) || (e->second->sens->id == fe->second->sens->id)) {
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

		// CLOSE THE CIRCUIT
		for (auto& e : edges) {
			int connected_first = 0;
			int connected_second = 0;
			for (auto& fe : finaledges) {
				if ((e->first->sens->id == fe->first->sens->id) || (e->first->sens->id == fe->second->sens->id)) {
					connected_first++;
				}
				if ((e->second->sens->id == fe->first->sens->id) || (e->second->sens->id == fe->second->sens->id)) {
					connected_second++;
				}
			}
			if ((connected_first == 1) && (connected_second == 1)) {
				e->idTSP = finaledges.front()->idTSP;
				finaledges.push_back(e);
			}
		}

		SensorNode *nextS = &uavDummySensorNode;
		int counter = 500;
		//cout << "TSP final: UAV" << cc->clusterUAV->id;
		do {
			//for (auto& fe : finaledges) {
			for (auto it_fe = finaledges.begin(); it_fe != finaledges.end(); it_fe++) {
				if (((*it_fe)->first->sens->id == nextS->sens->id) || ((*it_fe)->second->sens->id == nextS->sens->id)) {
					if ((*it_fe)->first->sens->id == nextS->sens->id){
						finalcircuit.push_back(new TSP2MultiFlow(nextS, (*it_fe)->second, (*it_fe)->weight));
						nextS = (*it_fe)->second;
					}
					else {
						finalcircuit.push_back(new TSP2MultiFlow(nextS, (*it_fe)->first, (*it_fe)->weight));
						nextS = (*it_fe)->first;
					}

					//if (nextS->id != uavDummySensor->id) {
					//cout << "-S" << nextS->id;
					//}
					finaledges.erase(it_fe);
					break;
				}
			}
		} while ((nextS != &uavDummySensorNode) && ((counter--) > 0));

		if (counter == 0) {
			cerr << "ERRORE" << endl;
			exit(EXIT_FAILURE);
		}

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
					if (fe1->second->sens->id != fe2->first->sens->id){
						//double sumActCost = fe1->first->sens->coord.distance(fe1->second->sens->coord) + fe2->first->sens->coord.distance(fe2->second->sens->coord);
						double sumActCost = calculateCosts1Edge(fe1->first, fe1->second) + calculateCosts1Edge(fe2->first, fe2->second);
						//double sumSwitch = fe1->first->sens->coord.distance(fe2->first->sens->coord) + fe1->second->sens->coord.distance(fe2->second->sens->coord);
						double sumSwitch = calculateCosts1Edge(fe1->first, fe2->first) + calculateCosts1Edge(fe1->second, fe2->second);
						if (sumSwitch < sumActCost) {
							list<TSP2MultiFlow *> tmpcircuit;
							SensorNode *tmp;

							//cout << endl << "TEMP CIRCUIT BEFORE: ";
							//for (auto& fe : finalcircuit) cout << " S" << fe->first->id << "-S" << fe->second->id;
							//cout << endl;
							//cout << "Swapping S" << fe1->first->id << "-S" << fe1->second->id << " with S" << fe2->first->id << "-S" << fe2->second->id << endl;

							auto it_feSwitch = it_fe1;
							it_feSwitch++;
							while ((*it_feSwitch)->second->sens->id != (*it_fe2)->second->sens->id){
								tmp = (*it_feSwitch)->second;
								(*it_feSwitch)->second = (*it_feSwitch)->first;
								(*it_feSwitch)->first = tmp;

								it_feSwitch++;
							}

							tmp = fe1->second;
							fe1->second = fe2->first;
							//fe1->weight = fe1->first->sens->coord.distance(fe1->second->sens->coord);
							fe1->weight = calculateCosts1Edge(fe1->first, fe1->second);

							fe2->first = tmp;
							//fe2->weight = fe2->first->sens->coord.distance(fe2->second->sens->coord);
							fe2->weight = calculateCosts1Edge(fe2->first, fe2->second);

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
		tsp_time = 0;
		tsp_energy_cost = 0;
		for (auto& fe : finalcircuit) {
			//fCircuit.push_back(fe);
			if (fe->second->sens->id != TSP_UAV_CODE_MF) {
				newTSP.push_back(fe->second);

				tsp_time += calcTimeToWuData();
				tsp_energy_cost += calcEnergyToWuData(pwu);
			}
			tsp_time += calcTimeToTravel(fe->first->sens->coord, fe->second->sens->coord);
			tsp_energy_cost += calcEnergyToTravel(fe->first->sens->coord, fe->second->sens->coord);

			//cout << " S" << fe->first->id << "-S" << fe->second->id;
		}
		//cout << endl;

		for (auto& ef : edges) free(ef);
		edges.clear();

		//for (auto& ff : finaledges) free(ff);
		finaledges.clear();

		for (auto& ff : finalcircuit) free(ff);
		finalcircuit.clear();

		free(uavDummySensor);
	}
}

void MultiFlow::calculateTSP_and_UpdateMF(ChargingNode *leftmost) {
	int tk = leftmost->lastTimestamp;
	double tsp_time = leftmost->u->residual_energy / Generic::getInstance().pUfly;
	//double tsp_cost = 0;
	list<SensorNode *> sens_check;
	list<SensorNode *> actTSP;

	for (auto& s : sens_list) {
		sens_check.push_back(s);
	}

	while (!sens_check.empty()) {
		list<SensorNode *> newTSP;
		double energy_cost = std::numeric_limits<double>::max();
		double t_time_tmp = 0;

		double t_exp = tk + (tsp_time / 2.0);
		SensorNode *sj = getMinLossSensor(sens_check, t_exp);

		/*cerr << "Calculating TSP with U" << leftmost->u->id << " ";
		for(auto& s : actTSP) {
			cerr << "S" << s->sens->id << " ";
		}
		cerr << "NS" << sj->sens->id << " ";
		cerr << endl;*/

		calculateTSP_incremental(newTSP, actTSP, sj, leftmost, t_time_tmp, energy_cost);

		if (energy_cost <= leftmost->u->residual_energy) {//Generic::getInstance().initUAVEnergy) {
			actTSP.clear();
			for (auto& s : newTSP) {
				actTSP.push_back(s);
			}
			tsp_time = t_time_tmp;
			//tsp_cost = t_cost;

			/*cerr << "Calculated NEW TSP: " << endl << "C" << leftmost->u->id << " ";
			for (auto& s : newTSP) {
				cerr << "S" << s->sens->id << " ";
			}
			cerr << "C" << leftmost->u->id << " ";
			cerr << "- with time cost " << t_time_tmp
					<< " and with energy cost " << energy_cost
					<< " having energy " << leftmost->u->residual_energy
					<< endl << endl;*/
		}

		sens_check.remove(sj);
	}

	cerr << "Final TSP: " << "C" << leftmost->u->id << " ";
	for (auto& s : actTSP) {
		cerr << "S" << s->sens->id << " ";
	}
	cerr << "C" << leftmost->u->id << " with TSP-time of " << tsp_time << endl;

	activateTSPandRecharge(leftmost, actTSP);
}

/*
double MultiFlow::getPDF_Eloc(MyCoord e) {
	double meanGPS = 0;
	double sigmaGPS = 2;
	double meanPilot = 0;
	double sigmaPilot = 1;

	return RandomGenerator::get_PDF_normal(e.length(), meanGPS + meanPilot, sigmaGPS + sigmaPilot);
}

double MultiFlow::getPDF_Erot(MyCoord r) {
	double mean = 0;
	double sigma = 0.3;

	return RandomGenerator::get_PDF_normal(r.x, mean, sigma) * RandomGenerator::get_PDF_normal(r.y, mean, sigma);
}*/

double MultiFlow::getPDF_Eloc_single(double e) {
	double meanGPS = 0;
	double sigmaGPS = Generic::getInstance().sigmaGPS;
	double meanPilot = 0;
	double sigmaPilot = Generic::getInstance().sigmaPilot;

	return RandomGenerator::get_PDF_normal(e, meanGPS + meanPilot, sigmaGPS + sigmaPilot);
}

double MultiFlow::getPDF_Erot_single(double r) {
	double mean = 0;
	double sigma = Generic::getInstance().sigmaRot;

	return RandomGenerator::get_PDF_normal(r, mean, sigma);
}

double MultiFlow::calc_d2D_max(double h, double alpha_max) {
	double angle_h = M_PI_2 - (alpha_max / 2.0);
	double ris_sens = ((h/sin(angle_h)) * sin(alpha_max / 2.0));
	double ptx_dbm = 10.0 * log10(1000.0 * Generic::getInstance().wakeupTxPower);

	double d3D_max = pow(10.0,
			(ptx_dbm + Generic::getInstance().gUmax + Generic::getInstance().gSmax +
					(20.0*log10(Generic::getInstance().wakeupTxFrequency) -27.55) - Generic::getInstance().wakeupTxMinPower)/20.0);
	double ris_uav = sqrt(pow(d3D_max, 2.0) - pow(h, 2.0));

	cout << "Calculating d2D from ris_sens: " << ris_sens << " and ris_uav: " << ris_uav << " ris: " << min(ris_sens, ris_uav) << endl;

	//cout << "min: " << min(ris, 40.0) << endl;

	return min(ris_sens, ris_uav);
}

double MultiFlow::calcProb_EReceived_new(double h, double e) {
	double ris = 0;
	double distDiv = 50.0;
	double rotDiv = 20.0;

	double d2Dmax = calc_d2D_max(h, Generic::getInstance().alphaSmax);
	double deltad = d2Dmax/distDiv;

	double rmax = M_PI / 3.0;
	double deltar = rmax/rotDiv;

	double sumAll = 0;

	for (double x = -d2Dmax; x <= (d2Dmax + 0.001); x+=deltad) {
		for (double y = -d2Dmax; y <= (d2Dmax + 0.001); y+=deltad) {
			if (MyCoord(x, y).length() <= d2Dmax) {
				for (double rx = -rmax; rx <= (rmax + 0.001); rx+=deltar) {
					for (double ry = -rmax; ry <= (rmax + 0.001); ry+=deltar) {
						//cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry  << endl;
						double g = calc_Gamma(x, y, rx, ry);
						//cout << "deltad: " << deltad << " deltar:" << deltar <<
						//		" -> gamma: " << g << "; e: " << e << endl << endl;

						if (e == 0) {
							//cout << "E = 0" << endl;
							if (g == e) {
								double actProb = getPDF_Eloc_single(x) * getPDF_Eloc_single(y) * getPDF_Erot_single(rx) * getPDF_Erot_single(ry);
								//cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry;
								//cout << " -> gamma: " << g << "; e: " << e;
								//cout << " --> totProb = " << actProb << endl;
								sumAll += actProb;
							}
						}
						else {
							/*if (g > 0) {
								cout << "Calculated gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry;
								cout << " - deltad: " << deltad << " deltar:" << deltar <<
										" -> gamma: " << g << "; e: " << e << endl;
							}*/
							double closeoffset = e / 50.0;//1.0e-9 * Generic::getInstance().timeSlot;
							if (MyCoord::close(g, e, closeoffset)) {

								cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry;
								cout << " - deltad: " << deltad << " deltar:" << deltar <<
										" -> gamma: " << g << "; e: " << e << endl;

								cout << "they are close!!!" << endl;
								double actProb = getPDF_Eloc_single(x) * getPDF_Eloc_single(y) * getPDF_Erot_single(rx) * getPDF_Erot_single(ry);
								cout << "Calculating gamma --> totProb = " << actProb << endl;
								sumAll += actProb;
							}
						}
					}
				}
			}
		}
	}

	cout << "Sum all Prob = " << sumAll << " using"
				<< " d2Dmax: " << d2Dmax << " rmax:" << rmax
				<< " deltad: " << deltad << " deltar:" << deltar
				<< " --> ris = " << (deltad * deltad * deltar * deltar * sumAll) << endl;

	//if (e == 0)
	exit(0);

	ris = deltad * deltad * deltar * deltar * sumAll;

	return ris;
}

double MultiFlow::calcProb_EReceived(double h, double e) {
	//double ris = 0;

	double d2Dmax = calc_d2D_max(h, Generic::getInstance().alphaSmax);
	double deltad = d2Dmax/50.0;

	double rmax = M_PI_2;
	double deltar = rmax/20.0;

	/*double sumX = 0;
	double sumY = 0;
	double sumRX = 0;
	double sumRY = 0;

	for (double x = -d2Dmax; x <= (d2Dmax + 0.001); x+=deltad) {
		sumX += getPDF_Eloc_single(x);
	}
	for (double y = -d2Dmax; y <= (d2Dmax + 0.001); y+=deltad) {
		sumY += getPDF_Eloc_single(y);
	}
	for (double rx = -rmax; rx <= (rmax + 0.001); rx+=deltar) {
		sumRX += getPDF_Erot_single(rx);
	}
	for (double ry = -rmax; ry <= (rmax + 0.001); ry+=deltar) {
		sumRY += getPDF_Erot_single(ry);
	}

	cout << "deltad: " << deltad << " deltar:" << deltar <<
			" -> sumX: " << sumX << "; sumY: " << sumY << "; sumRX: " << sumRX << "; sumRY: " << sumRY  << endl;

	return (deltad * sumX * deltad * sumY * deltar * sumRX * deltar * sumRY);
	*/

	double sumAll = 0;

	for (double x = -d2Dmax; x <= (d2Dmax + 0.001); x+=deltad) {
		for (double y = -d2Dmax; y <= (d2Dmax + 0.001); y+=deltad) {
			if (MyCoord(x, y).length() <= d2Dmax) {
				for (double rx = -rmax; rx <= (rmax + 0.001); rx+=deltar) {
					for (double ry = -rmax; ry <= (rmax + 0.001); ry+=deltar) {
						//cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry  << endl;
						double g = calc_Gamma(x, y, rx, ry);
						//cout << "deltad: " << deltad << " deltar:" << deltar <<
						//		" -> gamma: " << g << "; e: " << e << endl << endl;

						if (MyCoord::close(g, e, 0.0000001)) {
						//if (g == e) {
							double actProb = getPDF_Eloc_single(x) * getPDF_Eloc_single(y) * getPDF_Erot_single(rx) * getPDF_Erot_single(ry);
							//cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry;
							//cout << " -> gamma: " << g << "; e: " << e;
							//cout << " --> totProb = " << actProb << endl;
							sumAll += actProb;
						}
					}
				}
			}
		}
	}

	cout << "Sum all Prob = " << sumAll << " using"
			<< " d2Dmax: " << d2Dmax << " rmax:" << rmax
			<< " deltad: " << deltad << " deltar:" << deltar
			<< " --> ris = " << (deltad * deltad * deltar * deltar * sumAll) << endl;

	if (e == 0) exit(0);

	return (deltad * deltad * deltar * deltar * sumAll);

	/*std::list<MyCoord> cirDeltad;
	std::list<MyCoord> rotDeltar;

	for (double x = -d2Dmax; x <= d2Dmax; x+=deltad) {
		for (double y = -d2Dmax; y <= d2Dmax; y+=deltad) {
			MyCoord actCoord(x, y);
			if (actCoord.length() <= d2Dmax) {
				cirDeltad.push_back(actCoord);
			}
		}
	}
	for (double x = -rmax; x <= rmax; x+=deltar) {
		for (double y = -rmax; y <= rmax; y+=deltar) {
			rotDeltar.push_back(MyCoord(x, y));
		}
	}

	cout << endl;
	for (auto& earr : cirDeltad) {
		for (auto& r : rotDeltar) {
			double summ = getPDF_Eloc(earr) * getPDF_Erot(r);
			cout << summ << " ";
			ris += summ;
		}
		cout << endl;
	}
	cout << "RIS: " << ris << " -> deltas: " << (deltad * deltad) * (deltar * deltar) << endl;

	ris = (deltad * deltad) * (deltar * deltar) * ris;
	return ris;*/
}

void MultiFlow::calcProb_EReceivedTime_rec(double &acc, std::vector<double> &vect, double h, int t, double e, double deltae) {
	double ris = 0;

	/*cout << "vect -> [";
	for(auto& val : vect)
		cout << val << " ";
	cout << "]";*/

	double sume = 0;
	for (auto& ei : vect){
		sume += ei;
	}
	//if (MyCoord::close(sume, e)) {
	if (sume == e) {
		cout << "vect -> [";
		for(auto& val : vect)
			cout << val << " ";
		cout << "]";

		double product = 1.0;
		for (int j = 0; j < t; j++) {
			double pEReceived = calcProb_EReceived_new(h, vect[j]);
			cout << " " << pEReceived << " *";
			product *= pEReceived;
		}
		ris += product;

		cout << " --> OK --> " << ris << endl;
	}
	/*else {
		cout << endl;
	}*/

	acc += ris;

	if (sume < (t*e)) {
		for(int k = 0; k < t; k++){
			vect[k] += deltae;
			if (vect[k] > e) {
				vect[k] = 0;
			}
			else {
				break;
			}
		}

		return calcProb_EReceivedTime_rec(acc, vect, h, t, e, deltae);
	}
}

double MultiFlow::calcProb_EReceivedTime(double e, double deltae, double h, int t) {
	double ris = 0;
	std::vector<double> vect_e(t, 0);

	cout << "Making recursion for " << e << endl;
	calcProb_EReceivedTime_rec(ris, vect_e, h, t, e, deltae);
	cout << "End recursion - RIS: " << ris << endl << flush;

	return ris;
}

double MultiFlow::calculate_pWU(double h, int twu, double sigma2loc, double sigma2rho) {
	/*double pwu = 1;
	double ewu = Generic::getInstance().energyToWakeUp;
	double deltae = ewu / 20.0;

	double sumprob = 0;
	for (double e = 0; e <= ((ewu/deltae) - 1); e+= 1) {
		double actVal = calcProb_EReceivedTime(e * deltae, deltae, h, twu);
		cout << "Prob_EReceivedTime = " << actVal << endl << endl;
		sumprob += actVal;
	}
	pwu = 1.0 - (deltae * sumprob);

	return pwu;*/

	//TODO read from file
	return 1;
}

double MultiFlow::calc_Beta(double d3D, double h, double d2D) {
	return acos( (pow(d3D, 2.0) + pow(h, 2.0) - pow(d2D, 2.0)) / (2.0 * d3D * h) );
}

double MultiFlow::calc_smallGamma(double x, double y, double d3D, double h, double rho_x, double rho_y) {
	MyCoord uav(x, y, h);
	MyCoord sens(0, 0);
	MyCoord rho(x + h*tan(rho_y), y + h*tan(rho_x));

	double d_rho_2D = rho.distance(sens);
	double h_rho = rho.distance(uav);

	return acos((pow(h_rho, 2.0) + pow(d3D, 2.0) - pow(d_rho_2D, 2.0))/(2.0 * h_rho * d3D));
}

double MultiFlow::calc_Gain(double alpha, double gMAX, double alphaMAX) {
	if (alpha <= (alphaMAX / 2.0)) {
		return (gMAX + (-12 * pow(alpha / (alphaMAX / 2.0), 2.0)));
	}
	else {
		//return (-std::numeric_limits<double>::max());
		return -1000000000;
	}
}

double MultiFlow::calc_PathLoss(double d3D, double fMHz){
	return ((20.0 * log10(d3D)) + (20.0 * log10(fMHz)) - 27.55);
}

void MultiFlow::initEfficiencyMap(void) {
	efficiencyMap[-19] = 0;
	efficiencyMap[-18] = 0.005;
	efficiencyMap[-17] = 0.075;
	efficiencyMap[-16] = 0.15;
	efficiencyMap[-15] = 0.22;
	efficiencyMap[-14] = 0.28;
	efficiencyMap[-13] = 0.34;
	efficiencyMap[-12] = 0.385;
	efficiencyMap[-11] = 0.42;
	efficiencyMap[-10] = 0.44;
	efficiencyMap[-9] = 0.445;
	efficiencyMap[-8] = 0.442;
	efficiencyMap[-7] = 0.43;
	efficiencyMap[-6] = 0.415;
	efficiencyMap[-5] = 0.395;
	efficiencyMap[-4] = 0.37;
	efficiencyMap[-3] = 0.345;
	efficiencyMap[-2] = 0.315;
	efficiencyMap[-1] = 0.29;
	efficiencyMap[0] = 0.27;
	efficiencyMap[1] = 0.25;
	efficiencyMap[2] = 0.23;
	efficiencyMap[3] = 0.208;
	efficiencyMap[4] = 0.19;
	efficiencyMap[5] = 0.173;
	efficiencyMap[6] = 0.15;
	efficiencyMap[7] = 0.125;
	efficiencyMap[8] = 0.105;
	efficiencyMap[9] = 0.09;
	efficiencyMap[10] = 0.072;
	efficiencyMap[11] = 0.06;
	efficiencyMap[12] = 0.05;
	efficiencyMap[13] = 0.041;
	efficiencyMap[14] = 0.036;
	efficiencyMap[15] = 0.028;
	efficiencyMap[16] = 0.024;
	efficiencyMap[17] = 0.02;
	efficiencyMap[18] = 0.018;
	efficiencyMap[19] = 0;
}

double MultiFlow::calcRF2DC_efficiency(double rcvPow_dbm) {
	double eff_floor = 0;
	double eff_ceil = 0;
	int rcvPow_floor = floor(rcvPow_dbm);
	int rcvPow_ceil = ceil(rcvPow_dbm);

	if ((rcvPow_dbm >= -19) && (rcvPow_dbm <= 19)) {
		eff_floor = efficiencyMap[rcvPow_floor];
		eff_ceil = efficiencyMap[rcvPow_ceil];
	}

	//return ((1-(J5-FLOOR(J5)))*L5)+(((J5-FLOOR(J5)))*M5);
	return ( ( (1.0-(rcvPow_dbm-rcvPow_floor))*eff_floor ) + ( ((rcvPow_dbm-rcvPow_floor))*eff_ceil ) );
}

double MultiFlow::calc_Gamma(double x, double y, double rho_x, double rho_y) {
	double ris = 0;

	double beta, gamma, d3D;

	d3D = sqrt( pow(MyCoord(x, y).length(), 2.0) + pow(Generic::getInstance().flightAltitudeUAV, 2.0) );
	//cout << "d3D -> " << d3D << endl;

	beta = calc_Beta(d3D, Generic::getInstance().flightAltitudeUAV, MyCoord(x, y).length());
	//cout << "beta -> " << beta << endl;

	gamma = calc_smallGamma(x, y, d3D, Generic::getInstance().flightAltitudeUAV, rho_x, rho_y);
	//cout << "gamma -> " << gamma << endl;

	//cout << "wakeupTxPower -> " << Generic::getInstance().wakeupTxPower << endl;
	//cout << "gainU -> " << calc_Gain(gamma, Generic::getInstance().gUmax, Generic::getInstance().alphaUmax) << endl;
	//cout << "gainS -> " << calc_Gain(beta, Generic::getInstance().gSmax, Generic::getInstance().alphaSmax) << endl;
	//cout << "PL -> " << calc_PathLoss(d3D, Generic::getInstance().wakeupTxFrequency) << endl;

	ris += 10.0 * log10(Generic::getInstance().wakeupTxPower);
	ris += calc_Gain(gamma, Generic::getInstance().gUmax, Generic::getInstance().alphaUmax);
	ris += calc_Gain(beta, Generic::getInstance().gSmax, Generic::getInstance().alphaSmax);
	ris -= calc_PathLoss(d3D, Generic::getInstance().wakeupTxFrequency);

	double ris_W = pow(10.0, ris / 10.0) / 1000.0;
	double ris_store = ris_W * calcRF2DC_efficiency(ris_W);
	double ris_J = ris_store * Generic::getInstance().timeSlot;

	return ris_J;
}

void MultiFlow::init(void) {
	initEfficiencyMap();
}

void MultiFlow::run(int end_time) {

	/*for(double k = -5; k <= 5; k+=0.2) {
		cout << k << " " << RandomGenerator::get_PDF_normal(k, 0, 1) << endl;
	}
	exit(EXIT_FAILURE);*/

	pWU = calculate_pWU(Generic::getInstance().flightAltitudeUAV, 4,
			Generic::getInstance().sigmaGPS + Generic::getInstance().sigmaPilot, Generic::getInstance().sigmaRot);
	cerr << "fine pWU: " << pWU << endl;

	ChargingNode *leftmost = getLeftMostUAV(end_time); //cs_map.begin()->second;
	while(actSensorTimeStamp < end_time){
		if (actSensorTimeStamp < actUAVTimeStamp) {
			bool sensUP = updateSensorsEnergy(actSensorTimeStamp, actUAVTimeStamp);
			actSensorTimeStamp = actUAVTimeStamp;

			if (!sensUP) {
				break;
			}
		}

		calculateTSP_and_UpdateMF(leftmost);

		leftmost = getLeftMostUAV(end_time);
		if (leftmost == nullptr) {
			actUAVTimeStamp = end_time;
			if (actSensorTimeStamp < actUAVTimeStamp) {
				bool sensUP = updateSensorsEnergy(actSensorTimeStamp, actUAVTimeStamp);
				actSensorTimeStamp = actUAVTimeStamp;

				if (!sensUP) {
					break;
				}
			}

			break;
		}
		else {
			double lastTS = actUAVTimeStamp;
			actUAVTimeStamp = leftmost->lastTimestamp;
			if (actUAVTimeStamp < lastTS) {
				cerr << "ERROR incrementing time stamp" << endl;
				exit(EXIT_FAILURE);
			}
		}
	}

	cerr << "Simulation FINISHED!!!" << endl;
}


double MultiFlow::getLastSensorRead(void) {
	double ris = 0;

	for (auto& s : sens_list) {
		for (auto& r : s->readings) {
			if (r.readTime > ris) {
				ris = r.readTime;
			}
		}
	}

	return ris;
}

double MultiFlow::calcIndex(void) {
	double ris = 0;

	for (auto& s : sens_list) {
		for (auto& r : s->readings) {
			double actIndex = 1.0 - calcLossSensorOriginal(s, sens_list, r.readTime);
			if (actIndex < 0) {
				cerr << "Error in calculating index: " << actIndex << endl;
				exit(EXIT_FAILURE);
			}
			ris += actIndex;
		}
	}

	return ris;
}



double MultiFlow::getLastSensorRead_Tree(void) {
	double ris = 0;

	for (auto& s : sens_list) {
		for (auto& r : s->readings) {
			if (r.readTime > ris) {
				ris = r.readTime;
			}
		}
	}

	return ris;
}


double MultiFlow::calcLossSensorOriginal_Tree(SensorNode *s_check, std::list<SensorNode *> &sList, int texp) {
	double ris = 0;

	for (auto& s : sList) {
		for (auto& r : s->real_readings) {
			if (r.readTime < texp) {
				double actLoss = Loss::getInstance().calculate_loss_distance(s->sens, s_check->sens)
						* Loss::getInstance().calculate_loss_time(r.readTime, texp);

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
	}

	return ris;
}

double MultiFlow::calcIndex_Tree(void) {
	double ris = 0;

	for (auto& s : sens_list) {
		for (auto& r : s->real_readings) {
			double actIndex = 1.0 - calcLossSensorOriginal_Tree(s, sens_list, r.readTime);
			if (actIndex < 0) {
				cerr << "Error in calculating index: " << actIndex << endl;
				exit(EXIT_FAILURE);
			}
			ris += actIndex;
		}
	}

	return ris;
}















void MultiFlow::run_distributed(double end_time) {
	double sim_time = 0;
	bool sensAlive = true;

	cout << "Initializing simulation" << endl << flush;
/*
	for (auto& s : sens_list) {
		cout << "  S->" << s->sens->id << endl << flush;
	}
	cout << "End check sens_list" << endl << flush;
*/
	//init uavs
	for (auto& uav : uav_list) {
		for (auto& s : sens_list) {
			uav->sensMap[s->sens->id].sens = s;
			uav->sensMap[s->sens->id].lastTimeStamp = sim_time;
			uav->sensMap[s->sens->id].lastResidualEnergy = s->sens->residual_energy;
		}

		uav->us = UavDistributed::RECHARGING;
		uav->tsp2update = false;
	}

	//init wuVal	//TODO
	wuVal.estimatedIrrEnergyPerSlot_uJ = 0.1;
	wuVal.maxTwakeup = 10;
	wuVal.h = 5;
	wuVal.commProb = 0.9;

	cout << "Starting simulation" << endl << flush;

	while((sim_time <= end_time) && (sensAlive)) {

		//cout << "Updating NeighMaps" << endl << flush;
		updateNeighMaps(sim_time);
		//cout << "End check UAVs AFTER updateNeighMaps" << endl << flush;

		for (auto& uav : uav_list) {
			//cout << "Running UAV " << uav->cn->u->id << endl << flush;
			run_uav(uav, sim_time);
		}

		// remove self-discharge energy from sensors
		//cout << "Removing self-discharge energy from sensors" << endl << flush;
		for (auto& s : sens_list) {
			s->sens->residual_energy -= calcPowEtaSens(s->sens->residual_energy, Generic::getInstance().timeSlot);

			if (s->sens->residual_energy < 0) {
				sensAlive = false;
			}
		}

		cout << "Sim-Time: "<< sim_time << endl << flush;
		sim_time += Generic::getInstance().timeSlot;
	}

	cerr << "Simulation FINISHED!!!" << endl;
}

void MultiFlow::updateNeighMaps(double timenow) {
	for (auto& u1 : uav_list) {
		bool toUpdate = false;

		for (auto& u2 : uav_list) {
			if (u1->cn->id != u2->cn->id) {
				if (u1->cn->u->actual_coord.distance(u2->cn->u->actual_coord) <= Generic::getInstance().uavComRange) {
					//update neigh map
					if (u1->neighMap.count(u2->cn->id) == 0) {
						u1->neighMap[u2->cn->id].uav = u2;
						toUpdate = true;
					}
					u1->neighMap[u2->cn->id].lastTimeStamp = timenow;

					//update sensor timestamp
					for (auto& s2 : u2->sensMap) {
						if (s2.second.lastTimeStamp > u1->sensMap[s2.first].lastTimeStamp) {
							u1->sensMap[s2.first].lastTimeStamp = s2.second.lastTimeStamp;
							toUpdate = true;
						}

						if (s2.second.lastResidualEnergy < u1->sensMap[s2.first].lastResidualEnergy) {
							u1->sensMap[s2.first].lastResidualEnergy = s2.second.lastResidualEnergy;
							toUpdate = true;
						}
					}

					//update sensor timestamp V2
					for (auto& s2 : u2->sensMapTree) {
						for (auto& c2 : s2.second.timestamp_read_tslot) {
							bool found = false;
							for (auto& c1 : u1->sensMapTree[s2.first].timestamp_read_tslot) {
								if (c1 == c2) {
									found = true;
									break;
								}
							}
							if (!found) {
								u1->sensMapTree[s2.first].timestamp_read_tslot.push_back(c2);
								u1->sensMapTree[s2.first].timestamp_read.push_back(((double) c2) * Generic::getInstance().timeSlot);
								toUpdate = true;
							}
						}

						if (s2.second.lastResidualEnergy < u1->sensMapTree[s2.first].lastResidualEnergy) {
							u1->sensMapTree[s2.first].lastResidualEnergy = s2.second.lastResidualEnergy;
							toUpdate = true;
						}
					}
				}
				else {
					if (u1->neighMap.count(u2->cn->id) > 0) {
						if ((timenow - u1->neighMap[u2->cn->id].lastTimeStamp) > Generic::getInstance().neighUAVTimeout) {
							//u1->sensMap.erase(u2->cn->id);
							u1->neighMap.erase(u2->cn->id);
							toUpdate = true;
						}
					}
				}
			}
		}

		if (toUpdate) {
			u1->tsp2update = true;
		}
	}
}

void MultiFlow::run_uav(UavDistributed *uav, double simTime) {
	SensorNode *sn;

	switch (uav->us) {
	case UavDistributed::MOVING:
		//cout << "UAV " << uav->cn->u->id << " is in state MOVING" << endl << flush;
		uav->cn->u->residual_energy -= Generic::getInstance().singleMotorPowerUAV * 4.0 * Generic::getInstance().timeSlot;
		if ((uav->activeTSP.size() == 0) && (uav->cn->u->actual_coord == uav->cn->pos)) {
			//arrived back to the charging station
			uav->cn->u->actual_coord = uav->cn->pos;	// set the exact position due to possible numerical error
			uav->us = UavDistributed::RECHARGING;
			//cout << "UAV " << uav->cn->u->id << " moving on state RECHARGING" << endl << flush;
		}
		else if ((uav->activeTSP.size() > 0) && (uav->cn->u->actual_coord == (*(uav->activeTSP.begin()))->sens->coord)) {
			//arrived to destination sensor

			//check if nobody is there
			sn = *(uav->activeTSP.begin());
			if (sn->irradiatingUAV == nullptr) {
				sn->irradiatingUAV = uav->cn->u;
				sn->irradiatingTimeSlots = 0;
				sn->accumulatedEnergy_uJ = 0;
				uav->us = UavDistributed::WAKINGUP;
				//cout << "UAV " << uav->cn->u->id << " moving on state WAKINGUP" << endl << flush;
			}
			else {
				// the sensor is being used by another UAV. Go to the next
				uav->activeTSP.pop_front();
			}
		}
		else {
			MyCoord endP;

			if (uav->tsp2update) {
				calculateTSP_distributed(uav, uav->cn->u->actual_coord, uav->cn->pos, simTime);
				uav->tsp2update = false;
			}

			if (uav->activeTSP.size() == 0) endP = uav->cn->pos;
			else endP = (*(uav->activeTSP.begin()))->sens->coord;

			double dist2dest = uav->cn->u->actual_coord.distance(endP);
			double oneStepDist = ((double) Generic::getInstance().timeSlot) * Generic::getInstance().maxVelocity;
			if (oneStepDist >= dist2dest) {
				// I will arrive to destination
				uav->cn->u->actual_coord = endP;
			}
			else {
				MyCoord unit = endP - uav->cn->u->actual_coord;
				unit.normalize();
				unit *= oneStepDist;
				uav->cn->u->actual_coord += unit;
			}
		}
		break;

	case UavDistributed::WAKINGUP:
		//cout << "UAV " << uav->cn->u->id << " is in state WAKINGUP" << endl << flush;
		uav->cn->u->residual_energy -= Generic::getInstance().singleMotorPowerUAV * 4.0 * Generic::getInstance().timeSlot;
		uav->cn->u->residual_energy -= Generic::getInstance().wakeupTxPower * Generic::getInstance().timeSlot;

		if (uav->activeTSP.empty()){
			cerr << "uav->activeTSP is empty in WAKINGUP!" << endl;
			exit(EXIT_FAILURE);
		}
		sn = *(uav->activeTSP.begin());

		if (sn->irradiatingUAV == uav->cn->u) {
			// Ok I'm already irradiating
			sn->accumulatedEnergy_uJ += wuVal.estimatedIrrEnergyPerSlot_uJ;
			sn->irradiatingTimeSlots++;
			if (sn->accumulatedEnergy_uJ >= Generic::getInstance().energyToWakeUp) {
				// OK the sensor wakes-up
				uav->us = UavDistributed::STARTINGUP;
				//cout << "UAV " << uav->cn->u->id << " moving on state STARTINGUP" << endl << flush;
				sn->startupTimeSlots = 0;
			}
			else if(sn->irradiatingTimeSlots >= ceil(wuVal.maxTwakeup / Generic::getInstance().timeSlot)) {
				// the sensor didn't wake-up within maxTwakeup time. Go to the next
				uav->activeTSP.pop_front();
				sn->irradiatingUAV = nullptr;
				uav->us = UavDistributed::MOVING;
				//cout << "UAV " << uav->cn->u->id << " moving on state MOVING" << endl << flush;
			}
		}
		else {
			throw std::logic_error("Impossible that irradiating a sensor not mine");
		}
		break;

	case UavDistributed::STARTINGUP:
		//cout << "UAV " << uav->cn->u->id << " is in state STARTINGUP" << endl << flush;
		uav->cn->u->residual_energy -= Generic::getInstance().singleMotorPowerUAV * 4.0 * Generic::getInstance().timeSlot;
		uav->cn->u->residual_energy -= Generic::getInstance().pUstartup * Generic::getInstance().timeSlot;

		if (uav->activeTSP.empty()){
			cerr << "uav->activeTSP is empty in STARTINGUP!" << endl;
			exit(EXIT_FAILURE);
		}
		sn = *(uav->activeTSP.begin());

		sn->sens->residual_energy -= Generic::getInstance().pSstartup * Generic::getInstance().timeSlot;

		sn->startupTimeSlots++;
		if(sn->startupTimeSlots >= ceil(Generic::getInstance().tstartup / Generic::getInstance().timeSlot)) {
			sn->commTimeSlots = 0;
			sn->nCommAttempt = 0;
			uav->us = UavDistributed::READING;
			//cout << "UAV " << uav->cn->u->id << " moving on state READING" << endl << flush;
		}
		break;

	case UavDistributed::READING:
		//cout << "UAV " << uav->cn->u->id << " is in state READING" << endl << flush;
		uav->cn->u->residual_energy -= Generic::getInstance().singleMotorPowerUAV * 4.0 * Generic::getInstance().timeSlot;

		if (uav->activeTSP.empty()){
			cerr << "uav->activeTSP is empty in READING!" << endl;
			exit(EXIT_FAILURE);
		}
		sn = *(uav->activeTSP.begin());

		sn->commTimeSlots++;
		if(sn->commTimeSlots >= ceil(Generic::getInstance().ttimeout / Generic::getInstance().timeSlot)) {
			// ending one timeout
			uav->cn->u->residual_energy -= (Generic::getInstance().pUrx +
					(	(Generic::getInstance().pUtx - Generic::getInstance().pUrx) /
						ceil(Generic::getInstance().ttimeout / Generic::getInstance().timeSlot)
					) ) * Generic::getInstance().timeSlot;

			sn->sens->residual_energy -= (Generic::getInstance().pSrx +
					(	(Generic::getInstance().pStx - Generic::getInstance().pSrx) /
						ceil(Generic::getInstance().ttimeout / Generic::getInstance().timeSlot)
					) ) * Generic::getInstance().timeSlot;

			sn->nCommAttempt++;
			if (RandomGenerator::getInstance().getRealUniform(0, 1) <= wuVal.commProb) {
				// ok successful data gathering

				//read the data
				SensorNode::SensorRead sr;
				sr.readTime = simTime;
				sr.uav = uav->cn->u;
				sn->readings.push_back(sr);
				uav->sensMap[sn->sens->id].lastTimeStamp = simTime;

				// update last read energy
				uav->sensMap[sn->sens->id].lastResidualEnergy = sn->sens->residual_energy;

				//go away
				sn->commTimeSlots = 0;
				sn->nCommAttempt = 0;
				uav->activeTSP.pop_front();
				sn->irradiatingUAV = nullptr;
				uav->us = UavDistributed::MOVING;
				//cout << "UAV " << uav->cn->u->id << " moving on state MOVING" << endl << flush;
			}
			else if (sn->nCommAttempt >= Generic::getInstance().nr) {
				// too much attempts
				// go away
				sn->commTimeSlots = 0;
				sn->nCommAttempt = 0;
				uav->activeTSP.pop_front();
				sn->irradiatingUAV = nullptr;
				uav->us = UavDistributed::MOVING;
				//cout << "UAV " << uav->cn->u->id << " moving on state MOVING" << endl << flush;
			}
			else {
				sn->commTimeSlots = 0;	// start new attempt
			}
		}

		break;

	case UavDistributed::RECHARGING:
	default:
		//cout << "UAV " << uav->cn->u->id << " is in state RECHARGING" << endl << flush;
		//cout << "  I'm recharging" << endl << flush;
		uav->cn->u->residual_energy += Generic::getInstance().rechargeStation_power * ((double) Generic::getInstance().timeSlot);
		if (uav->cn->u->residual_energy >= uav->cn->u->max_energy) {
			uav->cn->u->residual_energy = uav->cn->u->max_energy;

			//cout << "  Start calculating TSP" << endl << flush;
			calculateTSP_distributed(uav, uav->cn->pos, uav->cn->pos, simTime);
			//calculateTSP_distributed_dummy(uav, uav->cn->pos, uav->cn->pos, simTime);
			//cout << "  End calculating TSP" << endl << flush;
			if (uav->activeTSP.size() > 0) {		// IDLE but calculated TSP... let's move!
				uav->us = UavDistributed::MOVING;
				//cout << "UAV " << uav->cn->u->id << " moving on state MOVING" << endl << flush;
			}
		}
		break;
	}

	//cout << "  Inside UAV " << uav->cn->u->id << " END" << endl << flush;
}

double MultiFlow::calcLossSensor_distributed(SensorNode *sens, UavDistributed *uav, double texp) {
	double ris = 0;

	//cout << "        Inside calcLossSensor. START" << endl << flush;

	/*std::list<Sensor *> ssList;
	for (auto& sn : sens_list){
		ssList.push_back(sn->sens);
	}*/

	std::list<long double> ldList;
	for (auto& sn : sens_list){
		if (sn->sens->id != sens->sens->id) {
			ldList.push_back(uav->sensMap[sn->sens->id].lastResidualEnergy);
		}
	}

	for (auto& s : uav->sensMap) {
		if (s.second.lastTimeStamp < texp) {

			double actLoss = Loss::algebraic_sum(Loss::getInstance().calculate_loss_distance(sens->sens, s.second.sens->sens)
											* Loss::getInstance().calculate_loss_time(s.second.lastTimeStamp, texp),
											//Loss::getInstance().calculate_loss_energy(sens->sens, texp, ssList));
											Loss::getInstance().calculate_loss_energy_only(uav->sensMap[sens->sens->id].lastResidualEnergy, ldList));

			//cout << "        Inside calcLossSensor. Not discounted loss: " << actLoss << endl << flush;

			double closestThenMe = 0;
			double myDist = uav->cn->pos.distance(sens->sens->coord);
			for (auto& u : uav->neighMap) {
				if (u.second.uav->cn->pos.distance(sens->sens->coord) < myDist) {
					closestThenMe += 1.0;
				}
			}

			//double discount = (closestThenMe + 1.0) / (((double) uav->neighMap.size()) + 1.0);
			double penalty = 0.0;
			if (uav->neighMap.size() > 0) {
				penalty = closestThenMe / ((double) uav->neighMap.size());
			}
			//cout << "        Inside calcLossSensor. Calculated discount: " << discount << endl << flush;

			//actLoss *= discount;
			actLoss = Loss::algebraic_sum(actLoss, penalty);

			if (actLoss > ris) {
				ris = actLoss;
			}
		}
	}

	//cout << "        Inside calcLossSensor. END" << endl << flush;

	return ris;
}

SensorNode *MultiFlow::getMinLossSensor_distributed(UavDistributed *uav, list<SensorNode *> &sList, double texp) {
	SensorNode *ris = nullptr;
	double minLoss = std::numeric_limits<double>::max();
	//double minLoss = -1000;
	//cout << "      Inside MinLossSensor. START" << endl << flush;

	//cerr << "Calculating minLossSensor with Texp: " << texp << " - ";
	for (auto& s : sList) {
		//double actLoss = calcLossSensor(s, sens_list, texp);
		double actLoss = calcLossSensor_distributed(s, uav, texp);
		//cerr << "S" << s->sens->id << ":" << actLoss << " ";
		if (actLoss < minLoss) {
			minLoss = actLoss;
			ris = s;
		}
	}
	//cerr << " -> winner: S" << ris->sens->id << endl;
	//cout << "      Inside MinLossSensor. END" << endl << flush;

	return ris;
}

void MultiFlow::calculateTSP_distributed_dummy(UavDistributed *uav, MyCoord startPoint, MyCoord endPoint, double simTime) {
	double getProb = 1.0 / ((double) sens_list.size());

	uav->activeTSP.clear();


	for (auto& s : sens_list) {
		double randNum = RandomGenerator::getInstance().getRealUniform(0, 1);
		if (randNum <= getProb) {
			uav->activeTSP.push_back(s);
		}
	}
}

void MultiFlow::calculateTSP_distributed(UavDistributed *uav, MyCoord startPoint, MyCoord endPoint, double simTime) {
	double tsp_time = uav->cn->u->residual_energy / Generic::getInstance().pUfly;
	//double tsp_cost = 0;
	list<SensorNode *> sens_check;
	list<SensorNode *> actTSP;

	//cout << "    Inside TSP for UAV " << uav->cn->u->id << " START" << endl << flush;

	//uav->activeTSP.clear();

	for (auto& s : sens_list) {
		sens_check.push_back(s);
	}


	while (!sens_check.empty()) {
		list<SensorNode *> newTSP;
		double energy_cost = std::numeric_limits<double>::max();
		double t_time_tmp = 0;

		double t_exp = simTime + (tsp_time / 2.0);
		//cout << "    Inside TSP. Calculating minLossSensor with expected time " << t_exp << endl << flush;
		SensorNode *sj = getMinLossSensor_distributed(uav, sens_check, t_exp);
		//cout << "    Inside TSP. Calculated minLossSensor: S" << sj->sens->id << endl << flush;
		//SensorNode *sj = getMinLossSensor(sens_check, t_exp);

		/*cerr << "Calculating TSP with U" << leftmost->u->id << " ";
		for(auto& s : actTSP) {
			cerr << "S" << s->sens->id << " ";
		}
		cerr << "NS" << sj->sens->id << " ";
		cerr << endl;*/

		calculateTSP_incremental_distributed(newTSP, actTSP, sj, startPoint, endPoint, t_time_tmp, energy_cost);

		if (energy_cost <= uav->cn->u->residual_energy) {//Generic::getInstance().initUAVEnergy) {
			actTSP.clear();
			for (auto& s : newTSP) {
				actTSP.push_back(s);
			}
			tsp_time = t_time_tmp;
			//tsp_cost = t_cost;

			/*cerr << "Calculated NEW TSP: " << endl << "C" << leftmost->u->id << " ";
			for (auto& s : newTSP) {
				cerr << "S" << s->sens->id << " ";
			}
			cerr << "C" << leftmost->u->id << " ";
			cerr << "- with time cost " << t_time_tmp
					<< " and with energy cost " << energy_cost
					<< " having energy " << leftmost->u->residual_energy
					<< endl << endl;*/
		}

		sens_check.remove(sj);
	}

	cerr << "Final TSP: " << "B" << startPoint << " ";
	for (auto& s : actTSP) {
		cerr << "S" << s->sens->id << " ";
	}
	cerr << "E" << endPoint << " with TSP-time of " << tsp_time << endl;

	for (auto& s : actTSP) {
		uav->activeTSP.push_back(s);
	}

	//activateTSPandRecharge(leftmost, actTSP);
	//cout << "    Inside TSP for UAV " << uav->cn->u->id << " END" << endl << flush;
}

void MultiFlow::calculateTSP_incremental_distributed(list<SensorNode *> &newTSP, list<SensorNode *> &actTSP,
			SensorNode *sj, MyCoord startPoint, MyCoord endPoint, double &tsp_time, double &tsp_energy_cost) {
	double pwu = 1; //TODO

	if (actTSP.empty()) {
		newTSP.push_back(sj);
		tsp_time = calcTimeToTravel(startPoint, sj->sens->coord) + calcTimeToTravel(sj->sens->coord, endPoint);
		tsp_energy_cost = 	calcEnergyToTravel(startPoint, sj->sens->coord) +
				calcEnergyToWuData(pwu) +
				calcEnergyToTravel(sj->sens->coord, endPoint);
	}
	else {
		Sensor *uavDummySensorStart = new Sensor(startPoint, 1, TSP_DUMMY_CODE_START_MF);
		Sensor *uavDummySensorEnd = new Sensor(endPoint, 1, TSP_DUMMY_CODE_END_MF);
		SensorNode uavDummySensorNodeStart;
		SensorNode uavDummySensorNodeEnd;
		list<SensorNode *> allSens;
		list<TSP2MultiFlow *> edges;
		list<TSP2MultiFlow *> finaledges;
		list<TSP2MultiFlow *> finalcircuit;

		uavDummySensorNodeStart.sens = uavDummySensorStart;
		uavDummySensorNodeStart.lastTimestamp = tsp_time;
		uavDummySensorNodeEnd.sens = uavDummySensorEnd;
		uavDummySensorNodeEnd.lastTimestamp = tsp_time;

		for (auto& s : actTSP) {
			allSens.push_back(s);
		}
		allSens.push_back(sj);
		allSens.push_back(&uavDummySensorNodeStart);
		allSens.push_back(&uavDummySensorNodeEnd);

		auto it1 = allSens.begin();
		while (it1 != allSens.end()) {
			auto it2 = it1;
			it2++;
			while (it2 != allSens.end()) {
				//double t;
				TSP2MultiFlow *ne = new TSP2MultiFlow(*it1, *it2, 0);

				ne->weight = calculateCosts1Edge(ne);
				edges.push_back(ne);

				it2++;
			}
			it1++;
		}
		edges.sort(TSP2MultiFlow::sortEdges);

		int idFinal = 0;
		for (auto& e : edges) {
			//check if the sensors in the edge are already caught
			int connected_first = 0;
			int connected_second = 0;
			int idFirst = -1;
			int idSecond = -2;

			for (auto& fe : finaledges) {
				if ((e->first->sens->id == fe->first->sens->id) || (e->first->sens->id == fe->second->sens->id)) {
					connected_first++;
					idFirst = fe->idTSP;
				}
				if ((e->second->sens->id == fe->first->sens->id) || (e->second->sens->id == fe->second->sens->id)) {
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

		// CLOSE THE CIRCUIT
		for (auto& e : edges) {
			int connected_first = 0;
			int connected_second = 0;
			for (auto& fe : finaledges) {
				if ((e->first->sens->id == fe->first->sens->id) || (e->first->sens->id == fe->second->sens->id)) {
					connected_first++;
				}
				if ((e->second->sens->id == fe->first->sens->id) || (e->second->sens->id == fe->second->sens->id)) {
					connected_second++;
				}
			}
			if ((connected_first == 1) && (connected_second == 1)) {
				e->idTSP = finaledges.front()->idTSP;
				finaledges.push_back(e);
			}
		}

		SensorNode *nextS = &uavDummySensorNodeStart;
		int counter = 500;
		//cout << "TSP final: UAV" << cc->clusterUAV->id;
		do {
			//for (auto& fe : finaledges) {
			for (auto it_fe = finaledges.begin(); it_fe != finaledges.end(); it_fe++) {
				if (((*it_fe)->first->sens->id == nextS->sens->id) || ((*it_fe)->second->sens->id == nextS->sens->id)) {
					if ((*it_fe)->first->sens->id == nextS->sens->id){
						if ((nextS->sens->id == TSP_DUMMY_CODE_START_MF) && ((*it_fe)->second->sens->id == TSP_DUMMY_CODE_END_MF)) {
							continue;
						}
						finalcircuit.push_back(new TSP2MultiFlow(nextS, (*it_fe)->second, (*it_fe)->weight));
						nextS = (*it_fe)->second;
					}
					else {
						if ((nextS->sens->id == TSP_DUMMY_CODE_END_MF) && ((*it_fe)->second->sens->id == TSP_DUMMY_CODE_START_MF)) {
							continue;
						}
						finalcircuit.push_back(new TSP2MultiFlow(nextS, (*it_fe)->first, (*it_fe)->weight));
						nextS = (*it_fe)->first;
					}

					//if (nextS->id != uavDummySensor->id) {
					//cout << "-S" << nextS->id;
					//}
					finaledges.erase(it_fe);
					break;
				}
			}
		} while ((nextS != &uavDummySensorNodeEnd) && ((counter--) > 0));

		if (counter == 0) {
			cerr << "ERRORE" << endl;
			exit(EXIT_FAILURE);
		}

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
					if (fe1->second->sens->id != fe2->first->sens->id){
						//double sumActCost = fe1->first->sens->coord.distance(fe1->second->sens->coord) + fe2->first->sens->coord.distance(fe2->second->sens->coord);
						double sumActCost = calculateCosts1Edge(fe1->first, fe1->second) + calculateCosts1Edge(fe2->first, fe2->second);
						//double sumSwitch = fe1->first->sens->coord.distance(fe2->first->sens->coord) + fe1->second->sens->coord.distance(fe2->second->sens->coord);
						double sumSwitch = calculateCosts1Edge(fe1->first, fe2->first) + calculateCosts1Edge(fe1->second, fe2->second);
						if (sumSwitch < sumActCost) {
							list<TSP2MultiFlow *> tmpcircuit;
							SensorNode *tmp;

							//cout << endl << "TEMP CIRCUIT BEFORE: ";
							//for (auto& fe : finalcircuit) cout << " S" << fe->first->id << "-S" << fe->second->id;
							//cout << endl;
							//cout << "Swapping S" << fe1->first->id << "-S" << fe1->second->id << " with S" << fe2->first->id << "-S" << fe2->second->id << endl;

							auto it_feSwitch = it_fe1;
							it_feSwitch++;
							while ((*it_feSwitch)->second->sens->id != (*it_fe2)->second->sens->id){
								tmp = (*it_feSwitch)->second;
								(*it_feSwitch)->second = (*it_feSwitch)->first;
								(*it_feSwitch)->first = tmp;

								it_feSwitch++;
							}

							tmp = fe1->second;
							fe1->second = fe2->first;
							//fe1->weight = fe1->first->sens->coord.distance(fe1->second->sens->coord);
							fe1->weight = calculateCosts1Edge(fe1->first, fe1->second);

							fe2->first = tmp;
							//fe2->weight = fe2->first->sens->coord.distance(fe2->second->sens->coord);
							fe2->weight = calculateCosts1Edge(fe2->first, fe2->second);

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
		tsp_time = 0;
		tsp_energy_cost = 0;
		for (auto& fe : finalcircuit) {
			//fCircuit.push_back(fe);
			if ((fe->second->sens->id != TSP_DUMMY_CODE_START_MF) && (fe->second->sens->id != TSP_DUMMY_CODE_END_MF)) {
				newTSP.push_back(fe->second);

				tsp_time += calcTimeToWuData();
				tsp_energy_cost += calcEnergyToWuData(pwu);
			}
			tsp_time += calcTimeToTravel(fe->first->sens->coord, fe->second->sens->coord);
			tsp_energy_cost += calcEnergyToTravel(fe->first->sens->coord, fe->second->sens->coord);

			//cout << " S" << fe->first->id << "-S" << fe->second->id;
		}
		//cout << endl;

		for (auto& ef : edges) free(ef);
		edges.clear();

		//for (auto& ff : finaledges) free(ff);
		finaledges.clear();

		for (auto& ff : finalcircuit) free(ff);
		finalcircuit.clear();

		free(uavDummySensorStart);
		free(uavDummySensorEnd);
	}
}


void MultiFlow::writeHitmaps_distr(std::string filename) {

	// all in one
	std::string allHit = Generic::getInstance().hitmapFilename + ".log";
	std::ofstream ofs (allHit, std::ofstream::out);
	if (ofs.is_open()) {
		for (auto& s : sens_list) {
			ofs << s->sens->id << " " << s->sens->coord.x << " " << s->sens->coord.y << " " << s->readings.size() << endl;
		}
		ofs.close();
	}

	// single for each UAV
	for (auto& u : uav_list) {
		std::string allHit = Generic::getInstance().hitmapFilename + "_U" + std::to_string(u->cn->u->id) + ".log";

		std::ofstream ofs_single (allHit, std::ofstream::out);
		if (ofs_single.is_open()) {
			for (auto& s : sens_list) {
				int sumReadUAV = 0;
				for(auto& r : s->readings) {
					if (r.uav->id == u->cn->u->id) {
						++sumReadUAV;
					}
				}
				ofs_single << s->sens->id << " " << s->sens->coord.x << " " << s->sens->coord.y << " " << sumReadUAV << endl;
			}
			ofs_single.close();
		}
	}
}


void MultiFlow::writeHitmaps_multiflow(std::string filename) {

	// all in one
	std::string allHit = Generic::getInstance().hitmapFilename + ".log";
	std::ofstream ofs (allHit, std::ofstream::out);
	if (ofs.is_open()) {
		for (auto& s : sens_list) {
			ofs << s->sens->id << " " << s->sens->coord.x << " " << s->sens->coord.y << " " << s->readings.size() << endl;
		}
		ofs.close();
	}

	// single for each UAV
	for (auto& cnm : cs_map) {
		ChargingNode *cn = cnm.second;

		std::string allHit = Generic::getInstance().hitmapFilename + "_U" + std::to_string(cn->u->id) + ".log";

		std::ofstream ofs_single (allHit, std::ofstream::out);
		if (ofs_single.is_open()) {
			for (auto& s : sens_list) {
				int sumReadUAV = 0;
				for(auto& r : s->readings) {
					if (r.uav->id == cn->u->id) {
						++sumReadUAV;
					}
				}
				ofs_single << s->sens->id << " " << s->sens->coord.x << " " << s->sens->coord.y << " " << sumReadUAV << endl;
			}
			ofs_single.close();
		}
	}
}






void MultiFlow::init_treeMF(double endTS, double timeoffset) {
	int nTslot = ceil(endTS / timeoffset);

	cerr << "Start making the multiflow graph" << endl;

	//init maps
	for (auto& s : sens_list) {
		mfTreeMAP_sensor[s->sens->id] = map<double, SensorNodeTree *>();
	}

	for (auto& cn : cs_map) {
		mfTreeMAP_uav[cn.second->u->id] = map<double, ChargingNodeTree *>();
	}

	cerr << "Start building the nodes" << endl;

	//init the nodes
	for (int ts = 0; ts <= nTslot; ts++) {
		double t = ((double) ts) * timeoffset;
		for (auto& s : sens_list) {
			SensorNodeTree *newsnt = new(SensorNodeTree);
			newsnt->id = s->sens->id;
			newsnt->timestamp = t;
			newsnt->timestamp_Tslot = ts;
			newsnt->timestamp_offset = timeoffset;
			newsnt->read = false;
			newsnt->s = s->sens;
			newsnt->pos = s->sens->coord;

			mfTreeMAP_sensor[s->sens->id][t] = newsnt;
		}

		for (auto& cn : cs_map) {
			ChargingNodeTree *newcnt = new ChargingNodeTree();
			newcnt->id = cn.second->u->id;
			newcnt->timestamp = t;
			newcnt->timestamp_Tslot = ts;
			newcnt->timestamp_offset = timeoffset;
			newcnt->charging = false;
			newcnt->u = cn.second->u;
			newcnt->pos = cn.second->u->recharge_coord;

			mfTreeMAP_uav[cn.second->u->id][t] = newcnt;
		}
	}

	cerr << "End building the nodes" << endl << endl;

	cerr << "Start building the edges for UAVs" << endl;

	//set the arcs for the charging stations
	for (auto& cn : cs_map) {

		cerr << " Start building the edges for U" << cn.second->u->id << endl;
		cerr << "\r U" << cn.second->u->id << " T0";

		//set for zero
		ChargingNodeTree *zeroCNT = mfTreeMAP_uav[cn.second->u->id][0];
		if (1 <= nTslot){
			ArcTree *at = new ArcTree();
			at->cnt_start = zeroCNT;
			at->cnt_end = mfTreeMAP_uav[cn.second->u->id][timeoffset];
			at->timecost = timeoffset;
			at->timecost_Tslot = 1;

			zeroCNT->nextCN.push_back(at);
		}
		for (auto& s : sens_list) {
			double timeArrival = 50;		//TODO
			int timeArrival_Tslot = ceil (timeArrival / timeoffset);
			if (timeArrival_Tslot <= nTslot) {
				ArcTree *at = new ArcTree();
				at->cnt_start = zeroCNT;
				at->snt_end = mfTreeMAP_sensor[s->sens->id][((double) timeArrival_Tslot) * timeoffset];
				at->timecost = ((double) timeArrival_Tslot) * timeoffset;
				at->timecost_Tslot = timeArrival_Tslot;

				zeroCNT->nextS.push_back(at);
			}
		}

		//set the others
		for (int ts = 1; ts <= nTslot; ts++) {
			double t = ((double) ts) * timeoffset;
			ChargingNodeTree *actCNT = mfTreeMAP_uav[cn.second->u->id][t];

			cerr << "\r U" << cn.second->u->id << " T" << t;

			if ((ts+1) <= nTslot){
				ArcTree *at = new ArcTree();
				at->cnt_start = actCNT;
				at->cnt_end = mfTreeMAP_uav[cn.second->u->id][((double) (ts+1)) * timeoffset];
				at->timecost = timeoffset;
				at->timecost_Tslot = 1;

				zeroCNT->nextCN.push_back(at);
			}

			for (auto& l : zeroCNT->nextS){
				int zeroTslot_arr = l->snt_end->timestamp_Tslot;
				if ((zeroTslot_arr + ts) <= nTslot) {
					double newArrTime = ((double) (zeroTslot_arr + ts)) * timeoffset;
					ArcTree *at = new ArcTree();
					at->cnt_start = actCNT;
					at->snt_end = mfTreeMAP_sensor[l->snt_end->id][newArrTime];
					at->timecost = l->timecost;
					at->timecost_Tslot = l->timecost_Tslot;

					actCNT->nextS.push_back(at);
				}
			}
		}
		cerr << " Finish" << endl;
	}

	cerr << "Start building the edges for sensors" << endl;
	//set the arcs for the sensors
	for (auto& s : sens_list) {

		cerr << " Start building the edges for S" << s->sens->id << endl;
		cerr << "\r S" << s->sens->id << " T0";

		//set for zero
		SensorNodeTree *zeroSNT = mfTreeMAP_sensor[s->sens->id][0];
		for (auto& s1 : sens_list) {
			if (s->sens->id != s1->sens->id) {
				double timeArrival = 50;		//TODO
				int timeArrival_Tslot = ceil (timeArrival / timeoffset);
				if (timeArrival_Tslot <= nTslot) {
					ArcTree *at = new ArcTree();
					at->snt_start = zeroSNT;
					at->snt_end = mfTreeMAP_sensor[s1->sens->id][((double) timeArrival_Tslot) * timeoffset];
					at->timecost = timeArrival;
					at->timecost_Tslot = timeArrival_Tslot;

					zeroSNT->nextS.push_back(at);
				}
			}
		}
		for (auto& cn : cs_map) {
			double timeArrival = 50;		//TODO
			int timeArrival_Tslot = ceil (timeArrival / timeoffset);
			if (timeArrival_Tslot <= nTslot) {
				ArcTree *at = new ArcTree();
				at->snt_start = zeroSNT;
				at->cnt_end = mfTreeMAP_uav[cn.second->u->id][((double) timeArrival_Tslot) * timeoffset];
				at->timecost = timeArrival;
				at->timecost_Tslot = timeArrival_Tslot;

				zeroSNT->nextCN.push_back(at);
			}
		}

		//set the others
		for (int ts = 1; ts <= nTslot; ts++) {
			double t = ((double) ts) * timeoffset;
			SensorNodeTree *actSNT = mfTreeMAP_sensor[s->sens->id][t];

			cerr << "\r S" << s->sens->id << " T" << t;

			for (auto& l : zeroSNT->nextS){
				int zeroTslot_arr = l->snt_end->timestamp_Tslot;
				if ((zeroTslot_arr + ts) <= nTslot) {
					double newArrTime = ((double) (zeroTslot_arr + ts)) * timeoffset;

					ArcTree *at = new ArcTree();
					at->snt_start = actSNT;
					at->snt_end = mfTreeMAP_sensor[l->snt_end->id][newArrTime];
					at->timecost = l->timecost;
					at->timecost_Tslot = l->timecost_Tslot;

					actSNT->nextS.push_back(at);
				}
			}

			for (auto& l : zeroSNT->nextCN){
				int zeroTslot_arr = l->cnt_end->timestamp_Tslot;
				if ((zeroTslot_arr + ts) <= nTslot) {
					double newArrTime = ((double) (zeroTslot_arr + ts)) * timeoffset;

					ArcTree *at = new ArcTree();
					at->snt_start = zeroSNT;
					at->cnt_end = mfTreeMAP_uav[l->cnt_end->id][newArrTime];
					at->timecost = l->timecost;
					at->timecost_Tslot = l->timecost_Tslot;

					actSNT->nextCN.push_back(at);
				}
			}
		}
		cerr << " Finish" << endl;
	}
}

















int MultiFlow::calculateMatrixTimeSlot(int id1, int id2) {
	MyCoord p1, p2;

	if (Sensor::isSensorID(id1)) {
		p1 = sens_map[id1]->sens->coord;
	}
	else {
		p1 = cs_map[id1]->u->recharge_coord;
	}

	if (Sensor::isSensorID(id2)) {
		p2 = sens_map[id2]->sens->coord;
	}
	else {
		p2 = cs_map[id2]->u->recharge_coord;
	}

	double ttt = p1.distance(p2) / Generic::getInstance().maxVelocity;
	int fly = ceil(ttt / Generic::getInstance().timeSlot);

	if (Sensor::isSensorID(id2)) {
		double wuANDcom = 0;
		wuANDcom += Generic::getInstance().twakeup;
		wuANDcom += Generic::getInstance().tstartup;
		wuANDcom += Generic::getInstance().ttimeout * Generic::getInstance().nr;

		fly += ceil(wuANDcom / Generic::getInstance().timeSlot);
	}

	return fly;
}

double MultiFlow::calculateMatrixEnergy(int id1, int id2) {
	int flyTime = calculateMatrixTimeSlot(id1, id2);
	double ris = ((double) flyTime) * Generic::getInstance().pUfly;

	if (Sensor::isSensorID(id2)) {
		ris += Generic::getInstance().twakeup * Generic::getInstance().wakeupTxPower;
		ris += Generic::getInstance().tstartup * Generic::getInstance().pUstartup;
		ris += (Generic::getInstance().ttimeout * Generic::getInstance().nr) *
				(Generic::getInstance().pUrx +
						( ( Generic::getInstance().pUtx - Generic::getInstance().pUrx) /
							Generic::getInstance().ttimeout));
	}

	return ris;
}

void MultiFlow::init_matrix_treeMF(void) {
	for (auto& s : sens_list) {
		mfTreeMatrix_time[s->sens->id] = map<int, double>();
		mfTreeMatrix_timeslot[s->sens->id] = map<int, int>();
		mfTreeMatrix_energy[s->sens->id] = map<int, double>();
	}
	for (auto& cn : cs_map) {
		mfTreeMatrix_time[cn.second->u->id] = map<int, double>();
		mfTreeMatrix_timeslot[cn.second->u->id] = map<int, int>();
		mfTreeMatrix_energy[cn.second->u->id] = map<int, double>();
	}

	for (auto& vt : mfTreeMatrix_time) {
		for (auto& s : sens_list) {
			//mfTreeMatrix_time[vt.first][s->sens->id] = calculateMatrixTimeSlot(vt.first, s->sens->id);
			//mfTreeMatrix_timeslot[vt.first][s->sens->id] = ceil(mfTreeMatrix_time[vt.first][s->sens->id] / Generic::getInstance().timeSlot);
			mfTreeMatrix_timeslot[vt.first][s->sens->id] = calculateMatrixTimeSlot(vt.first, s->sens->id);
			mfTreeMatrix_time[vt.first][s->sens->id] = mfTreeMatrix_timeslot[vt.first][s->sens->id] * Generic::getInstance().timeSlot;
		}
		for (auto& cn : cs_map) {
			//mfTreeMatrix_time[vt.first][cn.second->u->id] = calculateMatrixTimeSlot(vt.first, cn.second->u->id);
			//mfTreeMatrix_timeslot[vt.first][cn.second->u->id] = ceil(mfTreeMatrix_time[vt.first][cn.second->u->id] / Generic::getInstance().timeSlot);
			mfTreeMatrix_timeslot[vt.first][cn.second->u->id] = calculateMatrixTimeSlot(vt.first, cn.second->u->id);
			mfTreeMatrix_time[vt.first][cn.second->u->id] = mfTreeMatrix_timeslot[vt.first][cn.second->u->id] * Generic::getInstance().timeSlot;
		}
	}

	for (auto& ve : mfTreeMatrix_energy) {
		for (auto& s : sens_list) {
			mfTreeMatrix_energy[ve.first][s->sens->id] = calculateMatrixEnergy(ve.first, s->sens->id);
		}
		for (auto& cn : cs_map) {
			mfTreeMatrix_energy[ve.first][cn.second->u->id] = calculateMatrixEnergy(ve.first, cn.second->u->id);
		}
	}
	/*
	for (auto& vt : mfTreeMatrix_time) {
		cout << vt.first << "\t|\t";

		for (auto& s : sens_list) {
			cout << mfTreeMatrix_time[vt.first][s->sens->id] << " ";
		}
		cout << endl;
	}
	exit(0);*/
}

void MultiFlow::run_tree_multiflow(double end_time) {

	/*pWU = calculate_pWU(Generic::getInstance().flightAltitudeUAV, 4,
			Generic::getInstance().sigmaGPS + Generic::getInstance().sigmaPilot, Generic::getInstance().sigmaRot);
	cerr << "fine pWU: " << pWU << endl;*/

	ChargingNode *leftmost = getLeftMostUAV(end_time); //cs_map.begin()->second;
	while(actSensorTimeStamp < end_time){
		if (actSensorTimeStamp < actUAVTimeStamp) {
			bool sensUP = updateSensorsEnergy_Tree(actSensorTimeStamp, actUAVTimeStamp);
			actSensorTimeStamp = actUAVTimeStamp;

			if (!sensUP) {
				break;
			}
		}

		calculateTreeBSF_and_UpdateMF(leftmost, end_time);

		leftmost = getLeftMostUAV(end_time);
		if (leftmost == nullptr) {
			actUAVTimeStamp = end_time;
			if (actSensorTimeStamp < actUAVTimeStamp) {
				bool sensUP = updateSensorsEnergy_Tree(actSensorTimeStamp, actUAVTimeStamp);
				actSensorTimeStamp = actUAVTimeStamp;

				if (!sensUP) {
					break;
				}
			}

			break;
		}
		else {
			double lastTS = actUAVTimeStamp;
			actUAVTimeStamp = leftmost->lastTimestamp;
			if (actUAVTimeStamp < lastTS) {
				cerr << "ERROR incrementing time stamp" << endl;
				exit(EXIT_FAILURE);
			}
		}
	}

	cerr << "Simulation FINISHED!!!" << endl;
}


void MultiFlow::calculateTreeBSF_and_UpdateMF(ChargingNode *leftmost, double end_time) {
	list<pair<pathStats, list<SensorNode *>>> bestTSPs;
	//double startUAVenergy = leftmost->u->residual_energy;
	//double timek = leftmost->lastTimestamp;
	//int timek_tslot = leftmost->lastTimestamp_tslot;
	//int bestTimeSlot = numeric_limits<int>::max();
	int rechargeBulk = ceil(
				(leftmost->u->max_energy * Generic::getInstance().rechargeRatio) /
				(Generic::getInstance().timeSlot * Generic::getInstance().rechargeStation_power)
			);

	double uavEnergy = leftmost->u->residual_energy;
	double uavTime = leftmost->lastTimestamp;
	int uavTimeTSlot = leftmost->lastTimestamp_tslot;
	int bulkExecuted = 0;

	//do {
	while (true) {
		list<SensorNode *> bTSP;
		pathStats pstat;

		//cout << "U" << leftmost->u->id << " calculating path with energy " << uavEnergy
		//		<< " and max energy of " << leftmost->u->max_energy
		//		<< " with " << bulkExecuted << " bulks of recharge"
		//		<< endl;

		calculatePath(bTSP, leftmost, uavTime, uavTimeTSlot, uavEnergy, true, nullptr, pstat);
		if (bTSP.size() > 0) {
			pstat.recharge_bulk = bulkExecuted;
			bestTSPs.push_back(make_pair(pstat, bTSP));
		}

		if ((uavEnergy >= leftmost->u->max_energy) || (uavTime >= end_time)) break;

		uavTimeTSlot += rechargeBulk;
		uavTime = ((double) uavTimeTSlot) * Generic::getInstance().timeSlot;
		uavEnergy = min(leftmost->u->max_energy, uavEnergy +
				(((double) rechargeBulk) * Generic::getInstance().timeSlot * Generic::getInstance().rechargeStation_power));
		++bulkExecuted;

	}
	//while ((uavEnergy < leftmost->u->max_energy) && (uavTime < end_time));

	list<SensorNode *> bestTSP;
	pathStats topStat;

	switch (mfAlgoType) {
	case ALGO_BSF:
	case ALGO_DSF:
	default:
		for (auto& bTSP : bestTSPs) {
			topStat.total_gain = 0;
			topStat.time_cost = numeric_limits<double>::max();
			if ( 	(bTSP.first.total_gain > topStat.total_gain) ||
					((bTSP.first.total_gain == topStat.total_gain) && (bTSP.first.time_cost < topStat.time_cost)) ) {
				topStat = bTSP.first;
				bestTSP = bTSP.second;
			}
		}
		break;

	case ALGO_BSF_DISTANCE:
	case ALGO_BSF_ENERGY:
	case ALGO_DSF_DISTANCE:
	case ALGO_DSF_ENERGY:
		for (auto& bTSP : bestTSPs) {
			topStat.total_gain = numeric_limits<double>::max();
			topStat.time_cost = numeric_limits<double>::max();
			if ( (bTSP.first.total_gain < topStat.total_gain) ||
					((bTSP.first.total_gain == topStat.total_gain) && (bTSP.first.time_cost < topStat.time_cost)) ) {
				topStat = bTSP.first;
				bestTSP = bTSP.second;
			}
		}
		break;
	}

	if (bestTSP.size() > 0) {
		leftmost->lastTimestamp_tslot += rechargeBulk * topStat.recharge_bulk;
		leftmost->lastTimestamp = ((double) leftmost->lastTimestamp_tslot) * Generic::getInstance().timeSlot;
		leftmost->u->residual_energy = min(leftmost->u->max_energy,
				leftmost->u->residual_energy +
				(((double) (rechargeBulk * topStat.recharge_bulk)) * Generic::getInstance().timeSlot * Generic::getInstance().rechargeStation_power));
	}
	else {
		leftmost->lastTimestamp = end_time;
		leftmost->lastTimestamp_tslot = ceil(end_time / Generic::getInstance().timeSlot);
		leftmost->u->residual_energy = leftmost->u->max_energy;
	}

	/*
	double gainTSP = 0;
	double timeTSP = numeric_limits<double>::max();
	int bulkToRecharge = 0;

	for (auto& bTSP : bestTSPs) {
		double g = 0;
		double t = ((double) (bTSP.first * rechargeBulk)) * Generic::getInstance().timeSlot;
		int actID = leftmost->u->id;
		double actTime = leftmost->lastTimestamp;
		list<pair<int, int>> lsensread;

		for (auto& el : bTSP.second) {
			double actArcTim = mfTreeMatrix_time[actID][el->sens->id];

			t += actArcTim;
			actTime += actArcTim;

			g += 1.0 - calculateLossBSF(lsensread, sens_map[el->sens->id], actTime, true, nullptr);
			lsensread.push_back(make_pair(el->sens->id, actTime));

			actID = el->sens->id;
		}
		t += mfTreeMatrix_time[actID][leftmost->u->id];

		if ( (g > gainTSP) || ((g == gainTSP) && (t < timeTSP)) ) {
			gainTSP = g;
			timeTSP = t;
			bulkToRecharge = bTSP.first;
			bestTSP = bTSP.second;
		}

		//cout << "U" << leftmost->u->id << " calculated path with " << bTSP.first << " bulks of recharge"
		//		<< " having gain: " << g << " and time " << t
		//		<< endl;
	}

	//cout << "The winner is the one with " << bulkToRecharge << " bulks of recharge"
	//			<< " having gain: " << gainTSP << " and time " << timeTSP
	//			<< endl;

	if (bestTSP.size() > 0) {
		leftmost->lastTimestamp_tslot += rechargeBulk * bulkToRecharge;
		leftmost->lastTimestamp = ((double) leftmost->lastTimestamp_tslot) * Generic::getInstance().timeSlot;
		leftmost->u->residual_energy = min(leftmost->u->max_energy,
				leftmost->u->residual_energy +
				(((double) (rechargeBulk * bulkToRecharge)) * Generic::getInstance().timeSlot * Generic::getInstance().rechargeStation_power));
	}
	else {
		leftmost->lastTimestamp = end_time;
		leftmost->lastTimestamp_tslot = ceil(end_time / Generic::getInstance().timeSlot);
		leftmost->u->residual_energy = leftmost->u->max_energy;
	}*/


	/*do {
		double batteryRatio = min(leftmost->u->residual_energy / leftmost->u->max_energy, 1.0);
		double checkNum = pow(batteryRatio, Generic::getInstance().bsfExponent);

		if ( 	(leftmost->u->residual_energy >= leftmost->u->max_energy) ||
				(RandomGenerator::getInstance().getRealUniform(0, 1) <= checkNum) ) {

			//cout << "Calculating BSF!!!" << endl << flush;
			calculatePath(bestTSP,
					leftmost,
					leftmost->lastTimestamp,
					leftmost->lastTimestamp_tslot,
					leftmost->u->residual_energy,
					true, nullptr);

			//cout << "Calculated BSF!!! - " << flush;
			//for (auto& bsf : bestTSP) {
			//	cout << "S" << bsf->sens->id << " " << flush;
			//}
			//cout << endl << flush;
		}

		leftmost->lastTimestamp_tslot += rechargeBulk;
		leftmost->lastTimestamp = ((double) leftmost->lastTimestamp_tslot) * Generic::getInstance().timeSlot;
		leftmost->u->residual_energy = min(leftmost->u->max_energy,
				leftmost->u->residual_energy +
					(((double) rechargeBulk) * Generic::getInstance().timeSlot * Generic::getInstance().rechargeStation_power));

		//startUAVenergy += min(leftmost->u->max_energy,
		//		Generic::getInstance().timeSlot * Generic::getInstance().rechargeStation_power);

	} while ((bestTSP.size() == 0) && (leftmost->lastTimestamp < end_time));*/


	activateBSFandRecharge(leftmost, bestTSP);

	//exit(EXIT_FAILURE);
}


void MultiFlow::calculatePath(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e,
		bool centralized, UavDistributed *uav, pathStats &pstat) {
	switch (mfAlgoType) {
		case ALGO_BSF:
		default:
			calculateBSF(path, cn, tk, tk_tslot, uav_e, centralized, uav, pstat);
			break;

		case ALGO_BSF_DISTANCE:
			calculateBSF_distance(path, cn, tk, tk_tslot, uav_e, centralized, uav, pstat);
			break;

		case ALGO_BSF_ENERGY:
			calculateBSF_energy(path, cn, tk, tk_tslot, uav_e, centralized, uav, pstat);
			break;

		case ALGO_DSF:
			calculateDSF(path, cn, tk, tk_tslot, uav_e, centralized, uav, pstat);
			break;

		case ALGO_DSF_DISTANCE:
			calculateDSF_distance(path, cn, tk, tk_tslot, uav_e, centralized, uav, pstat);
			break;

		case ALGO_DSF_ENERGY:
			calculateDSF_energy(path, cn, tk, tk_tslot, uav_e, centralized, uav, pstat);
			break;
	}
}


void MultiFlow::calculateDSF(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e,
		bool centralized, UavDistributed *uav, pathStats &pstat) {
	list<pair<int, int>> q;

	map<pair<int,int>, double> energymap;
	list<pair<int, int>> llist;
	map<pair<int,int>, list<pair<int,int>>> pathmap;
	//map<pair<int,int>, list<pair<int,int>>> phimap;

	//init
	energymap[make_pair(cn->u->id, tk_tslot)] = uav_e;
	llist.push_back(make_pair(cn->u->id, tk_tslot));
	pathmap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();
	//phimap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();

	q.push_back(make_pair(cn->u->id, tk_tslot));

	while (!q.empty()) {
		pair<int, int> qt_pair = q.front();
		q.pop_front();

		list<pair<SensorNode *, double>> allSens;
		for (auto& s : sens_map) {
			if (s.second->sens->id != qt_pair.first) {
				int tm_tslot = qt_pair.second + mfTreeMatrix_timeslot[qt_pair.first][s.second->sens->id];
				allSens.push_back(make_pair(s.second, calculateLossBSF(pathmap[qt_pair], s.second, tm_tslot, centralized, uav)));
			}
		}
		allSens.sort(MultiFlow::sortEdgesBSF);
		/*for (auto& pr : allSens){
			cout << "S" << pr.first->sens->id << "|" << pr.second << " ";
		}
		cout << endl;*/

		for (auto& sj : allSens) {
			int tm_tslot = qt_pair.second + mfTreeMatrix_timeslot[qt_pair.first][sj.first->sens->id];
			double residualAfter = energymap[qt_pair]
									- mfTreeMatrix_energy[qt_pair.first][sj.first->sens->id]
									- mfTreeMatrix_energy[sj.first->sens->id][cn->u->id];
			pair<int, int> sj_pair = make_pair(sj.first->sens->id, tm_tslot);

			if ( (pathmap.count(sj_pair) == 0) && (residualAfter > 0) ){
				q.push_back(sj_pair);

				pathmap[sj_pair] = list<pair<int,int>>();
				for (auto& oldp : pathmap[qt_pair]) {
					pathmap[sj_pair].push_back(oldp);
				}
				pathmap[sj_pair].push_back(sj_pair);

				energymap[sj_pair] = energymap[qt_pair] - mfTreeMatrix_energy[qt_pair.first][sj_pair.first];

				llist.remove(qt_pair);
				llist.push_back(sj_pair);

				break;
			}
		}
	}

	pair<int, int> final_pair = make_pair(-1, -1);
	double final_gain = 0;

	//cout << "Calculated PATHS: " << endl;
	for (auto& stprime : llist) {
		double sumgain = 0;
		double sumloss = 0;
		double sumenergy = 0;
		double sumtime = 0;
		int sumtime_tslot = 0;
		//cout << "From <" << stprime.first << ";" << stprime.second << "> -> ";

		int actID = cn->u->id;
		for (auto& stpath : pathmap[stprime]) {
			double actLossSens = calculateLossBSF(pathmap[stprime], sens_map[stpath.first], stpath.second, centralized, uav);

			sumloss += actLossSens;
			sumgain += 1.0 - actLossSens;
			sumenergy += mfTreeMatrix_energy[actID][stpath.first];
			sumtime += mfTreeMatrix_time[actID][stpath.first];
			sumtime_tslot += mfTreeMatrix_timeslot[actID][stpath.first];

			actID = stpath.first;

			//cout << "S" << stpath.first << "(" << stpath.second*Generic::getInstance().timeSlot << "|" << actLossSens << ") ";
		}
		//cout << " GAIN: " << sumgain << endl;
		sumenergy += mfTreeMatrix_energy[actID][cn->u->id];
		sumtime += mfTreeMatrix_time[actID][cn->u->id];
		sumtime_tslot += mfTreeMatrix_timeslot[actID][cn->u->id];

		double sumGainTime = sumgain / sumtime;

		//if(sumgain > final_gain) {
		if(sumGainTime > final_gain) {
			final_pair = stprime;
			//final_gain = sumgain;
			final_gain = sumGainTime;

			pstat.total_gain = sumgain;
			pstat.total_loss = sumloss;
			pstat.energy_cost = sumenergy;
			pstat.time_cost = sumtime;
			pstat.time_cost_tslot = sumtime_tslot;
		}
	}

	path.clear();
	for (auto& f : pathmap[final_pair]) {
		path.push_back(sens_map[f.first]);
	}
}


void MultiFlow::calculateDSF_distance(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e,
		bool centralized, UavDistributed *uav, pathStats &pstat) {
	list<pair<int, int>> q;

	map<pair<int,int>, double> energymap;
	list<pair<int, int>> llist;
	map<pair<int,int>, list<pair<int,int>>> pathmap;
	//map<pair<int,int>, list<pair<int,int>>> phimap;

	//init
	energymap[make_pair(cn->u->id, tk_tslot)] = uav_e;
	llist.push_back(make_pair(cn->u->id, tk_tslot));
	pathmap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();
	//phimap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();

	q.push_back(make_pair(cn->u->id, tk_tslot));

	while (!q.empty()) {
		pair<int, int> qt_pair = q.front();
		q.pop_front();

		list<pair<SensorNode *, double>> allSens;
		for (auto& s : sens_map) {
			if (s.second->sens->id != qt_pair.first) {
				if (Sensor::isSensorID(qt_pair.first)) {
					allSens.push_back(make_pair(s.second, s.second->sens->coord.distance(sens_map[qt_pair.first]->sens->coord)));
				}
				else {
					allSens.push_back(make_pair(s.second, s.second->sens->coord.distance(cn->u->recharge_coord)));
				}
			}
		}
		allSens.sort(MultiFlow::sortEdgesBSF);
		/*for (auto& pr : allSens){
			cout << "S" << pr.first->sens->id << "|" << pr.second << " ";
		}
		cout << endl;*/

		for (auto& sj : allSens) {
			int tm_tslot = qt_pair.second + mfTreeMatrix_timeslot[qt_pair.first][sj.first->sens->id];
			double residualAfter = energymap[qt_pair]
									- mfTreeMatrix_energy[qt_pair.first][sj.first->sens->id]
									- mfTreeMatrix_energy[sj.first->sens->id][cn->u->id];
			pair<int, int> sj_pair = make_pair(sj.first->sens->id, tm_tslot);

			if ( (pathmap.count(sj_pair) == 0) && (residualAfter > 0) ){
				q.push_back(sj_pair);

				pathmap[sj_pair] = list<pair<int,int>>();
				for (auto& oldp : pathmap[qt_pair]) {
					pathmap[sj_pair].push_back(oldp);
				}
				pathmap[sj_pair].push_back(sj_pair);

				energymap[sj_pair] = energymap[qt_pair] - mfTreeMatrix_energy[qt_pair.first][sj_pair.first];

				llist.remove(qt_pair);
				llist.push_back(sj_pair);

				break;
			}
		}
	}

	pair<int, int> final_pair = make_pair(-1, -1);
	double final_dist = numeric_limits<double>::max();

	for (auto& stprime : llist) {
		double sumdist = 0;
		MyCoord lastCoord = cn->pos;
		double sumenergy = 0;
		double sumtime = 0;
		int sumtime_tslot = 0;

		int actID = cn->u->id;
		for (auto& stpath : pathmap[stprime]) {
			sumdist += lastCoord.distance(sens_map[stpath.first]->sens->coord);
			lastCoord = sens_map[stpath.first]->sens->coord;

			sumenergy += mfTreeMatrix_energy[actID][stpath.first];
			sumtime += mfTreeMatrix_time[actID][stpath.first];
			sumtime_tslot += mfTreeMatrix_timeslot[actID][stpath.first];

			actID = stpath.first;
		}
		sumdist += lastCoord.distance(cn->pos);
		sumenergy += mfTreeMatrix_energy[actID][cn->u->id];
		sumtime += mfTreeMatrix_time[actID][cn->u->id];
		sumtime_tslot += mfTreeMatrix_timeslot[actID][cn->u->id];

		if(sumdist < final_dist) {
			final_pair = stprime;
			final_dist = sumdist;

			pstat.total_gain = sumdist;
			pstat.total_loss = sumdist;
			pstat.energy_cost = sumenergy;
			pstat.time_cost = sumtime;
			pstat.time_cost_tslot = sumtime_tslot;
		}
	}

	path.clear();
	for (auto& f : pathmap[final_pair]) {
		path.push_back(sens_map[f.first]);
	}
}


void MultiFlow::calculateDSF_energy(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e,
		bool centralized, UavDistributed *uav, pathStats &pstat) {
	list<pair<int, int>> q;

	map<pair<int,int>, double> energymap;
	list<pair<int, int>> llist;
	map<pair<int,int>, list<pair<int,int>>> pathmap;
	//map<pair<int,int>, list<pair<int,int>>> phimap;

	//init
	energymap[make_pair(cn->u->id, tk_tslot)] = uav_e;
	llist.push_back(make_pair(cn->u->id, tk_tslot));
	pathmap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();
	//phimap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();

	q.push_back(make_pair(cn->u->id, tk_tslot));

	while (!q.empty()) {
		pair<int, int> qt_pair = q.front();
		q.pop_front();

		list<pair<SensorNode *, double>> allSens;
		for (auto& s : sens_map) {
			if (s.second->sens->id != qt_pair.first) {
				if (centralized){
					allSens.push_back(make_pair(s.second, s.second->sens->residual_energy));
				}
				else {
					allSens.push_back(make_pair(s.second, uav->sensMapTree[s.second->sens->id].lastResidualEnergy));
				}
			}
		}
		allSens.sort(MultiFlow::sortEdgesBSF);
		allSens.reverse();
		/*for (auto& pr : allSens){
			cout << "S" << pr.first->sens->id << "|" << pr.second << " ";
		}
		cout << endl;*/

		for (auto& sj : allSens) {
			int tm_tslot = qt_pair.second + mfTreeMatrix_timeslot[qt_pair.first][sj.first->sens->id];
			double residualAfter = energymap[qt_pair]
									- mfTreeMatrix_energy[qt_pair.first][sj.first->sens->id]
									- mfTreeMatrix_energy[sj.first->sens->id][cn->u->id];
			pair<int, int> sj_pair = make_pair(sj.first->sens->id, tm_tslot);

			if ( (pathmap.count(sj_pair) == 0) && (residualAfter > 0) ){
				q.push_back(sj_pair);

				pathmap[sj_pair] = list<pair<int,int>>();
				for (auto& oldp : pathmap[qt_pair]) {
					pathmap[sj_pair].push_back(oldp);
				}
				pathmap[sj_pair].push_back(sj_pair);

				energymap[sj_pair] = energymap[qt_pair] - mfTreeMatrix_energy[qt_pair.first][sj_pair.first];

				llist.remove(qt_pair);
				llist.push_back(sj_pair);

				break;
			}
		}
	}

	pair<int, int> final_pair = make_pair(-1, -1);
	double final_gain = numeric_limits<double>::max();

	for (auto& stprime : llist) {
		double sumSensEnergy = 0;
		double sumenergy = 0;
		double sumtime = 0;
		int sumtime_tslot = 0;

		int actID = cn->u->id;
		for (auto& stpath : pathmap[stprime]) {
			sumenergy += mfTreeMatrix_energy[actID][stpath.first];
			sumtime += mfTreeMatrix_time[actID][stpath.first];
			sumtime_tslot += mfTreeMatrix_timeslot[actID][stpath.first];
			if (centralized){
				sumSensEnergy += sens_map[stpath.first]->sens->residual_energy;
			}
			else {
				sumSensEnergy += uav->sensMapTree[stpath.first].lastResidualEnergy;
			}

			actID = stpath.first;
		}
		sumenergy += mfTreeMatrix_energy[actID][cn->u->id];
		sumtime += mfTreeMatrix_time[actID][cn->u->id];
		sumtime_tslot += mfTreeMatrix_timeslot[actID][cn->u->id];

		if(sumSensEnergy < final_gain) {
			final_pair = stprime;
			final_gain = sumSensEnergy;

			pstat.total_gain = sumSensEnergy;
			pstat.total_loss = sumSensEnergy;
			pstat.energy_cost = sumenergy;
			pstat.time_cost = sumtime;
			pstat.time_cost_tslot = sumtime_tslot;
		}
	}

	path.clear();
	for (auto& f : pathmap[final_pair]) {
		path.push_back(sens_map[f.first]);
	}
}


void MultiFlow::calculateBSF_energy(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e,
		bool centralized, UavDistributed *uav, pathStats &pstat) {
	list<pair<int, int>> q;

	map<pair<int,int>, double> energymap;
	list<pair<int, int>> llist;
	map<pair<int,int>, list<pair<int,int>>> pathmap;
	//map<pair<int,int>, list<pair<int,int>>> phimap;

	//init
	energymap[make_pair(cn->u->id, tk_tslot)] = uav_e;
	llist.push_back(make_pair(cn->u->id, tk_tslot));
	pathmap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();
	//phimap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();

	q.push_back(make_pair(cn->u->id, tk_tslot));

	while (!q.empty()) {
		pair<int, int> qt_pair = q.front();
		q.pop_front();

		list<pair<SensorNode *, double>> allSens;
		for (auto& s : sens_map) {
			if (s.second->sens->id != qt_pair.first) {
				if (centralized){
					allSens.push_back(make_pair(s.second, s.second->sens->residual_energy));
				}
				else {
					allSens.push_back(make_pair(s.second, uav->sensMapTree[s.second->sens->id].lastResidualEnergy));
				}
			}
		}
		allSens.sort(MultiFlow::sortEdgesBSF);
		allSens.reverse();
		/*for (auto& pr : allSens){
			cout << "S" << pr.first->sens->id << "|" << pr.second << " ";
		}
		cout << endl;*/

		for (auto& sj : allSens) {
			int tm_tslot = qt_pair.second + mfTreeMatrix_timeslot[qt_pair.first][sj.first->sens->id];
			double residualAfter = energymap[qt_pair]
									- mfTreeMatrix_energy[qt_pair.first][sj.first->sens->id]
									- mfTreeMatrix_energy[sj.first->sens->id][cn->u->id];
			pair<int, int> sj_pair = make_pair(sj.first->sens->id, tm_tslot);

			if ( (pathmap.count(sj_pair) == 0) && (residualAfter > 0) ){
				q.push_back(sj_pair);

				pathmap[sj_pair] = list<pair<int,int>>();
				for (auto& oldp : pathmap[qt_pair]) {
					pathmap[sj_pair].push_back(oldp);
				}
				pathmap[sj_pair].push_back(sj_pair);

				energymap[sj_pair] = energymap[qt_pair] - mfTreeMatrix_energy[qt_pair.first][sj_pair.first];

				llist.remove(qt_pair);
				llist.push_back(sj_pair);

				if (llist.size() > sens_map.size()) {
					pair<int, int> st_pair = make_pair(-1, -1);
					double gain = 0;

					//cout << "Check L" << endl;
					for (auto& stprime : llist) {
						double sumSensEnergy = 0;

						//cout << "<" << stprime.first << ", " << stprime.second << "> -> ";
						for (auto& stpath : pathmap[stprime]) {
							if (centralized){
								sumSensEnergy += sens_map[stpath.first]->sens->residual_energy;
							}
							else {
								sumSensEnergy += uav->sensMapTree[stpath.first].lastResidualEnergy;
							}
						}
						//cout << "--> SUM:" << sumgain<< endl;

						if(sumSensEnergy > gain) {
							st_pair = stprime;
							gain = sumSensEnergy;
						}
					}

					//cout << "Removing <" << st_pair.first << ", " << st_pair.second << ">" << endl <<  endl;

					llist.remove(st_pair);
					q.remove(st_pair);
				}

				break;
			}
		}
	}

	pair<int, int> final_pair = make_pair(-1, -1);
	double final_dist = numeric_limits<double>::max();

	for (auto& stprime : llist) {
		double sumSensEnergy = 0;
		double sumenergy = 0;
		double sumtime = 0;
		int sumtime_tslot = 0;

		int actID = cn->u->id;
		for (auto& stpath : pathmap[stprime]) {

			sumenergy += mfTreeMatrix_energy[actID][stpath.first];
			sumtime += mfTreeMatrix_time[actID][stpath.first];
			sumtime_tslot += mfTreeMatrix_timeslot[actID][stpath.first];
			if (centralized){
				sumSensEnergy += sens_map[stpath.first]->sens->residual_energy;
			}
			else {
				sumSensEnergy += uav->sensMapTree[stpath.first].lastResidualEnergy;
			}

			actID = stpath.first;
		}
		sumenergy += mfTreeMatrix_energy[actID][cn->u->id];
		sumtime += mfTreeMatrix_time[actID][cn->u->id];
		sumtime_tslot += mfTreeMatrix_timeslot[actID][cn->u->id];

		if(sumSensEnergy < final_dist) {
			final_pair = stprime;
			final_dist = sumSensEnergy;

			pstat.total_gain = sumSensEnergy;
			pstat.total_loss = sumSensEnergy;
			pstat.energy_cost = sumenergy;
			pstat.time_cost = sumtime;
			pstat.time_cost_tslot = sumtime_tslot;
		}
	}

	path.clear();
	for (auto& f : pathmap[final_pair]) {
		path.push_back(sens_map[f.first]);
	}
}


void MultiFlow::calculateBSF_distance(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e,
		bool centralized, UavDistributed *uav, pathStats &pstat) {
	list<pair<int, int>> q;

	map<pair<int,int>, double> energymap;
	list<pair<int, int>> llist;
	map<pair<int,int>, list<pair<int,int>>> pathmap;
	//map<pair<int,int>, list<pair<int,int>>> phimap;

	//init
	energymap[make_pair(cn->u->id, tk_tslot)] = uav_e;
	llist.push_back(make_pair(cn->u->id, tk_tslot));
	pathmap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();
	//phimap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();

	q.push_back(make_pair(cn->u->id, tk_tslot));

	while (!q.empty()) {
		pair<int, int> qt_pair = q.front();
		q.pop_front();

		list<pair<SensorNode *, double>> allSens;
		for (auto& s : sens_map) {
			if (s.second->sens->id != qt_pair.first) {
				if (Sensor::isSensorID(qt_pair.first)) {
					allSens.push_back(make_pair(s.second, s.second->sens->coord.distance(sens_map[qt_pair.first]->sens->coord)));
				}
				else {
					allSens.push_back(make_pair(s.second, s.second->sens->coord.distance(cn->u->recharge_coord)));
				}
			}
		}
		allSens.sort(MultiFlow::sortEdgesBSF);
		/*for (auto& pr : allSens){
			cout << "S" << pr.first->sens->id << "|" << pr.second << " ";
		}
		cout << endl;*/

		for (auto& sj : allSens) {
			int tm_tslot = qt_pair.second + mfTreeMatrix_timeslot[qt_pair.first][sj.first->sens->id];
			double residualAfter = energymap[qt_pair]
									- mfTreeMatrix_energy[qt_pair.first][sj.first->sens->id]
									- mfTreeMatrix_energy[sj.first->sens->id][cn->u->id];
			pair<int, int> sj_pair = make_pair(sj.first->sens->id, tm_tslot);

			if ( (pathmap.count(sj_pair) == 0) && (residualAfter > 0) ){
				q.push_back(sj_pair);

				pathmap[sj_pair] = list<pair<int,int>>();
				for (auto& oldp : pathmap[qt_pair]) {
					pathmap[sj_pair].push_back(oldp);
				}
				pathmap[sj_pair].push_back(sj_pair);

				energymap[sj_pair] = energymap[qt_pair] - mfTreeMatrix_energy[qt_pair.first][sj_pair.first];

				llist.remove(qt_pair);
				llist.push_back(sj_pair);

				if (llist.size() > sens_map.size()) {
					pair<int, int> st_pair = make_pair(-1, -1);
					double final_dist = 0;

					//cout << "Check L" << endl;
					for (auto& stprime : llist) {
						double sumdist = 0;
						MyCoord lastCoord = cn->pos;

						//cout << "<" << stprime.first << ", " << stprime.second << "> -> ";
						for (auto& stpath : pathmap[stprime]) {
							sumdist += lastCoord.distance(sens_map[stpath.first]->sens->coord);
							lastCoord = sens_map[stpath.first]->sens->coord;
						}
						//cout << "--> SUM:" << sumgain<< endl;
						sumdist += lastCoord.distance(cn->pos);

						if(sumdist > final_dist) {
							st_pair = stprime;
							final_dist = sumdist;
						}
					}
					//cout << "Removing <" << st_pair.first << ", " << st_pair.second << ">" << endl <<  endl;

					llist.remove(st_pair);
					q.remove(st_pair);
				}

				break;
			}
		}
	}

	pair<int, int> final_pair = make_pair(-1, -1);
	double final_dist = numeric_limits<double>::max();

	for (auto& stprime : llist) {
		double sumdist = 0;
		MyCoord lastCoord = cn->pos;
		double sumenergy = 0;
		double sumtime = 0;
		int sumtime_tslot = 0;

		int actID = cn->u->id;
		for (auto& stpath : pathmap[stprime]) {
			sumdist += lastCoord.distance(sens_map[stpath.first]->sens->coord);
			lastCoord = sens_map[stpath.first]->sens->coord;

			sumenergy += mfTreeMatrix_energy[actID][stpath.first];
			sumtime += mfTreeMatrix_time[actID][stpath.first];
			sumtime_tslot += mfTreeMatrix_timeslot[actID][stpath.first];

			actID = stpath.first;
		}
		sumdist += lastCoord.distance(cn->pos);
		sumenergy += mfTreeMatrix_energy[actID][cn->u->id];
		sumtime += mfTreeMatrix_time[actID][cn->u->id];
		sumtime_tslot += mfTreeMatrix_timeslot[actID][cn->u->id];

		if(sumdist < final_dist) {
			final_pair = stprime;
			final_dist = sumdist;

			pstat.total_gain = sumdist;
			pstat.total_loss = sumdist;
			pstat.energy_cost = sumenergy;
			pstat.time_cost = sumtime;
			pstat.time_cost_tslot = sumtime_tslot;
		}
	}

	path.clear();
	for (auto& f : pathmap[final_pair]) {
		path.push_back(sens_map[f.first]);
	}
}


void MultiFlow::calculateBSF(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e,
		bool centralized, UavDistributed *uav, pathStats &pstat) {
	list<pair<int, int>> q;

	map<pair<int,int>, double> energymap;
	list<pair<int, int>> llist;
	map<pair<int,int>, list<pair<int,int>>> pathmap;
	//map<pair<int,int>, list<pair<int,int>>> phimap;

	//init
	energymap[make_pair(cn->u->id, tk_tslot)] = uav_e;
	llist.push_back(make_pair(cn->u->id, tk_tslot));
	pathmap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();
	//phimap[make_pair(cn->u->id, tk_tslot)] = list<pair<int,int>>();

	q.push_back(make_pair(cn->u->id, tk_tslot));

	while (!q.empty()) {
		pair<int, int> qt_pair = q.front();
		q.pop_front();

		list<pair<SensorNode *, double>> allSens;
		for (auto& s : sens_map) {
			if (s.second->sens->id != qt_pair.first) {
				int tm_tslot = qt_pair.second + mfTreeMatrix_timeslot[qt_pair.first][s.second->sens->id];
				allSens.push_back(make_pair(s.second, calculateLossBSF(pathmap[qt_pair], s.second, tm_tslot, centralized, uav)));
			}
		}
		allSens.sort(MultiFlow::sortEdgesBSF);
		/*for (auto& pr : allSens){
			cout << "S" << pr.first->sens->id << "|" << pr.second << " ";
		}
		cout << endl;*/

		for (auto& sj : allSens) {
			int tm_tslot = qt_pair.second + mfTreeMatrix_timeslot[qt_pair.first][sj.first->sens->id];
			double residualAfter = energymap[qt_pair]
									- mfTreeMatrix_energy[qt_pair.first][sj.first->sens->id]
									- mfTreeMatrix_energy[sj.first->sens->id][cn->u->id];
			pair<int, int> sj_pair = make_pair(sj.first->sens->id, tm_tslot);

			if ( (pathmap.count(sj_pair) == 0) && (residualAfter > 0) ){
				q.push_back(sj_pair);

				pathmap[sj_pair] = list<pair<int,int>>();
				for (auto& oldp : pathmap[qt_pair]) {
					pathmap[sj_pair].push_back(oldp);
				}
				pathmap[sj_pair].push_back(sj_pair);

				energymap[sj_pair] = energymap[qt_pair] - mfTreeMatrix_energy[qt_pair.first][sj_pair.first];

				llist.remove(qt_pair);
				llist.push_back(sj_pair);

				if (llist.size() > (sens_map.size() * 2)) {
					pair<int, int> st_pair = make_pair(-1, -1);
					double gain = numeric_limits<double>::max();

					//cout << "Check L" << endl;
					for (auto& stprime : llist) {
						double sumgain = 0;
						double sumtime = 0;

						//cout << "<" << stprime.first << ", " << stprime.second << "> -> ";
						int actID = cn->u->id;
						for (auto& stpath : pathmap[stprime]) {
							double actgain = 1.0 - calculateLossBSF(pathmap[stprime], sens_map[stpath.first], stpath.second, centralized, uav);
							//cout << "S" << stpath.first << "|T" << stpath.second << "|" << actgain << " ";
							sumgain += actgain;
							sumtime += mfTreeMatrix_time[actID][stpath.first];
						}
						sumtime += mfTreeMatrix_time[actID][cn->u->id];
						//cout << "--> SUM:" << sumgain<< endl;

						sumgain = sumgain / sumtime;

						if(sumgain < gain) {
							st_pair = stprime;
							gain = sumgain;
						}
					}

					//cout << "Removing <" << st_pair.first << ", " << st_pair.second << ">" << endl <<  endl;

					llist.remove(st_pair);
					q.remove(st_pair);
				}
			}
		}
	}

	pair<int, int> final_pair = make_pair(-1, -1);
	double final_gain = 0;

	//cout << "Calculated PATHS: " << endl;
	for (auto& stprime : llist) {
		double sumgain = 0;
		double sumloss = 0;
		double sumenergy = 0;
		double sumtime = 0;
		int sumtime_tslot = 0;
		//cout << "From <" << stprime.first << ";" << stprime.second << "> -> ";

		int actID = cn->u->id;
		for (auto& stpath : pathmap[stprime]) {
			double actLossSens = calculateLossBSF(pathmap[stprime], sens_map[stpath.first], stpath.second, centralized, uav);

			sumloss += actLossSens;
			sumgain += 1.0 - actLossSens;
			sumenergy += mfTreeMatrix_energy[actID][stpath.first];
			sumtime += mfTreeMatrix_time[actID][stpath.first];
			sumtime_tslot += mfTreeMatrix_timeslot[actID][stpath.first];

			actID = stpath.first;

			//cout << "S" << stpath.first << "(" << stpath.second*Generic::getInstance().timeSlot << "|" << actLossSens << ") ";
		}
		//cout << " GAIN: " << sumgain << endl;
		sumenergy += mfTreeMatrix_energy[actID][cn->u->id];
		sumtime += mfTreeMatrix_time[actID][cn->u->id];
		sumtime_tslot += mfTreeMatrix_timeslot[actID][cn->u->id];

		double sumGainTime = sumgain / sumtime;

		//if(sumgain > final_gain) {
		if(sumGainTime > final_gain) {
			final_pair = stprime;
			//final_gain = sumgain;
			final_gain = sumGainTime;

			pstat.total_gain = sumgain;
			pstat.total_loss = sumloss;
			pstat.energy_cost = sumenergy;
			pstat.time_cost = sumtime;
			pstat.time_cost_tslot = sumtime_tslot;
		}
	}
	//cout << endl << endl;

	path.clear();
	for (auto& f : pathmap[final_pair]) {
		path.push_back(sens_map[f.first]);
	}
}

double MultiFlow::calculateLossBSF(list<pair<int,int>> &phi, SensorNode *sn, int tm_tslot, bool centralized, UavDistributed *uav) {
	double ris = 0;
	double tm = ((double) tm_tslot) * Generic::getInstance().timeSlot;

	std::list<Sensor *> ssList;
	for (auto& sn : sens_list){
		ssList.push_back(sn->sens);
	}

	// check over the active readings
	for (auto& s : sens_list) {
		if (centralized){
			for (auto& r : s->readings) {
				double actLoss = Loss::algebraic_sum(
						Loss::getInstance().calculate_loss_distance(s->sens, sn->sens)
						* Loss::getInstance().calculate_loss_time(r.readTime, tm),
						Loss::getInstance().calculate_loss_energy(sn->sens, tm_tslot, ssList));

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
		else {
			for (auto& r : uav->sensMapTree[s->sens->id].timestamp_read) {
				list<double> eeList;
				for (auto& ee : uav->sensMapTree){
					if (ee.first != sn->sens->id) {
						eeList.push_back(ee.second.lastResidualEnergy);
					}
				}
				double actLoss = Loss::algebraic_sum(
						Loss::getInstance().calculate_loss_distance(s->sens, sn->sens)
						* Loss::getInstance().calculate_loss_time(r, tm),
						Loss::getInstance().calculate_loss_energy_onNumber(uav->sensMapTree[sn->sens->id].lastResidualEnergy, tm_tslot, eeList));

				double closestThenMe = 0;
				double myDist = uav->cn->pos.distance(sn->sens->coord);
				for (auto& u : uav->neighMap) {
					if (u.second.uav->cn->pos.distance(sn->sens->coord) < myDist) {
						closestThenMe += 1.0;
					}
				}

				//double discount = (closestThenMe + 1.0) / (((double) uav->neighMap.size()) + 1.0);
				double penalty = 0.0;
				if (uav->neighMap.size() > 0) {
					penalty = closestThenMe / ((double) uav->neighMap.size());
				}
				//cout << "        Inside calcLossSensor. Calculated discount: " << discount << endl << flush;

				//actLoss *= discount;
				actLoss = Loss::algebraic_sum(actLoss, penalty);

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
	}

	// check over the hypothetical readings
	for (auto& p : phi) {
		if (tm_tslot > p.second){
			double ptime = ((double) p.second) * Generic::getInstance().timeSlot;
			if (centralized){
				double actLoss = Loss::algebraic_sum(
						Loss::getInstance().calculate_loss_distance(sens_map[p.first]->sens, sn->sens)
						* Loss::getInstance().calculate_loss_time(ptime, tm),
						Loss::getInstance().calculate_loss_energy(sn->sens, tm_tslot, ssList));

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
			else {
				list<double> eeList;
				for (auto& ee : uav->sensMapTree){
					if (ee.first != sn->sens->id) {
						eeList.push_back(ee.second.lastResidualEnergy);
					}
				}
				double actLoss = Loss::algebraic_sum(
						Loss::getInstance().calculate_loss_distance(sens_map[p.first]->sens, sn->sens)
						* Loss::getInstance().calculate_loss_time(ptime, tm),
						Loss::getInstance().calculate_loss_energy_onNumber(uav->sensMapTree[sn->sens->id].lastResidualEnergy, tm_tslot, eeList));

				double closestThenMe = 0;
				double myDist = uav->cn->pos.distance(sn->sens->coord);
				for (auto& u : uav->neighMap) {
					if (u.second.uav->cn->pos.distance(sn->sens->coord) < myDist) {
						closestThenMe += 1.0;
					}
				}

				//double discount = (closestThenMe + 1.0) / (((double) uav->neighMap.size()) + 1.0);
				double penalty = 0.0;
				if (uav->neighMap.size() > 0) {
					penalty = closestThenMe / ((double) uav->neighMap.size());
				}
				//cout << "        Inside calcLossSensor. Calculated discount: " << discount << endl << flush;

				//actLoss *= discount;
				actLoss = Loss::algebraic_sum(actLoss, penalty);

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
	}

	return ris;
}

void MultiFlow::activateBSFandRecharge(ChargingNode *cnode, list<SensorNode *> &tsp) {
	double actTime = cnode->lastTimestamp;
	int actTime_tslot = cnode->lastTimestamp_tslot;
	MyCoord uav_pos = cnode->pos;
	double uav_energy = cnode->u->residual_energy;

	cerr << "Updating the UAV" << cnode->u->id
			<< " starting from time " << cnode->lastTimestamp << " at position " << uav_pos
			<< " with energy: " << cnode->u->residual_energy << " over max energy of " << cnode->u->max_energy
			<< "(ratio: " << (cnode->u->residual_energy / cnode->u->max_energy) << ")"
			<< endl;

	int oldID = cnode->u->id;

	cerr << "BSF: C" << cnode->u->id << "(" << actTime << ") ";
	for (auto& s : tsp) {
		// calculate time for the arc
		actTime += mfTreeMatrix_time[oldID][s->sens->id];
		actTime_tslot += mfTreeMatrix_timeslot[oldID][s->sens->id];
		uav_energy -= mfTreeMatrix_energy[oldID][s->sens->id];

		//move the UAV
		uav_pos = s->sens->coord;

		SensorNode::SensorRead sr;
		sr.readTime = actTime;
		sr.readTime_tslot = actTime_tslot;
		sr.uav = cnode->u;
		s->readings.push_back(sr);

		cerr << "S" << s->sens->id << "(" << actTime << ") ";

		if (RandomGenerator::getInstance().getRealUniform(0, 1) <= Generic::getInstance().pwakeup) {
			SensorNode::SensorRead rwu;
			rwu.readTime = actTime;
			rwu.readTime_tslot = actTime_tslot;
			rwu.uav = cnode->u;
			s->real_wakeup.push_back(rwu);

			if (RandomGenerator::getInstance().getRealUniform(0, 1) <= Generic::getInstance().pcomunication) {
				SensorNode::SensorRead rsr;
				rsr.readTime = actTime;
				rsr.readTime_tslot = actTime_tslot;
				rsr.uav = cnode->u;
				s->real_readings.push_back(rsr);
			}
		}

		if (actTime > s->lastTimestamp) {
			s->lastTimestamp = actTime;		// a cosa serve s->lastTimestamp?
			s->lastTimestamp_tslot = actTime_tslot;
		}

		oldID = s->sens->id;
	}

	if (oldID != cnode->u->id) {
		// calculate time for the arc
		actTime += mfTreeMatrix_time[oldID][cnode->u->id];
		actTime_tslot += mfTreeMatrix_timeslot[oldID][cnode->u->id];
		uav_energy -= mfTreeMatrix_energy[oldID][cnode->u->id];

		//move the UAV
		uav_pos = cnode->pos;

		cerr << "C" << cnode->u->id << "(" << actTime << ") " << endl;
	}

	//double tspTime = actTime;

	// calculate time to recharge
	//double e2recharge = cnode->u->max_energy - max(uav_energy, 0.0);
	//actTime += ((int) (ceil(e2recharge / Generic::getInstance().rechargeStation_power)));
	//actTime += e2recharge / Generic::getInstance().rechargeStation_power;

	cerr << "Updated the UAV" << cnode->u->id << " arriving at time " << actTime
			<< " at position " << uav_pos
			<< " with residual energy " << uav_energy << " energy"
			<< ", i.e. consuming " << (cnode->u->residual_energy - max(uav_energy, 0.0)) << " energy"
			<< endl << endl;

	cnode->lastTimestamp = actTime;
	cnode->lastTimestamp_tslot = actTime_tslot;
	cnode->u->residual_energy = uav_energy;
}



bool MultiFlow::updateSensorsEnergy_Tree(int starttime, int endtime) {
	bool ris = true;
	long double minEnergy = 1000000;
	SensorNode *snMin = nullptr;

	for (auto& s : sens_list) {
		// update the energy with the self-discharge
		for (int i = (starttime+1); i <= endtime; i++) {
			//s->sens->residual_energy -= calcPowEtaSens(s->sens->residual_energy) * Generic::getInstance().timeSlot;
			s->sens->residual_energy -= calcPowEtaSens(s->sens->residual_energy, Generic::getInstance().timeSlot);
		}

		for (auto& l : s->real_wakeup) {
			if ((l.readTime > starttime) && (l.readTime <= endtime)) {
				if (s->lastTimestamp < l.readTime) {
					s->lastTimestamp = l.readTime;
				}

				// update the energy because of the reading
				double pwu = 1;	//TODO
				s->sens->residual_energy -= sensor_energy_loss_read(pwu);

				/*if (s->lastTimestamp == l.readTime) {
					s->lastTimestamp += Generic::getInstance().tstartup + (Generic::getInstance().nr * Generic::getInstance().ttimeout);
					s->sens->residual_energy -= energy_loss_onArc(l.readTime);
				}
				else {
					cerr << "Error in updateSensorsEnergy" << endl;
					exit(EXIT_FAILURE);
				}*/
			}
		}

		if (s->sens->residual_energy < minEnergy) {
			minEnergy = s->sens->residual_energy;
			snMin = s;
		}

		if (s->sens->residual_energy <= 0) {
			ris = false;
		}
	}

	if (snMin != nullptr) {
		cout << "SENSOR S" << snMin->sens->id << " has minimal energy of " << snMin->sens->residual_energy << endl;
	}

	/*cout << "SENSORS ";
	cout << "- Eta: " << calcPowEtaSens((*sens_list.begin())->sens->residual_energy, Generic::getInstance().timeSlot);
	cout << "- ReadLoss: " << sensor_energy_loss_read(1) << " - ";
	for (auto& s : sens_list) {
		cout << "S" << s->sens->id << "(" << s->sens->residual_energy << ") ";
	}
	cout << endl;*/

	return ris;
}









void MultiFlow::run_tree_multiflow_distr(double end_time) {
	double sim_time = 0;
	int sim_time_tslot = 0;
	bool sensAlive = true;

	cout << "Initializing simulation" << endl << flush;

	//init uavs
	for (auto& uav : uav_list) {
		for (auto& s : sens_list) {
			uav->sensMap[s->sens->id].sens = s;
			uav->sensMap[s->sens->id].lastTimeStamp = sim_time;
			uav->sensMap[s->sens->id].lastTimeStamp_tslot = sim_time_tslot;
			uav->sensMap[s->sens->id].lastResidualEnergy = s->sens->residual_energy;

			uav->sensMapTree[s->sens->id].sens = s;
			uav->sensMap[s->sens->id].lastResidualEnergy = s->sens->residual_energy;
		}

		uav->us = UavDistributed::RECHARGING;
		uav->tsp2update = false;
		uav->rechargeBulk = 0;
	}


	cout << "Starting simulation" << endl << flush;

	while((sim_time <= end_time) && (sensAlive)) {


		//cout << "Updating NeighMaps" << endl << flush;
		//updateNeighMaps(sim_time);
		//cout << "End check UAVs AFTER updateNeighMaps" << endl << flush;


		for (auto& uav : uav_list) {
			//cout << "Running UAV " << uav->cn->u->id << endl << flush;
			run_uav_tree(uav, sim_time, sim_time_tslot);
		}

		// remove self-discharge energy from sensors
		//cout << "Removing self-discharge energy from sensors" << endl << flush;
		for (auto& s : sens_list) {
			s->sens->residual_energy -= calcPowEtaSens(s->sens->residual_energy, Generic::getInstance().timeSlot);

			if (s->sens->residual_energy < 0) {
				sensAlive = false;
			}
		}

		//cout << "Sim-Time: "<< sim_time << endl << flush;
		++sim_time_tslot;
		sim_time = ((double) sim_time_tslot) * Generic::getInstance().timeSlot;
	}

	cout << "Simulation FINISHED!!!" << endl;
}


void MultiFlow::run_uav_tree(UavDistributed *uav, double simTime, int simTime_tslot) {
	SensorNode *sn;

	switch (uav->us) {
	case UavDistributed::MOVING:
		//cout << "UAV " << uav->cn->u->id << " is in state MOVING" << endl << flush;
		uav->cn->u->residual_energy -= Generic::getInstance().pUfly * Generic::getInstance().timeSlot;
		if ((uav->activeTSP.size() == 0) && (uav->cn->u->actual_coord == uav->cn->pos)) {
			//arrived back to the charging station
			uav->cn->u->actual_coord = uav->cn->pos;	// set the exact position due to possible numerical error
			uav->us = UavDistributed::RECHARGING;
			//cout << "UAV " << uav->cn->u->id << " moving on state RECHARGING" << endl << flush;
		}
		else if ((uav->activeTSP.size() > 0) && (uav->cn->u->actual_coord == (*(uav->activeTSP.begin()))->sens->coord)) {
			//arrived to destination sensor

			//check if nobody is there
			sn = *(uav->activeTSP.begin());
			if (sn->irradiatingUAV == nullptr) {
				sn->irradiatingUAV = uav->cn->u;
				sn->irradiatingTimeSlots = 0;
				sn->accumulatedEnergy_uJ = 0;
				uav->us = UavDistributed::WAKINGUP;
				//cout << "UAV " << uav->cn->u->id << " moving on state WAKINGUP" << endl << flush;

				//every time I set this reading
				SensorNode::SensorRead sr;
				sr.readTime = simTime;
				sr.readTime_tslot = simTime_tslot;
				sr.uav = uav->cn->u;
				sn->readings.push_back(sr);

			}
			else {
				// the sensor is being used by another UAV. Go to the next
				uav->activeTSP.pop_front();
			}
		}
		else {
			MyCoord endP;

			/*if (uav->tsp2update) {
				calculateTSP_distributed(uav, uav->cn->u->actual_coord, uav->cn->pos, simTime);
				uav->tsp2update = false;
			}*/

			if (uav->activeTSP.size() == 0) endP = uav->cn->pos;
			else endP = (*(uav->activeTSP.begin()))->sens->coord;

			double dist2dest = uav->cn->u->actual_coord.distance(endP);
			double oneStepDist = ((double) Generic::getInstance().timeSlot) * Generic::getInstance().maxVelocity;
			if (oneStepDist >= dist2dest) {
				// I will arrive to destination
				uav->cn->u->actual_coord = endP;
			}
			else {
				MyCoord unit = endP - uav->cn->u->actual_coord;
				unit.normalize();
				unit *= oneStepDist;
				uav->cn->u->actual_coord += unit;
			}
		}
		break;

	case UavDistributed::WAKINGUP:
		//cout << "UAV " << uav->cn->u->id << " is in state WAKINGUP" << endl << flush;
		uav->cn->u->residual_energy -= Generic::getInstance().singleMotorPowerUAV * 4.0 * Generic::getInstance().timeSlot;
		uav->cn->u->residual_energy -= Generic::getInstance().wakeupTxPower * Generic::getInstance().timeSlot;

		if (uav->activeTSP.empty()){
			cerr << "uav->activeTSP is empty in WAKINGUP!" << endl;
			exit(EXIT_FAILURE);
		}
		sn = *(uav->activeTSP.begin());

		if (sn->irradiatingUAV == uav->cn->u) {
			// Ok I'm already irradiating
			//sn->accumulatedEnergy_uJ += wuVal.estimatedIrrEnergyPerSlot_uJ;
			sn->irradiatingTimeSlots++;
			/*if (sn->accumulatedEnergy_uJ >= Generic::getInstance().energyToWakeUp) {
				// OK the sensor wakes-up
				uav->us = UavDistributed::STARTINGUP;
				//cout << "UAV " << uav->cn->u->id << " moving on state STARTINGUP" << endl << flush;
				sn->startupTimeSlots = 0;
			}
			else */
			if(sn->irradiatingTimeSlots >= ceil(Generic::getInstance().twakeup / Generic::getInstance().timeSlot)) {
				// I irradiated, let's see if the sensor is woken-up
				if (RandomGenerator::getInstance().getRealUniform(0, 1) <= Generic::getInstance().pwakeup) {
					// OK the sensor wakes-up
					uav->us = UavDistributed::STARTINGUP;
					//cout << "UAV " << uav->cn->u->id << " moving on state STARTINGUP" << endl << flush;
					sn->startupTimeSlots = 0;

					//set node wake up the data
					SensorNode::SensorRead sr;
					sr.readTime = simTime;
					sr.readTime_tslot = simTime_tslot;
					sr.uav = uav->cn->u;
					sn->real_wakeup.push_back(sr);

					uav->sensMapTree[sn->sens->id].lastResidualEnergy = sn->sens->residual_energy;
				}
				else {
					//no wake-up... go ahead
					uav->activeTSP.pop_front();
					sn->irradiatingUAV = nullptr;
					uav->us = UavDistributed::MOVING;
				}
				//cout << "UAV " << uav->cn->u->id << " moving on state MOVING" << endl << flush;
			}
		}
		else {
			throw std::logic_error("Impossible that irradiating a sensor not mine");
		}
		break;

	case UavDistributed::STARTINGUP:
		//cout << "UAV " << uav->cn->u->id << " is in state STARTINGUP" << endl << flush;
		uav->cn->u->residual_energy -= Generic::getInstance().pUfly * Generic::getInstance().timeSlot;
		uav->cn->u->residual_energy -= Generic::getInstance().pUstartup * Generic::getInstance().timeSlot;

		if (uav->activeTSP.empty()){
			cerr << "uav->activeTSP is empty in STARTINGUP!" << endl;
			exit(EXIT_FAILURE);
		}
		sn = *(uav->activeTSP.begin());

		sn->sens->residual_energy -= Generic::getInstance().pSstartup * Generic::getInstance().timeSlot;

		sn->startupTimeSlots++;
		if(sn->startupTimeSlots >= ceil(Generic::getInstance().tstartup / Generic::getInstance().timeSlot)) {
			sn->commTimeSlots = 0;
			sn->nCommAttempt = 0;
			uav->us = UavDistributed::READING;
			//cout << "UAV " << uav->cn->u->id << " moving on state READING" << endl << flush;
		}
		break;

	case UavDistributed::READING:
		//cout << "UAV " << uav->cn->u->id << " is in state READING" << endl << flush;
		uav->cn->u->residual_energy -= Generic::getInstance().pUfly * Generic::getInstance().timeSlot;

		if (uav->activeTSP.empty()){
			cerr << "uav->activeTSP is empty in READING!" << endl;
			exit(EXIT_FAILURE);
		}
		sn = *(uav->activeTSP.begin());

		uav->cn->u->residual_energy -= (Generic::getInstance().pUrx +
				(	(Generic::getInstance().pUtx - Generic::getInstance().pUrx) / Generic::getInstance().ttimeout
				) ) * Generic::getInstance().timeSlot;

		sn->sens->residual_energy -= (Generic::getInstance().pSrx +
				(	(Generic::getInstance().pStx - Generic::getInstance().pSrx) / Generic::getInstance().ttimeout
				) ) * Generic::getInstance().timeSlot;

		sn->commTimeSlots++;
		if(sn->commTimeSlots >= ceil((Generic::getInstance().ttimeout * Generic::getInstance().nr) / Generic::getInstance().timeSlot)) {
			// ending nr timeout

			//sn->nCommAttempt++;
			if (RandomGenerator::getInstance().getRealUniform(0, 1) <= Generic::getInstance().pcomunication) {
				// ok successful data gathering

				//read the data
				SensorNode::SensorRead rsr;
				rsr.readTime = simTime;
				rsr.readTime_tslot = simTime_tslot;
				rsr.uav = uav->cn->u;
				sn->real_readings.push_back(rsr);

				uav->sensMapTree[sn->sens->id].timestamp_read.push_back(simTime);
				uav->sensMapTree[sn->sens->id].timestamp_read_tslot.push_back(simTime_tslot);
				uav->sensMapTree[sn->sens->id].lastResidualEnergy = sn->sens->residual_energy;

				uav->sensMap[sn->sens->id].lastTimeStamp = simTime;

				// update last read energy
				uav->sensMap[sn->sens->id].lastResidualEnergy = sn->sens->residual_energy;

				//go away
				sn->commTimeSlots = 0;
				sn->nCommAttempt = 0;
				uav->activeTSP.pop_front();
				sn->irradiatingUAV = nullptr;
				uav->us = UavDistributed::MOVING;
				//cout << "UAV " << uav->cn->u->id << " moving on state MOVING" << endl << flush;

				if (uav->tsp2update) {
					//calculateTSP_distributed(uav, uav->cn->u->actual_coord, uav->cn->pos, simTime);
					/*calculatePath(uav->activeTSP,
							uav->cn,
							uav->cn->lastTimestamp,
							uav->cn->lastTimestamp_tslot,
							uav->cn->u->residual_energy,
							false, uav);*/

					uav->tsp2update = false;
				}
			}
			else {
				// too much attempts

				// go away
				sn->commTimeSlots = 0;
				sn->nCommAttempt = 0;
				uav->activeTSP.pop_front();
				sn->irradiatingUAV = nullptr;
				uav->us = UavDistributed::MOVING;
				//cout << "UAV " << uav->cn->u->id << " moving on state MOVING" << endl << flush;

				if (uav->tsp2update) {
					//calculateTSP_distributed(uav, uav->cn->u->actual_coord, uav->cn->pos, simTime);

					/*calculatePath(uav->activeTSP,
							uav->cn,
							uav->cn->lastTimestamp,
							uav->cn->lastTimestamp_tslot,
							uav->cn->u->residual_energy,
							false, uav);*/

					uav->tsp2update = false;
				}
			}
		}

		break;

	case UavDistributed::RECHARGING:
	default:
		//cout << "UAV " << uav->cn->u->id << " is in state RECHARGING" << endl << flush;
		//cout << "  I'm recharging" << endl << flush;
		uav->cn->u->residual_energy += Generic::getInstance().rechargeStation_power * ((double) Generic::getInstance().timeSlot);
		if (uav->cn->u->residual_energy >= uav->cn->u->max_energy) {
			uav->cn->u->residual_energy = uav->cn->u->max_energy;
		}

		uav->rechargeBulk = max(0, uav->rechargeBulk - 1);

		if (uav->rechargeBulk == 0) {
			if (uav->activeTSP.size() > 0) {
				uav->us = UavDistributed::MOVING;
			}
			else {
				list<pair<pathStats, list<SensorNode *>>> bestTSPs;
				double uavEnergy = uav->cn->u->residual_energy;
				double uavTime = simTime;
				int uavTimeTSlot = simTime_tslot;
				int bulkExecuted = 0;
				int rechargeBulkSlot = ceil(
						(uav->cn->u->max_energy * Generic::getInstance().rechargeRatio) /
						(Generic::getInstance().timeSlot * Generic::getInstance().rechargeStation_power)
				);

				//do {
				while (true) {
					list<SensorNode *> bTSP;
					pathStats pstat;

					//cout << "U" << uav->cn->u->id << " calculating path with energy " << uavEnergy
					//		<< " and max energy of " << uav->cn->u->max_energy
					//		<< " with " << bulkExecuted << " bulks of recharge of size " << rechargeBulkSlot
					//		<< endl;

					calculatePath(bTSP, uav->cn, uavTime, uavTimeTSlot, uavEnergy, false, uav, pstat);
					if (bTSP.size() > 0) {
						bestTSPs.push_back(make_pair(pstat, bTSP));
					}
					//cout << "Calculated" << endl;

					if (uavEnergy >= uav->cn->u->max_energy) break;

					uavTimeTSlot += rechargeBulkSlot;
					uavTime = ((double) uavTimeTSlot) * Generic::getInstance().timeSlot;
					uavEnergy = min(uav->cn->u->max_energy, uavEnergy +
							(((double) rechargeBulkSlot) * Generic::getInstance().timeSlot * Generic::getInstance().rechargeStation_power));
					++bulkExecuted;
				//} while (uavEnergy < uav->cn->u->max_energy);
				}

				list<SensorNode *> bestTSP;
				pathStats topStat;

				switch (mfAlgoType) {
				case ALGO_BSF:
				case ALGO_DSF:
				default:
					for (auto& bTSP : bestTSPs) {
						topStat.total_gain = 0;
						topStat.time_cost = numeric_limits<double>::max();
						if ( 	(bTSP.first.total_gain > topStat.total_gain) ||
								((bTSP.first.total_gain == topStat.total_gain) && (bTSP.first.time_cost < topStat.time_cost)) ) {
							topStat = bTSP.first;
							bestTSP = bTSP.second;
						}
					}
					break;

				case ALGO_BSF_DISTANCE:
				case ALGO_BSF_ENERGY:
				case ALGO_DSF_DISTANCE:
				case ALGO_DSF_ENERGY:
					for (auto& bTSP : bestTSPs) {
						topStat.total_gain = numeric_limits<double>::max();
						topStat.time_cost = numeric_limits<double>::max();
						if ( (bTSP.first.total_gain < topStat.total_gain) ||
								((bTSP.first.total_gain == topStat.total_gain) && (bTSP.first.time_cost < topStat.time_cost)) ) {
							topStat = bTSP.first;
							bestTSP = bTSP.second;
						}
					}
					break;
				}

				if (bestTSP.size() > 0) {
					uav->rechargeBulk = rechargeBulkSlot * topStat.recharge_bulk;
					uav->activeTSP = bestTSP;

					cerr << "PATH at " << simTime << ": C" << uav->cn->u->id << " ";
					for (auto& s : uav->activeTSP) {
						cerr << "S" << s->sens->id << " ";
					}
					cerr << "C" << uav->cn->u->id
							<< " with time to recharge: " << uav->rechargeBulk << endl;

					if (uav->rechargeBulk == 0) {
						uav->us = UavDistributed::MOVING;
					}
				}


				/*

				double gainTSP = 0;
				double timeTSP = numeric_limits<double>::max();
				int bulkToRecharge = 0;

				cout << "Calculated path at time " << simTime << endl;
				for (auto& bTSP : bestTSPs) {
					double g = 0;
					double t = ((double) (bTSP.first * rechargeBulkSlot)) * Generic::getInstance().timeSlot;
					int actID = uav->cn->u->id;
					double actTime = simTime + t;
					double actTime_tslot = simTime_tslot + (bTSP.first * rechargeBulkSlot);
					list<pair<int, int>> lsensread;

					cout << "Recharging for " << bTSP.first << " bulks -> C" << uav->cn->u->id << "(" << actTime << ")";

					for (auto& el : bTSP.second) {
						double actArcTim = mfTreeMatrix_time[actID][el->sens->id];
						double actArcTimSlot = mfTreeMatrix_timeslot[actID][el->sens->id];

						t += actArcTim;
						actTime += actArcTim;
						actTime_tslot += actArcTimSlot;

						double actLossSens = calculateLossBSF(lsensread, sens_map[el->sens->id], actTime_tslot, true, nullptr);

						g += 1.0 - actLossSens;
						lsensread.push_back(make_pair(el->sens->id, actTime_tslot));

						cout << " S" << el->sens->id << "(" << actTime << "|" << actLossSens << ")";

						actID = el->sens->id;
					}
					t += mfTreeMatrix_time[actID][uav->cn->u->id];

					cout << " C" << uav->cn->u->id << "(" << actTime + mfTreeMatrix_time[actID][uav->cn->u->id] << ")" << endl;

					if ( (g > gainTSP) || ((g == gainTSP) && (t < timeTSP)) ) {
						gainTSP = g;
						timeTSP = t;
						bulkToRecharge = bTSP.first;
						bestTSP = bTSP.second;
					}

					//cout << "U" << uav->cn->u->id << " calculated path with " << bTSP.first << " bulks of recharge"
					//		<< " having gain: " << g << " and time " << t
					//		<< endl;
				}

				//cout << "The winner is the one with " << bulkToRecharge << " bulks of recharge"
				//			<< " having gain: " << gainTSP << " and time " << timeTSP
				//			<< endl;

				if (bestTSP.size() > 0) {
					uav->rechargeBulk = rechargeBulkSlot * bulkToRecharge;
					uav->activeTSP = bestTSP;

					cerr << "PATH at " << simTime << ": C" << uav->cn->u->id << " ";
					for (auto& s : uav->activeTSP) {
						cerr << "S" << s->sens->id << " ";
					}
					cerr << "C" << uav->cn->u->id
							<< " with time to recharge: " << uav->rechargeBulk << endl;

					if (uav->rechargeBulk == 0) {
						uav->us = UavDistributed::MOVING;
					}
				}*/
			}
		}

		/*{
			double batteryRatio = min(uav->cn->u->residual_energy / uav->cn->u->max_energy, 1.0);
			double checkNum = pow(batteryRatio, Generic::getInstance().bsfExponent);

			if ( 	(uav->cn->u->residual_energy >= uav->cn->u->max_energy) ||
					(RandomGenerator::getInstance().getRealUniform(0, 1) <= checkNum) ) {
				calculatePath(uav->activeTSP,
						uav->cn,
						uav->cn->lastTimestamp,
						uav->cn->lastTimestamp_tslot,
						uav->cn->u->residual_energy,
						false, uav);

				if (uav->activeTSP.size() > 0) {		// IDLE but calculated TSP... let's move!
					uav->us = UavDistributed::MOVING;
					//cout << "UAV " << uav->cn->u->id << " moving on state MOVING" << endl << flush;
				}
			}
		}*/
		break;
	}

}




















