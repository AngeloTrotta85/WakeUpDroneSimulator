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

MultiFlow::MultiFlow() {
	actSensorTimeStamp = 0;
	actUAVTimeStamp = 0;

	pWU = 0;
}

void MultiFlow::addSensor(Sensor *s) {
	SensorNode *newsn = new SensorNode();

	newsn->sens = s;
	newsn->lastTimestamp = 0;

	sens_list.push_back(newsn);
}

void MultiFlow::addChargStationAndUAV(MyCoord c, UAV *u) {
	ChargingNode *newcs = new ChargingNode();

	newcs->id = u->id;
	newcs->pos = c;
	newcs->u = u;
	newcs->lastTimestamp = 0;

	cs_map[u->id] = newcs;
}

void MultiFlow::addChargStationAndUAV_distributed(MyCoord c, UAV *u) {
	ChargingNode *newcs = new ChargingNode();

	newcs->id = u->id;
	newcs->pos = c;
	newcs->u = u;
	newcs->lastTimestamp = 0;

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

double MultiFlow::calcLossSensor(SensorNode *s_check, std::list<SensorNode *> &sList, int texp) {
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

double MultiFlow::calcLossSensorPresentFuture(SensorNode *s_check, std::list<SensorNode *> &sList, int texp) {
	double ris = 0;

	for (auto& s : sList) {
		for (auto& r : s->readings) {
			double actLoss = Loss::getInstance().calculate_loss_distance(s->sens, s_check->sens)
								+ Loss::getInstance().calculate_loss_time(r.readTime, texp);

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
	double ris = 0;
	double pwu = 1;	//TODO

	ris += calcEnergyToTravel(e->first->sens->coord, e->second->sens->coord);	//time to travel
	ris += calcEnergyToWuData(pwu);	//time to wake-up

	return ris;
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
		Sensor *uavDummySensor = new Sensor(cnode->pos, 1, TSP_UAV_CODE);
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
			if (fe->second->sens->id != TSP_UAV_CODE) {
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

		for (auto& ff : finaledges) free(ff);
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
			double actIndex = 1.0 - calcLossSensor(s, sens_list, r.readTime);
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

	for (auto& uav : uav_list) {
		for (auto& s : sens_list) {
			UavDistributed::sensElem ns;
			ns.sens = s;
			ns.lastTimeStamp = sim_time;

			uav->sensMap[s->sens->id] = ns;
		}
	}

	while(sim_time <= end_time) {

		updateNeighMaps(sim_time);

		for (auto& uav : uav_list) {
			run_uav(uav, sim_time);
		}

		sim_time += Generic::getInstance().timeSlot;
	}

	cerr << "Simulation FINISHED!!!" << endl;
}

void MultiFlow::updateNeighMaps(double timenow) {
	for (auto& u1 : uav_list) {
		for (auto& u2 : uav_list) {
			if (u1->cn->id != u2->cn->id) {
				if (u1->cn->u->actual_coord.distance(u2->cn->u->actual_coord) <= Generic::getInstance().uavComRange) {
					UavDistributed::neighUAV nuav;
					nuav.uav = u2;
					nuav.lastTimeStamp = timenow;

					u1->neighMap[u2->cn->id] = nuav;
				}
			}
		}
	}
}

void MultiFlow::run_uav(UavDistributed *uav, double simTime) {

}




