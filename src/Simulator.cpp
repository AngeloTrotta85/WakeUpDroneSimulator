/*
 * Simulator.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include <iostream>
#include <fstream>

#include "Simulator.h"
#include "Generic.h"
#include "Statistics.h"
#include "RandomGenerator.h"
#include "Loss.h"

#include "clustering/ClusteringEqRandomLoss.h"
#include "clustering/ClusteringKMeans.h"
#include "clustering/ClusteringKMeansRead.h"
#include "clustering/ClusteringRandom.h"
#include "clustering/ClusteringRRobinMinimumLoss.h"
#include "clustering/ClusteringRRobinMinimumLossLocal.h"

#include "tsp/TSPRandom.h"
#include "tsp/TSPNoFullRandom.h"
#include "tsp/TSP2Opt.h"
#include "tsp/TSP2OptEnergy.h"
#include "tsp/TSP2OptEnergyMinDist.h"
#include "tsp/TSP2OptEnergyMinEnergy.h"

#ifndef TSP_UAV_CODE
#define TSP_UAV_CODE 100000
#endif

using namespace std;

Simulator::Simulator() {
	simulation_time = 0;
	end_time = -1;

	timeSlot = 1;

	endSimulation = false;
	makeLog = false;

	clust = nullptr;
	tsp = nullptr;
	mainalgo = ALGO_BEE;
	simtype = SIMU_NORMAL;

	mf = nullptr;
}

void Simulator::init(Simu_type st, int stime, int etime) {
	simtype = st;
	simulation_time = stime;
	end_time = etime;
}

void Simulator::setMainAlgo(std::string algotype_main) {
	if (!algotype_main.empty()) {
		if (algotype_main.compare("bee") == 0) {
			mainalgo = ALGO_BEE;
		}
		else if (algotype_main.compare("beenc") == 0) {
			mainalgo = ALGO_BEE_NOCLUST;
		}
		else if (algotype_main.compare("close") == 0) {
			mainalgo = ALGO_CLOSEST;
		}
		else if (algotype_main.compare("lowbatt") == 0) {
			mainalgo = ALGO_LOWERBATT;
		}
		else {
			std::cerr << "Unknown algotype for main: \"" << algotype_main << "\". Using default BEE-DRONES" << std::endl;
			mainalgo = ALGO_BEE;
		}
	}
	else {
		//default simple k-means
		std::cerr << "Undefined algotype for main. Using default BEE-DRONES" << std::endl;
		mainalgo = ALGO_BEE;
	}
}

void Simulator::setClusteringAlgo(std::string algotype_clustering) {
	if (!algotype_clustering.empty()) {
		if (algotype_clustering.compare("kmS") == 0) {
			clust = new ClusteringKMeans(); //simple k-means
		}
		else if (algotype_clustering.compare("kmR") == 0) {
			clust = new ClusteringKMeansRead();	//k-means with loss in the distance calculus
		}
		else if (algotype_clustering.compare("eqLoss") == 0) {
			clust = new ClusteringRandom();	// just random for now
		}
		else if (algotype_clustering.compare("eqRandLoss") == 0) {
			clust = new ClusteringEqRandomLoss();	// from each random sensor choose the closest cluster
		}
		else if (algotype_clustering.compare("rrMinLoss") == 0) {
			clust = new ClusteringRRobinMinimumLoss();	// round robin cluster, choose mis loss sensor
		}
		else if (algotype_clustering.compare("rrMinLossLocal") == 0) {
			clust = new ClusteringRRobinMinimumLossLocal();	// round robin cluster, choose min loss sensor
		}
		else {
			std::cerr << "Unknown algotype for clustering: \"" << algotype_clustering << "\". Using default simple k-means" << std::endl;
			clust = new ClusteringKMeans(); //simple k-means
		}
	}
	else {
		//default simple k-means
		std::cerr << "Undefined algotype for clustering. Using default simple k-means" << std::endl;
		clust = new ClusteringKMeans(); //simple k-means
	}
}

void Simulator::setAlgoType(MultiFlow::Algo_type algotype) {
	mfAT = algotype;
}

void Simulator::setTSPAlgo(std::string algotype_tsp) {
	if (!algotype_tsp.empty()) {
		if (algotype_tsp.compare("frTSP") == 0) {
			tsp = new TSPRandom(); //full random TSP
		}
		else if (algotype_tsp.compare("nfrTSP") == 0) {
			tsp = new TSPNoFullRandom(); //no full random TSP
		}
		else if (algotype_tsp.compare("tsp2opt") == 0) {
			tsp = new TSP2Opt(); // 2-opt algorithm
		}
		else if (algotype_tsp.compare("tsp2optE") == 0) {
			tsp = new TSP2OptEnergy(); // 2-opt algorithm with energy constraint
		}
		else if (algotype_tsp.compare("tsp2optEMinDist") == 0) {
			tsp = new TSP2OptEnergyMinDist(); // 2-opt algorithm with energy constraint - min distance
		}
		else if (algotype_tsp.compare("tsp2optEMinEnergy") == 0) {
			tsp = new TSP2OptEnergyMinEnergy(); // 2-opt algorithm with energy constraint - min energy
		}
		else {
			cerr << "Unknown algotype for tsp: \"" << algotype_tsp << "\". Using default full random TSP" << endl;
			tsp = new TSPRandom(); //full random TSP
		}
	}
	else {
		//default simple k-means
		cerr << "Undefined algotype for tsp. Using default full random TSP" << endl;
		tsp = new TSPRandom(); //full random TSP
	}
}

void Simulator::finish(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	switch (simtype) {
		case SIMU_NORMAL:
		default:
			finish_normal(clustVec, sensList, allReadings);
			break;

		case SIMU_MULTI_FLOW:
			finish_multiflow(clustVec, sensList, allReadings);
			break;

		case SIMU_DISTRIBUTED:
			finish_distributed(clustVec, sensList, allReadings);
			break;

		case SIMU_TREE_MULTI_FLOW:
			finish_tree_multiflow(clustVec, sensList, allReadings);
			break;

		case SIMU_TREE_MULTI_FLOW_DISTR:
			finish_tree_multiflow_distr(clustVec, sensList, allReadings);
			break;
	}
}

void Simulator::finish_normal(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {

	if (!Generic::getInstance().statFilename.empty()) {
		double avg, min, max, var;

		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out | std::ofstream::app);
		if (ofs.is_open()) {
			ofs << "FINISH LifetimeSecs " << simulation_time << endl;
			ofs << "FINISH LifetimeDays " << ((double) simulation_time) / 86400.0 << endl;

			ofs << "FINISH Index " << Statistics::getInstance().calculate_andSave_index(simulation_time, clustVec, sensList, false) << endl;
			ofs << "FINISH CorrelationGain " << Loss::getInstance().calculate_correlationGain_full_reading(simulation_time, sensList) << endl;
			ofs << "FINISH CorrelationLoss " << Loss::getInstance().calculate_correlationLoss_full_reading(simulation_time, sensList) << endl;
			ofs << "FINISH EnergyGain " << Loss::getInstance().calculate_energyGain_full_reading(simulation_time, sensList) << endl;
			ofs << "FINISH EnergyLoss " << Loss::getInstance().calculate_energyLoss_full_reading(simulation_time, sensList) << endl;

			Statistics::getInstance().calculate_minmax_sensor_energy(sensList, avg, min, max, var);
			ofs << "FINISH EnergySensorsAvg " << avg << endl;
			ofs << "FINISH EnergySensorsMin " << min << endl;
			ofs << "FINISH EnergySensorsMax " << max << endl;
			ofs << "FINISH EnergySensorsDiff " << (max - min) << endl;
			ofs << "FINISH EnergySensorsVar " << var << endl;
			ofs << "FINISH EnergySensorsStddev " << sqrt(var) << endl;

			ofs << "FINISH AvgIndexDerivative " << Statistics::getInstance().calculateAvgIndexDerivative() << endl;
			ofs << "FINISH NumberReadings " << allReadings.size() << endl;

			Statistics::getInstance().calculate_minmax_uav_charge_time(clustVec, avg, min, max, var);
			ofs << "FINISH UavChargeTimesAvg " << avg << endl;
			ofs << "FINISH UavChargeTimesMin " << min << endl;
			ofs << "FINISH UavChargeTimesMax " << max << endl;
			ofs << "FINISH UavChargeTimesDiff " << (max - min) << endl;
			ofs << "FINISH UavChargeTimesVar " << var << endl;
			ofs << "FINISH UavChargeTimesStddev " << sqrt(var) << endl;

			Statistics::getInstance().calculate_minmax_sensor_visiting(simulation_time, clustVec, sensList, avg, min, max, var);
			ofs << "FINISH VisitingSensorsAvg " << avg << endl;
			ofs << "FINISH VisitingSensorsMin " << min << endl;
			ofs << "FINISH VisitingSensorsMax " << max << endl;
			ofs << "FINISH VisitingSensorsDiff " << (max - min) << endl;
			ofs << "FINISH VisitingSensorsVar " << var << endl;
			ofs << "FINISH VisitingSensorsStddev " << sqrt(var) << endl;

			ofs.close();
		}
	}

	if (!Generic::getInstance().hitmapFilename.empty()) {
		std::ofstream ofs (Generic::getInstance().hitmapFilename, std::ofstream::out);
		if (ofs.is_open()) {
			for (auto& s : sensList) {
				ofs << s->id << " " << s->coord.x << " " << s->coord.y << " " << s->mySensorReadings.size() << endl;
			}
			ofs.close();
		}
	}
}

void Simulator::run(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	switch (simtype) {
		case SIMU_NORMAL:
		default:
			run_normal(clustVec, sensList, allReadings);
			break;

		case SIMU_MULTI_FLOW:
			run_multiflow(clustVec, sensList, allReadings);
			break;

		case SIMU_DISTRIBUTED:
			run_distributed(clustVec, sensList, allReadings);
			break;

		case SIMU_TREE_MULTI_FLOW:
			run_tree_multiflow(clustVec, sensList, allReadings);
			break;

		case SIMU_TREE_MULTI_FLOW_DISTR:
			run_tree_multiflow_distr(clustVec, sensList, allReadings);
			break;
	}
}

void Simulator::run_normal(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	if (!Generic::getInstance().statFilename.empty()) {
		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out);
		if (ofs.is_open()) {
			ofs << "";
			ofs.close();
		}
	}

	timeSlot = Generic::getInstance().timeSlot;
	makeLog = false;
	endSimulation = false;

	while (((end_time < 0) || (simulation_time < end_time)) && (!endSimulation)) {

		if ((simulation_time % 10000000) == 0) makeLog = true;
		else makeLog = false;

		if (makeLog) cout << "Simulation time: " << ((double) simulation_time) / (3600 * 24) << " - ";

		// move and update the UAVs
		for (auto& c : clustVec) {
			if (makeLog) cout << "[UAV" << c->clusterUAV->id << " S" << c->clusterUAV->state << " E" << c->clusterUAV->residual_energy
					<< " P" <<c->clusterUAV->actual_coord << " ";

			switch (c->clusterUAV->state) {
			case UAV::IDLE:

				switch (mainalgo) {
				case ALGO_BEE:
				default:
					calc_path_bee(clustVec, sensList, allReadings, c);
					break;

				case ALGO_BEE_NOCLUST:
					calc_path_beenoclust(clustVec, sensList, allReadings, c);
					break;

				case ALGO_CLOSEST:
					calc_path_closest(clustVec, sensList, allReadings, c);
					break;

				case ALGO_LOWERBATT:
					calc_path_lowerbatt(clustVec, sensList, allReadings, c);
					break;
				}

				if (c->pointsTSP_listFinal.size() > 0) {		// IDLE but calculated TSP... let's move!
					c->clusterUAV->state = UAV::MOVING;

					c->startingSensor = nullptr;
					c->nextSensor = *c->pointsTSP_listFinal.begin();
				}
				break;

			case UAV::MOVING:
				c->clusterUAV->residual_energy -= Generic::getInstance().singleMotorPowerUAV * 4.0 * Generic::getInstance().timeSlot;
				if ((c->nextSensor == nullptr) && (c->clusterUAV->actual_coord == c->clusterUAV->recharge_coord)) {
					//arrived back to the charging station
					c->clusterUAV->state = UAV::RECHARGING;

					//reset the variables
					c->nextSensor = c->startingSensor = nullptr;

					//unset the chosen sensors as booked
					for (auto& s: c->pointsTSP_listFinal) {
						s->uavBookedReading[c->clusterUAV->id] = false;
					}
					c->pointsTSP_listFinal.clear();
				}
				else if ((c->nextSensor != nullptr) && (c->clusterUAV->actual_coord == c->nextSensor->coord)) {
					//arrived to destination sensor
					c->clusterUAV->state = UAV::WAKINGUP_READING;

					c->timeSpentInWakeRead = 0;	//reset variable
				}
				else {
					MyCoord endP;
					if (c->nextSensor == nullptr) endP = c->clusterUAV->recharge_coord;
					else endP = c->nextSensor->coord;

					double dist2dest = c->clusterUAV->actual_coord.distance(endP);
					double oneStepDist = ((double) Generic::getInstance().timeSlot) * Generic::getInstance().maxVelocity;
					if (oneStepDist >= dist2dest) {
						// I will arrive to destination
						c->clusterUAV->actual_coord = endP;
					}
					else {
						MyCoord unit = endP - c->clusterUAV->actual_coord;
						unit.normalize();
						unit *= oneStepDist;
						c->clusterUAV->actual_coord += unit;
					}
				}
				break;

			case UAV::RECHARGING:
				c->clusterUAV->residual_energy += Generic::getInstance().rechargeStation_power * ((double) Generic::getInstance().timeSlot);
				if (c->clusterUAV->residual_energy >= c->clusterUAV->max_energy) {
					c->clusterUAV->residual_energy = c->clusterUAV->max_energy;
					c->clusterUAV->state = UAV::IDLE;
					c->clusterUAV->chargeCount++;
				}
				break;

			case UAV::WAKINGUP_READING:
				if (makeLog) cout << " R" << c->nextSensor->id;
				c->clusterUAV->residual_energy -= Generic::getInstance().singleMotorPowerUAV * 4.0 * Generic::getInstance().timeSlot;
				if (c->timeSpentInWakeRead >= Generic::getInstance().getTime2WakeRead(c->clusterUAV->actual_coord, c->nextSensor->coord)) {
					// ok, finished
					c->clusterUAV->state = UAV::MOVING;

					//remove at the end the wakeup+reading energy
					c->clusterUAV->residual_energy -= Generic::getInstance().getEnergy2WakeRead(c->clusterUAV->actual_coord, c->nextSensor->coord);
					c->nextSensor->residual_energy -= Generic::getInstance().energyBOOT + Generic::getInstance().energyON;

					//Add the new Reading
					Readings *read_new = new Readings(c->nextSensor, c->clusterUAV, simulation_time, RandomGenerator::getInstance().getRealUniform(0.0, 1.0));
					Loss::getInstance().calculate_reading_par(simulation_time, read_new, sensList);
					c->nextSensor->mySensorReadings.push_front(read_new);
					c->clusterUAV->mySensorReadings.push_front(read_new);
					allReadings.push_front(read_new);
					//c->nextSensor->mySensorReadings.push_back(read_new);
					//c->clusterUAV->mySensorReadings.push_back(read_new);
					//allReadings.push_back(read_new);

					// remove the booking
					c->nextSensor->uavBookedReading[c->clusterUAV->id] = false;

					if (c->nextSensor->residual_energy <= 0) {
						endSimulation = true;
					}

					cout << "UAV" << c->clusterUAV->id << " visited S" << c->nextSensor->id << " at time " << simulation_time << endl;
					cout << "Made new reading with:" <<
							" fullLoss: " << read_new->full_loss <<
							" fullGain: " << read_new->gain <<
							" correlationLoss: " << read_new->correlation_loss <<
							" correlationGain: " << read_new->correlation_gain <<
							" energyLoss: " << read_new->energy_loss <<
							" energyGain: " << read_new->energy_gain <<
							endl;

					c->timeSpentInWakeRead = 0;

					c->startingSensor = c->nextSensor;
					for (auto it = c->pointsTSP_listFinal.begin(); it != c->pointsTSP_listFinal.end(); it++) {
						if ((*it)->id == c->nextSensor->id) {
							it++;
							if (it != c->pointsTSP_listFinal.end()) {
								c->nextSensor = *it;
							}
							else {
								c->nextSensor = nullptr;
							}
							break;
						}
					}
				}
				else {
					c->timeSpentInWakeRead += Generic::getInstance().timeSlot;
				}
				break;
			}

			if (c->clusterUAV->residual_energy < 0) {
				cerr << "UAV" << c->clusterUAV->id << " run out of energy" << endl;
				exit(EXIT_FAILURE);
			}

			if (makeLog) cout << "] ";
		}

		for (auto& ss : sensList) {
			if (makeLog) cout << "[S" << ss->id << " E" << ss->residual_energy << "]";

			//selfdischarge
			ss->residual_energy = ((long double) ss->residual_energy) * Generic::getInstance().sensorSelfDischargePerSlot;
		}
		if (makeLog) cout << endl << endl;

		//if (simulation_time > 100) break;  //TODO remove

		//STATISTICS
		if ((Generic::getInstance().makeRunSimStat) && (!Generic::getInstance().statFilename.empty())) {
			if (Statistics::getInstance().isTimeToLog(simulation_time)) {
				double avg, min, max, var;

				std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out | std::ofstream::app);
				if (ofs.is_open()) {
					ofs << "SIMULATION TimeSecs " << simulation_time << endl;
					ofs << "SIMULATION TimeDays " << ((double) simulation_time) / 86400.0 << endl;

					ofs << "SIMULATION Index " << Statistics::getInstance().calculate_andSave_index(simulation_time, clustVec, sensList, true) << endl;
					ofs << "SIMULATION CorrelationGain " << Loss::getInstance().calculate_correlationGain_full_reading(simulation_time, sensList) << endl;
					ofs << "SIMULATION CorrelationLoss " << Loss::getInstance().calculate_correlationLoss_full_reading(simulation_time, sensList) << endl;
					ofs << "SIMULATION EnergyGain " << Loss::getInstance().calculate_energyGain_full_reading(simulation_time, sensList) << endl;
					ofs << "SIMULATION EnergyLoss " << Loss::getInstance().calculate_energyLoss_full_reading(simulation_time, sensList) << endl;

					Statistics::getInstance().calculate_actual_minmax_sensor_energy(sensList, avg, min, max, var);
					ofs << "SIMULATION EnergySensorsAvg " << avg << endl;
					ofs << "SIMULATION EnergySensorsMin " << min << endl;
					ofs << "SIMULATION EnergySensorsMax " << max << endl;
					ofs << "SIMULATION EnergySensorsDiff " << (max - min) << endl;
					ofs << "SIMULATION EnergySensorsVar " << var << endl;
					ofs << "SIMULATION EnergySensorsStddev " << sqrt(var) << endl;

					ofs << "SIMULATION AvgIndexDerivative " << Statistics::getInstance().calculateActualAvgIndexDerivative() << endl;
					ofs << "SIMULATION NumberReadings " << allReadings.size() << endl;

					Statistics::getInstance().calculate_actual_minmax_uav_charge_time(clustVec, avg, min, max, var);
					ofs << "SIMULATION UavChargeTimesAvg " << avg << endl;
					ofs << "SIMULATION UavChargeTimesMin " << min << endl;
					ofs << "SIMULATION UavChargeTimesMax " << max << endl;
					ofs << "SIMULATION UavChargeTimesDiff " << (max - min) << endl;
					ofs << "SIMULATION UavChargeTimesVar " << var << endl;
					ofs << "SIMULATION UavChargeTimesStddev " << sqrt(var) << endl;

					Statistics::getInstance().calculate_actual_minmax_sensor_visiting(simulation_time, clustVec, sensList, avg, min, max, var);
					ofs << "SIMULATION VisitingSensorsAvg " << avg << endl;
					ofs << "SIMULATION VisitingSensorsMin " << min << endl;
					ofs << "SIMULATION VisitingSensorsMax " << max << endl;
					ofs << "SIMULATION VisitingSensorsDiff " << (max - min) << endl;
					ofs << "SIMULATION VisitingSensorsVar " << var << endl;
					ofs << "SIMULATION VisitingSensorsStddev " << sqrt(var) << endl;

					ofs << endl;

					ofs.close();
				}

				Statistics::getInstance().logging(simulation_time);
			}
		}

		simulation_time += timeSlot;
	}
}

void Simulator::cluster_and_tour(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, CoordCluster *actClust) {
	actClust->pointsTSP_listFinal.clear();
	clust->cluster(clustVec, sensList, simulation_time, actClust->clusterUAV->id);

	cout << "Calculating TSP" << endl << flush;
	tsp->calculateTSP(actClust, sensList, simulation_time);
}


void Simulator::calc_path_bee(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings, CoordCluster *actClust) {
	//cout << endl << "UAV" << c->clusterUAV->id << " is choosing its tour" << endl;
	cluster_and_tour(clustVec, sensList, actClust);

	//set the chosen sensors as booked
	for (auto& s: actClust->pointsTSP_listFinal) {
		s->uavBookedReading[actClust->clusterUAV->id] = true;
	}
}

void Simulator::calc_path_beenoclust(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings, CoordCluster *actClust) {
	actClust->pointsTSP_listFinal.clear();
	//clust->cluster(clustVec, sensList, simulation_time, c->clusterUAV->id);
	actClust->pointsList.clear();
	for (auto& ss : sensList) {
		bool isOthers = false;
		for (auto& cc : clustVec) {
			for (auto& sb : cc->pointsTSP_listFinal){
				if (ss->id == sb->id) {
					isOthers = true;
					break;
				}
			}
			if (isOthers) break;
		}
		if (!isOthers) {
			actClust->pointsList.push_back(ss);
		}
	}
	//update cluster head (useless)
	actClust->clusterHead->x = actClust->clusterHead->y = actClust->clusterHead->z = 0;// = MyCoord(0, 0);
	for (auto& ss : actClust->pointsList) {
		actClust->clusterHead->x += ss->coord.x;
		actClust->clusterHead->y += ss->coord.y;
	}
	if (actClust->pointsList.size() > 0) {
		actClust->clusterHead->x /= (double) actClust->pointsList.size();
		actClust->clusterHead->y /= (double) actClust->pointsList.size();
	}

	cout << "Calculating TSP" << endl << flush;
	tsp->calculateTSP(actClust, sensList, simulation_time);
}

void Simulator::calc_path_closest(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings, CoordCluster *actClust) {

	// are all the same, change only the tsp algorithm
	calc_path_beenoclust(clustVec, sensList, allReadings, actClust);


	/*
	actClust->pointsTSP_listFinal.clear();
	//clust->cluster(clustVec, sensList, simulation_time, c->clusterUAV->id);
	actClust->pointsList.clear();
	for (auto& ss : sensList) {
		actClust->pointsList.push_back(ss);
	}
	//update cluster head (useless)
	actClust->clusterHead->x = actClust->clusterHead->y = actClust->clusterHead->z = 0;// = MyCoord(0, 0);
	for (auto& ss : actClust->pointsList) {
		actClust->clusterHead->x += ss->coord.x;
		actClust->clusterHead->y += ss->coord.y;
	}
	if (actClust->pointsList.size() > 0) {
		actClust->clusterHead->x /= (double) actClust->pointsList.size();
		actClust->clusterHead->y /= (double) actClust->pointsList.size();
	}

	cout << "Calculating TSP" << endl << flush;
	tsp->calculateTSP(actClust, sensList, simulation_time);
	*/
}

void Simulator::calc_path_lowerbatt(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings, CoordCluster *actClust) {

	// are all the same, change only the tsp algorithm
	calc_path_beenoclust(clustVec, sensList, allReadings, actClust);


	/*
	actClust->pointsTSP_listFinal.clear();
	//clust->cluster(clustVec, sensList, simulation_time, c->clusterUAV->id);
	actClust->pointsList.clear();
	for (auto& ss : sensList) {
		actClust->pointsList.push_back(ss);
	}
	//update cluster head (useless)
	actClust->clusterHead->x = actClust->clusterHead->y = actClust->clusterHead->z = 0;// = MyCoord(0, 0);
	for (auto& ss : actClust->pointsList) {
		actClust->clusterHead->x += ss->coord.x;
		actClust->clusterHead->y += ss->coord.y;
	}
	if (actClust->pointsList.size() > 0) {
		actClust->clusterHead->x /= (double) actClust->pointsList.size();
		actClust->clusterHead->y /= (double) actClust->pointsList.size();
	}

	cout << "Calculating TSP" << endl << flush;
	tsp->calculateTSP(actClust, sensList, simulation_time);
	*/
}












void Simulator::run_multiflow(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	if (!Generic::getInstance().statFilename.empty()) {
		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out);
		if (ofs.is_open()) {
			ofs << "";
			ofs.close();
		}
	}

	mf = new MultiFlow(mfAT);
	mf->init();

	for (auto& cv : clustVec) {
		mf->addChargStationAndUAV(*(cv->clusterHead), cv->clusterUAV);
	}
	for (auto& s : sensList) {
		mf->addSensor(s);
	}

	mf->run(end_time);

}


void Simulator::finish_multiflow(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	if (!Generic::getInstance().statFilename.empty()) {

		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out | std::ofstream::app);
		if (ofs.is_open()) {

			double lifetimeSec = mf->getLastSensorRead();
			ofs << "FINISH LifetimeSecs " << lifetimeSec << endl;
			ofs << "FINISH LifetimeDays " << ((double) lifetimeSec) / 86400.0 << endl;

			double index = mf->calcIndex();
			ofs << "FINISH Index " << index << endl;

		}
	}

	if (!Generic::getInstance().hitmapFilename.empty()) {
		/*std::ofstream ofs (Generic::getInstance().hitmapFilename, std::ofstream::out);
		if (ofs.is_open()) {
			for (auto& s : sensList) {
				ofs << s->id << " " << s->coord.x << " " << s->coord.y << " " << s->mySensorReadings.size() << endl;
			}
			ofs.close();
		}*/
		mf->writeHitmaps_multiflow(Generic::getInstance().hitmapFilename);
	}
}








void Simulator::run_distributed(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	if (!Generic::getInstance().statFilename.empty()) {
		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out);
		if (ofs.is_open()) {
			ofs << "";
			ofs.close();
		}
	}

	mf = new MultiFlow(mfAT);
	mf->init();

	for (auto& cv : clustVec) {
		mf->addChargStationAndUAV_distributed(*(cv->clusterHead), cv->clusterUAV);
	}
	for (auto& s : sensList) {
		mf->addSensor(s);
	}

	mf->run_distributed(end_time);

}


void Simulator::finish_distributed(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	if (!Generic::getInstance().statFilename.empty()) {

		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out | std::ofstream::app);
		if (ofs.is_open()) {

			double lifetimeSec = mf->getLastSensorRead();
			ofs << "FINISH LifetimeSecs " << lifetimeSec << endl;
			ofs << "FINISH LifetimeDays " << ((double) lifetimeSec) / 86400.0 << endl;

			double index = mf->calcIndex();
			ofs << "FINISH Index " << index << endl;

		}
	}

	if (!Generic::getInstance().hitmapFilename.empty()) {
		/*std::ofstream ofs (Generic::getInstance().hitmapFilename, std::ofstream::out);
		if (ofs.is_open()) {
			for (auto& s : sensList) {
				ofs << s->id << " " << s->coord.x << " " << s->coord.y << " " << s->mySensorReadings.size() << endl;
			}
			ofs.close();
		}*/
		mf->writeHitmaps_distr(Generic::getInstance().hitmapFilename);
	}
}












void Simulator::run_tree_multiflow(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	/*if (!Generic::getInstance().statFilename.empty()) {
		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out);
		if (ofs.is_open()) {
			ofs << "";
			ofs.close();
		}
	}*/

	mf = new MultiFlow(mfAT);
	mf->init();

	for (auto& cv : clustVec) {
		mf->addChargStationAndUAV(*(cv->clusterHead), cv->clusterUAV);
	}
	for (auto& s : sensList) {
		mf->addSensor(s);
	}

	//mf->init_treeMF(end_time, Generic::getInstance().timeSlot);
	mf->init_matrix_treeMF();
	mf->run_tree_multiflow(end_time);

}


void Simulator::finish_tree_multiflow(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	if (!Generic::getInstance().statFilename.empty()) {

		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out | std::ofstream::app);
		if (ofs.is_open()) {

			double lifetimeSec = mf->getLastSensorRead();
			double lifetimeHour = ((double) lifetimeSec) / 3600.0;
			double lifetimeDay = ((double) lifetimeSec) / 86400.0;
			//ofs << "FINISH LifetimeSecs " << lifetimeSec << endl;
			//ofs << "FINISH LifetimeDays " << lifetimeDay << endl;

			double index = mf->calcIndex_Tree();
			//ofs << "FINISH Index " << index << endl;
			//ofs << "FINISH IndexOverHour " << (index / lifetimeHour) << endl;
			//ofs << "FINISH IndexOverDay " << (index / lifetimeDay) << endl;

			int readings = mf->calcNumRead_Tree();
			double avg_read = ((double) readings) / ((double) sensList.size());

			double minGain, maxGain, varGain, avgGain;
			mf->calcFinalGainsTree(minGain, maxGain, varGain, avgGain);

			int numRechargeTot = mf->calcNumRecharge_Tot();
			double numRechargeRelative = ((double) numRechargeTot) / ((double) clustVec.size());

			double numRechargeTimeTot = mf->calcNumRechargeTime_Tot();
			double numRechargeTimeRelative = numRechargeTimeTot / ((double) clustVec.size());

			ofs 	<< RandomGenerator::getInstance().getSeed() << ";"
					<< lifetimeSec << ";"
					<< lifetimeHour << ";"
					<< lifetimeDay << ";"
					<< index << ";"
					<< (index / lifetimeHour) << ";"
					<< (index / lifetimeDay) << ";"
					<< readings << ";"
					<< avg_read << ";"
					<< minGain << ";"
					<< maxGain << ";"
					<< varGain << ";"
					<< avgGain << ";"
					<< numRechargeTot << ";"
					<< numRechargeRelative << ";"
					<< numRechargeTimeTot << ";"
					<< numRechargeTimeRelative
					<< endl;

			ofs.close();
		}
	}

	if (!Generic::getInstance().hitmapFilename.empty()) {
		/*std::ofstream ofs (Generic::getInstance().hitmapFilename, std::ofstream::out);
		if (ofs.is_open()) {
			for (auto& s : sensList) {
				ofs << s->id << " " << s->coord.x << " " << s->coord.y << " " << s->mySensorReadings.size() << endl;
			}
			ofs.close();
		}*/
		mf->writeHitmaps_multiflow(Generic::getInstance().hitmapFilename);
	}
}














void Simulator::run_tree_multiflow_distr(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	/*if (!Generic::getInstance().statFilename.empty()) {
		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out);
		if (ofs.is_open()) {
			ofs << "";
			ofs.close();
		}
	}*/

	mf = new MultiFlow(mfAT);
	mf->init();

	for (auto& cv : clustVec) {
		mf->addChargStationAndUAV_distributed(*(cv->clusterHead), cv->clusterUAV);
	}
	for (auto& s : sensList) {
		mf->addSensor(s);
	}

	//mf->init_treeMF(end_time, Generic::getInstance().timeSlot);
	mf->init_matrix_treeMF();
	mf->run_tree_multiflow_distr(end_time);

}


void Simulator::finish_tree_multiflow_distr(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings) {
	if (Generic::getInstance().makeRunSimStat) return;  // don't make final stats if doing during the simulation

	if (!Generic::getInstance().statFilename.empty()) {

		std::ofstream ofs (Generic::getInstance().statFilename, std::ofstream::out | std::ofstream::app);
		if (ofs.is_open()) {

			double lifetimeSec = mf->getLastSensorRead();
			double lifetimeHour = ((double) lifetimeSec) / 3600.0;
			double lifetimeDay = ((double) lifetimeSec) / 86400.0;
			//ofs << "FINISH LifetimeSecs " << lifetimeSec << endl;
			//ofs << "FINISH LifetimeDays " << lifetimeDay << endl;

			double index = mf->calcIndex_Tree();
			//ofs << "FINISH Index " << index << endl;
			//ofs << "FINISH IndexOverHour " << (index / lifetimeHour) << endl;
			//ofs << "FINISH IndexOverDay " << (index / lifetimeDay) << endl;

			int readings = mf->calcNumRead_Tree();
			double avg_read = ((double) readings) / ((double) sensList.size());

			double minGain, maxGain, varGain, avgGain;
			mf->calcFinalGainsTree(minGain, maxGain, varGain, avgGain);

			int numRechargeTot = mf->calcNumRecharge_Tot();
			double numRechargeRelative = ((double) numRechargeTot) / ((double) clustVec.size());

			double numRechargeTimeTot = mf->calcNumRechargeTime_Tot();
			double numRechargeTimeRelative = numRechargeTimeTot / ((double) clustVec.size());

			ofs 	<< RandomGenerator::getInstance().getSeed() << ";"
					<< lifetimeSec << ";"
					<< lifetimeHour << ";"
					<< lifetimeDay << ";"
					<< index << ";"
					<< (index / lifetimeHour) << ";"
					<< (index / lifetimeDay) << ";"
					<< readings << ";"
					<< avg_read << ";"
					<< minGain << ";"
					<< maxGain << ";"
					<< varGain << ";"
					<< avgGain << ";"
					<< numRechargeTot << ";"
					<< numRechargeRelative << ";"
					<< numRechargeTimeTot << ";"
					<< numRechargeTimeRelative
					<< endl;
			ofs.close();
		}
	}

	if (!Generic::getInstance().hitmapFilename.empty()) {
		/*std::ofstream ofs (Generic::getInstance().hitmapFilename, std::ofstream::out);
		if (ofs.is_open()) {
			for (auto& s : sensList) {
				ofs << s->id << " " << s->coord.x << " " << s->coord.y << " " << s->mySensorReadings.size() << endl;
			}
			ofs.close();
		}*/
		mf->writeHitmaps_multiflow(Generic::getInstance().hitmapFilename);
	}
}


