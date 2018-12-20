/*
 * Simulator.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "Simulator.h"
#include "Generic.h"

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

#ifndef TSP_UAV_CODE
#define TSP_UAV_CODE 100000
#endif

using namespace std;

Simulator::Simulator() {
	simulation_time = 0;
	end_time = -1;

	clust = nullptr;
	tsp = nullptr;
}

void Simulator::init(int stime, int etime) {
	simulation_time = stime;
	end_time = etime;
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

void Simulator::finish() {

}

void Simulator::run(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList) {
	int timeSlot = Generic::getInstance().timeSlot;
	while ((end_time < 0) || (simulation_time < end_time)) {

		cout << "Simulation time: " << simulation_time << endl;

		// move and update the UAVs
		for (auto& c : clustVec) {
			switch (c->clusterUAV->state) {
			case UAV::IDLE:
				cluster_and_tour(clustVec, sensList, c);

				//set the chosen sensors as booked
				for (auto& s: c->pointsTSP_listFinal) {
					s->uavBookedReading[c->clusterUAV->id] = true;
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
				}
				break;

			case UAV::WAKINGUP_READING:
				c->clusterUAV->residual_energy -= Generic::getInstance().singleMotorPowerUAV * 4.0 * Generic::getInstance().timeSlot;
				if (c->timeSpentInWakeRead >= Generic::getInstance().getTime2WakeRead(c->clusterUAV->actual_coord, c->nextSensor->coord)) {
					// ok, finished
					c->clusterUAV->state = UAV::MOVING;

					//remove at the end the wakeup+reading energy
					c->clusterUAV->residual_energy -= Generic::getInstance().getEnergy2WakeRead(c->clusterUAV->actual_coord, c->nextSensor->coord);

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
		}

		for (auto& s: sensList) {
			s->update_energy();
		}

		if (simulation_time > 100) break;  //TODO remove

		simulation_time += timeSlot;
	}
}

void Simulator::cluster_and_tour(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, CoordCluster *actClust) {
	actClust->pointsTSP_listFinal.clear();
	clust->cluster(clustVec, sensList, simulation_time, actClust->clusterUAV->id);

	tsp->calculateTSP(actClust, sensList, simulation_time);
}

