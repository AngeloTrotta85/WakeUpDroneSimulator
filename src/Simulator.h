/*
 * Simulator.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <stdlib.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <list>       // std::list
#include <stack>
#include <map>       // std::list

#include "clustering/Clustering.h"
#include "tsp/TSP.h"
#include "CoordCluster.h"
#include "MultiFlow.h"

class Simulator {
public:
	static Simulator& getInstance(void) {
		static Simulator    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Simulator(void);         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Simulator(Simulator const&)	= delete;
	void operator=(Simulator const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status
public:
	typedef enum {
		ALGO_BEE,
		ALGO_BEE_NOCLUST,
		ALGO_CLOSEST,
		ALGO_LOWERBATT
	} Main_Algo;

public:
	typedef enum {
		SIMU_NORMAL,
		SIMU_MULTI_FLOW,
		SIMU_DISTRIBUTED,
		SIMU_TREE_MULTI_FLOW,
		SIMU_TREE_MULTI_FLOW_DISTR
	} Simu_type;

public:

	void init(Simu_type st, int stime, int etime);

	void run(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void run_normal(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void run_multiflow(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void run_distributed(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void run_tree_multiflow(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void run_tree_multiflow_distr(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);

	void finish(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void finish_normal(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void finish_multiflow(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void finish_distributed(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void finish_tree_multiflow(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void finish_tree_multiflow_distr(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);

	void setMainAlgo(std::string algotype_main);
	void setClusteringAlgo(std::string algotype_clustering);
	void setTSPAlgo(std::string algotype_tsp);
	void setAlgoType(MultiFlow::Algo_type algotype);

	int getEndTime() const { return end_time; }
	int getSimulationTime() const { return simulation_time; }

protected:
	void cluster_and_tour(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, CoordCluster *actClust);

	void mainalgo_bee(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void mainalgo_beenoclust(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void mainalgo_closest(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void mainalgo_lowerbatt(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);

	void calc_path_bee(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings, CoordCluster *actClust);
	void calc_path_beenoclust(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings, CoordCluster *actClust);
	void calc_path_closest(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings, CoordCluster *actClust);
	void calc_path_lowerbatt(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings, CoordCluster *actClust);

private:
	int simulation_time;
	int end_time;

	int timeSlot;

	bool endSimulation;
	bool makeLog;

	Clustering *clust;
	TSP *tsp;
	Main_Algo mainalgo;
	Simu_type simtype;

	MultiFlow *mf;
	MultiFlow::Algo_type mfAT;
};

#endif /* SIMULATOR_H_ */
