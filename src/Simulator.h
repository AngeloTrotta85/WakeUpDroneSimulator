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

	void init(int stime, int etime);
	void run(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);
	void finish(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, std::list<Readings *> &allReadings);

	void setClusteringAlgo(std::string algotype_clustering);
	void setTSPAlgo(std::string algotype_tsp);

	int getEndTime() const { return end_time; }
	int getSimulationTime() const { return simulation_time; }

protected:
	void cluster_and_tour(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, CoordCluster *actClust);

private:
	int simulation_time;
	int end_time;

	Clustering *clust;
	TSP *tsp;
};

#endif /* SIMULATOR_H_ */
