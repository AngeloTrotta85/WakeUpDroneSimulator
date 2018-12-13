/*
 * Simulator.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

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
	void run(std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList);
	void finish();

	int getEndTime() const { return end_time; }
	int getSimulationTime() const { return simulation_time; }

private:
	int simulation_time;
	int end_time;
};

#endif /* SIMULATOR_H_ */
