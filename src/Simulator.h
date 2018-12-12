/*
 * Simulator.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

class Simulator {
public:
	Simulator();

	void init(int time);
	void run();
	void finish();

public:
	int simulation_time;
	int end_time;
};

#endif /* SIMULATOR_H_ */
