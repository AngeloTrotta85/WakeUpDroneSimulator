/*
 * Simulator.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "Simulator.h"

Simulator::Simulator() {
	simulation_time = 0;
	end_time = -1;
}

void Simulator::init(int stime, int etime) {
	simulation_time = stime;
	end_time = etime;
}

void Simulator::finish() {

}

void Simulator::run() {
	while ((end_time < 0) || (simulation_time < end_time)) {

		++simulation_time;
	}
}
