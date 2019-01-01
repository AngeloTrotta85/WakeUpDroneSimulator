/*
 * Statistics.cpp
 *
 *  Created on: Dec 31, 2018
 *      Author: angelo
 */

#include "Statistics.h"
#include "Loss.h"

bool Statistics::isTimeToLog(int sim_time) {
	return ( (step_simulation_log > 0) && (sim_time >= next_step_simulation_log) );
}

void Statistics::logging(int sim_time) {
	next_step_simulation_log = sim_time + step_simulation_log;
}

double Statistics::calculate_andSave_index(int sim_time, std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, bool save) {
	double actIndex = Loss::getInstance().calculate_index_full_reading(sim_time, sensList);

	if (save) {
		IndexPair newIP;
		newIP.indexVal = actIndex;
		newIP.timeStamp = sim_time;

		//std::cout << " [Adding new Index at time: "<< sim_time << " with value: " << actIndex << "] ";

		indexesList.push_front(newIP);
	}

	return actIndex;
}

double Statistics::calculateAvgIndexDerivative(void) {
	double ris = 0;

	if (indexesList.size() > 1) {
		double first, second;
		//int timeFirst, timeSecond;

		auto it = indexesList.begin();
		first = it->indexVal;
		//timeFirst = it->timeStamp;

		it++;
		second = it->indexVal;
		//timeSecond = it->timeStamp;

		ris = first - second;

		//std::cout << " [OK indexesList.size() > 1... "
		//		<< "first: " << first << "(" << timeFirst << ")"
		//		<< " and second: " << second << "(" << timeSecond << ")" << "] ";
	}

	return ris;
}

double Statistics::calculateActualAvgIndexDerivative(void) {
	return calculateAvgIndexDerivative();
}

void Statistics::calculate_minmax_sensor_energy(std::list<Sensor *> &sensList, double &avg, double &min, double &max, double &var) {
	double sum = 0;

	avg = var = 0;
	min = std::numeric_limits<double>::max();
	max = -1;

	for (auto& s: sensList) {
		sum += s->residual_energy;
		if (s->residual_energy > max) max = s->residual_energy;
		if (s->residual_energy < min) min = s->residual_energy;
	}
	avg = sum / ((double) sensList.size());

	sum = 0;
	for (auto& s: sensList) {
		sum += pow((double)(s->residual_energy - avg), 2.0);
	}
	var = sum / ((double) (sensList.size() - 1));
}

void Statistics::calculate_actual_minmax_sensor_energy(std::list<Sensor *> &sensList, double &avg, double &min, double &max, double &var) {
	return calculate_minmax_sensor_energy(sensList, avg, min, max, var);
}

void Statistics::calculate_minmax_sensor_visiting(int sim_time, std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, double &avg, double &min, double &max, double &var) {
	double sum = 0;

	avg = var = 0;
	min = std::numeric_limits<double>::max();
	max = -1;

	for (auto& s: sensList) {
		double actVal = 0;
		if (sim_time > 0) {
			actVal = ((double) s->mySensorReadings.size()) / (((double) sim_time) / 3600.0);
		}
		sum += actVal;
		if (actVal > max) max = actVal;
		if (actVal < min) min = actVal;
	}
	avg = sum / ((double) sensList.size());

	sum = 0;
	for (auto& s: sensList) {
		double actVal = 0;
		if (sim_time > 0) {
			actVal = ((double) s->mySensorReadings.size()) / (((double) sim_time) / 3600.0);
		}
		sum += pow(actVal - avg, 2.0);
	}
	var = sum / ((double) (sensList.size() - 1));
}

void Statistics::calculate_actual_minmax_sensor_visiting(int sim_time, std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, double &avg, double &min, double &max, double &var) {
	return calculate_minmax_sensor_visiting(sim_time, clustVec, sensList, avg, min, max, var);
}

