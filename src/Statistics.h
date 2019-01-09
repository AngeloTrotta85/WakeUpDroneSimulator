/*
 * Statistics.h
 *
 *  Created on: Dec 31, 2018
 *      Author: angelo
 */

#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "MyCoord.h"
#include "Sensor.h"
#include "Readings.h"
#include "UAV.h"
#include "CoordCluster.h"

class Statistics {
public:
	typedef struct IndexPair {
		int timeStamp;
		double indexVal;
	} IndexPair;

public:
	static Statistics& getInstance(void) {
		static Statistics    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Statistics(void){};         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Statistics(Statistics const&)	= delete;
	void operator=(Statistics const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status

public:
	void init(int stepSimLog) {
		step_simulation_log = stepSimLog;
		next_step_simulation_log = 0;
	}

	bool isTimeToLog(int sim_time);
	void logging(int sim_time);

	double calculate_andSave_index(int sim_time, std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, bool save);

	double calculateAvgIndexDerivative(void);
	double calculateActualAvgIndexDerivative(void);

	void calculate_minmax_sensor_energy(std::list<Sensor *> &sensList, double &avg, double &min, double &max, double &var);
	void calculate_actual_minmax_sensor_energy(std::list<Sensor *> &sensList, double &avg, double &min, double &max, double &var);

	void calculate_minmax_sensor_visiting(int sim_time, std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, double &avg, double &min, double &max, double &var);
	void calculate_actual_minmax_sensor_visiting(int sim_time, std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList, double &avg, double &min, double &max, double &var);

	void calculate_minmax_uav_charge_time(std::vector<CoordCluster *> &clustVec, double &avg, double &min, double &max, double &var);
	void calculate_actual_minmax_uav_charge_time(std::vector<CoordCluster *> &clustVec, double &avg, double &min, double &max, double &var);

public:
	int step_simulation_log;
	int next_step_simulation_log;

private:
	std::list<IndexPair> indexesList;
};

#endif /* STATISTICS_H_ */
