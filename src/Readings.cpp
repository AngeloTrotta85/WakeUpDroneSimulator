/*
 * Readings.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include <boost/range/irange.hpp>
#include <boost/math/special_functions/factorials.hpp>

#include "Readings.h"
#include "RandomGenerator.h"

int Readings::sequenceReadingCounter = 0;

Readings::Readings(Sensor *s, UAV *u, int timestamp, double val) {
	sensor = s;
	uav = u;
	value = val;
	read_time = timestamp;
	sequenceReading = ++sequenceReadingCounter;

	gain = full_loss = energy_loss = energy_gain = correlation_loss = correlation_gain = 0;
}

void Readings::generate_readings(std::list<Sensor *> &sl, std::list<UAV *> &ul, int maxTime) {
	for (auto& ss : sl) {
		for (int i = 0 ; i < maxTime; i++) {
			double r_reading = RandomGenerator::getInstance().getRealUniform(0, 1);

			if (r_reading < 0.025) {
				int r_uav = RandomGenerator::getInstance().getIntUniform(0, ul.size()-1);

				//UAV *uOK = *ul.begin();
				auto uOK = ul.begin();
				while (r_uav > 0) {
					--r_uav;
					uOK++;
				}
				UAV *u_reading = *uOK;
				Readings *read_new = new Readings(ss, u_reading, i, RandomGenerator::getInstance().getRealUniform(0, 1));
				ss->mySensorReadings.push_front(read_new);
				u_reading->mySensorReadings.push_front(read_new);
			}
		}
	}
}

bool compare_readings (const Readings *first, const Readings *second) {
	return ( first->read_time < second->read_time );
}
void Readings::writeOnFileReadings(std::string fn, std::list<Sensor *> &sl, int t) {
	std::ofstream fout(fn, std::ofstream::out);
	if (fout.is_open()) {
		std::list <Readings *> rl;
		for (auto& s : sl) {
			for (auto& r : s->mySensorReadings) {
				rl.push_back(r);
			}
		}
		rl.sort(compare_readings);
		for (auto& r : rl) {
			fout << r->read_time << ";" << r->value << ";" << r->sensor->id << ";" << r->uav->id << std::endl;
		}
		fout.close();
	}
}

void Readings::importReadingsFromFile(std::string inputFileName, std::list<Sensor *> &sl, std::list<UAV *> &ul, int t) {
	std::ifstream fileInput(inputFileName, std::ifstream::in);
	std::string str;
	int t_in, s, u;
	double val;

	if(fileInput.is_open()) {
		while (std::getline(fileInput, str)) {
			Sensor *s_r = nullptr;
			UAV *u_r = nullptr;

			sscanf(str.c_str(), "%d;%lf;%d;%d", &t_in, &val, &s, &u);

			for (auto& s_ok : sl) {
				if (s_ok->id == s) {
					s_r = s_ok;
					break;
				}
			}
			for (auto& u_ok : ul) {
				if (u_ok->id == u) {
					u_r = u_ok;
					break;
				}
			}

			if ((s_r != nullptr) && (u_r != nullptr)) {
				Readings *nr = new Readings(s_r, u_r, t_in, val);
				s_r->mySensorReadings.push_back(nr);
				u_r->mySensorReadings.push_back(nr);
			}
			//cout << "From file: " << str << ". Parsed sensor: " << x << ";" << y << endl;
		}
		fileInput.close();
	}
}
