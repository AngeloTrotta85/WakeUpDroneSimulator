/*
 * Sensor.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include <boost/range/irange.hpp>
#include <boost/math/special_functions/factorials.hpp>

#include "Sensor.h"
#include "Readings.h"
#include "RandomGenerator.h"
#include "Loss.h"

int Sensor::idSensGen = 0;

Sensor::Sensor(MyCoord sensCoord, double re) {
	coord = sensCoord;
	residual_energy = re;
	bookedReading = false;
	id = idSensGen++;
}

Sensor::Sensor(MyCoord sensCoord, double re, int id_new) {
	coord = sensCoord;
	residual_energy = re;
	bookedReading = false;
	id = id_new;
}

void Sensor::generateRandomSensors(std::list<Sensor *> &pl, int ss, int ns) {
	for (int i : boost::irange(0, ns)) { // i goes from 0 to ns-1
		Sensor *newS = new Sensor(
				MyCoord(RandomGenerator::getInstance().getRealUniform(0, ss), RandomGenerator::getInstance().getRealUniform(0, ss)),
				RandomGenerator::getInstance().getRealNormal(10000, 1000)
		);
		pl.push_back(newS);
		std::cout << "Sensor: " << i << " --> " << newS->coord << " - Energy: " << newS->residual_energy << std::endl;
	}
}

void Sensor::writeOnFileSensors(std::string fn, std::list<Sensor *> pointList) {
	std::ofstream fout(fn, std::ofstream::out);
	if (fout.is_open()) {
		for (auto& p : pointList) { // i goes from 0 to (ni-1) inclusive
			fout << p->id << ";" << p->coord.x << ";" << p->coord.y << ";" << p->residual_energy << std::endl;
		}
		fout.close();
	}
}

void Sensor::importSensorsFromFile(std::string inputFileName, std::list<Sensor *> &pl) {
	std::ifstream fileInput(inputFileName, std::ifstream::in);
	std::string str;
	int ids;
	double x, y, e;
	if(fileInput.is_open()) {
		while (std::getline(fileInput, str)) {
			sscanf(str.c_str(), "%d;%lf;%lf;%lf", &ids, &x, &y, &e);
			Sensor *np = new Sensor(MyCoord(x, y), e, ids);
			pl.push_back(np);
			//cout << "From file: " << str << ". Parsed sensor: " << x << ";" << y << endl;
		}
		fileInput.close();
	}
}

void Sensor::printLogsSensors (std::list<Sensor *> &sl, int timeNow) {
	int count = 0;
	for (auto& p : sl) {
		double sLossFull = Loss::getInstance().calculate_loss_full(p, timeNow, sl);
		double sLossEnergy = Loss::getInstance().calculate_loss_energy(p, timeNow, sl);
		double sLossCorr = Loss::getInstance().calculate_loss_correlation(p, timeNow, sl);

		std::cout  << "Sensor " << count
				<< " --> " << p->coord << ", Energy: " << p->residual_energy
				<< " LF: " << sLossFull
				<< " LE: " << sLossEnergy
				<< " LC: " << sLossCorr
				<< " - ";

		std::cout << "(";
		for (auto& r : p->mySensorReadings) {
			std::cout << r->read_time << " ";
		}
		std::cout << ")" << std::endl;

		count++;
	}
}

void Sensor::update_energy(void) {

}
