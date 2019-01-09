/*
 * UAV.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include <boost/range/irange.hpp>
#include <boost/math/special_functions/factorials.hpp>

#include "UAV.h"
#include "RandomGenerator.h"
#include "Generic.h"

int UAV::idUAVGen = 0;

UAV::UAV(MyCoord recCoord, double re) {
	actual_coord = recharge_coord = recCoord;
	max_energy = residual_energy = re;
	chargeCount = 0;
	state = IDLE;
	id = idUAVGen++;
}

UAV::UAV(MyCoord recCoord, double re, int id_new) {
	actual_coord = recharge_coord = recCoord;
	max_energy = residual_energy = re;
	chargeCount = 0;
	state = IDLE;
	id = id_new;
}

void UAV::generateRandomUAVs(std::list<UAV *> &pl, int ss, int nu) {
	for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
		UAV *newU = new UAV(
					MyCoord(RandomGenerator::getInstance().getRealUniform(0, ss), RandomGenerator::getInstance().getRealUniform(0, ss)),
					RandomGenerator::getInstance().getRealNormal(Generic::getInstance().initUAVEnergy, Generic::getInstance().initUAVEnergy/100.0)
				);
		pl.push_back(newU);
		//std::cout << "UAV: " << i << " --> " << newU->recharge_coord << " - Energy:" << newU->max_energy << std::endl;
	}
}

void UAV::importUAVsFromFile(std::string inputFileName, std::list<UAV *> &pl) {
	std::ifstream fileInput(inputFileName, std::ifstream::in);
	std::string str;
	int idu;
	double x, y, e;
	if(fileInput.is_open()) {
		while (std::getline(fileInput, str)) {
			sscanf(str.c_str(), "%d;%lf;%lf;%lf", &idu, &x, &y, &e);
			UAV *np = new UAV(MyCoord(x, y), e, idu);
			pl.push_back(np);
			//cout << "From file: " << str << ". Parsed sensor: " << x << ";" << y << endl;
		}
		fileInput.close();
	}
}

void UAV::writeOnFileUAVs(std::string fn, std::list<UAV *> pointList) {
	std::ofstream fout(fn, std::ofstream::out);
	if (fout.is_open()) {
		for (auto& p : pointList) { // i goes from 0 to (ni-1) inclusive
			fout << p->id << ";" << p->recharge_coord.x << ";" << p->recharge_coord.y << ";" << p->residual_energy << std::endl;
		}
		fout.close();
	}
}


