//============================================================================
// Name        : WakeUpDroneSimulator.cpp
// Author      : Angelo Trotta
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdlib.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <list>       // std::list
#include <stack>
#include <map>       // std::list
#include <cstdlib>
#include <ctime>
#include <random>
#include <chrono>

#include <boost/range/irange.hpp>
#include <boost/math/special_functions/factorials.hpp>

#include "MyCoord.h"
#include "Simulator.h"
#include "RandomGenerator.h"
#include "Sensor.h"
#include "UAV.h"
#include "Readings.h"
#include "CoordCluster.h"
#include "Loss.h"
#include "Generic.h"
#include "Simulator.h"

using namespace std;
using namespace boost;

const char* COLOR_LIST_10[] = {
		"green",
		"blue",
		"red",
		"gold",
		"magenta",
		"brown",
		"darkorange",
		"salmon",
		"greenyellow",
		"black"
};

class InputParser{
public:
	InputParser (int &argc, char **argv){
		for (int i=1; i < argc; ++i)
			this->tokens.push_back(std::string(argv[i]));
	}
	const std::string& getCmdOption(const std::string &option) const{
		std::vector<std::string>::const_iterator itr;
		itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
		if (itr != this->tokens.end() && ++itr != this->tokens.end()){
			return *itr;
		}
		static const std::string empty_string("");
		return empty_string;
	}
	bool cmdOptionExists(const std::string &option) const{
		return std::find(this->tokens.begin(), this->tokens.end(), option)
		!= this->tokens.end();
	}
private:
	std::vector <std::string> tokens;
};

void generateDOTfile(std::string outFileName, std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList,
		double sSize, double pSize, int timeNow){
	std::ofstream fout(outFileName, std::ofstream::out);

	if (fout.is_open()) {

		fout << "graph G{" << endl;

		fout 	<< "{node [style=invis] P00 P01 P10 P11}"
				<< "P00 [pos = \"0,0!\"]"
				<< "P01 [pos = \"0," << sSize << "!\"]"
				<< "P10 [pos = \"" << sSize << ",0!\"]"
				<< "P11 [pos = \"" << sSize << "," << sSize << "!\"]"
				<< endl << endl;

		for (auto& s : sensList) {
			double sLossFull = Loss::getInstance().calculate_loss_full(s, timeNow, sensList);
			double actSize = pSize * (2 - sLossFull);
			fout << "B" << s->id << " [shape=\"point\" color=\"" << "grey"
					<< "\" pos=\"" << s->coord.x << "," << s->coord.y << "!\" width="
					<< actSize << ", height=" << actSize << "]" << endl;
		}

		for (int i = 0; i < (int) clustVec.size(); i++) {
			std::string color = std::string(COLOR_LIST_10[i%10]);

			fout << "U" << clustVec[i]->clusterUAV->id << " [shape=\"star\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterUAV->recharge_coord.x << "," << clustVec[i]->clusterUAV->recharge_coord.y << "!\" width=" << pSize*3 << ", height=" << pSize*3 << "]" << endl;

			fout << "C" << clustVec[i]->clusterUAV->id << " [shape=\"diamond\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterHead->x << "," << clustVec[i]->clusterHead->y << "!\" width=" << pSize*2 << ", height=" << pSize*2 << "]" << endl;

			for (auto& p : clustVec[i]->pointsTSP_listFinal) {
				double sLossFull = Loss::getInstance().calculate_loss_full(p, timeNow, sensList);
				//double sLossLast = calculate_loss_last(p, timeNow, sensList);
				//double sLossEnergy = calculate_loss_energy(p, timeNow, sensList);
				//double sLossCorr = calculate_loss_correlation(p, timeNow, sensList);

				//cout << "Sensor " << count
				//		<< " has loss full: " << sLossFull
				//		<< " has loss last: " << sLossLast
				//		<< " has loss energy: " << sLossEnergy
				//		<< " has loss correlation: " << sLossCorr
				//		<< endl;

				double actSize = pSize * (2 - sLossFull);
				fout << "S" << p->id << " [shape=\"point\" color=\"" << color
						<< "\" pos=\"" << p->coord.x << "," << p->coord.y << "!\" width="
						<< actSize << ", height=" << actSize << "]" << endl;

				/*if (i == maxIdx) {
					fout << "S" << count << "_rad [shape=\"circle\" color=\"" << "black" << "\" style=\"dotted\" label=\"\" pos=\""
							<< p->x << "," << p->y << "!\" width=" << (2.0/maxCorrelation) << ", height=" << (2.0/maxCorrelation) << "]" << endl;
				}*/
			}

			auto p1 = clustVec[i]->pointsTSP_listFinal.begin();
			auto p2 = clustVec[i]->pointsTSP_listFinal.begin();
			if (p2 != clustVec[i]->pointsTSP_listFinal.end()) {
				p2++;

				fout << "U" << clustVec[i]->clusterUAV->id << " -- S" << (*p1)->id << " [color=\"" << color << "\"]" << endl;

				while (p2 != clustVec[i]->pointsTSP_listFinal.end()) {

					fout << "S" << (*p1)->id << " -- S" << (*p2)->id << " [color=\"" << color << "\"]" << endl;

					p1++;
					p2++;
				}

				fout << "S" << (*p1)->id << " -- U" << clustVec[i]->clusterUAV->id << " [color=\"" << color << "\"]" << endl;
			}

		}

		fout << "}" << endl;

		fout.close();
	}
}

int main(int argc, char **argv) {
	std::list<Sensor *> sensorsList;
	std::list<UAV *> uavsList;
	std::vector<CoordCluster *> clustersVec;

	// default values
	int scenarioSize = 100;
	int nSensors = 40;
	int nUAV = 3;
	int time_N = 100;
	double kd = 0.08;
	double kt = 0.02;
	double ke = 0.0005;
	double a = 0.5;

	cout << "Wake-up Drone BEGIN!!!" << endl;

	InputParser input(argc, argv);

	const std::string &inputSensorsFileName = input.getCmdOption("-is");
	const std::string &inputUAVsFileName = input.getCmdOption("-iu");
	const std::string &outputSensorsFileName = input.getCmdOption("-os");
	const std::string &outputUAVsFileName = input.getCmdOption("-ou");
	const std::string &inputNumSensors = input.getCmdOption("-ns");
	const std::string &inputNumUAV = input.getCmdOption("-nu");

	const std::string &scenarioMaxVal = input.getCmdOption("-scenario");
	const std::string &seedUser = input.getCmdOption("-seed");
	const std::string &dotFileOutput = input.getCmdOption("-dot");
	const std::string &inputTimeSim = input.getCmdOption("-time");

	const std::string &algotype_clustering = input.getCmdOption("-algoClust");
	const std::string &algotype_tsp = input.getCmdOption("-algoTSP");

	const std::string &costant_kd = input.getCmdOption("-kd");
	const std::string &costant_kt = input.getCmdOption("-kt");
	const std::string &costant_ke = input.getCmdOption("-ke");
	const std::string &costant_alpha = input.getCmdOption("-alpha");


	if (!seedUser.empty()) {
		int seedR = atoi(seedUser.c_str());
		RandomGenerator::getInstance().setSeed(seedR);
	}
	else {
		unsigned seedR = std::chrono::system_clock::now().time_since_epoch().count();
		RandomGenerator::getInstance().setSeed(seedR);
	}

	if (!scenarioMaxVal.empty()) {
		scenarioSize = atoi(scenarioMaxVal.c_str());
	}

	if (!inputNumSensors.empty()) {
		nSensors = atoi(inputNumSensors.c_str());
	}

	if (!inputNumUAV.empty()) {
		nUAV = atoi(inputNumUAV.c_str());
	}

	if (!inputTimeSim.empty()) {
		time_N = atoi(inputTimeSim.c_str());
	}

	if (inputSensorsFileName.empty()) {
		Sensor::generateRandomSensors(sensorsList, scenarioSize, nSensors);
		if (!outputSensorsFileName.empty()) {
			Sensor::writeOnFileSensors(outputSensorsFileName, sensorsList);
		}
	}
	else {
		Sensor::importSensorsFromFile(inputSensorsFileName, sensorsList);
	}

	if (inputUAVsFileName.empty()) {
		UAV::generateRandomUAVs(uavsList, scenarioSize, nUAV);
		if (!outputUAVsFileName.empty()) {
			UAV::writeOnFileUAVs(outputUAVsFileName, uavsList);
		}
	}
	else {
		UAV::importUAVsFromFile(inputUAVsFileName, uavsList);
	}


	if (!costant_kd.empty()) {
		kd = atof(costant_kd.c_str());
	}
	if (!costant_kt.empty()) {
		kt = atof(costant_kt.c_str());
	}
	if (!costant_ke.empty()) {
		ke = atof(costant_ke.c_str());
	}
	if (!costant_alpha.empty()) {
		a = atof(costant_alpha.c_str());
	}

	Generic::getInstance().init();
	Loss::getInstance().init(kd, kt, ke, a);

	clustersVec.resize(uavsList.size(), nullptr);
	int idd = 0;
	for (auto& uav : uavsList) {
		clustersVec[idd] = new CoordCluster(uav, idd);
		++idd;
	}

	Sensor::printLogsSensors(sensorsList, time_N);

	Simulator::getInstance().init(0, time_N);
	Simulator::getInstance().setClusteringAlgo(algotype_clustering);
	Simulator::getInstance().setTSPAlgo(algotype_tsp);
	Simulator::getInstance().run(clustersVec, sensorsList);
	Simulator::getInstance().finish();

	if (!dotFileOutput.empty()) {
		generateDOTfile(dotFileOutput, clustersVec, sensorsList, scenarioSize, ((double) scenarioSize)/50.0, Simulator::getInstance().getSimulationTime());
	}

	cout << "Wake-up Drone FINISH!!!" << endl;
	return EXIT_SUCCESS;
}
