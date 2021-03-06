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
#include "Statistics.h"

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
	double mf = 0.01;

	if (fout.is_open()) {

		fout << "graph G{" << endl;

		fout 	<< "{node [style=invis] P00 P01 P10 P11}"
				<< "P00 [pos = \"0,0!\"]"
				<< "P01 [pos = \"0," << sSize*mf << "!\"]"
				<< "P10 [pos = \"" << sSize*mf << ",0!\"]"
				<< "P11 [pos = \"" << sSize*mf << "," << sSize*mf << "!\"]"
				<< endl << endl;

		for (auto& s : sensList) {
			double sLossFull = Loss::getInstance().calculate_loss_full(s, timeNow, sensList);
			double actSize = pSize * (2 - sLossFull);
			fout << "B" << s->id << " [shape=\"point\" color=\"" << "grey"
					<< "\" pos=\"" << s->coord.x*mf << "," << s->coord.y*mf << "!\" width="
					<< actSize*mf << ", height=" << actSize*mf << "]" << endl;
		}

		for (int i = 0; i < (int) clustVec.size(); i++) {
			std::string color = std::string(COLOR_LIST_10[i%10]);

			fout << "U" << clustVec[i]->clusterUAV->id << " [shape=\"star\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterUAV->recharge_coord.x*mf << "," << clustVec[i]->clusterUAV->recharge_coord.y*mf << "!\" width=" << pSize*3*mf << ", height=" << pSize*3*mf << "]" << endl;

			//fout << "C" << clustVec[i]->clusterUAV->id << " [shape=\"diamond\" color=\"" << color << "\" pos=\""
			//		<< clustVec[i]->clusterHead->x << "," << clustVec[i]->clusterHead->y << "!\" width=" << pSize*2 << ", height=" << pSize*2 << "]" << endl;

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

				double actSize = pSize * (1.0 + (2.0 * (1.0 - sLossFull)));//;(2 - sLossFull);
				fout << "S" << p->id << " [shape=\"point\" color=\"" << color
						<< "\" pos=\"" << p->coord.x*mf << "," << p->coord.y*mf << "!\" width="
						<< actSize*mf << ", height=" << actSize*mf << "]" << endl;

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
	std::list<Readings *> allReadings;

	// default values
	int scenarioSize = 10000;
	int nSensors = 40;
	int nUAV = 3;
	int time_N = 3600;
	double timeSlot = 1;
	Simulator::Simu_type st = Simulator::SIMU_NORMAL;
	MultiFlow::Algo_type at = MultiFlow::ALGO_BSF;

	// (constant k at predefined %) k = (ln(1/%))/x       (used 0.1%)
	double kd = 0.004; //0.00460517;		//when 0.1? [0.00460517 at 500m] [0.1 at 26m][0.05 at 45m][0.02 at 115m][0.01 at 230m][0.005 at 460m]
	double kt = 0.0006; //0.000319803;	//when 0.1? [0.000319803 at 2h][0.000106601 at 6h][0.1 at 26s][0.05 at 45s][0.02 at 115s][0.01 at 230s][0.005 at 460s][0.002 at 1150s][0.001 at 2300s][0.00065 at 3600s][0.0005 at 4600s]
	double ke = 0.005; //0.002878231;	//when 0.9? [0.002878231 at 800J][0.001151293 at 2000J][0.1 at 26J][0.05 at 45J][0.02 at 115J][0.01 at 230J][0.005 at 460J][0.002 at 1150J][0.001 at 2300J][0.00065 at 3600s][0.0005 at 4600s][0.000105 at 21600J]

	double md = 2000; //1000;
	double mt = 15000; //7200;
	double me = 1800; //1200;
	bool useSigmoid = true; //false;

	double a = 0.5;

	// UAV parameters
	double initEnergyUAV = 119880;  	// Joule -> 3000mAh * 11.1Volt * 3.6
	double motorPower = 21.7;  			// Watt -> calcolato esempio da "https://www.ecalc.ch/xcoptercalc.php"
	double rechargePower = 20; 			// Watt	-> charging at 1C (it should recharges in 1h, but actually it needs 2.5h to finish the charge)
	double flightAltitude = 5;			// Meters (having Ptx=0.5 and Gtx=8.6 and Grx=1, at 25m it needs ~2sec to wakeup)
	double maxVelocity = 12;			// m/s
	double time2read = 1;				// seconds
	double energy2read = 0.0005;		// Joule
	double varGPS = 2;					// meter -> sigma^2 of the GPS error
	double varPilot = 1;				// meter -> sigma^2 of the Pilot error
	double varRot = 0.2;				// rad -> sigma^2 of the pilot-angle error

	//Sensor parameters
	double initEnergySensor = 3996;		// Joule -> 300mAh * 3.7Volt * 3.6
	double sensorSelfDischarge = 8;		// Percentage per month (https://batteryuniversity.com/learn/article/elevating_self_discharge)
	double eON = 0.000494;				// Joule -> from ICC
	double eBOOT = 0.0075;				// Joule -> from ICC
	double eSTB = 0;//0.00001;				// Watt -> from ICC
	bool fullRandomSensors = false;		// true to make random from 0 to MAX
	bool fullEnergySensors = false;		// true to make energy all to MAX

	//WakeUp
	double wakeupPower = 0.5;			// Watt -> trasmissione dell'illuminatore	//27dBm
	double wakeupMinPower = 0.000005;	// Watt -> min ricezione al sensore	//27dBm
	double wakeupFreq = 868;			// MHz
	double gainTx = 8.6;				// dBi (ex Yagi https://www.team-blacksheep.com/products/prod:868_yagi)
	double gainRx = 1;					// dBi
	double energy2wakeup = 0.6336;		// micro-Joule

	//MultiFlow
	double tsup = 20;					// sec -> t_{startup}
	double ttout = 20;					// sec -> t_{timeout}
	int numr = 3;						// n_r
	double ps_sup = 0.05;				// Watt -> p^S_{startup}
	double ps_tx = 0.1;					// Watt -> p^S_{tx}
	double ps_rx = 0.01;				// Watt -> p^S_{rx}
	double pu_sup = 0.05;				// Watt -> p^U_{startup}
	double pu_tx = 0.1;					// Watt -> p^U_{tx}
	double pu_rx = 0.01;				// Watt -> p^U_{rx}
	double gUmax = 8;					// dBi -> g^U_{max}
	double aUmax = M_PI_4;				// rad -> a^U_{max}
	double gSmax = 1;					// dBi -> g^S_{max}
	double aSmax = M_PI;				// rad -> a^S_{max}
	double uavComRange = 100.0;			// meters -> UAV communication range
	double neighUAVTout = 3600.0 * 4.0;	// sec -> UAV neigh timeout in the distributed method

	double twu = 1;						// sec -> t_{wakeup}
	double pwu = 0.99;					// probability of wake-up -> p_{wakeup}
	double pcom = 0.99;					// probability of comunication -> p_{com}
	double bsfExponent = 3.0;			// exponent for the probability to execute the BSF
	double rechargeRatio = 0.1;			// percentage of recarge before trying to BSFs
	double maxLoss = 0.8;				// max loss to take in consideration a sensor
	bool usePOT = false;				// use psi/time to search for the best path in BSF

	bool dynamicStatUAV = false;		// if true UAV 6,7 will start at 4H and UAV 4,5,6,7 will die at time 8H

	//Statistics
	int timeslots2log = 30;
	bool makeStateDuringSim = false;

	//cout << "Wake-up Drone BEGIN!!!" << endl;

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
	const std::string &timeSlot_string = input.getCmdOption("-tSlot");

	const std::string &stat2log_string = input.getCmdOption("-stat2l");
	const std::string &statOnrun_string = input.getCmdOption("-statOnrun");
	const std::string &statFile = input.getCmdOption("-statFile");
	const std::string &hitmapFile = input.getCmdOption("-hmFile");

	const std::string &algotype_clustering = input.getCmdOption("-algoClust");
	const std::string &algotype_tsp = input.getCmdOption("-algoTSP");
	const std::string &algotype_main = input.getCmdOption("-algoMain");

	const std::string &costant_kd = input.getCmdOption("-kd");
	const std::string &costant_kt = input.getCmdOption("-kt");
	const std::string &costant_ke = input.getCmdOption("-ke");
	const std::string &costant_md = input.getCmdOption("-md");
	const std::string &costant_mt = input.getCmdOption("-mt");
	const std::string &costant_me = input.getCmdOption("-me");
	const std::string &use_sigmoid = input.getCmdOption("-uSigm");
	const std::string &costant_alpha = input.getCmdOption("-alpha");

	const std::string &energy_UAV = input.getCmdOption("-ieUAV");
	const std::string &flight_altitude = input.getCmdOption("-h");
	const std::string &singleMotor_power = input.getCmdOption("-mp");
	const std::string &rechageStation_power = input.getCmdOption("-recp");
	const std::string &maxVel = input.getCmdOption("-mVel");
	const std::string &uavTime2Read = input.getCmdOption("-uT2R");
	const std::string &uavEnergy2Read = input.getCmdOption("-uE2R");
	const std::string &uavVarGPS = input.getCmdOption("-uVarGps");
	const std::string &uavVarPilot = input.getCmdOption("-uVarPil");
	const std::string &uavVarRot = input.getCmdOption("-uVarRot");

	const std::string &energy_sensor = input.getCmdOption("-ieSens");
	const std::string &energy_eon = input.getCmdOption("-eON");
	const std::string &energy_eboot = input.getCmdOption("-eBOOT");
	const std::string &energy_estb = input.getCmdOption("-eSTB");
	const std::string &sensor_self_discharge = input.getCmdOption("-ssd");
	const std::string &sensor_full_random = input.getCmdOption("-sFR");
	const std::string &sensor_full_energy = input.getCmdOption("-sFE");

	const std::string &wakeup_tx_power = input.getCmdOption("-wuPTx");
	const std::string &wakeup_tx_minpower = input.getCmdOption("-wumPTx");
	const std::string &wakeup_tx_freq = input.getCmdOption("-wuFreq");
	const std::string &energy_to_wakeup = input.getCmdOption("-e2wu_uJ");
	const std::string &gain_antenna_tx = input.getCmdOption("-gTx");
	const std::string &gain_antenna_rx = input.getCmdOption("-gRx");

	const std::string &simu_type = input.getCmdOption("-st");
	const std::string &algo_type = input.getCmdOption("-at");

	const std::string &multiflow_t_startup = input.getCmdOption("-mfTstartup");
	const std::string &multiflow_t_timeout = input.getCmdOption("-mfTtimeout");
	const std::string &multiflow_nr = input.getCmdOption("-mfnr");
	const std::string &multiflow_pS_startup = input.getCmdOption("-mfpSstartup");
	const std::string &multiflow_pS_tx = input.getCmdOption("-mfpStx");
	const std::string &multiflow_pS_rx = input.getCmdOption("-mfpSrx");
	const std::string &multiflow_pU_startup = input.getCmdOption("-mfpUstartup");
	const std::string &multiflow_pU_tx = input.getCmdOption("-mfpUtx");
	const std::string &multiflow_pU_rx = input.getCmdOption("-mfpUrx");

	const std::string &multiflow_gU_max = input.getCmdOption("-mfgUmax");
	const std::string &multiflow_aU_max = input.getCmdOption("-mfaUmax");
	const std::string &multiflow_gS_max = input.getCmdOption("-mfgSmax");
	const std::string &multiflow_aS_max = input.getCmdOption("-mfaSmax");

	const std::string &multiflow_uav_com_range = input.getCmdOption("-uavComR");
	const std::string &multiflow_uav_tout = input.getCmdOption("-uavNeighTout");

	const std::string &multiflow_static_twu = input.getCmdOption("-mfTwakeup");
	const std::string &multiflow_static_pwu = input.getCmdOption("-mfPwakeup");
	const std::string &multiflow_static_pcomm = input.getCmdOption("-mfPcomm");
	const std::string &multiflow_bsf_exponent = input.getCmdOption("-mfBSFexp");
	const std::string &multiflow_bsf_recharge_ratio = input.getCmdOption("-mfRecRatio");
	const std::string &multiflow_max_loss = input.getCmdOption("-mfMaxLoss");
	const std::string &multiflow_use_POT = input.getCmdOption("-mfUsePOT");

	const std::string &multiflow_dynamic_UAV = input.getCmdOption("-mfDynamicUAV");

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
	if (!timeSlot_string.empty()) {
		timeSlot = atof(timeSlot_string.c_str());
	}
	if (!stat2log_string.empty()) {
		timeslots2log = atoi(stat2log_string.c_str());
	}
	if (!statOnrun_string.empty()) {
		int tmp = atoi(statOnrun_string.c_str());
		makeStateDuringSim = (tmp != 0);
	}
	if (!energy_UAV.empty()) {
		initEnergyUAV = atof(energy_UAV.c_str());
	}
	if (!flight_altitude.empty()) {
		flightAltitude = atof(flight_altitude.c_str());
	}
	if (!maxVel.empty()) {
		maxVelocity = atof(maxVel.c_str());
	}
	if (!uavTime2Read.empty()) {
		time2read = atof(uavTime2Read.c_str());
	}
	if (!uavEnergy2Read.empty()) {
		energy2read = atof(uavEnergy2Read.c_str());
	}
	if (!singleMotor_power.empty()) {
		motorPower = atof(singleMotor_power.c_str());
	}
	if (!rechageStation_power.empty()) {
		rechargePower = atof(rechageStation_power.c_str());
	}
	if (!energy_sensor.empty()) {
		initEnergySensor = atof(energy_sensor.c_str());
	}
	if (!sensor_full_random.empty()) {
		fullRandomSensors = atoi(sensor_full_random.c_str()) != 0;
	}
	if (!sensor_full_energy.empty()) {
		fullEnergySensors = atoi(sensor_full_energy.c_str()) != 0;
	}
	if (!energy_eon.empty()) {
		eON = atof(energy_eon.c_str());
	}
	if (!energy_eboot.empty()) {
		eBOOT = atof(energy_eboot.c_str());
	}
	if (!energy_estb.empty()) {
		eSTB = atof(energy_estb.c_str());
	}
	if (!sensor_self_discharge.empty()) {
		sensorSelfDischarge = atof(sensor_self_discharge.c_str());
	}
	if (!wakeup_tx_power.empty()) {
		wakeupPower = atof(wakeup_tx_power.c_str());
	}
	if (!wakeup_tx_minpower.empty()) {
		wakeupMinPower = atof(wakeup_tx_minpower.c_str());
	}
	if (!wakeup_tx_freq.empty()) {
		wakeupFreq = atof(wakeup_tx_freq.c_str());
	}
	if (!energy_to_wakeup.empty()) {
		energy2wakeup = atof(energy_to_wakeup.c_str()) / 1000000.0;
	}
	if (!gain_antenna_tx.empty()) {
		gainTx = atof(gain_antenna_tx.c_str());
	}
	if (!gain_antenna_rx.empty()) {
		gainRx = atof(gain_antenna_rx.c_str());
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
	if (!costant_md.empty()) {
		md = atof(costant_md.c_str());
	}
	if (!costant_mt.empty()) {
		mt = atof(costant_mt.c_str());
	}
	if (!costant_me.empty()) {
		me = atof(costant_me.c_str());
	}
	if (!use_sigmoid.empty()) {
		useSigmoid = (atoi(use_sigmoid.c_str()) != 0);
	}
	if (!costant_alpha.empty()) {
		a = atof(costant_alpha.c_str());
	}
	if (!simu_type.empty()) {
		if (simu_type.compare("normal") == 0) {
			st = Simulator::SIMU_NORMAL;
		}
		else if (simu_type.compare("multiflow") == 0){
			st = Simulator::SIMU_MULTI_FLOW;
		}
		else if (simu_type.compare("distributed") == 0){
			st = Simulator::SIMU_DISTRIBUTED;
		}
		else if (simu_type.compare("treemultiflow") == 0){
			st = Simulator::SIMU_TREE_MULTI_FLOW;
		}
		else if (simu_type.compare("treemultiflowdistr") == 0){
			st = Simulator::SIMU_TREE_MULTI_FLOW_DISTR;
		}
	}
	if (!algo_type.empty()) {
		if (algo_type.compare("bsf") == 0) {
			at = MultiFlow::ALGO_BSF;
		}
		else if (algo_type.compare("bsfdist") == 0){
			at = MultiFlow::ALGO_BSF_DISTANCE;
		}
		else if (algo_type.compare("bsfenergy") == 0){
			at = MultiFlow::ALGO_BSF_ENERGY;
		}
		else if (algo_type.compare("dsf") == 0){
			at = MultiFlow::ALGO_DSF;
		}
		else if (algo_type.compare("dsfdist") == 0){
			at = MultiFlow::ALGO_DSF_DISTANCE;
		}
		else if (algo_type.compare("dsfenergy") == 0){
			at = MultiFlow::ALGO_DSF_ENERGY;
		}
	}
	if (!multiflow_t_startup.empty()) {
		tsup = atof(multiflow_t_startup.c_str());
	}
	if (!multiflow_t_timeout.empty()) {
		ttout = atof(multiflow_t_timeout.c_str());
	}
	if (!multiflow_nr.empty()) {
		numr = atoi(multiflow_nr.c_str());
	}
	if (!multiflow_pS_startup.empty()) {
		ps_sup = atof(multiflow_pS_startup.c_str());
	}
	if (!multiflow_pS_tx.empty()) {
		ps_tx = atof(multiflow_pS_tx.c_str());
	}
	if (!multiflow_pS_rx.empty()) {
		ps_rx = atof(multiflow_pS_rx.c_str());
	}
	if (!multiflow_pU_startup.empty()) {
		pu_sup = atof(multiflow_pU_startup.c_str());
	}
	if (!multiflow_pU_tx.empty()) {
		pu_tx = atof(multiflow_pU_tx.c_str());
	}
	if (!multiflow_pU_rx.empty()) {
		pu_rx = atof(multiflow_pU_rx.c_str());
	}

	if (!multiflow_gU_max.empty()) {
		gUmax = atof(multiflow_gU_max.c_str());
	}
	if (!multiflow_aU_max.empty()) {
		aUmax = atof(multiflow_aU_max.c_str());
	}
	if (!multiflow_gS_max.empty()) {
		gSmax = atof(multiflow_gS_max.c_str());
	}
	if (!multiflow_aS_max.empty()) {
		aSmax = atof(multiflow_aS_max.c_str());
	}
	if (!uavVarGPS.empty()) {
		varGPS = atof(uavVarGPS.c_str());
	}
	if (!uavVarPilot.empty()) {
		varPilot = atof(uavVarPilot.c_str());
	}
	if (!uavVarRot.empty()) {
		varRot = atof(uavVarRot.c_str());
	}
	if (!multiflow_uav_com_range.empty()) {
		uavComRange = atof(multiflow_uav_com_range.c_str());
	}
	if (!multiflow_uav_tout.empty()) {
		neighUAVTout = atof(multiflow_uav_tout.c_str());
	}
	if (!multiflow_static_twu.empty()) {
		twu = atof(multiflow_static_twu.c_str());
	}
	if (!multiflow_static_pwu.empty()) {
		pwu = atof(multiflow_static_pwu.c_str());
	}
	if (!multiflow_static_pcomm.empty()) {
		pcom = atof(multiflow_static_pcomm.c_str());
	}
	if (!multiflow_bsf_exponent.empty()) {
		bsfExponent = atof(multiflow_bsf_exponent.c_str());
	}
	if (!multiflow_bsf_recharge_ratio.empty()) {
		rechargeRatio = atof(multiflow_bsf_recharge_ratio.c_str());
	}
	if (!multiflow_max_loss.empty()) {
		maxLoss = atof(multiflow_max_loss.c_str());
	}
	if (!multiflow_use_POT.empty()) {
		usePOT = atoi(multiflow_use_POT.c_str()) != 0;
	}
	if (!multiflow_dynamic_UAV.empty()) {
		dynamicStatUAV = atoi(multiflow_dynamic_UAV.c_str()) != 0;
	}



	// HARD-CODED CHECK
	if (eSTB > 0) {
		wakeupPower = 0;
	}

	Generic::getInstance().init(timeSlot);
	Generic::getInstance().setSensorParam(initEnergySensor, sensorSelfDischarge, eON, eBOOT, eSTB, fullRandomSensors, fullEnergySensors);
	Generic::getInstance().setUAVParam(initEnergyUAV, flightAltitude, maxVelocity, motorPower, rechargePower, time2read, energy2read, varGPS, varPilot, varRot);
	Generic::getInstance().setWakeUpParam(wakeupPower, wakeupMinPower, wakeupFreq, energy2wakeup, gainTx, gainRx, gUmax, aUmax, gSmax, aSmax);
	Generic::getInstance().setStatParam(makeStateDuringSim, statFile, hitmapFile);
	Generic::getInstance().setMultiFlowParam(tsup, ttout, numr, ps_sup, ps_tx, ps_rx, pu_sup, pu_tx, pu_rx,
			4*motorPower, uavComRange, neighUAVTout, twu, pwu, pcom, bsfExponent, rechargeRatio, maxLoss,
			usePOT, dynamicStatUAV);
	Loss::getInstance().init(kd, kt, ke, md, mt, me, useSigmoid, a);
	Statistics::getInstance().init(timeslots2log);

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

	clustersVec.resize(uavsList.size(), nullptr);
	int idd = 0;
	for (auto& uav : uavsList) {
		clustersVec[idd] = new CoordCluster(uav, idd);
		++idd;
	}

	//Sensor::printLogsSensors(sensorsList, time_N);

	Simulator::getInstance().init(st, 0, time_N);
	Simulator::getInstance().setMainAlgo(algotype_main);
	Simulator::getInstance().setClusteringAlgo(algotype_clustering);
	Simulator::getInstance().setTSPAlgo(algotype_tsp);
	Simulator::getInstance().setAlgoType(at);
	Simulator::getInstance().run(clustersVec, sensorsList, allReadings);
	Simulator::getInstance().finish(clustersVec, sensorsList, allReadings);

	if (!dotFileOutput.empty()) {
		generateDOTfile(dotFileOutput, clustersVec, sensorsList, scenarioSize, ((double) scenarioSize)/50.0, Simulator::getInstance().getSimulationTime());
	}

	//cout << "Wake-up Drone FINISH!!!" << endl;
	return EXIT_SUCCESS;
}
