/*
 * MultiFlow.cpp
 *
 *  Created on: May 16, 2019
 *      Author: angelo
 */

#include "MultiFlow.h"
#include "Generic.h"
#include "RandomGenerator.h"

MultiFlow::MultiFlow() {
	actSensorTimeStamp = 0;
	actUAVTimeStamp = 0;

	pWU = 0;
}

void MultiFlow::addSensor(Sensor *s) {
	SensorNode *newsn = new SensorNode();

	newsn->sens = s;
	newsn->lastTimestamp = 0;

	sens_list.push_back(newsn);
}

void MultiFlow::addChargStationAndUAV(MyCoord c, UAV *u) {
	ChargingNode *newcs = new ChargingNode();

	newcs->id = u->id;
	newcs->pos = c;
	newcs->u = u;
	newcs->lastTimestamp = 0;

	cs_map[u->id] = newcs;
}

ChargingNode *MultiFlow::getLeftMostUAV(int end_time) {
	ChargingNode *ris = cs_map.begin()->second;
	int risTime = ris->lastTimestamp;
	for (auto& cs : cs_map) {
		if (cs.second->lastTimestamp < ris->lastTimestamp) {
			ris = cs.second;
			risTime = ris->lastTimestamp;
		}
	}
	if (risTime >= end_time) {
		ris = nullptr;
	}
	return ris;
}

double MultiFlow::calcPowEta(int t) {
	return 0;
}

double MultiFlow::energy_loss_onArc(int tstart) {
	double ris = 0;
	int tend = tstart + Generic::getInstance().tstartup + (Generic::getInstance().nr * Generic::getInstance().ttimeout);

	ris += calcPowEta(tend) * Generic::getInstance().timeSlot * (tend - tstart);

	double estartup = Generic::getInstance().pSstartup * Generic::getInstance().timeSlot * Generic::getInstance().tstartup;
	double ecomm = (Generic::getInstance().pSrx + ((Generic::getInstance().pStx - Generic::getInstance().pSrx) / Generic::getInstance().ttimeout))
			* Generic::getInstance().timeSlot * Generic::getInstance().nr * Generic::getInstance().ttimeout;
	ris += pWU * (estartup + ecomm);

	return ris;
}

int MultiFlow::updateSensorsEnergy(int starttime, int endtime) {
	for (auto& s : sens_list) {
		for (auto& l : s->readings) {
			if ((l.readTime > starttime) && (l.readTime <= endtime)) {
				if (s->lastTimestamp < l.readTime) {
					for (int i = 0; i < (l.readTime - s->lastTimestamp); i++) {
						s->sens->residual_energy -= calcPowEta(s->lastTimestamp + i + 1) * Generic::getInstance().timeSlot;
					}
					s->lastTimestamp = l.readTime;
				}

				if (s->lastTimestamp == l.readTime) {
					s->lastTimestamp += Generic::getInstance().tstartup + (Generic::getInstance().nr * Generic::getInstance().ttimeout);
					s->sens->residual_energy -= energy_loss_onArc(l.readTime);
				}
				else {
					cerr << "Error in updateSensorsEnergy" << endl;
					exit(EXIT_FAILURE);
				}
			}
		}
	}

	return endtime;
}

void MultiFlow::calculateTSP(ChargingNode *leftmost) {

}
/*
double MultiFlow::getPDF_Eloc(MyCoord e) {
	double meanGPS = 0;
	double sigmaGPS = 2;
	double meanPilot = 0;
	double sigmaPilot = 1;

	return RandomGenerator::get_PDF_normal(e.length(), meanGPS + meanPilot, sigmaGPS + sigmaPilot);
}

double MultiFlow::getPDF_Erot(MyCoord r) {
	double mean = 0;
	double sigma = 0.3;

	return RandomGenerator::get_PDF_normal(r.x, mean, sigma) * RandomGenerator::get_PDF_normal(r.y, mean, sigma);
}*/

double MultiFlow::getPDF_Eloc_single(double e) {
	double meanGPS = 0;
	double sigmaGPS = Generic::getInstance().sigmaGPS;
	double meanPilot = 0;
	double sigmaPilot = Generic::getInstance().sigmaPilot;

	return RandomGenerator::get_PDF_normal(e, meanGPS + meanPilot, sigmaGPS + sigmaPilot);
}

double MultiFlow::getPDF_Erot_single(double r) {
	double mean = 0;
	double sigma = Generic::getInstance().sigmaRot;

	return RandomGenerator::get_PDF_normal(r, mean, sigma);
}

double MultiFlow::calc_d2D_max(double h, double alpha_max) {
	double angle_h = M_PI_2 - (alpha_max / 2.0);
	double ris_sens = ((h/sin(angle_h)) * sin(alpha_max / 2.0));
	double ptx_dbm = 10.0 * log10(1000.0 * Generic::getInstance().wakeupTxPower);

	double d3D_max = pow(10.0,
			(ptx_dbm + Generic::getInstance().gUmax + Generic::getInstance().gSmax +
					(20.0*log10(Generic::getInstance().wakeupTxFrequency) -27.55) - Generic::getInstance().wakeupTxMinPower)/20.0);
	double ris_uav = sqrt(pow(d3D_max, 2.0) - pow(h, 2.0));

	cout << "Calculating d2D from ris_sens: " << ris_sens << " and ris_uav: " << ris_uav << " ris: " << min(ris_sens, ris_uav) << endl;

	//cout << "min: " << min(ris, 40.0) << endl;

	return min(ris_sens, ris_uav);
}

double MultiFlow::calcProb_EReceived_new(double h, double e) {
	double ris = 0;
	double distDiv = 50.0;
	double rotDiv = 20.0;

	double d2Dmax = calc_d2D_max(h, Generic::getInstance().alphaSmax);
	double deltad = d2Dmax/distDiv;

	double rmax = M_PI / 3.0;
	double deltar = rmax/rotDiv;

	double sumAll = 0;

	for (double x = -d2Dmax; x <= (d2Dmax + 0.001); x+=deltad) {
		for (double y = -d2Dmax; y <= (d2Dmax + 0.001); y+=deltad) {
			if (MyCoord(x, y).length() <= d2Dmax) {
				for (double rx = -rmax; rx <= (rmax + 0.001); rx+=deltar) {
					for (double ry = -rmax; ry <= (rmax + 0.001); ry+=deltar) {
						//cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry  << endl;
						double g = calc_Gamma(x, y, rx, ry);
						//cout << "deltad: " << deltad << " deltar:" << deltar <<
						//		" -> gamma: " << g << "; e: " << e << endl << endl;

						if (e == 0) {
							//cout << "E = 0" << endl;
							if (g == e) {
								double actProb = getPDF_Eloc_single(x) * getPDF_Eloc_single(y) * getPDF_Erot_single(rx) * getPDF_Erot_single(ry);
								//cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry;
								//cout << " -> gamma: " << g << "; e: " << e;
								//cout << " --> totProb = " << actProb << endl;
								sumAll += actProb;
							}
						}
						else {
							/*if (g > 0) {
								cout << "Calculated gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry;
								cout << " - deltad: " << deltad << " deltar:" << deltar <<
										" -> gamma: " << g << "; e: " << e << endl;
							}*/
							double closeoffset = e / 50.0;//1.0e-9 * Generic::getInstance().timeSlot;
							if (MyCoord::close(g, e, closeoffset)) {

								cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry;
								cout << " - deltad: " << deltad << " deltar:" << deltar <<
										" -> gamma: " << g << "; e: " << e << endl;

								cout << "they are close!!!" << endl;
								double actProb = getPDF_Eloc_single(x) * getPDF_Eloc_single(y) * getPDF_Erot_single(rx) * getPDF_Erot_single(ry);
								cout << "Calculating gamma --> totProb = " << actProb << endl;
								sumAll += actProb;
							}
						}
					}
				}
			}
		}
	}

	cout << "Sum all Prob = " << sumAll << " using"
				<< " d2Dmax: " << d2Dmax << " rmax:" << rmax
				<< " deltad: " << deltad << " deltar:" << deltar
				<< " --> ris = " << (deltad * deltad * deltar * deltar * sumAll) << endl;

	//if (e == 0)
	exit(0);

	ris = deltad * deltad * deltar * deltar * sumAll;

	return ris;
}

double MultiFlow::calcProb_EReceived(double h, double e) {
	//double ris = 0;

	double d2Dmax = calc_d2D_max(h, Generic::getInstance().alphaSmax);
	double deltad = d2Dmax/50.0;

	double rmax = M_PI_2;
	double deltar = rmax/20.0;

	/*double sumX = 0;
	double sumY = 0;
	double sumRX = 0;
	double sumRY = 0;

	for (double x = -d2Dmax; x <= (d2Dmax + 0.001); x+=deltad) {
		sumX += getPDF_Eloc_single(x);
	}
	for (double y = -d2Dmax; y <= (d2Dmax + 0.001); y+=deltad) {
		sumY += getPDF_Eloc_single(y);
	}
	for (double rx = -rmax; rx <= (rmax + 0.001); rx+=deltar) {
		sumRX += getPDF_Erot_single(rx);
	}
	for (double ry = -rmax; ry <= (rmax + 0.001); ry+=deltar) {
		sumRY += getPDF_Erot_single(ry);
	}

	cout << "deltad: " << deltad << " deltar:" << deltar <<
			" -> sumX: " << sumX << "; sumY: " << sumY << "; sumRX: " << sumRX << "; sumRY: " << sumRY  << endl;

	return (deltad * sumX * deltad * sumY * deltar * sumRX * deltar * sumRY);
	*/

	double sumAll = 0;

	for (double x = -d2Dmax; x <= (d2Dmax + 0.001); x+=deltad) {
		for (double y = -d2Dmax; y <= (d2Dmax + 0.001); y+=deltad) {
			if (MyCoord(x, y).length() <= d2Dmax) {
				for (double rx = -rmax; rx <= (rmax + 0.001); rx+=deltar) {
					for (double ry = -rmax; ry <= (rmax + 0.001); ry+=deltar) {
						//cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry  << endl;
						double g = calc_Gamma(x, y, rx, ry);
						//cout << "deltad: " << deltad << " deltar:" << deltar <<
						//		" -> gamma: " << g << "; e: " << e << endl << endl;

						if (MyCoord::close(g, e, 0.0000001)) {
						//if (g == e) {
							double actProb = getPDF_Eloc_single(x) * getPDF_Eloc_single(y) * getPDF_Erot_single(rx) * getPDF_Erot_single(ry);
							//cout << "Calculating gamma with -> x: " << x << "; y: " << y << "; rx: " << rx << "; ry: " << ry;
							//cout << " -> gamma: " << g << "; e: " << e;
							//cout << " --> totProb = " << actProb << endl;
							sumAll += actProb;
						}
					}
				}
			}
		}
	}

	cout << "Sum all Prob = " << sumAll << " using"
			<< " d2Dmax: " << d2Dmax << " rmax:" << rmax
			<< " deltad: " << deltad << " deltar:" << deltar
			<< " --> ris = " << (deltad * deltad * deltar * deltar * sumAll) << endl;

	if (e == 0) exit(0);

	return (deltad * deltad * deltar * deltar * sumAll);

	/*std::list<MyCoord> cirDeltad;
	std::list<MyCoord> rotDeltar;

	for (double x = -d2Dmax; x <= d2Dmax; x+=deltad) {
		for (double y = -d2Dmax; y <= d2Dmax; y+=deltad) {
			MyCoord actCoord(x, y);
			if (actCoord.length() <= d2Dmax) {
				cirDeltad.push_back(actCoord);
			}
		}
	}
	for (double x = -rmax; x <= rmax; x+=deltar) {
		for (double y = -rmax; y <= rmax; y+=deltar) {
			rotDeltar.push_back(MyCoord(x, y));
		}
	}

	cout << endl;
	for (auto& earr : cirDeltad) {
		for (auto& r : rotDeltar) {
			double summ = getPDF_Eloc(earr) * getPDF_Erot(r);
			cout << summ << " ";
			ris += summ;
		}
		cout << endl;
	}
	cout << "RIS: " << ris << " -> deltas: " << (deltad * deltad) * (deltar * deltar) << endl;

	ris = (deltad * deltad) * (deltar * deltar) * ris;
	return ris;*/
}

void MultiFlow::calcProb_EReceivedTime_rec(double &acc, std::vector<double> &vect, double h, int t, double e, double deltae) {
	double ris = 0;

	/*cout << "vect -> [";
	for(auto& val : vect)
		cout << val << " ";
	cout << "]";*/

	double sume = 0;
	for (auto& ei : vect){
		sume += ei;
	}
	//if (MyCoord::close(sume, e)) {
	if (sume == e) {
		cout << "vect -> [";
		for(auto& val : vect)
			cout << val << " ";
		cout << "]";

		double product = 1.0;
		for (int j = 0; j < t; j++) {
			double pEReceived = calcProb_EReceived_new(h, vect[j]);
			cout << " " << pEReceived << " *";
			product *= pEReceived;
		}
		ris += product;

		cout << " --> OK --> " << ris << endl;
	}
	/*else {
		cout << endl;
	}*/

	acc += ris;

	if (sume < (t*e)) {
		for(int k = 0; k < t; k++){
			vect[k] += deltae;
			if (vect[k] > e) {
				vect[k] = 0;
			}
			else {
				break;
			}
		}

		return calcProb_EReceivedTime_rec(acc, vect, h, t, e, deltae);
	}
}

double MultiFlow::calcProb_EReceivedTime(double e, double deltae, double h, int t) {
	double ris = 0;
	std::vector<double> vect_e(t, 0);

	cout << "Making recursion for " << e << endl;
	calcProb_EReceivedTime_rec(ris, vect_e, h, t, e, deltae);
	cout << "End recursion - RIS: " << ris << endl << flush;

	return ris;
}

double MultiFlow::calculate_pWU(double h, int twu) {
	double pwu = 1;
	double ewu = Generic::getInstance().energyToWakeUp;
	double deltae = ewu / 20.0;

	double sumprob = 0;
	for (double e = 1; e <= ((ewu/deltae) - 1); e+= 1) {	//TODO start from 0
		double actVal = calcProb_EReceivedTime(e * deltae, deltae, h, twu);
		cout << "Prob_EReceivedTime = " << actVal << endl << endl;
		sumprob += actVal;
	}
	pwu = 1.0 - (deltae * sumprob);

	return pwu;
}

double MultiFlow::calc_Beta(double d3D, double h, double d2D) {
	return acos( (pow(d3D, 2.0) + pow(h, 2.0) - pow(d2D, 2.0)) / (2.0 * d3D * h) );
}

double MultiFlow::calc_smallGamma(double x, double y, double d3D, double h, double rho_x, double rho_y) {
	MyCoord uav(x, y, h);
	MyCoord sens(0, 0);
	MyCoord rho(x + h*tan(rho_y), y + h*tan(rho_x));

	double d_rho_2D = rho.distance(sens);
	double h_rho = rho.distance(uav);

	return acos((pow(h_rho, 2.0) + pow(d3D, 2.0) - pow(d_rho_2D, 2.0))/(2.0 * h_rho * d3D));
}

double MultiFlow::calc_Gain(double alpha, double gMAX, double alphaMAX) {
	if (alpha <= (alphaMAX / 2.0)) {
		return (gMAX + (-12 * pow(alpha / (alphaMAX / 2.0), 2.0)));
	}
	else {
		//return (-std::numeric_limits<double>::max());
		return -1000000000;
	}
}

double MultiFlow::calc_PathLoss(double d3D, double fMHz){
	return ((20.0 * log10(d3D)) + (20.0 * log10(fMHz)) - 27.55);
}

void MultiFlow::initEfficiencyMap(void) {
	efficiencyMap[-19] = 0;
	efficiencyMap[-18] = 0.005;
	efficiencyMap[-17] = 0.075;
	efficiencyMap[-16] = 0.15;
	efficiencyMap[-15] = 0.22;
	efficiencyMap[-14] = 0.28;
	efficiencyMap[-13] = 0.34;
	efficiencyMap[-12] = 0.385;
	efficiencyMap[-11] = 0.42;
	efficiencyMap[-10] = 0.44;
	efficiencyMap[-9] = 0.445;
	efficiencyMap[-8] = 0.442;
	efficiencyMap[-7] = 0.43;
	efficiencyMap[-6] = 0.415;
	efficiencyMap[-5] = 0.395;
	efficiencyMap[-4] = 0.37;
	efficiencyMap[-3] = 0.345;
	efficiencyMap[-2] = 0.315;
	efficiencyMap[-1] = 0.29;
	efficiencyMap[0] = 0.27;
	efficiencyMap[1] = 0.25;
	efficiencyMap[2] = 0.23;
	efficiencyMap[3] = 0.208;
	efficiencyMap[4] = 0.19;
	efficiencyMap[5] = 0.173;
	efficiencyMap[6] = 0.15;
	efficiencyMap[7] = 0.125;
	efficiencyMap[8] = 0.105;
	efficiencyMap[9] = 0.09;
	efficiencyMap[10] = 0.072;
	efficiencyMap[11] = 0.06;
	efficiencyMap[12] = 0.05;
	efficiencyMap[13] = 0.041;
	efficiencyMap[14] = 0.036;
	efficiencyMap[15] = 0.028;
	efficiencyMap[16] = 0.024;
	efficiencyMap[17] = 0.02;
	efficiencyMap[18] = 0.018;
	efficiencyMap[19] = 0;
}

double MultiFlow::calcRF2DC_efficiency(double rcvPow_dbm) {
	double eff_floor = 0;
	double eff_ceil = 0;
	int rcvPow_floor = floor(rcvPow_dbm);
	int rcvPow_ceil = ceil(rcvPow_dbm);

	if ((rcvPow_dbm >= -19) && (rcvPow_dbm <= 19)) {
		eff_floor = efficiencyMap[rcvPow_floor];
		eff_ceil = efficiencyMap[rcvPow_ceil];
	}

	//return ((1-(J5-FLOOR(J5)))*L5)+(((J5-FLOOR(J5)))*M5);
	return ( ( (1.0-(rcvPow_dbm-rcvPow_floor))*eff_floor ) + ( ((rcvPow_dbm-rcvPow_floor))*eff_ceil ) );
}

double MultiFlow::calc_Gamma(double x, double y, double rho_x, double rho_y) {
	double ris = 0;

	double beta, gamma, d3D;

	d3D = sqrt( pow(MyCoord(x, y).length(), 2.0) + pow(Generic::getInstance().flightAltitudeUAV, 2.0) );
	//cout << "d3D -> " << d3D << endl;

	beta = calc_Beta(d3D, Generic::getInstance().flightAltitudeUAV, MyCoord(x, y).length());
	//cout << "beta -> " << beta << endl;

	gamma = calc_smallGamma(x, y, d3D, Generic::getInstance().flightAltitudeUAV, rho_x, rho_y);
	//cout << "gamma -> " << gamma << endl;

	//cout << "wakeupTxPower -> " << Generic::getInstance().wakeupTxPower << endl;
	//cout << "gainU -> " << calc_Gain(gamma, Generic::getInstance().gUmax, Generic::getInstance().alphaUmax) << endl;
	//cout << "gainS -> " << calc_Gain(beta, Generic::getInstance().gSmax, Generic::getInstance().alphaSmax) << endl;
	//cout << "PL -> " << calc_PathLoss(d3D, Generic::getInstance().wakeupTxFrequency) << endl;

	ris += 10.0 * log10(Generic::getInstance().wakeupTxPower);
	ris += calc_Gain(gamma, Generic::getInstance().gUmax, Generic::getInstance().alphaUmax);
	ris += calc_Gain(beta, Generic::getInstance().gSmax, Generic::getInstance().alphaSmax);
	ris -= calc_PathLoss(d3D, Generic::getInstance().wakeupTxFrequency);

	double ris_W = pow(10.0, ris / 10.0) / 1000.0;
	double ris_store = ris_W * calcRF2DC_efficiency(ris_W);
	double ris_J = ris_store * Generic::getInstance().timeSlot;

	return ris_J;
}

void MultiFlow::init(void) {
	initEfficiencyMap();
}

void MultiFlow::run(int end_time) {

	/*for(double k = -5; k <= 5; k+=0.2) {
		cout << k << " " << RandomGenerator::get_PDF_normal(k, 0, 1) << endl;
	}
	exit(EXIT_FAILURE);*/

	pWU = calculate_pWU(Generic::getInstance().flightAltitudeUAV, 4);
	cerr << "fine pWU: " << pWU << endl;
	exit(EXIT_FAILURE);

	ChargingNode *leftmost = cs_map.begin()->second;
	while(actSensorTimeStamp < end_time){
		if (actSensorTimeStamp < actUAVTimeStamp) {
			actSensorTimeStamp = updateSensorsEnergy(actSensorTimeStamp, actUAVTimeStamp);
		}

		calculateTSP(leftmost);

		leftmost = getLeftMostUAV(end_time);
		if (leftmost == nullptr) {
			actUAVTimeStamp = end_time;
			if (actSensorTimeStamp < actUAVTimeStamp) {
				actSensorTimeStamp = updateSensorsEnergy(actSensorTimeStamp, actUAVTimeStamp);
			}
			break;
		}
		else {
			actUAVTimeStamp = leftmost->lastTimestamp;
		}
	}
}


