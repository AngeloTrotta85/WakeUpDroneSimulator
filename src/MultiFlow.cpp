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
}

double MultiFlow::getPDF_Eloc_single(double e) {
	double meanGPS = 0;
	double sigmaGPS = 2;
	double meanPilot = 0;
	double sigmaPilot = 1;

	return RandomGenerator::get_PDF_normal(e, meanGPS + meanPilot, sigmaGPS + sigmaPilot);
}

double MultiFlow::getPDF_Erot_single(double r) {
	double mean = 0;
	double sigma = 0.3;

	return RandomGenerator::get_PDF_normal(r, mean, sigma);
}

double MultiFlow::calcProb_EReceived(double e) {
	//double ris = 0;

	double d2Dmax = 20.0;
	double deltad = d2Dmax/10.0;

	double rmax = M_PI_2;
	double deltar = rmax/10.0;

	double sumX = 0;
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

	//cout << "deltad: " << deltad << " deltar:" << deltar <<
	//		" -> sumX: " << sumX << "; sumY: " << sumY << "; sumRX: " << sumRX << "; sumRY: " << sumRY  << endl;

	return (deltad * sumX * deltad * sumY * deltar * sumRX * deltar * sumRY);

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

void MultiFlow::calcProb_EReceivedTime_rec(double &acc, std::vector<double> &vect, int t, double e, double deltae) {
	double ris = 0;

	cout << "vect -> [";
	for(auto& val : vect)
		cout << val << " ";
	cout << "]";

	double sume = 0;
	for (auto& ei : vect){
		sume += ei;
	}
	if (sume == e) {
		double product = 1.0;
		for (int j = 0; j < t; j++) {
			double pEReceived = calcProb_EReceived(vect[j]);
			cout << " " << pEReceived << " *";
			product *= pEReceived;
		}
		ris += product;

		cout << " --> OK --> " << ris << endl;
	}
	else {
		cout << endl;
	}

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

		return calcProb_EReceivedTime_rec(acc, vect, t, e, deltae);
	}
}

double MultiFlow::calcProb_EReceivedTime(double e, double deltae, int t) {
	double ris = 0;
	std::vector<double> vect_e(t, 0);

	cout << "Making recursion for " << e << endl;
	calcProb_EReceivedTime_rec(ris, vect_e, t, e, deltae);
	cout << "End recursion - RIS: " << ris << endl << flush;

	return ris;
}

double MultiFlow::calculate_pWU(void) {
	double pwu = 1;
	double ewu = 20;
	double deltae = 5;
	int twu = 3;

	double sumprob = 0;
	for (double e = 0; e <= (ewu/deltae); e+= 1) {
		double actVal = calcProb_EReceivedTime(e * deltae, deltae, twu);
		cout << "Prob_EReceivedTime = " << actVal << endl << endl;
		sumprob += actVal;
	}
	pwu = 1.0 - (deltae * sumprob);

	return pwu;
}

double MultiFlow::calc_Beta(double d3D, double h, double d2D) {
	return acos( (pow(d3D, 2.0) + pow(h, 2.0) - pow(d2D, 2.0)) / (2.0 * d3D * h) );
}

double MultiFlow::calc_Gain(double alpha, double gMAX, double alphaMAX) {
	if (alpha <= (alphaMAX / 2.0)) {
		return (gMAX + (-12 * pow(alpha / (alphaMAX / 2.0), 2.0)));
	}
	else {
		return (std::numeric_limits<double>::min());
	}
}

double MultiFlow::calc_PathLoss(double d3D, double fMHz){
	return ((20.0 * log10(d3D)) + (20.0 * log10(fMHz)) - 27.55);
}

double MultiFlow::calc_Gamma(double x, double y, double rho_x, double rho_y) {
	double ris = 0;

	double beta, gamma, d3D;

	d3D = sqrt( pow(MyCoord(x, y).length(), 2.0) + pow(Generic::getInstance().flightAltitudeUAV, 2.0) );

	beta = calc_Beta(d3D, Generic::getInstance().flightAltitudeUAV, MyCoord(x, y).length());

	ris += Generic::getInstance().wakeupTxPower;
	ris += calc_Gain(gamma, Generic::getInstance().gUmax, Generic::getInstance().alphaUmax);
	ris += calc_Gain(beta, Generic::getInstance().gSmax, Generic::getInstance().alphaSmax);
	ris -= calc_PathLoss(d3D, Generic::getInstance().wakeupTxFrequency);

	return ris;
}

void MultiFlow::run(int end_time) {

	/*for(double k = -5; k <= 5; k+=0.2) {
		cout << k << " " << RandomGenerator::get_PDF_normal(k, 0, 1) << endl;
	}
	exit(EXIT_FAILURE);*/

	pWU = calculate_pWU();
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


