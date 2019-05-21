/*
 * MultiFlow.cpp
 *
 *  Created on: May 16, 2019
 *      Author: angelo
 */

#include "MultiFlow.h"
#include "Generic.h"

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

double MultiFlow::calculate_pWU(void) {
	return 1;
}

void MultiFlow::run(int end_time) {

	pWU = calculate_pWU();

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


