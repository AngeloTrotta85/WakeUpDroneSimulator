/*
 * Generic.cpp
 *
 *  Created on: Dec 19, 2018
 *      Author: angelo
 */

#include "Generic.h"

double Generic::getTime2Travel(MyCoord start, MyCoord end) {
	return (start.distance(end) / maxVelocity);
}

double Generic::getEnergy2Travel(MyCoord start, MyCoord end) {
	return (getTime2Travel(start, end) * singleMotorPowerUAV * 4.0);	// quadcopter
}

double Generic::getPrx_watt(MyCoord uavCoord, MyCoord sensorCoord) {
	double pathloss = (20.0 * log10(flightAltitudeUAV)) + (20.0 * log10(wakeupTxFrequency)) - 27.55;
	double ptx_dbm = 10.0 * log10(1000.0 * wakeupTxPower);
	double prx_dbm = ptx_dbm + antennaGainTx + antennaGainRx - pathloss;
	double prx_watt = pow(10.0, prx_dbm/10.0) / 1000.0;

	return prx_watt;
}

double Generic::getTime2Wake(MyCoord uavCoord, MyCoord sensorCoord) {
	return (energyToWakeUp / getPrx_watt(uavCoord, sensorCoord));
}

double Generic::getTime2WakeRead(MyCoord uavCoord, MyCoord sensorCoord) {
	double ris = 0;

	// time to wakeup
	ris += getTime2Wake(uavCoord, sensorCoord); 		// time to wake-up
	ris += uavTime2Read;

	return ris;
}

double Generic::getEnergy2WakeRead(MyCoord uavCoord, MyCoord sensorCoord) {
	double ris = 0;

	ris += wakeupTxPower * getTime2Wake(uavCoord, sensorCoord);		// energy to wake-up
	ris += uavEnergy2Read;

	//ris = 5000;
	return ris;
}
