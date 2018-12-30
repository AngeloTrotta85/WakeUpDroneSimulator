/*
 * Generic.h
 *
 *  Created on: Dec 19, 2018
 *      Author: angelo
 */

#ifndef GENERIC_H_
#define GENERIC_H_

#include "MyCoord.h"

class Generic {
public:
	static Generic& getInstance(void) {
		static Generic    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Generic(void){};         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Generic(Generic const&)	= delete;
	void operator=(Generic const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status


public:
	void init(int ts) {
		timeSlot = ts;
	}
	void setSensorParam(double initEnergySensor, double sensorSelfDischarge, double eON, double eBOOT) {
		initSensorEnergy = initEnergySensor;
		energyON = eON;
		energyBOOT = eBOOT;

		sensorBatterySelfDischarge = sensorSelfDischarge;
		if (sensorBatterySelfDischarge > 0) {
			// sensorBatterySelfDischarge is per months
			long double selfDischargeRatio = (100.0 - sensorBatterySelfDischarge) / 100.0;
			long double slotsPerMonth = (30.0 * 24.0 * 60.0 * 60.0) / timeSlot;

			sensorSelfDischargePerSlot = powl(selfDischargeRatio, 1.0 / slotsPerMonth);
		}
		else {
			sensorSelfDischargePerSlot = 0;
		}
	}
	void setUAVParam(double initEnergyUAV, double flightAltitude, double maxVel, double motorPower, double rechargePower, double time2read, double energy2read) {
		initUAVEnergy = initEnergyUAV;
		flightAltitudeUAV = flightAltitude;
		maxVelocity = maxVel;
		singleMotorPowerUAV = motorPower;
		rechargeStation_power = rechargePower;
		uavTime2Read = time2read;
		uavEnergy2Read = energy2read;
	}
	void setWakeUpParam(double wakeupPower, double wakeupFrequency, double energy2wakeup, double gTx, double gRx) {
		wakeupTxPower = wakeupPower;
		wakeupTxFrequency = wakeupFrequency;
		energyToWakeUp = energy2wakeup;
		antennaGainTx = gTx;
		antennaGainRx = gRx;
	}

	double getTime2Travel(MyCoord start, MyCoord end);
	double getEnergy2Travel(MyCoord start, MyCoord end);

	double getPrx_watt(MyCoord uavCoord, MyCoord sensorCoord);
	double getTime2Wake(MyCoord uavCoord, MyCoord sensorCoord);
	double getTime2WakeRead(MyCoord uavCoord, MyCoord sensorCoord);
	double getEnergy2WakeRead(MyCoord uavCoord, MyCoord sensorCoord);

public:
	int timeSlot;
	double initSensorEnergy;
	double initUAVEnergy;
	double flightAltitudeUAV;
	double maxVelocity;
	double uavTime2Read;
	double uavEnergy2Read;
	double singleMotorPowerUAV;
	double rechargeStation_power;
	double sensorBatterySelfDischarge;
	long double sensorSelfDischargePerSlot;
	double energyON;
	double energyBOOT;
	double wakeupTxPower;
	double wakeupTxFrequency;
	double energyToWakeUp;
	double antennaGainTx;
	double antennaGainRx;
};

#endif /* GENERIC_H_ */
