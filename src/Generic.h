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
	void init(double ts) {
		timeSlot = ts;
	}
	void setSensorParam(double initEnergySensor, double sensorSelfDischarge, double eON, double eBOOT, bool fullRandom) {
		initSensorEnergy = initEnergySensor;
		energyON = eON;
		energyBOOT = eBOOT;
		fullRandomInit = fullRandom;

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
	void setUAVParam(double initEnergyUAV, double flightAltitude, double maxVel, double motorPower,
			double rechargePower, double time2read, double energy2read,
			double sGPS, double sPilot, double sRot) {
		initUAVEnergy = initEnergyUAV;
		flightAltitudeUAV = flightAltitude;
		maxVelocity = maxVel;
		singleMotorPowerUAV = motorPower;
		rechargeStation_power = rechargePower;
		uavTime2Read = time2read;
		uavEnergy2Read = energy2read;
		sigmaGPS = sGPS;
		sigmaPilot = sPilot;
		sigmaRot = sRot;
	}
	void setWakeUpParam(double wakeupPower, double wakeupMinPower, double wakeupFrequency, double energy2wakeup, double gTx, double gRx,
			double gUm, double aUm, double gSm, double aSm) {
		wakeupTxPower = wakeupPower;
		wakeupTxFrequency = wakeupFrequency;
		wakeupTxMinPower = wakeupMinPower;
		energyToWakeUp = energy2wakeup;
		antennaGainTx = gTx;
		antennaGainRx = gRx;
		gUmax = gUm;
		alphaUmax = aUm;
		gSmax = gSm;
		alphaSmax = aSm;
	}

	void setStatParam(bool makeOnrunStat, const std::string statFile, const std::string hitmapFile) {
		makeRunSimStat = makeOnrunStat;
		statFilename = std::string(statFile);
		hitmapFilename = std::string(hitmapFile);
	}

	void setMultiFlowParam(double tsup, double ttout, int numr, double ps_sup, double ps_tx,
			double ps_rx, double pu_sup, double pu_tx, double pu_rx, double pu_fly) {
		tstartup = tsup;
		ttimeout = ttout;
		nr = numr;
		pSstartup = ps_sup;
		pStx = ps_tx;
		pSrx = ps_rx;
		pUstartup = pu_sup;
		pUtx = pu_tx;
		pUrx = pu_rx;
		pUfly = pu_fly;
	}

	double getTime2Travel(MyCoord start, MyCoord end);
	double getEnergy2Travel(MyCoord start, MyCoord end);

	double getPrx_watt(MyCoord uavCoord, MyCoord sensorCoord);
	double getTime2Wake(MyCoord uavCoord, MyCoord sensorCoord);
	double getTime2WakeRead(MyCoord uavCoord, MyCoord sensorCoord);
	double getEnergy2WakeRead(MyCoord uavCoord, MyCoord sensorCoord);

public:
	double timeSlot;
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
	bool fullRandomInit;
	double energyON;
	double energyBOOT;
	double wakeupTxPower;
	double wakeupTxMinPower;
	double wakeupTxFrequency;
	double energyToWakeUp;
	double antennaGainTx;
	double antennaGainRx;
	bool makeRunSimStat;
	std::string statFilename;
	std::string hitmapFilename;

	double tstartup;
	double ttimeout;
	int nr;
	double pSstartup;
	double pStx;
	double pSrx;
	double pUstartup;
	double pUtx;
	double pUrx;
	double pUfly;
	double gUmax;
	double alphaUmax;
	double gSmax;
	double alphaSmax;
	double sigmaGPS;
	double sigmaPilot;
	double sigmaRot;
};

#endif /* GENERIC_H_ */
