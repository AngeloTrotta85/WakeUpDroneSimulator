/*
 * MultiFlow.h
 *
 *  Created on: May 16, 2019
 *      Author: angelo
 */

#ifndef MULTIFLOW_H_
#define MULTIFLOW_H_

#include <list>       // std::list
#include <map>       // std::map

#include "UAV.h"
#include "Sensor.h"
#include "MyCoord.h"
#include "Generic.h"

#ifndef TSP_UAV_CODE
#define TSP_UAV_CODE 100000
#endif

#ifndef TSP_DUMMY_CODE_START
#define TSP_DUMMY_CODE_START 200000
#endif
#ifndef TSP_DUMMY_CODE_END
#define TSP_DUMMY_CODE_END 200001
#endif

using namespace std;

class ChargingNode {
public:
	int id;
	UAV* u;
	MyCoord pos;
	double lastTimestamp;
};

class SensorNode {
public:
	typedef struct SensorRead{
		double readTime;
		UAV *uav;
	} SensorRead;
public:
	Sensor* sens;
	list<SensorRead> readings;
	int lastTimestamp;

	double accumulatedEnergy_uJ;
	int irradiatingTimeSlots;
	int startupTimeSlots;
	int commTimeSlots;
	int nCommAttempt;
	UAV *irradiatingUAV;
};

class TSP2MultiFlow {
public:
	static bool sortEdges (const TSP2MultiFlow *first, const TSP2MultiFlow *second) {
		return first->weight < second->weight;
	}
public:
	TSP2MultiFlow(SensorNode *s1, SensorNode *s2, double w) {
		first = s1;
		second = s2;
		weight = w;
		idTSP = -1;
	};

public:
	SensorNode *first;
	SensorNode *second;
	double weight;
	int idTSP;
};

class UavDistributed {
public:
	typedef enum uavState {
		RECHARGING,
		MOVING,
		WAKINGUP,
		STARTINGUP,
		READING
	} uavState;

	typedef struct neighUAV {
		UavDistributed *uav;
		double lastTimeStamp;
	} neighUAV;

	typedef struct sensElem {
		SensorNode *sens;
		double lastTimeStamp;
	} sensElem;

public:
	ChargingNode *cn;
	map<int, neighUAV> neighMap;
	map<int, sensElem> sensMap;

	list<SensorNode *> activeTSP;

	uavState us;
};

class MultiFlow {
public:
	typedef struct wakeupVal {
		double h;
		double estimatedIrrEnergyPerSlot_uJ;
		double maxTwakeup;
		double commProb;
	} wakeupVal;

public:
	MultiFlow();

	void addSensor(Sensor *s);
	void addChargStationAndUAV(MyCoord c, UAV *u);
	void addChargStationAndUAV_distributed(MyCoord c, UAV *u);

	void run(int end_time);
	void run_distributed(double end_time);

	void init(void);

	//double getPDF_Eloc(MyCoord e);
	//double getPDF_Erot(MyCoord r);

	double getPDF_Eloc_single(double e);
	double getPDF_Erot_single(double r);

	double calc_d2D_max(double h, double alpha_max);
	double calcProb_EReceived_new(double h, double e);
	double calcProb_EReceived(double h, double e);
	void calcProb_EReceivedTime_rec(double &acc, std::vector<double> &vect, double h, int t, double e, double deltae);
	double calcProb_EReceivedTime(double e, double deltae, double h, int t);
	double calculate_pWU(double h, int twu, double sigma2loc, double sigma2rho);

	ChargingNode *getLeftMostUAV(int end_time);
	double sensor_energy_loss_read(double pwu);
	bool updateSensorsEnergy(int starttime, int endtime);

	double calcTimeToTravel(MyCoord p1, MyCoord p2);
	double calcEnergyToTravel(MyCoord p1, MyCoord p2);
	double calcTimeToWuData(void);
	double calcEnergyToWuData(double pWU);

	void activateTSPandRecharge(ChargingNode *cnode, list<SensorNode *> &tsp);
	double calcLossSensor(SensorNode *s_check, list<SensorNode *> &sList, int texp);
	double calcLossSensorPresentFuture(SensorNode *s_check, std::list<SensorNode *> &sList, int texp);
	double calcLossSensor_distributed(SensorNode *sens, UavDistributed *uav, double texp);
	SensorNode *getMinLossSensor(list<SensorNode *> &sList, int texp);
	SensorNode *getMinLossSensor_distributed(UavDistributed *uav, list<SensorNode *> &sList, double texp);
	double calculateCosts1Edge(TSP2MultiFlow *e);
	double calculateCosts1Edge(SensorNode *s1, SensorNode *s2);
	void calculateTSP_incremental(list<SensorNode *> &newTSP, list<SensorNode *> &actTSP,
			SensorNode *sj, ChargingNode *cnode, double &tsp_time, double &tsp_energy_cost);
	void calculateTSP_incremental_distributed(list<SensorNode *> &newTSP, list<SensorNode *> &actTSP,
			SensorNode *sj, MyCoord startPoint, MyCoord endPoint, double &tsp_time, double &tsp_energy_cost);
	void calculateTSP_and_UpdateMF(ChargingNode *leftmost);

	double calcPowEta(int t);
	double calcPowEtaSens(double e, double t);
	double energy_loss_onArc(int tstart);

	void run_uav(UavDistributed *uav, double simTime) ;
	void updateNeighMaps(double timenow);

	void calculateTSP_distributed(UavDistributed *uav, MyCoord startPoint, MyCoord endPoint, double simTime);
	void calculateTSP_distributed_dummy(UavDistributed *uav, MyCoord startPoint, MyCoord endPoint, double simTime);

public:
	void initEfficiencyMap(void);
	double calc_Beta(double d3D, double h, double d2D);
	double calc_smallGamma(double x, double y, double d3D, double h, double rho_x, double rho_y);
	double calc_Gain(double alpha, double gMAX, double alphaMAX);
	double calc_PathLoss(double d3D, double fMHz);
	double calc_Gamma(double x, double y, double rho_x, double rho_y);
	double calcRF2DC_efficiency(double rcvPow);

	double getLastSensorRead(void);
	double calcIndex(void);

private:
	map<int, ChargingNode *> cs_map;
	list<UavDistributed *> uav_list;
	list<SensorNode *> sens_list;

	map<int, double> efficiencyMap;

	double actSensorTimeStamp;
	double actUAVTimeStamp;

	double pWU;
	wakeupVal wuVal;
};

#endif /* MULTIFLOW_H_ */
