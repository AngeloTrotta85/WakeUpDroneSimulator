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

#ifndef TSP_UAV_CODE_MF
#define TSP_UAV_CODE_MF 1000000
#endif

#ifndef TSP_DUMMY_CODE_START_MF
#define TSP_DUMMY_CODE_START_MF 2000000
#endif
#ifndef TSP_DUMMY_CODE_END_MF
#define TSP_DUMMY_CODE_END_MF 2000001
#endif

using namespace std;

class SensorNodeTree;
class ChargingNodeTree;

class ArcTree {
public:
	ArcTree(){
		snt_start = nullptr;
		snt_end = nullptr;
		cnt_start = nullptr;
		cnt_end = nullptr;
		u = nullptr;
		s = nullptr;
		timecost = timecost_Tslot = 0;
	}
public:
	SensorNodeTree *snt_start;
	SensorNodeTree *snt_end;
	ChargingNodeTree *cnt_start;
	ChargingNodeTree *cnt_end;

	UAV *u;
	Sensor *s;

	double timecost;
	int timecost_Tslot;

};

class ChargingNodeTree {
public:
	int id;
	double timestamp;
	int timestamp_Tslot;
	double timestamp_offset;
	bool charging;
	UAV* u;
	MyCoord pos;
	//list<ChargingNodeTree *> nextCN;
	//list<SensorNodeTree *> nextS;
	list<ArcTree *> nextCN;
	list<ArcTree *> nextS;
};

class SensorNodeTree {
public:
	int id;
	double timestamp;
	int timestamp_Tslot;
	double timestamp_offset;
	bool read;
	Sensor* s;
	MyCoord pos;
	//list<ChargingNodeTree *> nextCN;
	//list<SensorNodeTree *> nextS;
	list<ArcTree *> nextCN;
	list<ArcTree *> nextS;
};

class ChargingNode {
public:
	int id;
	UAV* u;
	MyCoord pos;
	double lastTimestamp;
	int lastTimestamp_tslot;
};

class SensorNode {
public:
	typedef struct SensorRead{
		double readTime;
		int readTime_tslot;
		UAV *uav;
	} SensorRead;
public:
	Sensor* sens;
	list<SensorRead> readings;
	list<SensorRead> real_readings;
	list<SensorRead> real_wakeup;
	double lastTimestamp;
	int lastTimestamp_tslot;

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

	typedef enum mftype {
		UAVNODE,
		SENSORNODE
	} mftype;

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
		int lastTimeStamp_tslot;
	} neighUAV;

	typedef struct sensElem {
		SensorNode *sens;
		double lastTimeStamp;
		int lastTimeStamp_tslot;
		long double lastResidualEnergy;
	} sensElem;

	typedef struct sensElemTree {
		SensorNode *sens;
		list<int> timestamp_read_tslot;
		list<double> timestamp_read;
		long double lastResidualEnergy;
	} sensElemTree;

public:
	ChargingNode *cn;
	map<int, neighUAV> neighMap;
	map<int, sensElem> sensMap;
	map<int, sensElemTree> sensMapTree;

	list<SensorNode *> activeTSP;

	bool tsp2update;
	int rechargeBulk;

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
	typedef enum {
		ALGO_BSF,
		ALGO_BSF_DISTANCE,
		ALGO_BSF_ENERGY,
		ALGO_DSF,
		ALGO_DSF_DISTANCE,
		ALGO_DSF_ENERGY
	} Algo_type;

public:
	MultiFlow(Algo_type at);

	void addSensor(Sensor *s);
	void addChargStationAndUAV(MyCoord c, UAV *u);
	void addChargStationAndUAV_distributed(MyCoord c, UAV *u);

	void run(int end_time);
	void run_distributed(double end_time);
	void run_tree_multiflow(double end_time);
	void run_tree_multiflow_distr(double end_time);

	void init(void);
	void init_treeMF(double endTS, double timeoffset);
	void init_matrix_treeMF(void);

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
	double calcLossSensorOriginal(SensorNode *s_check, list<SensorNode *> &sList, int texp);
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
	void calculateTreeBSF_and_UpdateMF(ChargingNode *leftmost, double end_time);

	double calcPowEta(int t);
	double calcPowEtaSens(double e, double t);
	double energy_loss_onArc(int tstart);

	void run_uav(UavDistributed *uav, double simTime) ;
	void updateNeighMaps(double timenow);

	void calculateTSP_distributed(UavDistributed *uav, MyCoord startPoint, MyCoord endPoint, double simTime);
	void calculateTSP_distributed_dummy(UavDistributed *uav, MyCoord startPoint, MyCoord endPoint, double simTime);

	int calculateMatrixTimeSlot(int id1, int id2);
	double calculateMatrixEnergy(int id1, int id2);
	double calculateLossBSF(list<pair<int,int>> &phi, SensorNode *sn, int tm_tslot, bool centralized, UavDistributed *uav);
	void activateBSFandRecharge(ChargingNode *cnode, list<SensorNode *> &tsp);

	void calculateBSF(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e, bool centralized, UavDistributed *uav);
	void calculateBSF_distance(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e, bool centralized, UavDistributed *uav);
	void calculateBSF_energy(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e, bool centralized, UavDistributed *uav);

	void calculateDSF(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e, bool centralized, UavDistributed *uav);
	void calculateDSF_distance(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e, bool centralized, UavDistributed *uav);
	void calculateDSF_energy(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e, bool centralized, UavDistributed *uav);

	void calculatePath(list<SensorNode *> &path, ChargingNode *cn, double tk, int tk_tslot, double uav_e, bool centralized, UavDistributed *uav);

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


	double getLastSensorRead_Tree(void);
	double calcLossSensorOriginal_Tree(SensorNode *s_check, list<SensorNode *> &sList, int texp);
	double calcIndex_Tree(void);
	bool updateSensorsEnergy_Tree(int starttime, int endtime);

	void run_uav_tree(UavDistributed *uav, double simTime, int simTime_tslot) ;

	void writeHitmaps_distr(std::string filename);
	void writeHitmaps_multiflow(std::string filename);

public:
	static bool sortEdgesBSF (const pair<SensorNode *, double> first, const pair<SensorNode *, double> second) {
		return first.second < second.second;
	}

private:
	map<int, ChargingNode *> cs_map;
	map<int, SensorNode *> sens_map;
	list<UavDistributed *> uav_list;
	list<SensorNode *> sens_list;

	map<int, double> efficiencyMap;

	//map<mftype, map<int, map<double, > > > multiflowTreeMAP;
	map<int, map<double, SensorNodeTree *> > mfTreeMAP_sensor;
	map<int, map<double, ChargingNodeTree *> > mfTreeMAP_uav;

	map<int, map<int, double> > mfTreeMatrix_time;
	map<int, map<int, int> > mfTreeMatrix_timeslot;
	map<int, map<int, double> > mfTreeMatrix_energy;

	double actSensorTimeStamp;
	double actUAVTimeStamp;

	double pWU;
	wakeupVal wuVal;

	Algo_type mfAlgoType;
};

#endif /* MULTIFLOW_H_ */
