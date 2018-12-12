/*
 * UAV.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "UAV.h"

int UAV::idUAVGen = 0;

UAV::UAV(MyCoord recCoord, double re) {
	recharge_coord = recCoord;
	residual_energy = re;
	id = idUAVGen++;
}

UAV::UAV(MyCoord recCoord, double re, int id_new) {
	recharge_coord = recCoord;
	residual_energy = re;
	id = id_new;
}

