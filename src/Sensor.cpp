/*
 * Sensor.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "Sensor.h"

int Sensor::idSensGen = 0;

Sensor::Sensor(MyCoord sensCoord, double re) {
	coord = sensCoord;
	residual_energy = re;
	id = idSensGen++;
}

Sensor::Sensor(MyCoord sensCoord, double re, int id_new) {
	coord = sensCoord;
	residual_energy = re;
	id = id_new;
}

