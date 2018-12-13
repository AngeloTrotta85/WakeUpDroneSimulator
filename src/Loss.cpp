/*
 * Loss.cpp
 *
 *  Created on: Dec 13, 2018
 *      Author: angelo
 */

#include "Loss.h"

#include "Sensor.h"
#include "Readings.h"

Loss::Loss() {
	k_d = 0.08;
	k_t = 0.02;
	k_e = 0.0005;

	alpha = 0.5;
}

double Loss::calculate_loss_energy(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		if (ss != se) {
			double actLoss = 0;
			if (se->residual_energy < ss->residual_energy) {
				double exp_exponent = (ss->residual_energy - se->residual_energy) * k_e;
				actLoss = 1.0 - (1.0 / exp(exp_exponent));

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
	}
	return ris;
}

double Loss::calculate_loss_distance(Sensor *s1, Sensor *s2) {
	double exp_exponent = s1->coord.distance(s2->coord) * k_d;
	return (1.0 / exp(exp_exponent));
}

double Loss::calculate_loss_time(int t1, int t2) {
	double exp_exponent = abs(t1 - t2) * k_t;
	return (1.0 / exp(exp_exponent));
}

double Loss::calculate_loss_correlation(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		for (auto& r : ss->mySensorReadings) {
			if (r->read_time < tk) {
				double loss_dist = calculate_loss_distance(se, ss);
				double loss_time = calculate_loss_time(tk, r->read_time);
				double actLoss = loss_dist * loss_time;
				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
	}
	return ris;
}

double Loss::calculate_loss_full(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double loss_energy = calculate_loss_energy(se, tk, sl);
	double loss_corr = calculate_loss_correlation(se, tk, sl);

	double ris = algebraic_sum( alpha * loss_corr, (1 - alpha) * loss_energy);

	//cout << "calculate_loss_full: " << ris << "; loss_energy: " << loss_energy << "; loss_corr: " << loss_corr m<< endl << flush;

	return ris;
}
