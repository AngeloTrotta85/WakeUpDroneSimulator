/*
 * Loss.cpp
 *
 *  Created on: Dec 13, 2018
 *      Author: angelo
 */

#include "Loss.h"

#include "Sensor.h"
#include "Readings.h"

#define MIN_TIME_LOSS 0.01

Loss::Loss() {
	k_d = 0.08;
	k_t = 0.02;
	k_e = 0.0005;

	m_d = 1000;
	m_t = 7200;
	m_e = 1200;
	use_Sigmoid = false;

	alpha = 0.5;
}

double Loss::calculate_loss_energy(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		if (ss->id != se->id) {
			double actLoss = 0;
			if (se->residual_energy < ss->residual_energy) {
				if(use_Sigmoid) {
					double exp_exponent = ((ss->residual_energy - se->residual_energy) - m_e) * k_e;
					actLoss = 1.0 - (1.0 / (1.0 + exp(exp_exponent)));
				}
				else {
					double exp_exponent = (ss->residual_energy - se->residual_energy) * k_e;
					actLoss = 1.0 - (1.0 / exp(exp_exponent));
				}

				if (actLoss > ris) {
					ris = actLoss;
				}
			}
		}
	}
	return ris;
}

double Loss::calculate_loss_distance(Sensor *s1, Sensor *s2) {
	if (use_Sigmoid) {
		double exp_exponent = (s1->coord.distance(s2->coord) - m_d) * k_d;
		return (1.0 / (1.0 + exp(exp_exponent)));
	}
	else {
		double exp_exponent = s1->coord.distance(s2->coord) * k_d;
		return (1.0 / exp(exp_exponent));
	}
}

double Loss::calculate_loss_time(int t1, int t2) {
	if (use_Sigmoid) {
		double exp_exponent = (abs(t1 - t2) - m_t) * k_t;
		return (1.0 / (1.0 + exp(exp_exponent)));
	}
	else {
		double exp_exponent = abs(t1 - t2) * k_t;
		return (1.0 / exp(exp_exponent));
	}
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

				if (loss_time < MIN_TIME_LOSS) break;
			}
			else if ((r->read_time == tk) && (r->sequenceReading < (*se->mySensorReadings.begin())->sequenceReadingCounter)) {
				double loss_dist = calculate_loss_distance(se, ss);
				double loss_time = calculate_loss_time(tk, r->read_time);
				double actLoss = loss_dist * loss_time;
				if (actLoss > ris) {
					ris = actLoss;
				}

				if (loss_time < MIN_TIME_LOSS) break;
			}
		}
		if (ss->isBooked()) {
			double loss_dist = calculate_loss_distance(se, ss);
			double loss_time = calculate_loss_time(tk, tk-1);
			double actLoss = loss_dist * loss_time;
			if (actLoss > ris) {
				ris = actLoss;
			}

			if (loss_time < MIN_TIME_LOSS) break;
		}
	}
	return ris;
}

double Loss::calculate_loss_full(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double loss_energy = calculate_loss_energy(se, tk, sl);
	double loss_corr = calculate_loss_correlation(se, tk, sl);

	//double ris = algebraic_sum( alpha * loss_corr, (1 - alpha) * loss_energy);
	double ris = (alpha * loss_corr) + ((1.0 - alpha) * loss_energy);

	//cout << "calculate_loss_full: " << ris << "; loss_energy: " << loss_energy << "; loss_corr: " << loss_corr m<< endl << flush;

	return ris;
}





double Loss::calculate_loss_energy_reading(Readings *re, int tk, std::list<Sensor *> &sl) {
	return calculate_loss_energy(re->sensor, tk, sl);
}

double Loss::calculate_loss_distance_reading(Readings *r1, Readings *r2) {
	return calculate_loss_distance(r1->sensor, r2->sensor);
}

double Loss::calculate_loss_time_reading(int t1, int t2) {
	return calculate_loss_time(t1, t2);
}

double Loss::calculate_loss_correlation_reading(Readings *re, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		for (auto& r : ss->mySensorReadings) {
			if ( 	(r->read_time < tk) ||
					((r->read_time == tk) && (r->sequenceReading < re->sequenceReading)) ){
				double loss_dist = calculate_loss_distance_reading(re, r);
				double loss_time = calculate_loss_time_reading(tk, r->read_time);
				double actLoss = loss_dist * loss_time;
				if (actLoss > ris) {
					ris = actLoss;
				}

				if (loss_time < MIN_TIME_LOSS) break;
			}
		}
	}
	return ris;
}

double Loss::calculate_loss_full_reading(Readings *re, int tk, std::list<Sensor *> &sl) {
	double loss_energy = calculate_loss_energy_reading(re, tk, sl);
	double loss_corr = calculate_loss_correlation_reading(re, tk, sl);
	//double ris = algebraic_sum( alpha * loss_corr, (1 - alpha) * loss_energy);
	double ris = (alpha * loss_corr) + ((1.0 - alpha) * loss_energy);
	//std::cout << " [Calculate_loss_full_reading: " << ris << "; loss_energy: " << loss_energy << "; loss_corr: " << loss_corr << "] ";
	return ris;
}

double Loss::calculate_index_full_reading(int tk, std::list<Sensor *> &sl) {
	double ris = 0;

	for (auto& ss : sl) {
		for (auto& r : ss->mySensorReadings) {
			if (r->read_time <= tk) {
				//double gain = 1.0 - calculate_loss_full_reading(r, tk, sl);
				//ris += gain;
				//std::cout << " [Gain for reading: " << r->sequenceReading << " at time " << r->read_time << ": " << gain << "] " << std::endl;
				ris += r->gain;
			}
		}
	}

	return ris;
}

double Loss::calculate_correlationGain_full_reading(int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		for (auto& r : ss->mySensorReadings) {
			if (r->read_time <= tk) {
				ris += r->correlation_gain;
			}
		}
	}
	return ris;
}

double Loss::calculate_correlationLoss_full_reading(int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		for (auto& r : ss->mySensorReadings) {
			if (r->read_time <= tk) {
				ris += r->correlation_loss;
			}
		}
	}
	return ris;
}

double Loss::calculate_energyGain_full_reading(int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		for (auto& r : ss->mySensorReadings) {
			if (r->read_time <= tk) {
				ris += r->energy_gain;
			}
		}
	}
	return ris;
}

double Loss::calculate_energyLoss_full_reading(int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		for (auto& r : ss->mySensorReadings) {
			if (r->read_time <= tk) {
				ris += r->energy_loss;
			}
		}
	}
	return ris;
}

void Loss::calculate_reading_par(int tk, Readings *r, std::list<Sensor *> &sl) {
	r->full_loss = calculate_loss_full_reading(r, tk, sl);
	r->gain = 1.0 - r->full_loss;

	r->energy_loss = calculate_loss_energy_reading(r, tk, sl);
	r->energy_gain = 1.0 - r->energy_loss;

	r->correlation_loss = calculate_loss_correlation_reading(r, tk, sl);
	r->correlation_gain = 1.0 - r->correlation_loss;
}


