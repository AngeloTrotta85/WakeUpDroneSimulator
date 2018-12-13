/*
 * Loss.h
 *
 *  Created on: Dec 13, 2018
 *      Author: angelo
 */

#ifndef LOSS_H_
#define LOSS_H_

#include <list>

class Sensor;

class Loss {
public:
	static Loss& getInstance(void) {
		static Loss    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Loss(void);         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Loss(Loss const&)	= delete;
	void operator=(Loss const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status

	static double algebraic_sum(double a, double b) {
		return (a + b - (a * b));
	}

public:
	void init(double kd, double kt, double ke, double a) {
		k_d = kd;
		k_t = kt;
		k_e = ke;
		alpha = a;
	}

	double calculate_loss_energy(Sensor *se, int tk, std::list<Sensor *> &sl);
	double calculate_loss_distance(Sensor *s1, Sensor *s2);
	double calculate_loss_time(int t1, int t2);
	double calculate_loss_correlation(Sensor *se, int tk, std::list<Sensor *> &sl);
	double calculate_loss_full(Sensor *se, int tk, std::list<Sensor *> &sl);

private:
	double k_d;
	double k_t;
	double k_e;

	double alpha;
};

#endif /* LOSS_H_ */