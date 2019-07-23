/*
 * RandomGenerator.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "RandomGenerator.h"

RandomGenerator::RandomGenerator(void) {
	generator_rand = std::default_random_engine();
	starting_seed = 0;
}

void RandomGenerator::setSeed(unsigned int seed) {
	starting_seed = seed;
	generator_rand.seed(seed);
}

double RandomGenerator::getRealUniform (double min, double max) {
	std::uniform_real_distribution<double> uniform_distribution(min, max);
	return uniform_distribution(generator_rand);
}

int RandomGenerator::getIntUniform (int min, int max) {
	std::uniform_int_distribution<int> uniform_distribution(min, max);
	return uniform_distribution(generator_rand);
}

double RandomGenerator::getRealNormal (double mean, double stdev) {
	std::normal_distribution<double> n_distribution(mean, stdev);
	return n_distribution(generator_rand);
}

