/*
 * RandomGenerator.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#include "RandomGenerator.h"

RandomGenerator::RandomGenerator(unsigned int seed) {
	generator_rand = new std::default_random_engine(seed);
}

