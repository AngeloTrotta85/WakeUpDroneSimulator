/*
 * RandomGenerator.h
 *
 *  Created on: Dec 12, 2018
 *      Author: angelo
 */

#ifndef RANDOMGENERATOR_H_
#define RANDOMGENERATOR_H_

#include <random>
#include <chrono>

class RandomGenerator {
public:
        static RandomGenerator& getInstance(void) {
            static RandomGenerator    instance; 	// Guaranteed to be destroyed.

            // Instantiated on first use.
            return instance;
        }
    private:
        RandomGenerator(void);         // Constructor? (the {} brackets) are needed here.

        // C++ 11
        // =======
        // We can use the better technique of deleting the methods
        // we don't want.
    public:
        RandomGenerator(RandomGenerator const&)	= delete;
        void operator=(RandomGenerator const&)  = delete;

        // Note: Scott Meyers mentions in his Effective Modern
        //       C++ book, that deleted functions should generally
        //       be public as it results in better error messages
        //       due to the compilers behavior to check accessibility
        //       before deleted status


    public:
        void setSeed(unsigned int seed);

        double getRealUniform (double min, double max);
        int getIntUniform (int min, int max);
        double getRealNormal (double mean, double stdev);

        static double get_PDF_normal(double x, double mu, double sigma2) {
        	if (sigma2 == 0){
        		if (x == mu) return 1;
        		else return 0;
        	}
        	else {
        		return (1.0 / sqrt(2.0 * M_PI * sigma2)) * exp(-(pow(x - mu, 2.0)/(2.0 * sigma2)));
        	}
        	//return ((1.0 / (sqrt(2.0 * M_PI * sigma2))) * exp());
        }

	unsigned int getSeed() const {
		return starting_seed;
	}

private:
	std::default_random_engine generator_rand;
	unsigned int starting_seed;
};

#endif /* RANDOMGENERATOR_H_ */
