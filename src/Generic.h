/*
 * Generic.h
 *
 *  Created on: Dec 19, 2018
 *      Author: angelo
 */

#ifndef GENERIC_H_
#define GENERIC_H_

#include "MyCoord.h"

class Generic {
public:
	static Generic& getInstance(void) {
		static Generic    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Generic(void);         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Generic(Generic const&)	= delete;
	void operator=(Generic const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status


public:
	void init(void) {

	}

	double getTime2Travel(MyCoord start, MyCoord end);
	double getEnergy2Travel(MyCoord start, MyCoord end);

	double getTime2WakeRead(MyCoord uavCoord, MyCoord sensorCoord);
	double getEnergy2WakeRead(MyCoord uavCoord, MyCoord sensorCoord);
};

#endif /* GENERIC_H_ */
