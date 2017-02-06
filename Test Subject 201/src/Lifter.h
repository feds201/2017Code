/*
 * Lifter.h
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */

#ifndef SRC_LIFTER_H_
#define SRC_LIFTER_H_

#include"WPILib.h"

class Lifter{


public:

	Lifter(int PWM);
	void Toggle();

private:

	struct lifterlist{

	Spark *liftermotor;

	bool ison;

	};

	struct lifterlist *llist;

};



#endif /* SRC_LIFTER_H_ */
