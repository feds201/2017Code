/*
 * GearFlipper.h
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */

#ifndef SRC_GEARFLIPPER_H_
#define SRC_GEARFLIPPER_H_

#include"WPILib.h"

class GearFlipper{

public:

	GearFlipper();
	void Toggle();


private:

	struct gearflipperlist{

		DoubleSolenoid *flipper;

	};

	struct gearflipperlist *gflist;

};




#endif /* SRC_GEARFLIPPER_H_ */
