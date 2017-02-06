/*
 * Pickup.h
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */

#ifndef SRC_PICKUP_H_
#define SRC_PICKUP_H_

#include"WPILib.h"
#include"CANTalon.h"

class Pickup{

public:

	Pickup(uint8_t canid);
	void Toggle();



private:

	struct pickuplist{

		CANTalon *pickupmotor;

		bool ison;

	};

	struct pickuplist *plist;


};




#endif /* SRC_PICKUP_H_ */
