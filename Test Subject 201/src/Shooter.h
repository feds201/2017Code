/*
 * Shooter.h
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */

#ifndef SRC_SHOOTER_H_
#define SRC_SHOOTER_H_

#include"WPILib.h"
#include"CANTalon.h"

class Shooter{

public:

	Shooter(uint8_t canid, uint8_t canid2);
	void SpinUp();
	void UpdateSpeed(float speed);
	void Shoot();
	void Stop();

private:

	struct shooterlist{

		CANTalon *shooter1;
		CANTalon *shooter2;

		DoubleSolenoid *shoot;

		float speed;

	};


	struct shooterlist *slist;


};




#endif /* SRC_SHOOTER_H_ */
