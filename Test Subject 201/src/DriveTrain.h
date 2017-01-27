/*
 * DriveTrain.h
 *
 *  Created on: Jan 27, 2017
 *      Author: Andy
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include "WPILib.h"
#include "CANTalon.h"

class DriveTrain {
public:

	DriveTrain(uint8_t Lcanid, uint8_t Lcanid2, uint8_t Rcanid, uint8_t Rcanid2, int PCMCanid, int shifter1fwd,
			int shifter1rev, int shifter2fwd, int shifter2rev);

	void Drive(float fwd, float trn);

	enum motorSide{leftSide, rightSide};

	float getMotorVel(motorSide encside);



private:

	struct motorlist{

		CANTalon *Lmotor1;
		CANTalon *Lmotor2;
		CANTalon *Rmotor1;
		CANTalon *Rmotor2;

		uint8_t Lmotorcanid;
		uint8_t Lmotor2canid;
		uint8_t Rmotorcanid;
		uint8_t Rmotor2canid;

		float Lmotors;
		float Rmotors;

	};

	struct shifterlist{

		DoubleSolenoid *shifter1;
		DoubleSolenoid *shifter2;

		int PCMid;
		int shifterfwd;
		int shiferrev;
		int shifter2fwd;
		int shifter2rev;

	};


	struct shifterlist *slist;
	struct motorlist *mlist;

};

#endif /* SRC_DRIVETRAIN_H_ */
