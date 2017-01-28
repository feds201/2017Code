/*
 * Auton.h
 *
 *  Created on: Jan 27, 2017
 *      Author: Andy
 */

#ifndef SRC_AUTON_H_
#define SRC_AUTON_H_

#include "DriveTrain.h"
#include "WPILib.h"
#include <timer.h>
#include "Vision.h"

class Auton {
public:

	Auton(uint8_t Lcanid, uint8_t Lcanid2, uint8_t Rcanid, uint8_t Rcanid2, int PCMCanid, int shifter1fwd,int shifter1rev, int shifter2fwd, int shifter2rev);
	void Run();





	enum pos{Left, Center, Right};
	enum mode{Gear, Shoot, GearAndShoot};
	enum color{Red, Blue};


private:


	struct autonList {

	DigitalInput *pos1;
	DigitalInput *pos3;
	DigitalInput *mode1;
	DigitalInput *mode3;
	DigitalInput *redblu;
	DigitalInput *IR;

	DriveTrain *drivetrain;

	bool stop;
	bool iroutput;
	float dist;

	Vision *vision;

	Timer time;

	pos Pos;
	mode Mode;
	color Color;

	};
	struct autonList *alist;
};

#endif /* SRC_AUTON_H_ */
