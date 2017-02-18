/*
 * Auton.h
 *
 *  Created on: Feb 11, 2017
 *      Author: feds
 */

#ifndef SRC_AUTON_H_
#define SRC_AUTON_H_

#include"WPILib.h"
#include"Drivetrain.h"
#include"Timer.h"

class Auton {

public:

	Auton(DriveTrain *drive);
	double Update();
	void Drive();
	int Routes(frc::SampleRobot *robot);

private:

	enum autonstate {
			outOfView, inRangeL, inRangeR, inRange, inView, done
		};

	enum autonmode{gear, shoot, gearandshoot};
	enum team{red, blue};
	enum pos{left, right, center};

	struct autonlist {

		double dist;
		double cenx;
		double cenx2;
		double wid;
		double wid2;
		double rect1;
		double rect2;
		double distcalc1;
		double distcalc2;
		double hit;
		double hit2;

		std::shared_ptr<NetworkTable> table;

		DriveTrain *drivetrain;

		std::vector<double> centerX;
		std::vector<double> width;
		std::vector<double> height;

		int state;
		bool iroutput;
		double drivedist;

		Timer time;

		bool done = false;

	};

	struct switches{

		DigitalInput *Pos1;
		DigitalInput *Pos3;

		DigitalInput *Red;

		DigitalInput *Gear;
		DigitalInput *Shoot;

		autonmode mode;
		team Team;
		pos poss;

	};

	struct autonlist *alist;
	struct switches *switches;

};

#endif /* SRC_AUTON_H_ */
