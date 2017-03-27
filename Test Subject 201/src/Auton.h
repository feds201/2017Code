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
#include"AutoAim.h"
#include"Shooter.h"

class Auton {

public:

	Auton(DriveTrain *drive, AutoAim *aim, Shooter *shoot);
	double Update();
	void ShootLow();
	void Drive();
	int Routes(frc::SampleRobot *robot);

private:

	enum aimstate{found, lost, atGoal, ag};

	enum autonstate {
			outOfView, inRange, inView, done
		};

	enum autonmode{gear, shoot, gearandshoot};
	enum team{red, blue};
	enum pos{left, right, center};
	enum dirOfRot{clockwise, counterclockwise};

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

		Shooter *shooter;

		bool shooterpos = false;

		DigitalInput *ir;

		AutoAim *aim;

		std::vector<double> centerX;
		std::vector<double> width;
		std::vector<double> height;

		aimstate aimState;


		int state;
		bool iroutput;
		double drivedist;

		Timer time;

		bool isBolth;

		bool ongoal = false;
		bool done = false;
		bool found = false;

		bool resenc = false;

		bool part1 = false;
		bool part2 = false;

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
		dirOfRot rotDir = clockwise;

	};

	struct autonlist *alist;
	struct switches *switches;

};

#endif /* SRC_AUTON_H_ */
