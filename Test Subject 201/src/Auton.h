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

	Auton();
	double Update();
	void Drive();

private:

	enum autonstate {
			outOfView, inRangeL, inRangeR, inRange, inView, done
		};


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

	struct autonlist *alist;

};

#endif /* SRC_AUTON_H_ */
