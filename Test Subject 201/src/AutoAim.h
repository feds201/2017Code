/*
 * AutoAim.h
 *
 *  Created on: Feb 18, 2017
 *      Author: feds
 */

#ifndef SRC_AUTOAIM_H_
#define SRC_AUTOAIM_H_

#include"WPILib.h"
#include"DriveTrain.h"

class AutoAim{

public:

	AutoAim(DriveTrain* drive);
	double MotorCalc();
	double DistCalc();
	bool Aim();
	double MotorSpeed();

	enum state{found, lost, done};

private:

	struct autoaimlist{

		DriveTrain *drivetrain;

		Solenoid *frontlight;

		double driveDist;

		std::shared_ptr<NetworkTable> table;
		std::vector<double> centerX;
		std::vector<double> height;

		state aimstate;

		double centx = 0;
		double centx2 = 0;

		double hit = 0;
		double hit2 = 0;
		double dist = 0;

	};
	struct autoaimlist *aalist;
};



#endif /* SRC_AUTOAIM_H_ */
