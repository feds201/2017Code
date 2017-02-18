/*
 * Auton.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: feds
 */
#include"WPILib.h"
#include"Auton.h"
#include <iostream>

Auton::Auton(DriveTrain* drive) {

	alist = new struct autonlist;
	switches = new struct switches;

	alist->table = NetworkTable::GetTable("GRIP/myContoursReport");
	alist->drivetrain = drive;

	alist->centerX = alist->table->GetNumberArray("centerX",
			llvm::ArrayRef<double>());
	alist->width = alist->table->GetNumberArray("width",
			llvm::ArrayRef<double>());
	alist->height = alist->table->GetNumberArray("height",
			llvm::ArrayRef<double>());

	switches->Gear = new DigitalInput(4);
	switches->Shoot = new DigitalInput(5);
	switches->Pos1 = new DigitalInput(2);
	switches->Pos3 = new DigitalInput(3);
	switches->Red = new DigitalInput(1);

}

double Auton::Update() {

	if (!alist->centerX.empty()) {
		alist->cenx2 = alist->centerX[1];
		alist->cenx = alist->centerX[0];
		alist->wid = alist->width[0];
		alist->wid2 = alist->width[1];
		alist->hit = alist->height[0];
		alist->hit2 = alist->height[1];

		alist->rect1 = alist->cenx + (alist->wid / 2);
		alist->rect2 = alist->cenx2 + (alist->wid2 / 2);

		alist->dist = (alist->rect1 + alist->rect2) / 2;

		alist->dist = alist->dist - (320 / 2);

		alist->distcalc1 = 2010 / alist->hit;
		alist->distcalc2 = 2010 / alist->hit2;

		alist->distcalc1 = (alist->distcalc1 + alist->distcalc2) / 2;

		SmartDashboard::PutNumber("dist", alist->distcalc1);

	}
	if (alist->centerX.empty())
		alist->dist = 0;

	if (alist->dist > 300)
		alist->dist = 0;

	SmartDashboard::PutNumber("Dist From Center", alist->dist);

	return alist->dist;

}

void Auton::Drive() {

	alist->drivedist = (Update() / 800);

	if (alist->drivedist == 0) {
		alist->state = outOfView;
	} else {
		alist->state = inView;
	}

	if (alist->iroutput)
		alist->state = inRange;

	if (alist->done) {
		alist->state = done;
	}

	switch (alist->state) {

	case outOfView:

		alist->drivetrain->Drive(0, 0.4);

		break;

	case inRange:

		alist->time.Start();
		alist->time.Reset();
		alist->done = true;
		while (alist->time.Get() < 4) {

			alist->drivetrain->Drive(0, 0);

		}
		alist->time.Reset();
		while (alist->time.Get() < 4) {

			alist->drivetrain->Drive(0, 0.4);
		}
		if (done) {

			alist->drivetrain->Drive(0, 0);

		}
		break;

	case inView:

		alist->drivetrain->Drive(-alist->dist, 0.3);

		break;

	case done:

		alist->drivetrain->Drive(0, 0);
		break;
	}
}

int Auton::Routes(frc::SampleRobot *robot) {

	//Setting Auton Mode

	if (!switches->Gear->Get()) {
		switches->mode = gear;
	} else if (!switches->Shoot->Get()) {
		switches->mode = shoot;
	} else {
		switches->mode = gearandshoot;
	}

	std::cout << "GEAR: " << switches->Gear->Get() << std::endl;
	std::cout << "SHOOT: " << switches->Shoot->Get() << std::endl;

	//Setting Pos

	if (!switches->Pos1->Get()) {
		switches->poss = left;
	} else if (!switches->Pos3->Get()) {
		switches->poss = right;
	} else {
		switches->poss = center;
	}

	//Setting Team

	if (!switches->Red->Get()) {
		switches->Team = blue;
	} else {
		switches->Team = red;
	}

	/*
	 *
	 *
	 *
	 *
	 *
	 *
	 *
	 *
	 *
	 *
	 *
	 */


	std::cout << "TEAM: " << switches->Team << std::endl;
	std::cout << "POS: " << switches->poss  << std::endl;
	std::cout << "MODE: " << switches->mode << std::endl;

	if (switches->Team == red && switches->poss == left
			&& switches->mode == gear) {

		//Go Forward
		//Go For Gear Placement

		alist->time.Start();

		while (alist->time.Get() < 4 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}
		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

	}

	if (switches->Team == red && switches->poss == left
			&& switches->mode == shoot) {

		//Print Error for Red, Left, Shoot

	}

	if (switches->Team == red && switches->poss == left
			&& switches->mode == gearandshoot) {

		//Print Error for Red, Left, Gear and Shoot

	}

	if (switches->Team == red && switches->poss == center
			&& switches->mode == gear) {

		//Go Forward
		//Go For Gear

		alist->time.Start();

		while (alist->time.Get() < 1 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

	}

	if (switches->Team == red && switches->poss == center
			&& switches->mode == shoot) {

		//Go Forward
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

	}

	if (switches->Team == red && switches->poss == center
			&& switches->mode == gearandshoot) {

		//Do Red, Center, Gear
		//Backup
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

		alist->time.Reset();
		alist->time.Start();

		while (alist->time.Get() < 1 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(-1, 0);
		}

		alist->drivetrain->Drive(0, 0);

	}

	if (switches->Team == red && switches->poss == right
			&& switches->mode == gear) {

		//Go Forward
		//Go For Gear

		alist->time.Start();

		while (alist->time.Get() < 1 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

	}

	if (switches->Team == red && switches->poss == right
			&& switches->mode == shoot) {

		//Go Forward
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

	}

	if (switches->Team == red && switches->poss == right
			&& switches->mode == gearandshoot) {

		//Do Red, Right, Gear and Shoot
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

		alist->time.Reset();
		alist->time.Start();

		while (alist->time.Get() < 1 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(-1, 0);
		}

		alist->drivetrain->Drive(0, 0);

	}

	if (switches->Team == blue && switches->poss == left
			&& switches->mode == gear) {

		//Go Forward
		//Go For Gear Placement

		alist->time.Start();

		while (alist->time.Get() < 4 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

	}

	if (switches->Team == blue && switches->poss == left
			&& switches->mode == shoot) {

		//Go Foward
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

	}

	if (switches->Team == blue && switches->poss == left
			&& switches->mode == gearandshoot) {

		//Do Blue Left Gear
		//Backup
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

		alist->time.Reset();
		alist->time.Start();

		while (alist->time.Get() < 1 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(-1, 0);
		}

		alist->drivetrain->Drive(0, 0);

	}

	if (switches->Team == blue && switches->poss == center
			&& switches->mode == gear) {
		//Go Foward
		//Go For Gear

		alist->time.Start();
		alist->time.Reset();

		std::cout << alist->time.Get() << std::endl;
		while (alist->time.Get() < 4 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(-3, 0);
			std::cout << "asdasdasdasd" << std::endl;
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

	}

	if (switches->Team == blue && switches->poss == center
			&& switches->mode == shoot) {

		//Go Forward
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

	}

	if (switches->Team == blue && switches->poss == center
			&& switches->mode == gearandshoot) {

		//Do  Blue Center Gear
		//Backup
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(1, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

		alist->time.Reset();
		alist->time.Start();

		while (alist->time.Get() < 1 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(-1, 0);
		}

		alist->drivetrain->Drive(0, 0);

	}

	if (switches->Team == blue && switches->poss == right
			&& switches->mode == gear) {

		//Same as Red Right Gear

		//Go Forward
		//Go For Gear

		alist->time.Start();

		return 0;

		while (alist->time.Get() < 4 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(-1, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

	}

	if (switches->Team == blue && switches->poss == right
			&& switches->mode == shoot) {

		//Print Error for Blue, Right, Shoot

	}

	if (switches->Team == blue && switches->poss == right
			&& switches->mode == gearandshoot) {

		//Print Error for, Blue, Right, Gear And Shoot

	}

	if (switches->mode == shoot) {
		return 1;
	} else if (switches->mode == gearandshoot) {
		return 2;
	} else {
		return 3;
	}

}

