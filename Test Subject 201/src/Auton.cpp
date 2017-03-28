/*
 * Auton.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: feds
 */
#include"WPILib.h"
#include"Auton.h"
#include <iostream>

Auton::Auton(DriveTrain* drive, AutoAim *aim, Shooter *shoot) {

	alist = new struct autonlist;
	switches = new struct switches;

	alist->table = NetworkTable::GetTable("GRIP/myContoursReport");
	alist->drivetrain = drive;
	alist->aim = aim;
	alist->shooter = shoot;

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

	alist->ir = new DigitalInput(0);

	alist->found = false;

}

double Auton::Update() {

	alist->centerX = alist->table->GetNumberArray("centerX",
		llvm::ArrayRef<double>());
	alist->width = alist->table->GetNumberArray("width",
		llvm::ArrayRef<double>());
	alist->height = alist->table->GetNumberArray("height",
		llvm::ArrayRef<double>());



	if (!alist->centerX.empty()) {

		alist->cenx2 = alist->centerX[1];
		alist->cenx = alist->centerX[0];
		alist->wid = alist->width[0];
		alist->wid2 = alist->width[1];
		alist->hit = alist->height[0];
		alist->hit2 = alist->height[1];

		/*

		alist->rect1 = alist->cenx + (alist->wid / 2);
		alist->rect2 = alist->cenx2 + (alist->wid2 / 2);

		alist->dist = (alist->rect1 + alist->rect2) / 2;

		alist->dist = alist->dist - 130;
*/

		if(alist->cenx < alist->cenx2){
			return alist->cenx - (320/2);
		}else{
			return alist->cenx2 - (320/2);
		}

		}

	SmartDashboard::PutNumber("Dist From Center", alist->dist);

	if(alist->centerX.empty()){
		alist->dist = 0;
	}

	if(alist->dist > 300){
		alist->dist = 0;
	}

	return alist->dist;

}

void Auton::Drive() {

	alist->drivedist = (Update()/800);


		alist->drivetrain->Drive(-0.3, alist->drivedist);


}

void Auton::ShootLow(){

	SmartDashboard::PutNumber("AMPS", alist->drivetrain->getAmps());

	if(alist->drivetrain->getAmps() > 5)
		alist->aimState = ag;
	else
		alist->aimState = found;

	if(alist->ongoal){
		alist->aimState = atGoal;
	}

	switch(alist->aimState){

	case found:

		alist->shooter->returnStirToHome();

		alist->drivetrain->Drive(0.4, 0);

	break;

	case ag:

		std::cout << "turning at goal" << std::endl;
		alist->time.Reset();
		alist->time.Start();
		while(alist->time.Get() < 1){
		alist->drivetrain->Drive(0.5, -0.3);
		alist->ongoal = true;
		}
		alist->drivetrain->Drive(0, 0);
		alist->time.Reset();

	break;

	case atGoal:

		std::cout << "at goal" << std::endl;

	std::cout << "out of at goal" << std::endl;

	alist->shooter->Stir();

	alist->drivetrain->Drive(0, 0);

	alist->time.Start();

	alist->shooter->SpinUp();
	alist->shooter->UpdateSpeed(1050);

	if(alist->time.Get() > 2){
		alist->shooter->Shoot();
		alist->shooterpos = true;
		alist->time.Reset();
	}

	break;

	default:

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


	//Setting Pos

	if (!switches->Pos1->Get()) {
		switches->poss = right;
	} else if (!switches->Pos3->Get()) {
		switches->poss = left;
	} else {
		switches->poss = center;
	}

	//Setting Team

	if (!switches->Red->Get()) {
		switches->Team = blue;
	} else {
		switches->Team = red;
	}

	if(switches->poss == right){
		switches->rotDir = counterclockwise;
	}else if(switches->poss == left){
		switches->rotDir = clockwise;
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


	if (switches->Team == red && switches->poss == left
			&& switches->mode == gear) {

		std::cout << "Red Left Gear" << std::endl;

				//Go Forward
				//Go For Gear

				alist->drivetrain->resEncPoss();

					//driving fwd
					while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 192651 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) > -198805){
					if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part1){
					alist->drivetrain->Drive(-0.4, 0);
					}else{
					break;
					}
					}
					alist->part1 = true;

					alist->drivetrain->resEncPoss();

					while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 31088 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) < 39919){
					if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part2){
					alist->drivetrain->Drive(0, 0.3);
					}else{
					break;
					}
					}
					alist->part2 = true;

					alist->drivetrain->resEncPoss();

					while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 67402 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) < -72183){
					if(robot->IsAutonomous() && robot->IsEnabled() && !alist->part3){
					alist->drivetrain->Drive(-0.4, 0);
					}else{
					break;
					}
					}
					alist->part3 = true;

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

		std::cout << "Red Center Gear" << std::endl;



				//Go Foward
				//Go For Gear

		alist->drivetrain->resEncPoss();

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 164591 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) > -161864){
			if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part1){
				alist->drivetrain->Drive(-0.4, 0);
			}else{
				break;
			}
		}
		alist->part1 = true;

	}

	if (switches->Team == red && switches->poss == center
			&& switches->mode == shoot) {

		//Go Forward
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(0.7, 0);
		}

		while(robot->IsEnabled() && robot->IsAutonomous()){
			ShootLow();
		}

	}

	if (switches->Team == red && switches->poss == center
			&& switches->mode == gearandshoot) {

		//Do Red, Center, Gear
		//Backup
		//Shoot

		alist->time.Start();
		alist->time.Reset();

		while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
			alist->drivetrain->Drive(-0.7, 0);
		}

		while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
			Drive();
		}

		alist->time.Reset();

		while(alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()){
			alist->drivetrain->Drive(-0.5, 0);
		}

		while(robot->IsEnabled() && robot->IsAutonomous()){
			ShootLow();
		}

	}

	if (switches->Team == red && switches->poss == right
			&& switches->mode == gear) {

		std::cout << "Red Right Gear" << std::endl;

				//Go Forward
				//Go For Gear
		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 192651 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) > -198805){
			if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part1){
				alist->drivetrain->Drive(-0.4, 0);
			}else{
				break;
			}
			}
			alist->part1 = true;

			alist->drivetrain->resEncPoss();

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) > -31088 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) > -39919){
			if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part2){
				alist->drivetrain->Drive(0, -0.3);
			}else{
				break;
			}
			}
			alist->part2 = true;

			alist->drivetrain->resEncPoss();

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 67402 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) < -72183){
			if(robot->IsAutonomous() && robot->IsEnabled() && !alist->part3){
				alist->drivetrain->Drive(-0.4, 0);
			}else{
				break;
			}
			}
			alist->part3 = true;

	}

	if (switches->Team == red && switches->poss == right
			&& switches->mode == shoot) {

		//Go Forward
		//Shoot

		alist->time.Start();

		while (alist->time.Get() < 3 && robot->IsEnabled() && robot->IsAutonomous()) {
					alist->drivetrain->Drive(0.7, 0);
		}

		while(robot->IsEnabled() && robot->IsAutonomous()){
			ShootLow();
		}

	}

	if (switches->Team == red && switches->poss == right
			&& switches->mode == gearandshoot) {

		//Do Red, Right, Gear and Shoot
		//Shoot

		alist->time.Start();
				alist->time.Reset();

				while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
					alist->drivetrain->Drive(-0.7, 0);
				}

				while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
					Drive();
				}

				alist->time.Reset();

				while(alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()){
					alist->drivetrain->Drive(-0.5, 0);
				}

				while(robot->IsEnabled() && robot->IsAutonomous()){
					ShootLow();
				}

	}

	if (switches->Team == blue && switches->poss == left
			&& switches->mode == gear) {

		std::cout << "Blue Left Gear" << std::endl;

				//Go Forward
				//Go For Gear

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 192651 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) > -198805){
			if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part1){
				alist->drivetrain->Drive(-0.4, 0);
			}else{
				break;
			}
			}
			alist->part1 = true;
			alist->drivetrain->resEncPoss();

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 31088 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) < 39919){
			if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part2){
				alist->drivetrain->Drive(0, 0.3);
			}else{
				break;
			}
			}
			alist->part2 = true;
			alist->drivetrain->resEncPoss();

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 67402 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) < -72183){
			if(robot->IsAutonomous() && robot->IsEnabled() && !alist->part3){
				alist->drivetrain->Drive(-0.4, 0);
			}else{
				break;
			}
			}
			alist->part3 = true;

	}

	if (switches->Team == blue && switches->poss == left
			&& switches->mode == shoot) {

		//Go Foward
		//Shoot

		alist->time.Start();

				while (alist->time.Get() < 3 && robot->IsEnabled() && robot->IsAutonomous()) {
							alist->drivetrain->Drive(0.7, 0);
				}

				while(robot->IsEnabled() && robot->IsAutonomous()){
					ShootLow();
				}
	}

	if (switches->Team == blue && switches->poss == left
			&& switches->mode == gearandshoot) {

		//Do Blue Left Gear
		//Backup
		//Shoot

		alist->time.Start();
						alist->time.Reset();

						while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
							alist->drivetrain->Drive(-0.7, 0);
						}

						while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
							Drive();
						}

						alist->time.Reset();

						while(alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()){
							alist->drivetrain->Drive(-0.5, 0);
						}

						while(robot->IsEnabled() && robot->IsAutonomous()){
							ShootLow();
						}


	}

	if (switches->Team == blue && switches->poss == center
			&& switches->mode == gear) {

		std::cout << "Blue Center Gear" << std::endl;

		alist->drivetrain->resEncPoss();

				while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 164591 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) > -161864){
					if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part1){
						alist->drivetrain->Drive(-0.4, 0);
					}else{
						break;
					}
				}
				alist->part1 = true;

	}

	if (switches->Team == blue && switches->poss == center
			&& switches->mode == shoot) {

		//Go Forward
		//Shoot

		alist->time.Start();

						while (alist->time.Get() < 3 && robot->IsEnabled() && robot->IsAutonomous()) {
									alist->drivetrain->Drive(0.7, 0);
						}

						while(robot->IsEnabled() && robot->IsAutonomous()){
							ShootLow();
						}
	}

	if (switches->Team == blue && switches->poss == center
			&& switches->mode == gearandshoot) {

		//Do  Blue Center Gear
		//Backup
		//Shoot

		alist->time.Start();
								alist->time.Reset();

								while (alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()) {
									alist->drivetrain->Drive(-0.7, 0);
								}

								while (!alist->done && robot->IsEnabled() && robot->IsAutonomous()) {
									Drive();
								}

								alist->time.Reset();

								while(alist->time.Get() < 2 && robot->IsEnabled() && robot->IsAutonomous()){
									alist->drivetrain->Drive(-0.5, 0);
								}

								while(robot->IsEnabled() && robot->IsAutonomous()){
									ShootLow();
								}

	}

	if (switches->Team == blue && switches->poss == right
			&& switches->mode == gear) {

		std::cout << "Blue Right Gear" << std::endl;

		//Same as Red Right Gear

		//Go Forward
		//Go For Gear

		alist->drivetrain->resEncPoss();

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 192651 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) > -198805){
			if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part1){
				alist->drivetrain->Drive(-0.4, 0);
			}else{
				break;
			}
			}
			alist->part1 = true;
			alist->drivetrain->resEncPoss();

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) > -31088 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) > -39919){
			if(robot->IsEnabled() && robot->IsAutonomous() && !alist->part2){
				alist->drivetrain->Drive(0, -0.3);
			}else{
				break;
			}
			}
			alist->part2 = true;
			alist->drivetrain->resEncPoss();

		while(alist->drivetrain->getEncPoss(DriveTrain::motorSide::leftSide) < 67402 or alist->drivetrain->getEncPoss(DriveTrain::motorSide::rightSide) < -72183){
			if(robot->IsAutonomous() && robot->IsEnabled() && !alist->part3){
				alist->drivetrain->Drive(-0.4, 0);
			}else{
				break;
			}
			}
			alist->part3 = true;

	}

	if (switches->Team == blue && switches->poss == right
			&& switches->mode == shoot) {

		//Print Error for Blue, Right, Shoot

	}

	if (switches->Team == blue && switches->poss == right
			&& switches->mode == gearandshoot) {

		std::cout << "blue right gear and shoot" << std::endl;

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

