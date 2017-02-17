/*
 * Auton.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: feds
 */
#include"WPILib.h"
#include"Auton.h"

Auton::Auton() {

	alist = new struct autonlist;
	switches = new struct switches;

	alist->table = NetworkTable::GetTable("GRIP/myContoursReport");
	alist->drivetrain = new DriveTrain(3, 4, 7, 5, 8, 2, 3);

	alist->centerX = alist->table->GetNumberArray("centerX",
			llvm::ArrayRef<double>());
	alist->width = alist->table->GetNumberArray("width",
			llvm::ArrayRef<double>());
	alist->height = alist->table->GetNumberArray("height",
			llvm::ArrayRef<double>());


	switches->Gear = new DigitalInput(1);
	switches->GearAndShoot = new DigitalInput(2);
	switches->Pos1 = new DigitalInput(3);
	switches->Pos3 = new DigitalInput(4);
	switches->Red = new DigitalInput(5);


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

		alist->drivetrain->Drive(0.4, 0);

		break;

	case inRange:

		alist->time->Start();
		alist->time->Reset();
		alist->done = true;
		while (alist->time->Get() < 4) {

			alist->drivetrain->Drive(0, 0);

		}
		alist->time->Reset();
		while (alist->time->Get() < 4) {

			alist->drivetrain->Drive(0, 0.4);

		}
		if (done) {

			alist->drivetrain->Drive(0, 0);

		}
		break;

	case inView:

		alist->drivetrain->Drive(alist->dist, 0.3);

		break;

	case done:

		alist->drivetrain->Drive(0, 0);

		break;
	}
}

int Auton::Routes(){

	//Setting Auton Mode

	if(!switches->Gear->Get()){
		switches->mode = gear;
	}else if(!switches->GearAndShoot->Get()){
		switches->mode = gearandshoot;
	}else{
		switches->mode = shoot;
	}

	//Setting Pos

	if(!switches->Pos1->Get()){
		switches->poss = left;
	}else if(!switches->Pos3->Get()){
		switches->poss = right;
	}else{
		switches->poss = center;
	}

	//Setting Team

	if(!switches->Red->Get()){
		switches->Team = red;
	}else{
		switches->Team = blue;
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


	if(switches->Team == red && switches->poss == left && switches->mode == gear){

		//Go Forward
		//Go For Gear Placement

		alist->time->Start();

		while(alist->time->Get() < 4){
			alist->drivetrain->Drive(1, 0);
		}

	}

	if(switches->Team == red && switches->poss == left && switches->mode == shoot){

			//Print Error for Red, Left, Shoot

	}

	if(switches->Team == red && switches->poss == left && switches->mode == gearandshoot){

			//Print Error for Red, Left, Gear and Shoot

	}

	if(switches->Team == red && switches->poss == center && switches->mode == gear){

		//Go Forward
		//Go For Gear

		alist->time->Start();

		while(alist->time->Get() < 1){
			alist->drivetrain->Drive(1, 0);
		}


	}

	if(switches->Team == red && switches->poss == center && switches->mode == shoot){

		//Go Forward
		//Shoot

		alist->time->Start();

		while(alist->time->Get() < 2){
			alist->drivetrain->Drive(1, 0);
		}


	}

	if(switches->Team == red && switches->poss == center && switches->mode == gearandshoot){

		//Do Red, Center, Gear
		//Backup
		//Shoot

		alist->time->Start();

		while(alist->time->Get() < 2){
			alist->drivetrain->Drive(1, 0);
		}

		while(!alist->done){
			Drive();
		}

		alist->time->Reset();
		alist->time->Start();

		while(alist->time->Get() < 1){
			alist->drivetrain->Drive(-1, 0);
		}

		alist->drivetrain->Drive(0, 0);

	}


	if(switches->Team == red && switches->poss == right && switches->mode == gear){

		//Turn Right
		//Go Forward
		//Turn Left 90 degrees
		//Go For Gear

	}

	if(switches->Team == red && switches->poss == right && switches->mode == shoot){

		//Turn Right
		//Go Forward
		//Shoot

	}

	if(switches->Team == red && switches->poss == right && switches->mode == gearandshoot){

		//Do Red, Right, Gear and Shoot
		//Shoot

	}

	if(switches->Team == blue && switches->poss == left && switches->mode == gear){

		//Same As Red Left Gear

		//Turn Left Slightly
		//Go Forward
		//Turn 90 degrees right
		//Go For Gear Placement


	}

	if(switches->Team == blue && switches->poss == left && switches->mode == shoot){

		//Turn Left
		//Go Foward
		//Shoot

	}

	if(switches->Team == blue && switches->poss == left && switches->mode == gearandshoot){

		//Do Blue Left Gear
		//Backup
		//Turn Left 90 degrees
		//Backup
		//Shoot

	}

	if(switches->Team == blue && switches->poss == center && switches->mode == gear){

		//Go Foward
		//Go For Gear

		Drive();

	}

	if(switches->Team == blue && switches->poss == center && switches->mode == shoot){

		//Turn Left
		//Go Forward
		//Shoot

	}

	if(switches->Team == blue && switches->poss == center && switches->mode == gearandshoot){

		//Do  Blue Center Gear
		//Backup
		//Turn Left
		//Go Foward
		//Shoot


	}


	if(switches->Team == blue && switches->poss == right && switches->mode == gear){

		//Same as Red Right Gear

		//Turn Right
		//Go Forward
		//Turn Left 90 degrees
		//Go For Gear


	}

	if(switches->Team == blue && switches->poss == right && switches->mode == shoot){

		//Print Error for Blue, Right, Shoot

	}

	if(switches->Team == blue && switches->poss == right && switches->mode == gearandshoot){

		//Print Error for, Blue, Right, Gear And Shoot

	}

	if(switches->mode == shoot){
		return 1;
	}else if(switches->mode == gearandshoot){
		return 2;
	}else{
		return 3;
	}

}





