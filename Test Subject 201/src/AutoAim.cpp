/*
 * AutoAim.cpp
 *
 *  Created on: Feb 18, 2017
 *      Author: feds
 */

#include"DriveTrain.h"
#include"AutoAim.h"
#include"WPILib.h"

AutoAim::AutoAim(DriveTrain* drive){

	aalist = new struct autoaimlist;

	aalist->table = NetworkTable::GetTable("GRIP/myContoursReport");

	aalist->drivetrain = drive;

	aalist->centerY = aalist->table->GetNumberArray("centerY",llvm::ArrayRef<double>());

	aalist->Width = aalist->table->GetNumberArray("width", llvm::ArrayRef<double>());

}


double AutoAim::MotorCalc(){

	aalist->centerY = aalist->table->GetNumberArray("centerY",llvm::ArrayRef<double>());

	aalist->Width = aalist->table->GetNumberArray("width", llvm::ArrayRef<double>());


	if(!aalist->centerY.empty()){
				aalist->centy = aalist->centerY[0];

				aalist->centy = aalist->centy - (720/2);
			}else{
				aalist->centy = 0;
			}

	if(aalist->centy > 300){
		aalist->centy = 0;
	}


			return aalist->centy;
}

double AutoAim::DistCalc(){

	aalist->centerY = aalist->table->GetNumberArray("centerY",llvm::ArrayRef<double>());

	aalist->Width = aalist->table->GetNumberArray("width", llvm::ArrayRef<double>());

	if(!aalist->Width.empty()){
		aalist->hit = aalist->Width[0];
		aalist->hit2 = aalist->Width[1];
			}

			if(aalist->hit2 > aalist->hit){
				aalist->hit = aalist->hit2;
			}

			aalist->dist = 5610.5/aalist->hit; //was 1315

			return aalist->dist;

		}



bool AutoAim::Aim(){


aalist->driveDist = (MotorCalc()/1000); //was 800

if(aalist->driveDist == 0){
	aalist->aimstate = lost;
}else if(MotorCalc() < 10){
	aalist->aimstate = done;
}else{
	aalist->aimstate = found;
}

switch(aalist->aimstate){

case lost:

aalist->drivetrain->Drive(0, 0.25);

	return false;

break;

case found:

	aalist->drivetrain->Drive(0, aalist->driveDist);

	return false;

break;

case done:

	aalist->drivetrain->Drive(0, aalist->driveDist);

return true;

break;

default:

return false;

break;

}



}

double AutoAim::MotorSpeed(){


	return 0;
}



