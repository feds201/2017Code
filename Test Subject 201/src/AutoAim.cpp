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

	aalist->centerX = aalist->table->GetNumberArray("centerX",llvm::ArrayRef<double>());

	aalist->height = aalist->table->GetNumberArray("height", llvm::ArrayRef<double>());

}


double AutoAim::MotorCalc(){

	aalist->centerX = aalist->table->GetNumberArray("centerY",llvm::ArrayRef<double>());

	aalist->height = aalist->table->GetNumberArray("width", llvm::ArrayRef<double>());


	if(!aalist->centerX.empty()){
				aalist->centx = aalist->centerX[0];
				aalist->centx2 = aalist->centerX[1];

				aalist->centx = (aalist->centx + aalist->centx2)/2;

				aalist->centx = aalist->centx - (320/2);
			}else{
				aalist->centx = 0;
			}

	if(aalist->centx > 300){
		aalist->centx = 0;
	}


			return aalist->centx;
}

double AutoAim::DistCalc(){

	aalist->centerX = aalist->table->GetNumberArray("centerX",llvm::ArrayRef<double>());

	aalist->height = aalist->table->GetNumberArray("width", llvm::ArrayRef<double>());

	if(!aalist->height.empty()){
		aalist->hit = aalist->height[0];
		aalist->hit2 = aalist->height[1];
			}

			if(aalist->hit2 > aalist->hit){
				aalist->hit = aalist->hit2;
			}

			aalist->dist = 5781/aalist->hit; //was 1315

			return aalist->dist;

		}



bool AutoAim::Aim(){


aalist->driveDist = (MotorCalc()/800);

if(aalist->driveDist == 0){
	aalist->aimstate = lost;
}else if(MotorCalc() < 5){
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



