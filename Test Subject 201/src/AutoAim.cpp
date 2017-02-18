/*
 * AutoAim.cpp
 *
 *  Created on: Feb 18, 2017
 *      Author: feds
 */

/*

#include"WPILib.h"

AutoAim::AutoAim(){

	aalist = new struct autoaimlist;

	aalist->table = NetworkTable::GetTable("GRIP/myContoursReport");

	aalist->drivetrain = new DriveTrain(3, 4, 7, 5, 8, 2, 3);

	aalist->centerX = aalist->table->GetNumberArray("centerX",
				llvm::ArrayRef<double>());

	aalist->height = aalist->table->GetNumberArray("height", llvm::ArrayRef<double>());

}


double AutoAim::MotorCalc(){

	if(!aalist->centerX->empty()){
				aalist->centx = aalist->centerX[0];
				aalist->centx2 = aalist->centerX[1];

				aalist->centx = (aalist->centx + aalist->centx2)/2;

				aalist->centx = aalist->centx - (320/2);
			}



			return aalist->centx;


	return 0;
}

double AutoAim::DistCalc(){

	if(!aalist->height->empty()){
		aalist->hit = aalist->height[0];
		aalist->hit2 = aalist->height[1];
			}

			if(aalist->hit2 > aalist->hit){
				aalist->hit = aalist->hit2;
			}

			aalist->dist = 1414/aalist->hit; //was 1315

			return aalist->dist;

		}



void AutoAim::Aim(){






}

double AutoAim::MotorSpeed(){


	return 0;
}

*/

