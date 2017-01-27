/*
 * Vision.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: Andy
 */

#include <Vision.h>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <stdlib.h>
#include <stdio.h>


Vision::Vision() {

	list = new struct vislist;


}

float Vision::Update(){

	std::vector<double> centerX = list->table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	std::vector<double> width = list->table->GetNumberArray("width", llvm::ArrayRef<double>());
	std::vector<double> height = list->table->GetNumberArray("height", llvm::ArrayRef<double>());

	if(!centerX.empty()){
		list->cenx2 = centerX[1];
		list->cenx = centerX[0];
		list->wid = width[0];
		list->wid2 = width[1];
		list->hit = height[0];
		list->hit2 = height[1];

		list->rect1 = list->cenx + (list->wid/2);
		list->rect2 = list->cenx2 + (list->wid2/2);

		list->dist = (list->rect1 + list->rect2)/2;

		list->dist = list->dist - (320/2);

		list->distcalc1 = 2010/list->hit;
		list->distcalc2 = 2010/list->hit2;

		list->distcalc1 = (list->distcalc1 + list->distcalc2)/2;

		SmartDashboard::PutNumber("dist", list->distcalc1);



		}
		if(centerX.empty())
			list->dist = 0;

			if(list->dist > 300)
				list->dist = 0;


		SmartDashboard::PutNumber("Dist From Center", list->dist);



		return list->dist;




}
