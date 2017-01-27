/*
 * Vision.h
 *
 *  Created on: Jan 27, 2017
 *      Author: Andy
 */

#ifndef SRC_VISION_H_
#define SRC_VISION_H_

#include "WPILib.h"
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <stdlib.h>
#include <stdio.h>

class Vision {
public:

	Vision();
	float Update();

private:

	struct vislist{

	std::shared_ptr<NetworkTable> table;
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
	};

	struct vislist *list;

};

#endif /* SRC_VISION_H_ */
