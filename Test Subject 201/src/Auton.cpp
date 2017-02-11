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

	alist->table = NetworkTable::GetTable("GRIP/myContoursReport");
	alist->drivetrain = new DriveTrain(3, 4, 7, 5, 8, 2, 3);

	alist->centerX = alist->table->GetNumberArray("centerX",
			llvm::ArrayRef<double>());
	alist->width = alist->table->GetNumberArray("width",
			llvm::ArrayRef<double>());
	alist->height = alist->table->GetNumberArray("height",
			llvm::ArrayRef<double>());

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

		alist->drivetrain->Drive(alist->dist, 0.3);

		break;

	case done:

		alist->drivetrain->Drive(0, 0);

		break;
	}
}

