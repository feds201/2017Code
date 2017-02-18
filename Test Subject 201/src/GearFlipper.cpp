/*
 * GearFlipper.cpp
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */
#include"WPILib.h"
#include"GearFlipper.h"

GearFlipper::GearFlipper(){

	gflist = new struct gearflipperlist;

	gflist->flipper = new DoubleSolenoid(8, 2, 3);

}

void GearFlipper::Toggle(){

	if(gflist->flipper->Get() == frc::DoubleSolenoid::Value::kReverse){
		gflist->flipper->Set(frc::DoubleSolenoid::Value::kForward);
	}else{
		gflist->flipper->Set(frc::DoubleSolenoid::Value::kReverse);
	}

}
