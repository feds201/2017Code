/*
 * Pickup.cpp
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */
#include"WPILib.h"
#include"CANTalon.h"
#include"Pickup.h"

Pickup::Pickup(uint8_t canid){

	plist = new struct pickuplist;

	plist->pickupmotor->SetControlMode(frc::CANSpeedController::ControlMode::kPercentVbus);

}

void Pickup::Toggle(){

	if(plist->ison){
		plist->pickupmotor->Set(0);
	}else{
		plist->pickupmotor->Set(1);
	}

}
