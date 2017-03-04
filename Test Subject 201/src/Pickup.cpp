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

	plist->pickupmotor = new CANTalon(canid);

	plist->pickupmotor->SetControlMode(frc::CANSpeedController::ControlMode::kPercentVbus);

}

void Pickup::Toggle(float speed){

	if(plist->ison){
		plist->pickupmotor->Set(0);
		plist->ison = false;
	}else{
		plist->pickupmotor->Set(speed);
		plist->ison = true;
	}

}
