/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: Andy
 */

#include <DriveTrain.h>
#include "WPILib.h"
#include "CANTalon.h"

DriveTrain::DriveTrain(uint8_t Lcanid, uint8_t Lcanid2, uint8_t Rcanid, uint8_t Rcanid2, int PCMCanid, int shifter1fwd,int shifter1rev) {

	mlist = new struct motorlist;
	slist = new struct shifterlist;

	mlist->Lmotorcanid = Lcanid;
	mlist->Lmotor2canid = Lcanid2;
	mlist->Rmotorcanid = Rcanid;
	mlist->Rmotor2canid = Rcanid2;

	slist->PCMid = PCMCanid;
	slist->shifterfwd = shifter1fwd;
	slist->shiferrev = shifter1rev;

	mlist->Lmotor1 = new CANTalon(mlist->Lmotorcanid);
	mlist->Lmotor2 = new CANTalon(mlist->Lmotor2canid);
	mlist->Rmotor1 = new CANTalon(mlist->Rmotorcanid);
	mlist->Rmotor2 = new CANTalon(mlist->Rmotor2canid);

	slist->shifter1 = new DoubleSolenoid(slist->PCMid, slist->shifterfwd, slist->shiferrev);

	mlist->Lmotor1->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);
	mlist->Rmotor1->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);




}

void DriveTrain::Drive(float trn, float fwd){

	mlist->Lmotors = fwd - trn/2;
	mlist->Rmotors = fwd + trn/2;

	mlist->Lmotor1->Set(mlist->Lmotors);
	mlist->Lmotor2->Set(mlist->Lmotors);

	mlist->Rmotor1->Set(mlist->Rmotors);
	mlist->Rmotor2->Set(mlist->Rmotors);

}


void DriveTrain::Shift(){

	if(slist->shifter1->Get() == frc::DoubleSolenoid::Value::kForward){
		slist->shifter1->Set(frc::DoubleSolenoid::Value::kReverse);
	}else{
		slist->shifter1->Set(frc::DoubleSolenoid::Value::kForward);
	}

}

float DriveTrain::getMotorVel(motorSide encside){

	switch(encside){

	case leftSide :

		return(mlist->Lmotor1->GetEncVel());

		break;
	case rightSide :

		return(mlist->Rmotor1->GetEncVel());

		break;
	default :

		return 0;
		break;

	}

}
