/*
 * Shooter.cpp
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */
#include"Shooter.h"
#include"WPILib.h"
#include"CANTalon.h"

Shooter::Shooter(uint8_t canid, uint8_t canid2){

	slist = new struct shooterlist;

	slist->shooter1 = new CANTalon(canid);
	slist->shooter2 = new CANTalon(canid2);

	slist->shooter1->SetControlMode(frc::CANSpeedController::ControlMode::kPercentVbus);
	slist->shooter2->SetControlMode(frc::CANSpeedController::ControlMode::kPercentVbus);

	slist->shooter1->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
	slist->shooter2->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);

	slist->shoot = new DoubleSolenoid(8, 0, 1);

	slist->speed = 0.7;

}

void Shooter::SpinUp(){

	slist->shooter1->Set(-slist->speed);
	slist->shooter2->Set(slist->speed);
}

void Shooter::UpdateSpeed(float speed){

	slist->speed = speed;

	SpinUp();

}

void Shooter::Shoot(){

	if(slist->shoot->Get() == frc::DoubleSolenoid::Value::kReverse){
		slist->shoot->Set(frc::DoubleSolenoid::Value::kForward);
	}else{
		slist->shoot->Set(frc::DoubleSolenoid::Value::kReverse);
	}

}

void Shooter::Stop(){

	slist->shooter1->Set(0);
	slist->shooter2->Set(0);

	slist->shoot->Set(frc::DoubleSolenoid::Value::kReverse);

}
float Shooter::getVel(){

	return slist->shooter1->GetEncVel();

}

float Shooter::getVel2(){

	return slist->shooter2->GetEncVel();

}

