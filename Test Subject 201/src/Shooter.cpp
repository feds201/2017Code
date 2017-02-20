/*
 * Shooter.cpp
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */
#include"Shooter.h"
#include"WPILib.h"
#include"CANTalon.h"
#include <iostream>

Shooter::Shooter(uint8_t canid, uint8_t canid2){

	slist = new struct shooterlist;

	slist->shooter1 = new CANTalon(canid);
	slist->shooter2 = new CANTalon(canid2);

	slist->stirer = new VictorSP(4);

	slist->stirenc = new AnalogInput(0);
	slist->limit = new DigitalInput(9);

	slist->shooter1->SetControlMode(frc::CANSpeedController::ControlMode::kSpeed);
	slist->shooter2->SetControlMode(frc::CANSpeedController::ControlMode::kSpeed);

	slist->shooter1->SetPID(0.00012, .000025, 0.00001);
	slist->shooter2->SetPID(0.00012, .000025, 0.00001);

	slist->shooter1->ConfigEncoderCodesPerRev(1024);
	slist->shooter2->ConfigEncoderCodesPerRev(1024);

	//slist->shooter1->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
	//slist->shooter2->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);

	slist->shoot = new DoubleSolenoid(8, 4, 5);

	slist->speed = 5250;

}

void Shooter::SpinUp(){

	slist->shooter1->Set(-slist->speed);
	slist->shooter2->Set(slist->speed);
	slist->ison = true;
	slist->started = false;

	//Stir();
}

void Shooter::UpdateSpeed(float speed){

	slist->speed = speed;

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

	slist->ison = false;

	slist->shoot->Set(frc::DoubleSolenoid::Value::kForward);

	slist->started = false;

}

void Shooter::Stir(){

if(!slist->started){
	if(slist->limit->Get()){
		slist->motorin = 0.45;
		slist->started = true;
		slist->direction = false;

	}else{
		slist->motorin = -0.45;
	}

}else{
	slist->input = slist->stirenc->GetValue();

	if(slist->input > 2900 && !slist->lasttime){
		slist->counts++;
		slist->lasttime = true;

	}else if(slist->input < 2900){
		slist->lasttime = false;

	}
	if(!slist->direction){

		if(slist->counts >= 79){
			slist->counts = 0;
			slist->motorin = slist->motorin*-1;
			slist->direction = true;
	}
	}else if(slist->limit->Get()){
		slist->counts = 0;
		slist->motorin = slist->motorin*-1;
		slist->direction = false;

	}

}

slist->stirer->Set(slist->motorin);
}


void Shooter::returnStirToHome(){

	slist->started = false;
	slist->direction = false;
	slist->counts = 0;
	slist->stirer->Set(0);

}


float Shooter::getVel(){

	return slist->shooter1->GetEncVel();

}

float Shooter::getVel2(){

	return slist->shooter2->GetEncVel();

}

