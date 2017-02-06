/*
 * Lifter.cpp
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */
#include"WPILib.h"
#include"CANTalon.h"
#include"Lifter.h"

Lifter::Lifter(int PWM){

	llist = new struct lifterlist;

	llist->liftermotor = new Spark(PWM);

	llist->ison = false;

}

void Lifter::Toggle(){

	if(llist->ison){
		llist->liftermotor->Set(0);
	}else{
		llist->liftermotor->Set(1);
	}

}
