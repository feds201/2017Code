/*
 * Shooter.h
 *
 *  Created on: Feb 5, 2017
 *      Author: Andy
 */

#ifndef SRC_SHOOTER_H_
#define SRC_SHOOTER_H_

#include"WPILib.h"
#include"CANTalon.h"
#include"Timer.h"

class Shooter{

public:



	Shooter(uint8_t canid, uint8_t canid2);
	void SpinUp();
	enum shooterSpeed{lowGoal, low, med, high};
	void UpdateSpeed(float speed);
	void Shoot();
	void Stop();
	void Stir();
	void returnStirToHome();
	float getVel();
	float getVel2();

private:

	struct shooterlist{

		CANTalon *shooter1;
		CANTalon *shooter2;

		VictorSP *stirer;

		int input = 0;
		double counts = 0;
		bool lasttime = false;
		double motorin = 0.45; //0.45
		bool direction = false;
		bool stopstir = false;

		AnalogInput *stirenc;
		DigitalInput *limit;

		bool ison = false;
		bool started = false;

		DoubleSolenoid *shoot;

		shooterSpeed spd;

		float speed;

		Timer time;

	};


	struct shooterlist *slist;


};




#endif /* SRC_SHOOTER_H_ */
