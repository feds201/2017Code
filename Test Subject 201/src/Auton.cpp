/*
 * Auton.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: Andy
 */

#include <Auton.h>
#include "WPILib.h"
#include "DriveTrain.h"
#include "Vision.h"
#include <timer.h>

Auton::Auton(uint8_t Lcanid, uint8_t Lcanid2, uint8_t Rcanid, uint8_t Rcanid2, int PCMCanid, int shifter1fwd, int shifter1rev) {

	alist = new struct autonList;

	alist->drivetrain = new DriveTrain(Lcanid, Lcanid2, Rcanid, Rcanid2, PCMCanid, shifter1fwd, shifter1rev);

	alist->stop = false;

	alist->mode1 = new DigitalInput(1);
	alist->mode3 = new DigitalInput(2);
	alist->pos1 = new DigitalInput(3);
	alist->pos3 = new DigitalInput(4);
	alist->redblu = new DigitalInput(5);

	//Finding Mode
	if(alist->mode1->Get()){
		alist->Mode = Gear;
	}else if(alist->mode3->Get()){
		alist->Mode = GearAndShoot;
	}else{
		alist->Mode = Shoot;
	}


	//Finding Pos
	if(alist->pos1->Get()){
		alist->Pos = Left;
	}else if(alist->pos3->Get()){
		alist->Pos = Right;
	}else{
		alist->Pos = Center;
	}


	//Red or Blue Team
	if(alist->redblu->Get()){
		alist->Color = Red;
	}else{
		alist->Color = Blue;
	}

}

bool Auton::Run(){

	if(alist->Pos == Left){

		if(alist->Mode == Gear){

			if(alist->Color == Red){
				//Red, Left, Gear

				alist->stop = true; //Change This When Code Is Done
				//END
			}else if(alist->Color == Blue){
				//Blue, Left, Gear

				alist->stop = true; //Change This When Code Is Done
				//END
			}

		}else if(alist->Mode == Shoot){

			if(alist->Color == Red){
				//NOT USING THIS, PRINT ERROR

				alist->stop = true; //Change This When Code Is Done
				//END
			}else if(alist->Color == Blue){
				//Blue, Left, Shoot

				alist->stop = true; //Change This When Code Is Done
				//END
			}

		}else if(alist->Mode == GearAndShoot){

			if(alist->Color == Red){
				//NOT USING THIS, PRINT ERROR

				alist->stop = true;
			}else if(alist->Color == Blue){
				//Blue, Left, Gear and Shoot

				alist->stop = true; //Change This When Code Is Done
				//END
			}

		}


	}else if(alist->Pos == Right){

		if(alist->Mode == Gear){

			if(alist->Color == Red){
				//Red, Right, Gear

				alist->stop = true; //Change This When Code Is Done
				//END
			}else if(alist->Color == Blue){
				//Blue, Right, Gear

				alist->stop = true; //Change This When Code Is Done
				//END
			}

		}else if(alist->Mode == Shoot){

			if(alist->Color == Red){
				//Red, Right, Shoot

				alist->stop = true; //Change This When Code Is Done
				//END
			}else if(alist->Color == Blue){
				//NOT USING THIS, PRINT ERROR

				alist->stop = true; //Change This When Code Is Done
				//END
			}

		}else if(alist->Mode == GearAndShoot){

			if(alist->Color == Red){
				//Red, Right, Gear and Shoot

				alist->stop = true; //Change This When Code Is Done
				//END
			}else if(alist->Color == Blue){
				//NOT USING THIS, PRINT ERROR

				alist->stop = true; //Change This When Code Is Done
				//END
			}

		}

	}else if(alist->Pos == Center){

		if(alist->Mode == Gear){

			//COPY OF CODE FORM Robot.cpp
			//Drives Robot Toward The Gear Hook

			//Red and Blue, Center, Gear

			alist->dist = alist->vision->Update();
			alist->iroutput = alist->IR->Get();

			if(alist->iroutput) /* What Runs When The Bot Is Close To Target */{
					alist->time.Start();
					alist->time.Reset();
					while(alist->time.Get() < 4){
						alist->drivetrain->Drive(0, 0);
					}
					alist->time.Reset();
					while(alist->time.Get() < 2){
						SmartDashboard::PutNumber("Timer", alist->time.Get());
						alist->drivetrain->Drive(0, 0.2);
					}
					alist->drivetrain->Drive(0,0);
					alist->stop = true; //Stops Auton

			}else /* What Runs When Bot Is Not Close To Target */ {

				alist->drivetrain->Drive(-0.2, alist->dist);

				alist->stop = false;


				//END

				}


			}


		}else if(alist->Mode == Shoot){

			if(alist->Color == Red){
				//Red, Center, Shoot

				alist->stop = true; //Change This When Code Is Done
				//END
			}else if(alist->Color == Blue){
				//Blue, Center, Shoot

				alist->stop = true; //Change This When Code Is Done
				//END
			}

		}else if(alist->Mode == GearAndShoot){

			if(alist->Color == Red){
				//Red, Center, Gear and Shoot

				alist->stop = true; //Change This When Code Is Done
				//END
			}else if(alist->Color == Blue){
				//Blue, Center, Gear and Shoot

				alist->stop = true; //Change This When Code Is Done
				//END
			}else{

				//Default Stops Auton

				alist->stop = true;
			}
		}

	return alist->stop;

	}






