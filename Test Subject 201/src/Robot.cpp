#include <thread>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <timer.h>
#include"WPILib.h"
#include"CANTalon.h"
#include"EdgeDetection.h"
#include"DriveTrain.h"
#include"GearFlipper.h"
#include"Lifter.h"
#include"Pickup.h"
#include"Shooter.h"


/*

 Pickup Class Crashed Program

 Can't Turn Off Lifter

 Can't Fire Shooter

 */


class Robot: public frc::SampleRobot {
	Joystick joy;
	Joystick joy2;
	DriveTrain drivetrain;
	Edge shift;
	Timer time;
	std::shared_ptr<NetworkTable> table;
	GearFlipper flipper;
	Lifter lifter;
	Pickup pickup;
	Shooter shooter;
	Edge flip;
	Edge lift;
	Edge shoot;
	Edge pick;
	Edge speedup;
	Edge speeddown;
	Edge SpinUp;

	static void startcam(){

		cs::AxisCamera camera = CameraServer::GetInstance()->AddAxisCamera("10.2.1.39");
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

	}

	enum autonstate{outOfView, inRangeL, inRangeR, inRange, inView};

public:
	Robot() :
		joy(0), joy2(1), drivetrain(3, 4, 7, 5, 8, 2, 3), shift(joy.GetRawButton(1)), table(NetworkTable::GetTable("GRIP/myContoursReport")),
		flipper(), lifter(0), pickup(6), shooter(1, 2), flip(joy2.GetRawButton(5)), lift(joy.GetRawButton(2)), shoot(joy2.GetRawButton(6)),
		pick(joy2.GetRawButton(3)), speedup(joy2.GetRawButton(8)), speeddown(joy2.GetRawButton(7)), SpinUp(joy2.GetRawButton(2))
	{

	}

/*
double Vision(){

				double dist;
				double cenx;
				double cenx2;
				double wid;
				double wid2;
				double rect1;
				double rect2;
				double distcalc1;
				double distcalc2;
				double hit;
				double hit2;


					std::vector<double> centerX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
					std::vector<double> width = table->GetNumberArray("width", llvm::ArrayRef<double>());
					std::vector<double> height = table->GetNumberArray("height", llvm::ArrayRef<double>());



					if(!centerX.empty()){
						cenx2 = centerX[1];
						cenx = centerX[0];
						wid = width[0];
						wid2 = width[1];
						hit = height[0];
						hit2 = height[1];

						rect1 = cenx + (wid/2);
						rect2 = cenx2 + (wid2/2);

						dist = (rect1 + rect2)/2;

						dist = dist - (320/2);

						distcalc1 = 2010/hit;
						distcalc2 = 2010/hit2;

						distcalc1 = (distcalc1 + distcalc2)/2;

						SmartDashboard::PutNumber("dist", distcalc1);



					}
					if(centerX.empty())
						dist = 0;

					if(dist > 300)
						dist = 0;


					SmartDashboard::PutNumber("Dist From Center", dist);



				return dist;

	}

*/

	void RobotInit() {
		std::thread visionThread(startcam);
		visionThread.detach();
	}


	void Autonomous() {

/*
		time.Reset();

		int state;
		float dist;
		bool iroutput;

		while(IsAutonomous() && IsEnabled()){

		dist = (Vision()/800);

				if(dist == 0){
					state = outOfView;
				}else{
					state = inView;
				}

				if(iroutput)
					state = inRange;

			switch(state){

			case outOfView:

				drivetrain.Drive(0.4, 0);

				break;

			case inRange:

				time.Start();
				time.Reset();

				while(time.Get() < 4){

					drivetrain.Drive(0, 0);

				}
				time.Reset();
				while(time.Get() < 4){

					drivetrain.Drive(0, 0.4);

				}
				while(IsAutonomous() && IsEnabled()){

					drivetrain.Drive(0, 0);

				}
					break;

			case inView:

				drivetrain.Drive(dist, 0.3);

				break;


			}
			}
					frc::Wait(0.005);
*/

		}



float deadzone(float f)
{
	if(fabs(f)<.15)
		return 0.0f;
	else
	{
		if(f>0)
			return (f-.15)/(1-.15);
		else
			return (f+.15)/(1-.15);
		}
	}

	void OperatorControl() override {

		float speed = 0.7;
		bool ison = false;

		while (IsOperatorControl() && IsEnabled()) {

			shift.update(joy.GetRawButton(1));
			flip.update(joy2.GetRawButton(5));
			lift.update(joy.GetRawButton(2));
			shoot.update(joy2.GetRawButton(6));
			pick.update(joy2.GetRawButton(3));
			speedup.update(joy2.GetRawButton(8));
			speeddown.update(joy2.GetRawButton(7));
			SpinUp.update(joy2.GetRawButton(2));

			if(shift.isPressed())
				drivetrain.Shift();



			if(flip.isPressed())
				flipper.Toggle();



			if(lift.isPressed())
				lifter.Toggle();




			if(shoot.isPressed())
				shooter.Shoot();

			if(SpinUp.isPressed()){
				if(ison == false){
				shooter.SpinUp();
				ison = true;
				}else{
					shooter.Stop();
					ison = false;
				}
			}

			if(speedup.isPressed()){
				speed+=0.05;
				shooter.UpdateSpeed(speed);
			}

			if(speeddown.isPressed()){
				speed-=0.05;
				shooter.UpdateSpeed(speed);
			}

			if(pick.isPressed())
				pickup.Toggle();

			if(ison == true){
				SmartDashboard::PutNumber("Shooter", shooter.getVel());
				SmartDashboard::PutNumber("Shooter 2", shooter.getVel2());
			}

			drivetrain.Drive(deadzone(joy.GetRawAxis(1)), deadzone(joy.GetRawAxis(4)));

			SmartDashboard::PutNumber("LMotors", drivetrain.getMotorVel(DriveTrain::leftSide));
			SmartDashboard::PutNumber("RMotors", drivetrain.getMotorVel(DriveTrain::rightSide));
			SmartDashboard::PutNumber("Speed", speed*100);

			frc::Wait(0.005);
		}
	}

	/*
	 * Runs during test mode
	 */
	void Test() override {


	}
};

START_ROBOT_CLASS(Robot)
