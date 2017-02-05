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

class Robot: public frc::SampleRobot {
	Joystick joy;
	DriveTrain drivetrain;
	Edge shift;

	static void startcam(){

		cs::AxisCamera camera = CameraServer::GetInstance()->AddAxisCamera("10.2.1.39");
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);
		cv::Mat mat;

		while (true) {

					if (cvSink.GrabFrame(mat) == 0) {

						outputStream.NotifyError(cvSink.GetError());

						continue;
					}

					outputStream.PutFrame(mat);
		}
	}

public:
	Robot() :
		joy(0), drivetrain(3, 4, 7, 5, 8, 2, 3), shift(joy.GetRawButton(1))
	{

	}

/*

DELETE THIS ONCE WE KNOW IT WORKS WITHOUT IT

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

		Original Auton Code

		time.Reset();

		float dist;
		bool iroutput;
		bool stop = false;

	while(IsAutonomous() && IsEnabled()){

		iroutput = ir.Get();

		dist = (vision.Update()/3000);

	if(iroutput){
		time.Start();
		time.Reset();
		while(time.Get() < 4){
			drivetrain.Drive(0, 0);
			light.Set(0);
		}
		time.Reset();
		while(time.Get() < 2){
		SmartDashboard::PutNumber("Timer", time.Get());
		drivetrain.Drive(0, 0.2);
		}
		drivetrain.Drive(0,0);
		stop = true;

	}else if(stop == false){

	drivetrain.Drive(-0.2, dist);

	light.Set(1);

	}

			}

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

		while (IsOperatorControl() && IsEnabled()) {

			//change.update(joy.GetRawButton(1));

			shift.update(joy.GetRawButton(1));

			if(shift.isPressed())
				drivetrain.Shift();


			drivetrain.Drive(deadzone(joy.GetRawAxis(4)), deadzone(joy.GetRawAxis(1)));

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
