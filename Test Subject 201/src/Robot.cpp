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
	std::shared_ptr<NetworkTable> table;
	DigitalInput ir;
	CANTalon light;
	Edge change;
	Timer time;
	AnalogInput ultrasonic;
	DriveTrain drivetrain;

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
		joy(0), table(NetworkTable::GetTable("GRIP/myContoursReport")), ir(0),
		light(5), change(joy.GetRawButton(1)), ultrasonic(0), drivetrain(3, 4, 1, 2, 5, 1, 2, 3, 4)
	{
		light.SetControlMode(frc::CANSpeedController::ControlMode::kPercentVbus);


	}

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

	void RobotInit() {
		std::thread visionThread(startcam);
		visionThread.detach();
	}


	void Autonomous() {

		time.Reset();

		float dist;
		bool iroutput;
		bool stop = false;

	while(IsAutonomous() && IsEnabled()){

		iroutput = ir.Get();

		dist = (Vision()/3000);

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


			frc::Wait(0.005);
		}



float deadzone(float raw)
{
	if(fabs(raw)<.15){
		return 0.0f;
	}else if(raw>0){
		return (raw-.15)/(1-.15);
	}else{
		return (+.15)/(1-.15);
	}
}

	void OperatorControl() override {
		bool lighton = false;
		while (IsOperatorControl() && IsEnabled()) {

			Vision();

			change.update(joy.GetRawButton(1));



			if(change.isPressed()){

				if(!lighton){
				light.Set(1);
				lighton = true;
				}else{
				light.Set(0);
				lighton = false;
				}
			}



			drivetrain.Drive(deadzone(joy.GetRawAxis(1)), deadzone(joy.GetRawAxis(4)));

			SmartDashboard::PutBoolean("IR", ir.Get());
			SmartDashboard::PutNumber("Ultrasonic", ultrasonic.GetValue()/0.37203);

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
