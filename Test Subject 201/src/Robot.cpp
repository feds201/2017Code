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
#include "EdgeDetection.h"
class Robot: public frc::SampleRobot {
	Joystick joy;
	CANTalon Lmotor1;
	CANTalon Lmotor2;
	CANTalon Rmotor1;
	CANTalon Rmotor2;
	frc::CANSpeedController::ControlMode controlmode;
	frc::CANSpeedController::ControlMode controlmode2;
	std::shared_ptr<NetworkTable> table;
	DigitalInput ir;
	CANTalon light;
	EdgeDetection change;
	EdgeDetection change2;
	Timer time;
	AnalogInput ultrasonic;

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
		joy(0), Lmotor1(3), Lmotor2(4), Rmotor1(1), Rmotor2(2), controlmode(frc::CANSpeedController::ControlMode::kPercentVbus),
		controlmode2(frc::CANSpeedController::ControlMode::kPercentVbus), table(NetworkTable::GetTable("GRIP/myContoursReport")), ir(0),
		light(5), change(joy.GetRawButton(1)), change2(joy.GetRawButton(2)), ultrasonic(0)
	{
		Lmotor1.SetControlMode(controlmode);
				Lmotor2.SetControlMode(controlmode2);

				Rmotor1.SetControlMode(controlmode);
				Rmotor2.SetControlMode(controlmode2);
				light.SetControlMode(frc::CANSpeedController::ControlMode::kPercentVbus);

				Lmotor1.SetSafetyEnabled(true);
				Lmotor2.SetSafetyEnabled(true);

				Rmotor1.SetSafetyEnabled(true);
				Rmotor2.SetSafetyEnabled(true);

				Lmotor2.SetClosedLoopOutputDirection(true);
				Lmotor1.SetClosedLoopOutputDirection(true);



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

		float speedL;
		float speedR;
		float dist;
		bool iroutput;
		bool stop = false;

	while(IsAutonomous() && IsEnabled()){

		iroutput = ir.Get();

		dist = (Vision()/3000);

	speedL = -0.2 - dist;
	speedR = -0.2 + dist;

	if(iroutput){
		time.Start();
		time.Reset();
		while(time.Get() < 4){
			Lmotor1.Set(0);
			Rmotor1.Set(0);
			Lmotor2.Set(0);
			Rmotor2.Set(0);
		light.Set(0);
		}
		time.Reset();
		while(time.Get() < 2){
		SmartDashboard::PutNumber("Timer", time.Get());
		Lmotor1.Set(0.2);
		Rmotor1.Set(-0.2);
		Lmotor2.Set(0.2);
		Rmotor2.Set(0.2);
		}

		Lmotor1.Set(0);
		Rmotor1.Set(0);
		Lmotor2.Set(0);
		Rmotor2.Set(0);
		stop = true;

	}else if(stop == false){

	Lmotor1.Set(speedL);
	Rmotor1.Set(-speedR);
	Lmotor2.Set(speedL);
	Rmotor2.Set(speedR);
	light.Set(1);

	}







			}


			frc::Wait(0.005);
		}



	float deadband(float f)
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
		float fwd;
		float trn;
		float Lin;
		float Rin;
		bool lighton = false;
		while (IsOperatorControl() && IsEnabled()) {
			fwd = deadband(joy.GetRawAxis(1));
			trn = deadband(joy.GetRawAxis(4));

			Vision();


			change.update(joy.GetRawButton(1));
			change2.update(joy.GetRawButton(2));

			if(change.isRising()){

				if(!lighton){
				light.Set(1);
				lighton = true;
				}else{
				light.Set(0);
				lighton = false;
				}
			}

			SmartDashboard::PutBoolean("IR", ir.Get());
			SmartDashboard::PutNumber("Ultrasonic", ultrasonic.GetValue()/0.37203);

			Lin = fwd-trn;
			Rin = fwd+trn;

			Lmotor1.Set(Lin);
			Rmotor1.Set(-Rin);
			Lmotor2.Set(Lin);
			Rmotor2.Set(Rin);


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
