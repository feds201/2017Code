#include <CameraServer.h>
#include"WPILib.h"
#include"CANTalon.h"
#include"EdgeDetection.h"
#include"DriveTrain.h"
#include"GearFlipper.h"
#include"Lifter.h"
#include"Pickup.h"
#include"Shooter.h"
#include"Auton.h"
#include"AutoAim.h"

class Robot: public frc::SampleRobot {
	Joystick joy;
	Joystick joy2;
	DriveTrain drivetrain;
	Auton auton;
	Edge shift;
	Timer time;
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
	Solenoid frontlight;
	Solenoid backlight;
	//AutoAim aim;

public:
	Robot() :
			joy(0), joy2(1), drivetrain(4, 3, 7, 5, 8, 0, 1), auton(&drivetrain), shift(joy.GetRawButton(1)),
			flipper(), lifter(0), pickup(6), shooter(1, 2), flip(joy2.GetRawButton(5)), lift(joy.GetRawButton(2)),
			shoot(joy2.GetRawButton(6)), pick(joy2.GetRawButton(3)), speedup(joy2.GetRawButton(8)),
			speeddown(joy2.GetRawButton(7)), SpinUp(joy2.GetRawButton(2)), frontlight(8, 7), backlight(8, 6)
	{


	}

	void RobotInit() {
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		cs::UsbCamera cam2 = CameraServer::GetInstance()->StartAutomaticCapture();

		camera.SetResolution(320, 240);
		camera.SetFPS(10);
		camera.SetExposureManual(1);
		cam2.SetResolution(320, 240);
		cam2.SetFPS(10);
		cam2.SetExposureManual(1);
	}

	void Autonomous() {
		int autonmode = auton.Routes(this);

		while(IsAutonomous() && IsEnabled()){

			backlight.Set(true);
			frontlight.Set(true);

		if(autonmode == 1){

		}else if(autonmode == 2){



		}else{
			auton.Drive();
		}

		}
	}

	float deadzone(float f) {
		if (fabs(f) < .15)
			return 0.0f;
		else {
			if (f > 0)
				return (f - .15) / (1 - .15);
			else
				return (f + .15) / (1 - .15);
		}
	}

	void OperatorControl() override {

		float speed = 5250;
		bool ison = false;

		shooter.returnStirToHome();
		shooter.Stop();

		while (IsOperatorControl() && IsEnabled()) {

			backlight.Set(true);
			frontlight.Set(true);

			//Button Updates

			shift.update(joy.GetRawButton(1));

			flip.update(joy2.GetRawButton(5));
			lift.update(joy.GetRawButton(2));
			shoot.update(joy2.GetRawButton(6));
			pick.update(joy2.GetRawButton(3));
			speedup.update(joy2.GetRawButton(8));
			speeddown.update(joy2.GetRawButton(7));
			SpinUp.update(joy2.GetRawButton(2));

			//Execution Of Robot Functions Based On Controller Input

			if (shift.isPressed())
				drivetrain.Shift();

			if (flip.isPressed())
				flipper.Toggle();

			if (lift.isPressed())
				lifter.Toggle();

			if (shoot.isPressed())
				shooter.Shoot();

			if (SpinUp.isPressed()) {
				if (ison == false) {
					shooter.SpinUp();
					ison = true;
				} else {
					shooter.Stop();
					ison = false;
				}
			}

			if (speedup.isPressed()) {
				speed += 525;
				shooter.UpdateSpeed(speed);
			}

			if (speeddown.isPressed()) {
				speed -= 525;
				shooter.UpdateSpeed(speed);
			}

			if (pick.isPressed())
				pickup.Toggle();

			//DriveTrain Controls

			drivetrain.Drive(deadzone(joy.GetRawAxis(1)), deadzone(joy.GetRawAxis(4)));

			if(ison == true){
				shooter.Stir();
			}else{
				shooter.returnStirToHome();
			}

			//Display To Dashboard

			SmartDashboard::PutNumber("LMotors",
					drivetrain.getMotorVel(DriveTrain::leftSide));
			SmartDashboard::PutNumber("RMotors",
					drivetrain.getMotorVel(DriveTrain::rightSide));
			SmartDashboard::PutNumber("Speed", (speed*100)/10500);

			SmartDashboard::PutNumber("Shooter", -shooter.getVel());
							SmartDashboard::PutNumber("Shooter 2", shooter.getVel2());

			//SmartDashboard::PutNumber("Dist From Center", aim.DistCalc());
			//SmartDashboard::PutNumber("Dist From Target", aim.DistCalc());

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
