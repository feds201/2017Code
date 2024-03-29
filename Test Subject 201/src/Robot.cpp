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

/*
 *
 * TODO:
 *
 * Update Firmware - 5 MIN
 * Check for floating current - 5 MIN
 * Double check camera feed - 2 MIN
 * Test input scaling - 7 MIN
 * Test encoders - 1 MIN
 * fix stirrer - 5 MIN
 * fix pressure trans - 5 MIN
 *
 *
 */


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
	Edge light;
	Edge revpickup;
	Solenoid frontlight;
	Solenoid backlight;
	AutoAim aim;
	DigitalInput ballIn;
	AnalogInput pressure;
	cs::UsbCamera camera;
	cs::UsbCamera cam2;
	PowerDistributionPanel pdp;

public:

	Robot() :
			joy(0), joy2(1), drivetrain(4, 3, 7, 5, 8, 0, 1), auton(&drivetrain, &aim, &shooter), shift(joy.GetRawButton(1)),
			flipper(), lifter(0), pickup(6), shooter(1, 2), flip(joy2.GetRawButton(5)), lift(joy.GetRawButton(2)),
			shoot(joy2.GetRawButton(6)), pick(joy2.GetRawButton(3)), speedup(joy2.GetRawButton(8)),
			speeddown(joy2.GetRawButton(7)), SpinUp(joy2.GetRawButton(2)), light(joy2.GetRawButton(9)), revpickup(joy2.GetRawButton(4)),
			frontlight(9, 0), backlight(9, 1), aim(&drivetrain), ballIn(6), pressure(1), camera(), cam2(), pdp(0)
	{


	}

	void RobotInit() {
		camera = CameraServer::GetInstance()->StartAutomaticCapture();
		cam2 = CameraServer::GetInstance()->StartAutomaticCapture();

		camera.SetFPS(10);
		cam2.SetResolution(480, 320);
		camera.SetResolution(1280, 720);
		camera.SetExposureManual(32);
		cam2.SetFPS(10);
		camera.SetExposureManual(2);
		cam2.SetExposureManual(2);
	}

	void Autonomous() {

		camera.SetExposureManual(2);
		cam2.SetExposureManual(2);

		backlight.Set(true);
		frontlight.Set(true);

		auton.Routes(this);

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

		float speed = 3360;
		bool ison = false;
		bool lightstat = false;
		bool shooterstat = false;

		//camera.SetExposureManual(2);
		//cam2.SetExposureManual(2);

		shooter.returnStirToHome();
		shooter.Stop();

		while (IsOperatorControl() && IsEnabled()) {

			camera.SetExposureAuto();
			cam2.SetExposureAuto();

			frontlight.Set(false);
			//Button Updates

			shift.update(joy.GetRawButton(1));

			flip.update(joy2.GetRawButton(5));
			lift.update(joy.GetRawButton(2));
			shoot.update(joy2.GetRawButton(6));
			pick.update(joy2.GetRawButton(3));
			speedup.update(joy2.GetRawButton(8));
			speeddown.update(joy2.GetRawButton(7));
			SpinUp.update(joy2.GetRawButton(2));
			light.update(joy2.GetRawButton(9));
			revpickup.update(joy2.GetRawButton(4));

			//Execution Of Robot Functions Based On Controller Input

			if(revpickup.isPressed()){
				pickup.Toggle(0.6);
			}

			if(light.isPressed()){
				if(lightstat == false)
					lightstat = true;
				else
					lightstat = false;
			}

			if (shift.isPressed())
				drivetrain.Shift();

			if (flip.isPressed())
				flipper.Toggle();

			if (lift.isPressed())
				lifter.Toggle();

			if (shoot.isPressed()){
				shooter.Shoot();
				if(shooterstat == false)
				shooterstat = true;
				else
				shooterstat = false;
			}

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
				speed += 105;
				shooter.UpdateSpeed(speed);
			}

			if (speeddown.isPressed()) {
				speed -= 105;
				shooter.UpdateSpeed(speed);
			}

			if (pick.isPressed())
				pickup.Toggle(-0.6);

			//DriveTrain Controls

			drivetrain.Drive(deadzone(joy.GetRawAxis(1)), deadzone(joy.GetRawAxis(4))/2);

			if(ison == true){
				shooter.SpinUp();
			}

			if(ison && !shooterstat){
				shooter.Stir();
			}else{
				shooter.returnStirToHome();
			}
			if(joy2.GetRawAxis(2) > 0.7)
				aim.Aim();

			backlight.Set(lightstat);

			if(ballIn.Get() && ison){
				joy2.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1);
			}else{
				joy2.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
			}


			//Display To Dashboard

			SmartDashboard::PutNumber("LMotors",
					drivetrain.getMotorVel(DriveTrain::leftSide));
			SmartDashboard::PutNumber("RMotors",
					drivetrain.getMotorVel(DriveTrain::rightSide));
			SmartDashboard::PutNumber("Speed", (speed*100)/10500);

			SmartDashboard::PutNumber("Shooter", -shooter.getVel());
							SmartDashboard::PutNumber("Shooter 2", shooter.getVel2());

			SmartDashboard::PutBoolean("Is Ball Ready?", ballIn.Get());

			SmartDashboard::PutNumber("Dist From Target", aim.DistCalc());

			SmartDashboard::PutNumber("Pressure", pressure.GetValue());

			SmartDashboard::PutNumber("AMPS of of PDP CH 14", pdp.GetCurrent(14));
			SmartDashboard::PutNumber("AMPS of of PDP CH 15", pdp.GetCurrent(15));
			SmartDashboard::PutNumber("AMPS of of PDP CH 1", pdp.GetCurrent(1));
			SmartDashboard::PutNumber("AMPS of of PDP CH 2", pdp.GetCurrent(2));
			SmartDashboard::PutNumber("AMPS of of PDP CH 3", pdp.GetCurrent(3));
			SmartDashboard::PutNumber("AMPS of of PDP CH 12", pdp.GetCurrent(12));
			SmartDashboard::PutNumber("AMPS of of PDP CH 13", pdp.GetCurrent(13));

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
