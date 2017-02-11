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
#include"Auton.h"

class Robot: public frc::SampleRobot {
	Joystick joy;
	Joystick joy2;
	DriveTrain drivetrain;
	Auton auton;
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

public:
	Robot() :
			joy(0), joy2(1), drivetrain(3, 4, 7, 5, 8, 2, 3), auton(), shift(joy.GetRawButton(1)), table(NetworkTable::GetTable("GRIP/myContoursReport")),
			flipper(), lifter(0), pickup(6), shooter(1, 2), flip(joy2.GetRawButton(5)), lift(joy.GetRawButton(2)),
			shoot(joy2.GetRawButton(6)), pick(joy2.GetRawButton(3)), speedup(joy2.GetRawButton(8)),
			speeddown(joy2.GetRawButton(7)), SpinUp(joy2.GetRawButton(2)) {

	}

	void RobotInit() {
		cs::UsbCamera camera =
				CameraServer::GetInstance()->StartAutomaticCapture();
	}

	void Autonomous() {

		while (IsAutonomous() && IsEnabled()) {

			auton.Drive();
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

		float speed = 0.7;
		bool ison = false;

		while (IsOperatorControl() && IsEnabled()) {

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
				speed += 1.2;
				shooter.UpdateSpeed(speed);
			}

			if (speeddown.isPressed()) {
				speed -= 1.2;
				shooter.UpdateSpeed(speed);
			}

			if (pick.isPressed())
				pickup.Toggle();

			//DriveTrain Controls

			drivetrain.Drive(deadzone(joy.GetRawAxis(1)),
					deadzone(joy.GetRawAxis(4)));

			//Display To Dashboard

			SmartDashboard::PutNumber("LMotors",
					drivetrain.getMotorVel(DriveTrain::leftSide));
			SmartDashboard::PutNumber("RMotors",
					drivetrain.getMotorVel(DriveTrain::rightSide));
			SmartDashboard::PutNumber("Speed", speed*100);

			SmartDashboard::PutNumber("Shooter", shooter.getVel());
							SmartDashboard::PutNumber("Shooter 2", shooter.getVel2());

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
