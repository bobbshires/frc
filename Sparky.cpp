/*
 * $Id$
 */

#include "Sparky.h"

/**
 * Sparky class.  Describes the 2012 FRC robot.
 */
Sparky::Sparky(void):
	drive(3, 2),
	stick1(1),
	stick2(2),
	stick3(3),
	targetingTask("targeting", (FUNCPTR)Targeting::VisionTracking, 102),
	blinkyLightsTask("blinkyLights", (FUNCPTR)Lights::BlinkyLights, 103),
	autoAimTask("autoAim", (FUNCPTR)Shooter::AutoAim),
	ds(DriverStation::GetInstance()),
	dsLCD(DriverStationLCD::GetInstance()),
	targeting(),
	loader(),
	shooter(this),
	bridgeArm(),
	lights()
{
	printf("Sparky: start\n");
	drive.SetExpiration(0.1);
	drive.SetSafetyEnabled(false);
	drive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	drive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	printf("Sparky: done\n");
}
	
/**
 * When disabled, suspend the targeting Task.
 */
void Sparky::Disabled()
{
	if(targetingTask.IsReady() && !targetingTask.IsSuspended())
		targetingTask.Suspend();
	
	if(blinkyLightsTask.IsReady() && !blinkyLightsTask.IsSuspended())
		blinkyLightsTask.Suspend();
}

/**
 * Overridden to avoid runtime message.
 */
void Sparky::RobotInit()
{	
}

/**
 * Score two baskets from the key.
 */
void Sparky::Autonomous(void)
{
	printf("Autonomous: start\n");
	drive.SetSafetyEnabled(false);
	/*
	if(targeting.IsSuspended())
		targetingTask.Resume();
	else
		targetingTask.Start();
	*/

	if(IsAutonomous() && IsEnabled())
	{
		if(ds->GetDigitalIn(1))
		{
			printf("Waiting 1...");
			Wait(3);
			printf("Waiting done!\n");
		}
		else if(ds->GetDigitalIn(2))
		{
			printf("Waiting 2...");
			Wait(5);
			printf("Waiting done!\n");
		}
		else if(ds->GetDigitalIn(3))
		{
			printf("Waiting 3...");
			Wait(7);
			printf("Waiting done!\n");
		}
		
		int p = 190;
		
		shooter.ArmToPosition(p);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", shooter.getTension());
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d",
				loader.getShooter(), loader.getTop(), loader.getMiddle());
		dsLCD->UpdateLCD();
		shooter.ReleaseNotifier(this);
		shooter.ArmToPositionNoEye(p);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", shooter.getTension());
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d",
				loader.getShooter(), loader.getTop(), loader.getMiddle());
		dsLCD->UpdateLCD();
		shooter.ReleaseNotifier(this);
		while(IsAutonomous() && IsEnabled())
		{
			Wait(0.05);
		}
	}
	//targetingTask.Suspend();
	printf("Autonomous: stop\n");
}

/**
 * Tele-op period.
 */
void Sparky::OperatorControl(void)
{
	printf("OperatorControl: start\n");
	Notifier armToPositionNotifier(Shooter::ArmToPositionNotifier, this);
	Notifier releaseNotifier(Shooter::ReleaseNotifier, this);
	Timer armTimer;
	int lastPosition = 0;
	drive.SetSafetyEnabled(false);
	shooter.setArmSet(false);
	shooter.setReleaseSet(false);
	loader.setIntakeOff(false);
	
	if(targetingTask.IsSuspended())
		targetingTask.Resume();
	else
		targetingTask.Start();
	
	if(blinkyLightsTask.IsSuspended())
		blinkyLightsTask.Resume();
	else
		blinkyLightsTask.Start((UINT32)(this));
	
	armTimer.Start();

	while (IsOperatorControl() && IsEnabled())
	{
		// drive
		if(!shooter.getAutoAimSet())
		{
			if(stick1.GetTrigger() && !stick2.GetTrigger())
			{
				drive.ArcadeDrive(stick1);
			}
			else if(stick1.GetTrigger() && stick2.GetTrigger())
			{
				drive.TankDrive(stick2, stick1);
			}
			else if(stick1.GetRawButton(8) && !ds->GetDigitalIn(5))
			{
				shooter.setAutoAimSet(true);
				autoAimTask.Start((UINT32)(this));
			}
			else
			{
				drive.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		
		// bridge arm
		if(stick1.GetRawButton(6))
		{
			if(!bridgeArm.getBridgeArmDown())
			{
				bridgeArm.setArmDown(true);
			}
			if(bridgeArm.isArmUp() && !bridgeArm.getBridgeArmUp())
			{
				bridgeArm.setArmUp(false);
			}
			if(!bridgeArm.isArmDown() || ds->GetDigitalIn(6))
			{
				bridgeArm.up();
			}
			else
			{
				bridgeArm.off();
			}
		}
		else if(stick1.GetRawButton(7))
		{
			if(!bridgeArm.getBridgeArmUp())
			{
				bridgeArm.setArmUp(true);
			}
			if(bridgeArm.isArmDown() && !bridgeArm.getBridgeArmDown())
			{
				bridgeArm.setArmDown(false);
			}
			if(!bridgeArm.isArmUp() || ds->GetDigitalIn(6))
			{
				bridgeArm.down();
			}
			else
			{
				bridgeArm.off();
			}
		}
		else
		{
			bridgeArm.off();
		}
		
		// shooter arm
		if(!shooter.isArmSet())
		{
			// zero encoder
			if(ds->GetDigitalIn(4))
			{
				if(stick3.GetRawButton(8))
				{
					shooter.zero();
				}
			}
			
			// coarse adjustment
			if(stick3.GetRawButton(3))
			{
				if(shooter.getTension() > 0 || ds->GetDigitalIn(4))
				{
					shooter.load(ARM_SPEED_COARSE_UNLOAD);
				}
			}
			else if(stick3.GetRawButton(2) && loader.getShooter())
			{
				shooter.load(ARM_SPEED_COARSE_LOAD);
			}
			// fine adjustment
			else if(stick3.GetRawButton(5) && loader.getShooter())
			{
				shooter.load(ARM_SPEED_FINE_LOAD);
			}
			else if(stick3.GetRawButton(4))
			{
				if(shooter.getTension() > 0 || ds->GetDigitalIn(4))
				{
					shooter.load(ARM_SPEED_FINE_UNLOAD);
				}
			}
			// move to preset
			else if(stick3.GetRawButton(9))
			{
				shooter.setEncPos(115);
				shooter.setArmSet(true);
				shooter.setArmSpeed(ARM_SPEED_COARSE);
				armToPositionNotifier.StartSingle(0);
			}
			else if(stick3.GetRawButton(8))
			{
				shooter.setEncPos(0);
				shooter.setArmSet(true);
				shooter.setArmSpeed(ARM_SPEED_FULL_UNLOAD);
				armToPositionNotifier.StartSingle(0);
			}
			else if(stick3.GetRawButton(10))
			{
				shooter.setEncPos(175);
				shooter.setArmSet(true);
				shooter.setArmSpeed(ARM_SPEED_COARSE);
				armToPositionNotifier.StartSingle(0);
			}
			else if(stick3.GetRawButton(11))
			{
				shooter.setEncPos(lastPosition);
				shooter.setArmSet(true);
				shooter.setArmSpeed(ARM_SPEED_COARSE);
				armToPositionNotifier.StartSingle(0);
			}
			else
			{
				shooter.load(TENSION_BRAKE); // brake spool
			}
		}
		
		// make sure that ball isn't settling in the arm
		if(loader.getShooter())
		{
			armTimer.Reset();
		}
		
		// ball loading
		if(!loader.isIntakeOff())
		{
			if(stick3.GetRawButton(6))
			{
				loader.load(armTimer, shooter.getTension());
			}
			else if(stick3.GetRawButton(7))
			{
				loader.unload();
			}
			else
			{
				loader.stop();
			}
		}
	
		// release
		if(!shooter.isReleaseSet())
		{
			if(stick3.GetTrigger())
			{
				lastPosition = shooter.getTension();
				shooter.setReleaseSet(true);
				releaseNotifier.StartSingle(0);
			}
		}
		
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "encoder: %d", shooter.getTension());
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "shooter: %d", loader.getShooter());
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "top:     %d", loader.getTop());
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "middle:  %d", loader.getMiddle());
		dsLCD->UpdateLCD();
		
		Wait(0.005); // wait for a motor update time
	}
	autoAimTask.Stop();
	targetingTask.Suspend();
	blinkyLightsTask.Suspend();
	armToPositionNotifier.Stop();
	releaseNotifier.Stop();
	printf("OperatorControl: stop\n");
}

Loader* Sparky::GetLoader()
{
	return &loader;
}

Shooter* Sparky::GetShooter()
{
	return &shooter;
}

RobotDrive* Sparky::GetDrive()
{
	return &drive;
}

int Sparky::GetTension()
{
	return shooter.getTension();
}

Targeting* Sparky::GetTargeting()
{
	return &targeting;
}

Lights* Sparky::GetLights()
{
	return &lights;
}

START_ROBOT_CLASS(Sparky);
