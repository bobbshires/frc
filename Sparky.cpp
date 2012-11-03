/*
 * $Id$
 */

#include "Sparky.h"

/**
 * Sparky class.  Describes the 2012 FRC robot.
 */
class Sparky : public SimpleRobot
{
	RobotDrive sparky;
	Joystick stick1, stick2, stick3;
	Task targetingTask, blinkyLightsTask, autoAimTask;
	DigitalInput top, middle, shooter, trigger, bridgeArmUp, bridgeArmDown;
	DriverStation *ds;
	DriverStationLCD *dsLCD;
	Jaguar arm;
	Victor floorPickup, shooterLoader, bridgeArm;
	Relay release, lights;
	Encoder tension;
	Targeting targeting;
	Loader loader;
	bool autoAimSet;
	static SEM_ID autoAimSem;
	
public:
	Sparky(void):
		sparky(3, 2),
		stick1(1),
		stick2(2),
		stick3(3),
		targetingTask("targeting", (FUNCPTR)Targeting::VisionTracking, 102),
		blinkyLightsTask("blinkyLights", (FUNCPTR)BlinkyLights, 103),
		autoAimTask("autoAim", (FUNCPTR)AutoAim),
		top(13),
		middle(14),
		shooter(12),
		trigger(11),
		bridgeArmUp(3),
		bridgeArmDown(4),
		ds(DriverStation::GetInstance()),
		dsLCD(DriverStationLCD::GetInstance()),
		arm(1),
		floorPickup(5),
		shooterLoader(4),
		bridgeArm(7),
		release(6),
		lights(4),
		tension(1,2),  // measures tension-revolutions
		targeting(),
		loader()
	{
		printf("Sparky: start\n");
		encPos = 0;
		armSet = false;
		autoAimSet = false;
		tension.Reset();
		tension.Start();
		sparky.SetExpiration(0.1);
		sparky.SetSafetyEnabled(false);
		sparky.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		sparky.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		printf("Sparky: done\n");
	}
	
	/**
	 * When disabled, suspend the targeting Task.
	 */
	void Disabled()
	{
		if(targetingTask.IsReady() && !targetingTask.IsSuspended())
			targetingTask.Suspend();
		
		if(blinkyLightsTask.IsReady() && !blinkyLightsTask.IsSuspended())
			blinkyLightsTask.Suspend();
	}
	
	/**
	 * Overridden to avoid runtime message.
	 */
	void RobotInit()
	{	
	}
	
	/**
	 * Score two baskets from the key.
	 */
	void Autonomous(void)
	{
		printf("Autonomous: start\n");
		sparky.SetSafetyEnabled(false);
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
			
			ArmToPosition(p);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", tension.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d", shooter.Get(), top.Get(), middle.Get());
			dsLCD->UpdateLCD();
			ReleaseNotifier(this);
			ArmToPositionNoEye(p);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", tension.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d", shooter.Get(), top.Get(), middle.Get());
			dsLCD->UpdateLCD();
			ReleaseNotifier(this);
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
	void OperatorControl(void)
	{
		printf("OperatorControl: start\n");
		Notifier armToPositionNotifier(ArmToPositionNotifier, this);
		Notifier releaseNotifier(ReleaseNotifier, this);
		Timer armTimer;
		bool armUp = false;
		bool armDown = false;
		int lastPosition = 0;
		sparky.SetSafetyEnabled(false);
		armSet = false;
		releaseSet = false;
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
			if(!autoAimSet)
			{
				if(stick1.GetTrigger() && !stick2.GetTrigger())
				{
					sparky.ArcadeDrive(stick1);
				}
				else if(stick1.GetTrigger() && stick2.GetTrigger())
				{
					sparky.TankDrive(stick2, stick1);
				}
				else if(stick1.GetRawButton(8) && !ds->GetDigitalIn(5))
				{
					autoAimSet = true;
					autoAimTask.Start((UINT32)(this));
				}
				else
				{
					sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
				}
			}
			
			// bridge arm
			if(stick1.GetRawButton(6))
			{
				if(!bridgeArmDown.Get())
				{
					armDown = true;
				}
				if(armUp && !bridgeArmUp.Get())
				{
					armUp = false;
				}
				if(!armDown || ds->GetDigitalIn(6))
				{
					bridgeArm.Set(BRIDGE_ARM_UP);
				}
				else
				{
					bridgeArm.Set(BRIDGE_ARM_OFF);
				}
			}
			else if(stick1.GetRawButton(7))
			{
				if(!bridgeArmUp.Get())
				{
					armUp = true;
				}
				if(armDown && !bridgeArmDown.Get())
				{
					armDown = false;
				}
				if(!armUp || ds->GetDigitalIn(6))
				{
					bridgeArm.Set(BRIDGE_ARM_DOWN);
				}
				else
				{
					bridgeArm.Set(BRIDGE_ARM_OFF);
				}
			}
			else
			{
				bridgeArm.Set(BRIDGE_ARM_OFF);
			}
			
			// shooter arm
			if(!armSet)
			{
				// zero encoder
				if(ds->GetDigitalIn(4))
				{
					if(stick3.GetRawButton(8))
					{
						tension.Reset();
					}
				}
				
				// coarse adjustment
				if(stick3.GetRawButton(3))
				{
					if(tension.Get() > 0 || ds->GetDigitalIn(4))
					{
						arm.Set(ARM_SPEED_COARSE_UNLOAD);
					}
				}
				else if(stick3.GetRawButton(2) && shooter.Get())
				{
					arm.Set(ARM_SPEED_COARSE_LOAD);
				}
				// fine adjustment
				else if(stick3.GetRawButton(5) && shooter.Get())
				{
					arm.Set(ARM_SPEED_FINE_LOAD);
				}
				else if(stick3.GetRawButton(4))
				{
					if(tension.Get() > 0 || ds->GetDigitalIn(4))
					{
						arm.Set(ARM_SPEED_FINE_UNLOAD);
					}
				}
				// move to preset
				else if(stick3.GetRawButton(9))
				{
					encPos = 115;
					armSet = true;
					armSpeed = ARM_SPEED_COARSE;
					armToPositionNotifier.StartSingle(0);
				}
				else if(stick3.GetRawButton(8))
				{
					encPos = 0;
					armSet = true;
					armSpeed = ARM_SPEED_FULL_UNLOAD;
					armToPositionNotifier.StartSingle(0);
				}
				else if(stick3.GetRawButton(10))
				{
					encPos = 175;
					armSet = true;
					armSpeed = ARM_SPEED_COARSE;
					armToPositionNotifier.StartSingle(0);
				}
				else if(stick3.GetRawButton(11))
				{
					encPos = lastPosition;
					armSet = true;
					armSpeed = ARM_SPEED_COARSE;
					armToPositionNotifier.StartSingle(0);
				}
				else
				{
					arm.Set(TENSION_BRAKE); // brake spool
				}
			}
			
			// make sure that ball isn't settling in the arm
			if(shooter.Get())
			{
				armTimer.Reset();
			}
			
			// ball loading
			if(!loader.isIntakeOff())
			{
				if(stick3.GetRawButton(6))
				{
					loader.load(armTimer, tension);
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
			if(!releaseSet)
			{
				if(stick3.GetTrigger())
				{
					lastPosition = tension.Get();
					releaseSet = true;
					releaseNotifier.StartSingle(0);
				}
			}
			
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "encoder: %d", tension.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "shooter: %d", shooter.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "top: %d", top.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "middle: %d", middle.Get());
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
	
	void ArmToPosition(int p)
	{
		if(tension.Get() < p && shooter.Get())
		{
			while(tension.Get() < p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_COARSE_LOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_COARSE_UNLOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		arm.Set(TENSION_BRAKE);
	}
	
	void ArmToPositionNoEye(int p)
	{
		if(tension.Get() < p)
		{
			while(tension.Get() < p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_COARSE_LOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_COARSE_UNLOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		arm.Set(TENSION_BRAKE);
	}
	
	void ArmToPositionFull(int p)
	{
		if(tension.Get() < p && shooter.Get())
		{
			while(tension.Get() < p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_FULL_LOAD);
				Wait(0.005);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p && this->IsEnabled())
			{
				arm.Set(ARM_SPEED_FULL_UNLOAD);
				Wait(0.005);

			}
		}
		arm.Set(TENSION_BRAKE);
	}
	
	Encoder* GetTension()
	{
		return &tension;
	}
	
	Jaguar* GetArm()
	{
		return &arm;
	}
	
	DigitalInput* GetShooter()
	{
		return &shooter;
	}
	
	DigitalInput* GetTop()
	{
		return &top;
	}
	
	DigitalInput* GetMiddle()
	{
		return &middle;
	}
	
	Victor* GetBridgeArm()
	{
		return &bridgeArm;
	}
	
	Relay* GetRelease()
	{
		return &release;
	}
	
	DigitalInput* GetTrigger()
	{
		return &trigger;
	}
	
	Victor* GetShooterLoader()
	{
		return &shooterLoader;
	}
	
	Relay* GetLights()
	{
		return &lights;
	}
	
	static void ArmToPositionNotifier(void* p)
	{
		Sparky *s = (Sparky *)p;
		Encoder *t = s->GetTension();
		Jaguar *a = s->GetArm();
		DigitalInput *shoot = s->GetShooter();
		{
			Synchronized sync(armSem);
			if(t->Get() < encPos && shoot->Get())
			{
				while(t->Get() < encPos)
				{
					a->Set(-armSpeed);
					Wait(0.005);
				}
			}
			else if(t->Get() > encPos)
			{
				while(t->Get() > encPos)
				{
					a->Set(armSpeed);
					Wait(0.005);
				}
			}
			a->Set(TENSION_BRAKE);
			armSet = false;
		}
	}
	
	static void ReleaseNotifier(void* p)
	{
		printf("ReleaseNotifier: start\n");
		Sparky *s = (Sparky *)p;
		Relay *r = s->GetRelease();
		Victor *sl = s->GetShooterLoader(); 
		DigitalInput *t = s->GetTrigger();
		DigitalInput *top = s->GetTop();
		Encoder *e = s->GetTension();
		{
			Synchronized sync(releaseSem);
			while(t->Get() && s->IsEnabled())
			{
				r->Set(Relay::kReverse);
				Wait(0.005);
			}
			Wait(0.1);
			r->Set(Relay::kOff);
			Wait(0.3);
			releaseSet = false;
			s->loader.setIntakeOff(true);
			armSet = true;
			s->ArmToPositionFull(0);
			while(e->Get() > ARM_ZERO_THRESH && s->IsEnabled())
			{
				Wait(0.1);
			}
			while(top->Get() && s->IsEnabled())
			{
				sl->Set(INTAKE_LOAD);
				Wait(0.005);
			}
			Wait(1.0);
			sl->Set(INTAKE_OFF);
			s->ArmToPosition(125);
			s->loader.setIntakeOff(false);
			armSet = false;
			printf("ReleaseNotifier: done\n");
		}
	}
	
	static int BlinkyLights(UINT32 argPtr)
	{
		printf("BlinkyLights: start\n");
		Sparky *s = (Sparky*)argPtr;
		while(true)
		{
			if(s->loader.getShooter() && s->loader.getTop() && s->loader.getMiddle())
			{
				s->lights.Set(Relay::kForward);
			}
			else
			{
				if(s->loader.getShooter())
				{
					s->lights.Set(Relay::kForward);
					Wait(0.2);
					s->lights.Set(Relay::kOff);
					Wait(0.1);
				}
				if(s->loader.getMiddle())
				{
					s->lights.Set(Relay::kForward);
					Wait(0.2);
					s->lights.Set(Relay::kOff);
					Wait(0.1);
				}
				if(s->loader.getTop())
				{
					s->lights.Set(Relay::kForward);
					Wait(0.2);
					s->lights.Set(Relay::kOff);
					Wait(0.1);
				}
			}
			//printf("Blinking!\n");
			Wait(1.0);
		}
		printf("BlinkyLights: done\n");
		return 0;
	}
	
	static int AutoAim(UINT32 argPtr)
	{
		Synchronized sync(autoAimSem);
		printf("AutoAim: start\n");
		
		Sparky *s = (Sparky*)argPtr;
		
		targetAlignment ta = s->targeting.getTargetAlign();
		double d = s->targeting.getTargetDistance();
		
		while(ta != TARGET_CENTER)
		{
			if(ta == TARGET_RIGHT)
			{
				s->sparky.TankDrive(AUTO_AIM_SPEED, -AUTO_AIM_SPEED);
			}
			else if(ta == TARGET_LEFT)
			{
				s->sparky.TankDrive(-AUTO_AIM_SPEED, AUTO_AIM_SPEED);
			}
			else if(ta == TARGET_NONE)
			{
				break;
			}
			Wait(0.1);
			ta = s->targeting.getTargetAlign();
			d = s->targeting.getTargetDistance();
		}

		s->sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
		s->autoAimSet = false;
		printf("AutoAim: done\n");
		return 0;
	}
};

START_ROBOT_CLASS(Sparky);
