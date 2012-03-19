/*
 * $Id$
 */

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"

static AxisCamera *camera;

// encoder
static SEM_ID armSem;
static int encPos;
static bool armSet;
static double armSpeed;

// bridge arm
static SEM_ID bridgeArmSem;
static bool bridgeArmSet;

// trigger release
static SEM_ID releaseSem;
static bool releaseSet;
static bool intakeOff;
 
/**
 * Sparky class.  Describes the 2012 FRC robot.
 */
class Sparky : public SimpleRobot
{
	RobotDrive sparky;
	Joystick stick1, stick2, stick3;
	Task targeting;
	DigitalInput top, middle, shooter, trigger, bridgeArmUp, bridgeArmDown;
	DriverStation *ds;
	DriverStationLCD *dsLCD;
	Jaguar arm;
	Victor floorPickup, shooterLoader;
	Relay release, bridgeArm, lights;
	Encoder tension;
	
	// constants
	static const double MOTOR_OFF = 0.0;
	static const double TENSION_BRAKE = -0.06;
	static const double ARM_SPEED_COARSE = 0.5;
	static const double ARM_SPEED_COARSE_LOAD = -0.5;
	static const double ARM_SPEED_COARSE_UNLOAD = 0.5;
	static const double ARM_SPEED_FINE_LOAD = -0.3;
	static const double ARM_SPEED_FINE_UNLOAD = 0.2;
	static const double ARM_SPEED_FULL = 1.0;
	static const double INTAKE_LOAD = 1.0;
	static const double INTAKE_UNLOAD = -1.0;
	static const double INTAKE_OFF = 0.0;

public:
	Sparky(void):
		sparky(3, 2),
		stick1(1),
		stick2(2),
		stick3(3),
		targeting("targeting", (FUNCPTR)Targeting),
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
		release(6),
		bridgeArm(5),
		lights(4),
		tension(1,2)  // measures tension-revolutions 
	{
		printf("Sparky: start\n");
		encPos = 0;
		armSet = false;
		bridgeArmSet = false;
		tension.Reset();
		tension.Start();
		sparky.SetExpiration(0.1);
		sparky.SetSafetyEnabled(false);
		sparky.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		sparky.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		Wait(5);
		camera = &AxisCamera::GetInstance("10.3.84.11");
		camera->WriteResolution(AxisCameraParams::kResolution_640x480);
		camera->WriteWhiteBalance(AxisCameraParams::kWhiteBalance_Hold);
		camera->WriteExposureControl(AxisCameraParams::kExposure_Hold);
		camera->WriteColorLevel(100);
	    camera->WriteCompression(30);
		camera->WriteBrightness(30);
		camera->WriteMaxFPS(10);
		Wait(5);
		printf("Sparky: done\n");
	}
	
	/**
	 * When disabled, suspend the targeting Task.
	 */
	void Disabled()
	{
		if(targeting.IsReady() && !targeting.IsSuspended())
			targeting.Suspend();
		
		lights.Set(Relay::kReverse);
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
			targeting.Resume();
		else
			targeting.Start();
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
		//targeting.Suspend();
		printf("Autonomous: stop\n");
	}
	
	/**
	 * Tele-op period.
	 */
	void OperatorControl(void)
	{
		printf("OperatorControl: start\n");
		Notifier bridgeArmUpNotifier(BridgeArmUpNotifier, this);
		Notifier bridgeArmDownNotifier(BridgeArmDownNotifier, this);
		Notifier armToPositionNotifier(ArmToPositionNotifier, this);
		Notifier releaseNotifier(ReleaseNotifier, this);
		Notifier lightsNotifier(LightsNotifier, this);
		Timer armTimer;
		bool armUp = false;
		bool armDown = false;
		int lastPosition = 0;
		sparky.SetSafetyEnabled(false);
		armSet = false;
		bridgeArmSet = false;
		releaseSet = false;
		intakeOff = false;
		
		if(targeting.IsSuspended())
			targeting.Resume();
		else
			targeting.Start();
		
		lights.Set(Relay::kForward);
		//lightsNotifier.StartSingle(0);
		armTimer.Start();

		while (IsOperatorControl() && IsEnabled())
		{
			// drive
			if(stick1.GetTrigger() && !stick2.GetTrigger())
			{
				sparky.ArcadeDrive(stick1);
			}
			else if(stick1.GetTrigger() && stick2.GetTrigger())
			{
				sparky.TankDrive(stick2, stick1);
			}
			else
			{
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
			
			// bridge arm
			if(!bridgeArmSet)
			{
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
						bridgeArm.Set(Relay::kReverse);
					}
					else
					{
						bridgeArm.Set(Relay::kOff);
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
						bridgeArm.Set(Relay::kForward);
					}
					else
					{
						bridgeArm.Set(Relay::kOff);
					}
				}
				else
				{
					bridgeArm.Set(Relay::kOff);
				}
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
					armSpeed = ARM_SPEED_FULL;
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
			if(!intakeOff)
			{
				if(stick3.GetRawButton(6))
				{
					if(shooter.Get() && top.Get() && middle.Get())
					{
						floorPickup.Set(INTAKE_OFF);
					}
					else if(top.Get() && middle.Get() && tension.Get() > 75)
					{
						floorPickup.Set(INTAKE_OFF);
					}
					else
					{
						floorPickup.Set(INTAKE_LOAD);
					}
					if(!shooter.Get() && tension.Get() < 75 && armTimer.Get() > 1.0)
					{
						shooterLoader.Set(INTAKE_LOAD);
					}
					else if(!top.Get() && shooter.Get())

					{
						shooterLoader.Set(INTAKE_LOAD);
					}
					else if(!top.Get())
					{
						shooterLoader.Set(INTAKE_LOAD);
					}
					else
					{
						shooterLoader.Set(INTAKE_OFF);
					}
				}
				else if(stick3.GetRawButton(7))
				{
					floorPickup.Set(INTAKE_UNLOAD);
					shooterLoader.Set(INTAKE_UNLOAD);
				}
				else
				{
					floorPickup.Set(INTAKE_OFF);
					shooterLoader.Set(INTAKE_OFF);
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
		targeting.Suspend();
		bridgeArmUpNotifier.Stop();
		bridgeArmDownNotifier.Stop();
		armToPositionNotifier.Stop();
		releaseNotifier.Stop();
		printf("OperatorControl: stop\n");
	}
	
	/**
	 * Task to handle targeting with the webcam.  Displays distance and target offset.
	 */
	static int Targeting(void)
	{
		printf("Targeting: start\n");
		vector<Threshold> thresholds;
		thresholds.push_back(Threshold(126, 224, 210, 255, 0, 138));  // field
		//thresholds.push_back(Threshold(0, 177, 165, 255, 0, 141));    // practice field
		//thresholds.push_back(Threshold(0, 158, 123, 255, 0, 160)); // night
		//thresholds.push_back(Threshold(107, 189, 150, 255, 68, 167)); // day
		//thresholds.push_back(Threshold(78, 210, 184, 255, 0, 190)); // day close
		ParticleFilterCriteria2 criteria[] = {
			{IMAQ_MT_BOUNDING_RECT_WIDTH, 10, 400, false, false},
			{IMAQ_MT_BOUNDING_RECT_HEIGHT, 10, 400, false, false}
		};
		double degs = 24;
		double degsVert = 20;
		double pi = 3.141592653589;
		double rads = pi / (double)180;
		double tapeWidth = 2;
		double tapeHeight = 1.5;
		ColorImage *image = NULL;
		double d, dv;
		double lastDist = 0;
		double distCount = 0;
		int centerMassX;
		int centerWidth = 320;
		int centerThresh = 20;
		bool found = false;
		
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
		dsLCD->UpdateLCD();
		
		DriverStation *ds = DriverStation::GetInstance();

		while(true) {
			if(!camera->IsFreshImage()) 
			{
				printf("Image is not fresh.\n");
				Wait(1.0);
				continue;
			}
			if(ds->GetDigitalIn(5))
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Targeting Disabled");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
				dsLCD->UpdateLCD();
				Wait(1.0);
				continue;
			}
			
			found = false;
			image = new RGBImage();
			camera->GetImage(image);
						
			// loop through our threshold values
			for(unsigned i = 0; i < thresholds.size() && !found; i++)
			{
				ParticleAnalysisReport *target = NULL;
				BinaryImage *thresholdImage = NULL;
				BinaryImage *convexHullImage = NULL;
				BinaryImage *bigObjectsImage = NULL;
				BinaryImage *filteredImage = NULL;
				vector<ParticleAnalysisReport> *reports = NULL;
				bool imageError = false;
				
				thresholdImage = image->ThresholdRGB(thresholds.at(i));
				if(!thresholdImage)
				{
					imageError = true;
				}
				if(!imageError)
				{
					convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
					if(!convexHullImage)
					{
						imageError = true;
					}
				}
				if(!imageError)
				{
					bigObjectsImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
					if(!bigObjectsImage)
					{
						imageError = true;
					}
				}
				if(!imageError)
				{
					filteredImage = bigObjectsImage->RemoveSmallObjects(false, 2);  // remove small objects (noise)
					if(!filteredImage)
					{
						imageError = true;
					}
				}
				if(!imageError)
				{
					reports = filteredImage->GetOrderedParticleAnalysisReports();  // get the results
				}
				
				// loop through the reports
				for (unsigned j = 0; reports && j < reports->size(); j++)
				{
					ParticleAnalysisReport *r = &(reports->at(j));
					double fov = (double)(tapeWidth * (double)r->imageWidth) / (double)r->boundingRect.width;
					double distance = (double)(fov / (double)2) / tan(degs * rads);
					double fovVert = (double)(tapeHeight * (double)r->imageHeight) / (double)r->boundingRect.height;
					double distanceVert = (double)(fovVert / (double)2) / tan(degsVert * rads);
					// get the bottom-most basket
					if(!target || target->center_mass_y < r->center_mass_y)
					{

						target = r;
						d = distance;
						dv = distanceVert;
						centerMassX = target->center_mass_x;
					}
					found = true;
					
					/*
					printf("center_mass_x: %d\n", r->center_mass_x);
					printf("center_mass_y: %d\n", r->center_mass_y);
					printf("percent: %f\n", r->particleToImagePercent);
					printf("area: %f\n", r->particleArea);
					printf("image width: %d\n", r->imageWidth);
					printf("image height: %d\n", r->imageHeight);
					printf("rect height: %d\n", r->boundingRect.height);
					printf("rect width: %d\n", r->boundingRect.width);
					printf("rect top: %d\n", r->boundingRect.top);
					printf("rect left: %d\n", r->boundingRect.left);
					printf("fov: %f\n", fov);
					printf("fovVert: %f\n", fovVert);
					printf("distance: %f\n", distance);
					printf("distanceVert: %f\n", distanceVert);
					printf("\n");
					*/
				}
				
				if(reports && !reports->size())
				{
					printf("No particles found.\n");
				}
				else if(imageError)
				{
					printf("Image processing error.\n");
				}
				else
				{
					printf("Particles found.\n");
				}
				
			    delete filteredImage;
				delete convexHullImage;
				delete bigObjectsImage;
				delete thresholdImage;
				delete reports;
				target = NULL;
			}
			
			// determine how many times we've seen a reading
			if(!distCount)
			{
				distCount++;
			}
			else
			{	
				if(lastDist == dv ||
				   (lastDist < dv && dv - lastDist < 1) ||
				   (lastDist > dv && lastDist - dv < 1))
				{
					distCount++;
				}
				else
				{
					distCount = 0;
				}	
			}
			lastDist = dv;
			
			// write to the dashboard if we've seen the same value a certain number of times
			if(distCount > 3)
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "target: %f", dv);
				if(centerMassX == centerWidth ||
				   (centerMassX > centerWidth && centerMassX - centerWidth < centerThresh) ||
				   (centerMassX < centerWidth && centerWidth - centerMassX < centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "%s (%d px %s)", "CENTER",
							centerMassX > centerWidth ? centerMassX - centerWidth : centerWidth - centerMassX,
						    centerMassX > centerWidth ? "right" : "left");
				}
				else if((centerMassX > centerWidth && centerMassX - centerWidth > centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s", "RIGHT");
				}
				else if ((centerMassX < centerWidth && centerWidth - centerMassX > centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s", "LEFT");
				}
			}
			else
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "*** NO TARGET ***");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
			}
			dsLCD->UpdateLCD();
			
			dv = 0;
			
			delete image;
			Wait(0.1);
		}
		printf("Targeting: stop\n");
		return 1;
			
		/*
		if(!loopCount && camera.IsFreshImage()) {
			if(image) image->Write("sparky.jpg");
			if(thresholdImage) thresholdImage->Write("sparky-Thresh.bmp");
			if(bigObjectsImage) bigObjectsImage->Write("sparky-bigObjects.bmp");
			if(convexHullImage) convexHullImage->Write("sparky-convexHull.bmp");
			if(filteredImage) filteredImage->Write("sparky-filtered.bmp");
			loopCount++;
		}
		//*/
	}
	
	void ArmToPosition(int p)
	{
		if(tension.Get() < p && shooter.Get())
		{
			while(tension.Get() < p)
			{
				arm.Set(ARM_SPEED_COARSE_LOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p)
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
			while(tension.Get() < p)
			{
				arm.Set(ARM_SPEED_COARSE_LOAD);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p)
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
			while(tension.Get() < p)
			{
				arm.Set(-1.0);
				Wait(0.005);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p)
			{
				arm.Set(1.0);
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
	
	Relay* GetBridgeArm()
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
	
	static void BridgeArmDownNotifier(void* p)
	{
		printf("BridgeArmDownNotifier: start\n");
		Sparky *s = (Sparky *)p;
		Relay *a = s->GetBridgeArm();
		{
			Synchronized sync(bridgeArmSem);
			a->Set(Relay::kReverse);
			Wait(0.5);
			a->Set(Relay::kOff);
			bridgeArmSet = false;
			printf("BridgeArmDownNotifier: done\n");
		}
	}
	
	static void BridgeArmUpNotifier(void* p)
	{
		printf("BridgeArmUpNotifier: start\n");
		Sparky *s = (Sparky *)p;
		Relay *a = s->GetBridgeArm();
		{
			Synchronized sync(bridgeArmSem);
			a->Set(Relay::kForward);
			Wait(0.5);
			a->Set(Relay::kOff);
			bridgeArmSet = false;
			printf("BridgeArmUpNotifier: done\n");
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
			while(t->Get())
			{
				r->Set(Relay::kReverse);
				Wait(0.005);
			}
			Wait(0.1);
			r->Set(Relay::kOff);
			Wait(0.3);
			releaseSet = false;
			intakeOff = true;
			s->ArmToPositionFull(0);
			while(e->Get() > 75)
			{
				Wait(0.1);
			}
			while(top->Get())
			{
				sl->Set(INTAKE_LOAD);
				Wait(0.005);
			}
			Wait(1.0);
			sl->Set(INTAKE_OFF);
			s->ArmToPosition(125);
			intakeOff = false;
			printf("ReleaseNotifier: done\n");
		}
	}
	
	static void LightsNotifier(void* p)
	{
		printf("Lights Notifier: start\n");
		Sparky *s = (Sparky *)p;
		DigitalInput *top = s->GetTop();
		DigitalInput *middle = s->GetMiddle();
		DigitalInput *shooter = s->GetShooter();
		Relay *lights = s->GetLights();
		while(s->IsOperatorControl() && s->IsEnabled())
		{
			if(shooter->Get() && top->Get() && middle->Get())
			{
				lights->Set(Relay::kForward);
			}
			else
			{
				if(shooter->Get())
				{
					lights->Set(Relay::kForward);
					Wait(0.2);
					lights->Set(Relay::kOff);
					Wait(0.1);
				}
				if(middle->Get())
				{
					lights->Set(Relay::kForward);
					Wait(0.2);
					lights->Set(Relay::kOff);
					Wait(0.1);
				}
				if(top->Get())
				{
					lights->Set(Relay::kForward);
					Wait(0.2);
					lights->Set(Relay::kOff);
					Wait(0.1);
				}
			}
			Wait(1.0);
		}
		printf("LightsNotifier: done\n");
	}
};

START_ROBOT_CLASS(Sparky);
