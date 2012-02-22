/*
 * $Id$
 */

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"

static AxisCamera &camera = AxisCamera::GetInstance("10.3.84.11");
// encoder
static SEM_ID armSem;
static int encPos;
static bool armSet;
 
/**
 * Sparky class.  Describes the 2012 FRC robot.
 */
class Sparky : public SimpleRobot
{
	RobotDrive sparky;
	Joystick stick1, stick2, stick3;
	Task targeting;
	DigitalInput top, middle, shooter;
	DriverStation *ds;
	DriverStationLCD *dsLCD;
	Jaguar arm;
	Relay floorPickup, shooterLoader, release;
	Encoder tension;
	
	// constants
	static const double MOTOR_OFF = 0.0;
	static const double TENSION_BRAKE = -0.06;
	static const double ARM_SPEED_COARSE_LOAD = -0.5;
	static const double ARM_SPEED_COARSE_UNLOAD = 0.5;
	static const double ARM_SPEED_FINE_LOAD = -0.3;
	static const double ARM_SPEED_FINE_UNLOAD = 0.1;

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
		ds(DriverStation::GetInstance()),
		dsLCD(DriverStationLCD::GetInstance()),
		arm(1),
		floorPickup(7),
		shooterLoader(8),
		release(6),
		tension(1,2)  // measures tension-revolutions 
		//reserved(10) // not used yet
	{
		printf("Sparky: start\n");
		encPos = 0;
		armSet = 0;
		tension.Reset();
		tension.Start();
		sparky.SetExpiration(0.1);
		sparky.SetSafetyEnabled(false);
		sparky.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		camera.WriteResolution(AxisCameraParams::kResolution_640x480);
		camera.WriteWhiteBalance(AxisCameraParams::kWhiteBalance_Hold);
		camera.WriteExposureControl(AxisCameraParams::kExposure_Hold);
		camera.WriteColorLevel(100);
	    camera.WriteCompression(30);
		camera.WriteBrightness(30);
		Wait(5);
		printf("Sparky: done\n");
	}
	
	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		printf("Autonomous: start\n");
		sparky.SetSafetyEnabled(false);
		targeting.Start();

		if(IsAutonomous() && IsEnabled()) {
			/*
			if(ds->GetDigitalIn(1))
			{
				printf("Waiting...");
				Wait(5);
				printf("Waiting done!");
			}
			else
			{
				
			}
			*/
			
			int p = 185;
			double wait = 0.73;
			
			ArmToPosition(p);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", tension.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d", shooter.Get(), top.Get(), middle.Get());
			dsLCD->UpdateLCD();
			release.Set(Relay::kReverse);
			Wait(wait);
			release.Set(Relay::kOff);
			ArmToPositionFull(0);
			while(!shooter.Get())
			{
				floorPickup.Set(Relay::kForward);
				shooterLoader.Set(Relay::kForward);
			}
			floorPickup.Set(Relay::kOff);
			shooterLoader.Set(Relay::kOff);
			ArmToPosition(p);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", tension.Get());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d", shooter.Get(), top.Get(), middle.Get());
			dsLCD->UpdateLCD();
			release.Set(Relay::kReverse);
			Wait(wait);
			release.Set(Relay::kOff);
			ArmToPositionFull(0);
		}
		targeting.Stop();
		printf("Autonomous: stop\n");
	}
	
	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		printf("OperatorControl: start\n");
		armSet = false;
		targeting.Start();
		//tension.Reset();
		//tension.Start();
		sparky.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
		{
			// drive
			if(stick1.GetTrigger() && !stick2.GetTrigger())
			{
				sparky.ArcadeDrive(stick1);
			}
			else if(stick1.GetTrigger() && stick2.GetTrigger())
			{
				sparky.TankDrive(stick1, stick2);
			}
			else
			{
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
			
			// shooter arm
			if(!armSet)
			{
				// coarse adjustment
				if(stick3.GetRawButton(3))
				{
					if(tension.Get() > 0)
					{
						arm.Set(ARM_SPEED_COARSE_UNLOAD);
					}
				}
				else if(stick3.GetRawButton(4) && shooter.Get())
				{
					arm.Set(ARM_SPEED_COARSE_LOAD);
				}
				// fine adjustment
				else if(stick3.GetRawButton(6) && shooter.Get())
				{
					arm.Set(ARM_SPEED_FINE_LOAD);
				}
				else if(stick3.GetRawButton(5))
				{
					if(tension.Get() > 0)
					{
						arm.Set(ARM_SPEED_FINE_UNLOAD);
					}
				}
				// move to preset
				else if(stick3.GetRawButton(7))
				{
					/*
					encPos = 220;
					armSet = true;
					Notifier n(ArmToPositionNotifier, this);
					n.StartSingle(0);
					*/
					ArmToPosition(220);
				}
				else if(stick3.GetRawButton(2))
				{
					/*
					encPos = 0;
					armSet = true;
					Notifier n(ArmToPositionNotifier, this);
					n.StartSingle(0);
					*/
					ArmToPosition(0);
				}
				else if(stick3.GetRawButton(9))
				{
					/*
					encPos = 265;
					armSet = true;
					Notifier n(ArmToPositionNotifier, this);
					n.StartSingle(0);
					*/
					ArmToPosition(265);
				}
				else
				{
					arm.Set(TENSION_BRAKE); // brake spool
				}
			}
			
			// ball loading
			if(stick3.GetRawButton(12))
			{
				if(shooter.Get() && top.Get() && middle.Get())
				{
					floorPickup.Set(Relay::kOff);
				}
				else
				{
					floorPickup.Set(Relay::kForward);
				}
				if(!shooter.Get())
				{
					shooterLoader.Set(Relay::kForward);
				}
				else if(!top.Get() && shooter.Get())
				{
					shooterLoader.Set(Relay::kForward);
				}
				else
				{
					shooterLoader.Set(Relay::kOff);
				}
			}
			else if(stick3.GetRawButton(11))
			{
				floorPickup.Set(Relay::kReverse);
				shooterLoader.Set(Relay::kReverse);
			}
			else
			{
				floorPickup.Set(Relay::kOff);
				shooterLoader.Set(Relay::kOff);
			}
			
			// release
			if(stick3.GetTrigger())
			{
				release.Set(Relay::kReverse);
			}
			else
			{
				release.Set(Relay::kOff);
			}
			
			//dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "volts: %f", ds->GetBatteryVoltage());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "encoder: %d", tension.Get());
			//dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Raw: %d", tension.GetRaw());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "s: %d, t: %d, m: %d", shooter.Get(), top.Get(), middle.Get());
			dsLCD->UpdateLCD();
			
			Wait(0.005);				// wait for a motor update time
		}
		targeting.Stop();
		printf("OperatorControl: stop\n");
	}
	
	/**
	 * 
	 */
	static int Targeting(void)
	{
		printf("Targeting: start\n");
		vector<Threshold> thresholds;
		thresholds.push_back(Threshold(0, 158, 123, 255, 0, 160)); // night
		thresholds.push_back(Threshold(107, 189, 150, 255, 68, 167)); // day
		thresholds.push_back(Threshold(78, 210, 184, 255, 0, 190)); // day close
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
		
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
		dsLCD->UpdateLCD();

		while(true) {
			bool found = false;
			image = new RGBImage();
			camera.GetImage(image);
						
			// loop through our threshold values
			for(unsigned i = 0; i < thresholds.size() && !found; i++)
			{
				ParticleAnalysisReport *target = NULL;
				BinaryImage *thresholdImage = image->ThresholdRGB(thresholds.at(i));
				BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
				BinaryImage *bigObjectsImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
				BinaryImage *filteredImage = bigObjectsImage->RemoveSmallObjects(false, 2);  // remove small objects (noise)
				vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  // get the results
				
				// loop through the reports
				for (unsigned j = 0; j < reports->size(); j++) {
					ParticleAnalysisReport *r = &(reports->at(j));
					double fov = (double)(tapeWidth * (double)r->imageWidth) / (double)r->boundingRect.width;
					double distance = (double)(fov / (double)2) / tan(degs * rads);
					double fovVert = (double)(tapeHeight * (double)r->imageHeight) / (double)r->boundingRect.height;
					double distanceVert = (double)(fovVert / (double)2) / tan(degsVert * rads);
					// get the topmost basket
					//if(!target || target->center_mass_y < r->center_mass_y) {
					if(!target || target->center_mass_y > r->center_mass_y) {

						target = r;
						d = distance;
						dv = distanceVert;
						centerMassX = target->center_mass_x;
						printf("*** Top Basket ***\n");
					}
					found = true;
					
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
				}
				
				if(!reports->size())
				{
					printf("No particles found.\n");
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
				if(lastDist == dv)
				{
					distCount++;
				}
				else if(lastDist < dv && dv - lastDist < 1)
				{
					distCount++;	
				}
				else if(lastDist > dv && lastDist - dv < 1)
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
			if(distCount > 3) {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "target: %f", dv);
				if(centerMassX == centerWidth ||
				   (centerMassX > centerWidth && centerMassX - centerWidth < centerThresh) ||
				   (centerMassX < centerWidth && centerWidth - centerMassX < centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s (%d px)", "CENTER",
							centerMassX > centerWidth ? centerMassX - centerWidth : centerWidth - centerMassX);
				}
				else if((centerMassX > centerWidth && centerMassX - centerWidth > centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s", "RIGHT");
				}
				else if ((centerMassX < centerWidth && centerWidth - centerMassX > centerThresh))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s", "LEFT");
				}
				dsLCD->UpdateLCD();
			}
			else {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "*** NO TARGET ***");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
				dsLCD->UpdateLCD();
			}
			
			dv = 0;
			
			delete image;
			Wait(0.1);
		}
		printf("Targeting: stop\n");
		return 1;
			
		/* orignal 
		BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 2);  // remove small objects (noise)
		BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);  // fill in partial and full rectangles
		BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
		*/

		// extra crispy
		/*
		BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
		BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
		*/
		// experimental

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
	
	void ArmToPositionFull(int p)
	{
		if(tension.Get() < p && shooter.Get())
		{
			while(tension.Get() < p)
			{
				arm.Set(-1.0);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
			}
		}
		else if(tension.Get() > p)
		{
			while(tension.Get() > p)
			{
				arm.Set(1.0);
				sparky.TankDrive(MOTOR_OFF, MOTOR_OFF);
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
	
	static void ArmToPositionNotifier(void* p)
	{
		Sparky *s = (Sparky *)p;
		Encoder *t = s->GetTension();
		Jaguar *a = s->GetArm();
		DigitalInput *shoot = s->GetShooter();
		{
			Synchronized sync(armSem);
			if(s->tension.Get() < encPos && shoot->Get())
			{
				while(t->Get() < encPos)
				{
					a->Set(ARM_SPEED_COARSE_LOAD);
				}
			}
			else if(t->Get() > encPos)
			{
				while(t->Get() > encPos)
				{
					a->Set(ARM_SPEED_COARSE_UNLOAD);
				}
			}
			a->Set(TENSION_BRAKE);
		}
		armSet = false;
	}
};

START_ROBOT_CLASS(Sparky);
