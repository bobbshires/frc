#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"

static AxisCamera &camera = AxisCamera::GetInstance("10.3.84.11");
 
/**
 * Sparky class.  Describes the 2012 FRC robot.
 */
class Sparky : public SimpleRobot
{
	RobotDrive sparky; // robot drive system
	Joystick stick1, stick2;
	Task targeting;
	DigitalInput top, middle, bottom;
	DriverStation *ds;
	Jaguar arm;
	Relay floorPickup, shooterLoader;//, reserved;

public:
	Sparky(void):
		sparky(3, 2),
		stick1(1),
		stick2(2),
		targeting("targeting", (FUNCPTR)Targeting),
		top(14),
		middle(13),
		bottom(3), // Not used yet.
		ds(DriverStation::GetInstance()),
		arm(1),
		floorPickup(7),
		shooterLoader(8)/*,
		reserved(10) // not used yet
		*/
	{
		printf("Sparky: start\n");
		sparky.SetExpiration(0.1);
		sparky.SetSafetyEnabled(false);
		sparky.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		//sparky.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
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
		int count = 0;
		sparky.SetSafetyEnabled(false);
		targeting.Start();
		while (IsAutonomous() && IsEnabled()) {
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
			
			if(count % 100 == 0) {
				printf("count: %d\n", count);
			}
			//sparky.Drive(0, 0);
			//sparky.TankDrive(-1.0, 1.0);
			count++;

			Wait(0.01);
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
		targeting.Start();
		sparky.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
		{
			if(stick2.GetTrigger() && !stick1.GetTrigger())
			{
				sparky.ArcadeDrive(stick2);
			}
			else if(stick1.GetTrigger() && stick2.GetTrigger())
			{
				sparky.TankDrive(stick1, stick2);
			}
			else
			{
				sparky.TankDrive(0.0, 0.0);
			}
			
			if(stick1.GetRawButton(3))
			{
				arm.Set(0.5);
			}
			else if(stick1.GetRawButton(4))
			{
				arm.Set(-0.5);
			}
			else
			{
				arm.Set(-0.05);
			}
			
			if(stick2.GetRawButton(2))
			{
				floorPickup.Set(Relay::kForward);
				shooterLoader.Set(Relay::kForward);
			}
			else if(stick2.GetRawButton(3))
			{
				floorPickup.Set(Relay::kReverse);
				shooterLoader.Set(Relay::kReverse);
			}
			else
			{
				floorPickup.Set(Relay::kOff);
				shooterLoader.Set(Relay::kOff);
			}
			
			/*
			printf("top: %d\n", top.Get());
			printf("middle: %d\n", middle.Get());
			printf("bottom: %d\n", bottom.Get());
			*/
			
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
		//int loopCount = 0;
		double d, dv;
		double lastDist = 0;
		double distCount = 0;
		int centerMassX;
		
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
					if(!target || target->center_mass_y < r->center_mass_y) {
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
				if(centerMassX == 320 ||
				   (centerMassX > 320 && centerMassX - 320 < 40) ||
				   (centerMassX < 320 && 320 - centerMassX < 40))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "alignment: %s", "CENTER");
				}
				else if((centerMassX > 320 && centerMassX - 320 > 40))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "alignment: %s", "RIGHT");
				}
				else if ((centerMassX < 320 && 320 - centerMassX > 40))
				{
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "alignment: %s", "LEFT");
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
};

START_ROBOT_CLASS(Sparky);
