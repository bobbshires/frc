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
	RobotDrive myRobot; // robot drive system
	Joystick stick1;
	Joystick stick2;
	Task targeting;
	Jaguar conveyor;

public:
	Sparky(void):
		myRobot(1, 2),
		stick1(1),
		stick2(2),
		targeting("targeting", (FUNCPTR)Targeting),
		conveyor(3) // Not a definite port, just putting in pre-code
	{
		printf("Sparky: start\n");
		myRobot.SetExpiration(0.1);
		myRobot.SetSafetyEnabled(false);
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
		targeting.Start();
		while (IsAutonomous() && IsEnabled()) {
			if(count % 10 == 0) {
				printf("count: %d\n", count);
			}
			myRobot.Drive(0, 0);
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
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
		{
			if(stick2.GetTrigger() && !stick1.GetTrigger())
			{
				myRobot.ArcadeDrive(stick1);
			}
			else if(stick1.GetTrigger() && stick2.GetTrigger())
			{
				myRobot.TankDrive(stick1, stick2);
			}
			else if(stick1.GetRawButton(2))
			{
				conveyor.SetSpeed(1);
			}
			else
			{
				myRobot.Drive(0, 0);
				conveyor.SetSpeed(0);
			}
			
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
		//Threshold greenThreshold(0, 52, 60, 255, 0, 88);  // 6ft
		//Threshold greenThreshold(0, 78, 81, 255, 5, 136); // 30ft
		//Threshold greenThreshold(21, 78, 76, 255, 0, 88);
		Threshold greenThreshold(0, 158, 123, 255, 0, 160);
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
		ColorImage *image;
		int loopCount = 0;
		ParticleAnalysisReport *target = NULL;
		double d, dv;
		double lastDist = 0;
		double distCount = 0;
		
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
		dsLCD->UpdateLCD();

		while(true) {
			image = new RGBImage();
			camera.GetImage(image);
			
			BinaryImage *thresholdImage = image->ThresholdRGB(greenThreshold);	// get just the red target pixels
			/*
			BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 2);  // remove small objects (noise)
			BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);  // fill in partial and full rectangles
			*/
			BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
			BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
			///*
			if(!loopCount && camera.IsFreshImage()) {
				image->Write("sparky.jpg");
				thresholdImage->Write("sparky-Thresh.bmp");
				//bigObjectsImage->Write("sparky-bigObjects.bmp");
				convexHullImage->Write("sparky-convexHull.bmp");
				filteredImage->Write("sparky-filtered.bmp");
				loopCount++;
			}
			//*/
			vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  // get the results
					
			for (unsigned i = 0; i < reports->size(); i++) {
				ParticleAnalysisReport *r = &(reports->at(i));
				double fov = (double)(tapeWidth * (double)r->imageWidth) / (double)r->boundingRect.width;
				double distance = (double)(fov / (double)2) / tan(degs * rads);
				double fovVert = (double)(tapeHeight * (double)r->imageHeight) / (double)r->boundingRect.height;
				double distanceVert = (double)(fovVert / (double)2) / tan(degsVert * rads);
				// get the topmost basket
				if(!target || target->center_mass_y < r->center_mass_y) {
					target = r;
					d = distance;
					dv = distanceVert;
					printf("*** Top Basket ***\n");
				}
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
			printf("\n");
			
			if(!distCount)
			{
				distCount++;
				lastDist = dv;
			}
			else
			{	
				if(lastDist == dv)
				{
					distCount++;
				}
				else if(lastDist < dv)
				{
					if(dv - lastDist < 1)
					{
						distCount++;
					}	
				}
				else if(lastDist > dv)
				{
					if(lastDist - dv < 1)
					{
						distCount++;
					}	
				}
				else
				{
					distCount = 0;
				}
				
				lastDist = dv;
			}
			
			if(distCount > 3) {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "distance: %f", d);
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "distanceVert: %f", dv);
				dsLCD->UpdateLCD();
			}
			else {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "*** TARGET NOT FOUND ***");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
				dsLCD->UpdateLCD();
			}
					
			if(!reports->size()){
				printf("No particles found.\n");
			}
			
			// be sure to delete images after using them
			delete reports;
			delete filteredImage;
			delete convexHullImage;
			//delete bigObjectsImage;
			delete thresholdImage;
			delete image;
			
			target = NULL;
			dv = 0;
			
			Wait(0.1);
		}
		printf("Targeting: stop\n");
		return 1;
	}
};

START_ROBOT_CLASS(Sparky);
