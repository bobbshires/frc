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
	Joystick stick; // only joystick

public:
	Sparky(void):
		myRobot(1, 2),
		stick(1)
	{
		myRobot.SetExpiration(0.1);
		myRobot.SetSafetyEnabled(false);
		camera.WriteResolution(AxisCameraParams::kResolution_640x480);
		camera.WriteBrightness(0);
		camera.WriteWhiteBalance(AxisCameraParams::kWhiteBalance_Automatic);
		Wait(3);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{	
		printf("Autonomous: start\n");
		int count = 0;
		Task targeting("targeting", (FUNCPTR)Targeting);
		targeting.Start();
		while (IsAutonomous() && IsEnabled()) {
			printf("count: %d\n", count++);
			Wait(1);
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
		Task targeting("targeting", (FUNCPTR)Targeting);
		targeting.Start();
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			if (stick.GetTrigger()) {
				myRobot.ArcadeDrive(stick);
			}
			else {
				myRobot.Drive(0, 0);
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
		//Threshold greenThreshold(0, 7, 0, 255, 86, 169);
		//Threshold greenThreshold(0, 10, 0, 255, 31, 110);
		Threshold greenThreshold(0, 10, 56, 255, 0, 29);
		ParticleFilterCriteria2 criteria[] = {
											{IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false, false},
											{IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false, false}
		};
		//double degs = 27;
		double degs = 24;
		double degsVert = 20;
		double pi = 3.141592653589;
		double rads = pi / (double)180;
		double tapeWidth = 2;
		double tapeHeight = 1.5;
		ColorImage *image;
		
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
		dsLCD->UpdateLCD();

		while(true) {
			ParticleAnalysisReport *target = NULL;
			double d, dv;
			image = new RGBImage();
			camera.GetImage(image);
			//image->Write("test-384.jpg");
	
			//BinaryImage *thresholdImage = image->ThresholdRGB(redThreshold);
			BinaryImage *thresholdImage = image->ThresholdRGB(greenThreshold);	// get just the red target pixels
			//thresholdImage->Write("test-384-thresh.bmp");
			BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 2);  // remove small objects (noise)
			BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);  // fill in partial and full rectangles
			BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
			vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  // get the results
			//filteredImage->Write("test-384-filtered.bmp");
					
			for (unsigned i = 0; i < reports->size(); i++) {
				ParticleAnalysisReport *r = &(reports->at(i));
				double fov = (double)(tapeWidth * (double)r->imageWidth) / (double)r->boundingRect.width;
				double distance = (double)(fov / (double)2) / tan(degs * rads);
				double fovVert = (double)(tapeHeight * (double)r->imageHeight) / (double)r->boundingRect.height;
				double distanceVert = (double)(fovVert / (double)2) / tan(degsVert * rads);
				// get the topmost basket
				if(!target || target->center_mass_y > r->center_mass_y) {
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
			
			if(target) {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "");
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "distance: %f", d);
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "distanceVert: %f", dv);
				dsLCD->UpdateLCD();
			}
			else {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "*** NO TARGET ***");
				dsLCD->UpdateLCD();
			}
					
			if(!reports->size()){
				printf("No particles found.\n");
			}
			
			// be sure to delete images after using them
			delete reports;
			delete filteredImage;
			delete convexHullImage;
			delete bigObjectsImage;
			delete thresholdImage;
			delete image;		
			
			Wait(0.01);
		}
		printf("Targeting: stop\n");
		return 0;
	}
};

START_ROBOT_CLASS(Sparky);
