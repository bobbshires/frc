#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"

//#define AxisCamera camera;
 
/**
 * Sample program to use NIVision to find rectangles in the scene that are illuminated
 * by a red ring light (similar to the model from FIRSTChoice). The camera sensitivity
 * is set very low so as to only show light sources and remove any distracting parts
 * of the image.
 * 
 * The CriteriaCollection is the set of criteria that is used to filter the set of
 * rectangles that are detected. In this example we're looking for rectangles with
 * a minimum width of 30 pixels and maximum of 400 pixels. Similar for height (see
 * the addCriteria() methods below.
 * 
 * The algorithm first does a color threshold operation that only takes objects in the
 * scene that have a significant red color component. Then removes small objects that
 * might be caused by red reflection scattered from other parts of the scene. Then
 * a convex hull operation fills all the rectangle outlines (even the partially occluded
 * ones). Finally a particle filter looks for all the shapes that meet the requirements
 * specified in the criteria collection.
 * Look in the VisionImages directory inside the project that is created for the sample
 * images as well as the NI Vision Assistant file that contains the vision command
 * chain (open it with the Vision Assistant)
 */
class VisionSample2012 : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	AxisCamera& camera;

public:
	VisionSample2012(void):
		myRobot(1, 2),	// these must be initialized in the same order
		stick(1),		// as they are declared above.
		camera(AxisCamera::GetInstance("10.3.84.11"))
	{
		myRobot.SetExpiration(0.1);
		myRobot.SetSafetyEnabled(false);
		//camera.WriteResolution(AxisCameraParams::kResolution_320x240);
		camera.WriteBrightness(0);
		Wait(3);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{	
		Threshold redThreshold(25, 255, 0, 45, 0, 47);
		//Threshold greenThreshold(0, 7, 0, 255, 86, 169);
		Threshold greenThreshold(0, 10, 0, 255, 31, 110);
		ParticleFilterCriteria2 criteria[] = {
											{IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false, false},
											{IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false, false}
		};
		
		//DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		
		while (IsAutonomous() && IsEnabled()) {
            /**
             * Do the image capture with the camera and apply the algorithm described above. This
             * sample will either get images from the camera or from an image file stored in the top
             * level directory in the flash memory on the cRIO. The file name in this case is "10ft2.jpg"
             * 
             */
	
			//AxisCamera& camera = AxisCamera::GetInstance("10.3.84.11");
			
			/*
			ColorImage *test;
			camera.GetImage(test);
			dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Hello World");
		    dsLCD->UpdateLCD();
			test->Write("test-384.jpg");
			*/
			
			ColorImage *image;
			//image = new RGBImage("/10ft2.jpg");		// get the sample image from the cRIO flash
			//image = new RGBImage("/20ft2.jpg");
			//image = new RGBImage("/30ft2.jpg");
			//image = new ColorImage(IMAQ_IMAGE_RGB);
			image = new RGBImage();
			camera.GetImage(image);
			//image->Write("test-384.jpg");

			//BinaryImage *thresholdImage = image->ThresholdRGB(redThreshold);
			BinaryImage *thresholdImage = image->ThresholdRGB(greenThreshold);	// get just the red target pixels
			//thresholdImage->Write("test-384-thresh.bmp");
			BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 2);  // remove small objects (noise)
			BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);  // fill in partial and full rectangles
			BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
			//thresholdImage->Write("test-384-rect.bmp");
			vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  // get the results
			//filteredImage->Write("test-filtered.jpg");
			
			//double degs = 27;
			double degs = 24;
			double pi = 3.14159;
			double tapeWidth = 2;
			
			for (unsigned i = 0; i < reports->size(); i++) {
				ParticleAnalysisReport *r = &(reports->at(i));
				double fov = (double)(tapeWidth * (double)r->imageWidth) / (double)r->boundingRect.width;
				double distance = (double)(fov / (double)2) / tan(degs * (pi / (double)180));
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
				printf("distance: %f\n", distance);
				printf("tan: %f\n", tan(27*(3.14159 / 180)));
				printf("\n");
				/*
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "particle: %d  center_mass_x: %d\n", i, r->center_mass_x);
				dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "center_mass_x: %d\n", r->center_mass_x);
				dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "percent: %f\n", r->particleToImagePercent);
				dsLCD->UpdateLCD();
				*/
			}
			printf("\n");
			
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
			
			Wait(5);
		}
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
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
	}
};

START_ROBOT_CLASS(VisionSample2012);

