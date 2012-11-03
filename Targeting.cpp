#include "Targeting.h"

Targeting::Targeting()
{
	targetDistance = 0;
	targetAlign = TARGET_NONE;
	Wait(5);
	/*
	camera = &AxisCamera::GetInstance("10.3.84.11");
	camera->WriteResolution(AxisCameraParams::kResolution_640x480);
	*/
	camera = &AxisCamera::GetInstance("10.3.84.12");
	camera->WriteResolution(AxisCameraParams::kResolution_320x240);
	camera->WriteWhiteBalance(AxisCameraParams::kWhiteBalance_Hold);
	camera->WriteExposureControl(AxisCameraParams::kExposure_Hold);
	camera->WriteColorLevel(100);
    camera->WriteCompression(30);
	camera->WriteBrightness(30);
	camera->WriteMaxFPS(10);
	Wait(5);
}

/**
 * Task to handle targeting with the webcam.  Displays distance and target offset.
 */
int Targeting::VisionTracking()
{
	printf("VisionTracking: start\n");
	vector<Threshold> thresholds;
	thresholds.push_back(Threshold(141, 253, 103, 253, 72, 255)); // LED flashlight
	//thresholds.push_back(Threshold(126, 224, 210, 255, 0, 138));  // field
	//thresholds.push_back(Threshold(0, 177, 165, 255, 0, 141));    // practice field
	//thresholds.push_back(Threshold(0, 158, 123, 255, 0, 160)); // night
	//thresholds.push_back(Threshold(107, 189, 150, 255, 68, 167)); // day
	//thresholds.push_back(Threshold(78, 210, 184, 255, 0, 190)); // day close
	ParticleFilterCriteria2 criteria[] = {
		{IMAQ_MT_BOUNDING_RECT_WIDTH, 10, 400, false, false},
		{IMAQ_MT_BOUNDING_RECT_HEIGHT, 10, 400, false, false}
	};
	double degsVert = 20;
	double pi = 3.141592653589;
	double rads = pi / (double)180;
	double tapeHeight = 1.5;
	ColorImage *image = NULL;
	double fovVert, dv = 0;
	double lastDist = 0;
	double distCount = 0;
	int centerMassX;
	int centerWidth = 320 / 2;
	int centerThresh = 20;
	bool found = false;
	ParticleAnalysisReport *target = NULL;
	BinaryImage *thresholdImage = NULL;
	BinaryImage *convexHullImage = NULL;
	BinaryImage *bigObjectsImage = NULL;
	BinaryImage *filteredImage = NULL;
	vector<ParticleAnalysisReport> *reports = NULL;
	ParticleAnalysisReport *r = NULL;
	bool imageError = false;
	unsigned i, j;
	
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
		
		if(image->GetWidth() == 0 || image->GetHeight() == 0)
		{
			printf("Image width or height is 0.\n");
			delete image;
			Wait(1.0);
			continue;
		}
					
		// loop through our threshold values
		for(i = 0; i < thresholds.size() && !found; i++)
		{
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
			for (j = 0; reports && j < reports->size(); j++)
			{
				r = &(reports->at(j));

				// get the bottom-most basket
				if(!target || target->center_mass_y < r->center_mass_y)
				{
					fovVert = (double)(tapeHeight * (double)r->imageHeight) / (double)r->boundingRect.height;
					dv = (double)(fovVert / (double)2) / tan(degsVert * rads);
					target = r;
					centerMassX = target->center_mass_x;
				}
				found = true;
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
			filteredImage = NULL;
			convexHullImage = NULL;
			bigObjectsImage = NULL;
			thresholdImage = NULL;
			reports = NULL;
			target = NULL;
			imageError = false;
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
				targetAlign = TARGET_CENTER;
			}
			else if((centerMassX > centerWidth && centerMassX - centerWidth > centerThresh))
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s", "RIGHT");
				targetAlign = TARGET_RIGHT;
			}
			else if ((centerMassX < centerWidth && centerWidth - centerMassX > centerThresh))
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "align: %s", "LEFT");
				targetAlign = TARGET_LEFT;
			}
		}
		else
		{
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "*** NO TARGET ***");
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
			targetAlign = TARGET_NONE;
		}
		dsLCD->UpdateLCD();
		
		targetDistance = dv;
		dv = 0;
		
		delete image;
		Wait(0.2);
	}
	printf("VisionTracking: stop\n");
	
	return 0;
}

double Targeting::getTargetDistance()
{
	return targetDistance;
}

targetAlignment Targeting::getTargetAlign()
{
	return targetAlign;
}
