#ifndef TARGETING_H
#define TARGETING_H

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"

typedef enum {TARGET_LEFT, TARGET_RIGHT, TARGET_CENTER, TARGET_NONE} targetAlignment;

class Targeting
{
private:
	static AxisCamera *camera;
	static double targetDistance;
	static targetAlignment targetAlign;
public:
	Targeting();
	static int VisionTracking();
	double getTargetDistance();
	targetAlignment getTargetAlign();
};

#endif
