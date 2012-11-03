#ifndef SPARKY_H
#define SPARKY_H

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"

static AxisCamera *camera;

// lights
static Relay *g_lights;
static DigitalInput *g_top;
static DigitalInput *g_middle;
static DigitalInput *g_shooter;

// encoder
static SEM_ID armSem;
static int encPos;
static bool armSet;
static double armSpeed;

// trigger release
static SEM_ID releaseSem;
static bool releaseSet;
static bool intakeOff;

// auto aim
static SEM_ID autoAimSem;
static bool g_autoAimSet;
static double g_targetDistance;
typedef enum {TARGET_LEFT, TARGET_RIGHT, TARGET_CENTER, TARGET_NONE} targetAlignment;
static targetAlignment g_targetAlign;

#endif
