#ifndef SPARKY_H
#define SPARKY_H

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"
#include "Targeting.h"
#include "Loader.h"

// encoder
static SEM_ID armSem;
static int encPos;
static bool armSet;
static double armSpeed;

// trigger release
static SEM_ID releaseSem;
static bool releaseSet;

// constants
static const double MOTOR_OFF = 0.0;
static const double TENSION_BRAKE = -0.06;
static const double ARM_SPEED_COARSE = 0.5;
static const double ARM_SPEED_COARSE_LOAD = -0.5;
static const double ARM_SPEED_COARSE_UNLOAD = 0.5;
static const double ARM_SPEED_FINE_LOAD = -0.3;
static const double ARM_SPEED_FINE_UNLOAD = 0.2;
static const double ARM_SPEED_FULL_LOAD = -1.0;
static const double ARM_SPEED_FULL_UNLOAD = 1.0;
static const double BRIDGE_ARM_DOWN = 0.9;
static const double BRIDGE_ARM_UP = -0.9;
static const double BRIDGE_ARM_OFF = 0.0;
static const double AUTO_AIM_SPEED = 0.2;

#endif
