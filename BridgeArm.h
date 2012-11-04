#ifndef BRIDGE_ARM_H
#define BRIDGE_ARM_H

#include "WPILib.h"

static const double BRIDGE_ARM_DOWN = 0.9;
static const double BRIDGE_ARM_UP = -0.9;
static const double BRIDGE_ARM_OFF = 0.0;

class BridgeArm
{
private:
	DigitalInput bridgeArmUp, bridgeArmDown;
	Victor bridgeArm;
	bool armUp;
	bool armDown;
public:
	BridgeArm();
	void up();
	void down();
	void off();
	bool isArmUp();
	bool isArmDown();
	void setArmUp(bool b);
	void setArmDown(bool b);
	int getBridgeArmUp();
	int getBridgeArmDown();
};

#endif
