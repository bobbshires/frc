#include "BridgeArm.h"

BridgeArm::BridgeArm():
	bridgeArmUp(3),
	bridgeArmDown(4),
	bridgeArm(7)
{
	armUp = false;
	armDown = false;
}

void BridgeArm::up()
{
	bridgeArm.Set(BRIDGE_ARM_UP);
}

void BridgeArm::down()
{
	bridgeArm.Set(BRIDGE_ARM_DOWN);
}

void BridgeArm::off()
{
	bridgeArm.Set(BRIDGE_ARM_OFF);
}

bool BridgeArm::isArmUp()
{
	return armUp;
}

bool BridgeArm::isArmDown()
{
	return armDown;
}

void BridgeArm::setArmUp(bool b)
{
	armUp = b;
}

void BridgeArm::setArmDown(bool b)
{
	armDown = b;
}

int BridgeArm::getBridgeArmUp()
{
	return bridgeArmUp.Get();
}

int BridgeArm::getBridgeArmDown()
{
	return bridgeArmDown.Get();
}
