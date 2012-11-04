#include "Loader.h"

Loader::Loader():
	top(13), middle(14), shooter(12), floorPickup(5), shooterLoader(4)
{
	intakeOff = false;
}

void Loader::load(Timer& armTimer, int tension)
{
	if(shooter.Get() && top.Get() && middle.Get())
	{
		floorPickup.Set(INTAKE_OFF);
	}
	else if(top.Get() && middle.Get() && tension > ARM_ZERO_THRESH)
	{
		floorPickup.Set(INTAKE_OFF);
	}
	else
	{
		floorPickup.Set(INTAKE_LOAD);
	}
	if(!shooter.Get() && tension < ARM_ZERO_THRESH && armTimer.Get() > 1.0)
	{
		shooterLoader.Set(INTAKE_LOAD);
	}
	else if(!top.Get() && shooter.Get())

	{
		shooterLoader.Set(INTAKE_LOAD);
	}
	else if(!top.Get())
	{
		shooterLoader.Set(INTAKE_LOAD);
	}
	else
	{
		shooterLoader.Set(INTAKE_OFF);
	}
}

void Loader::load()
{
	shooterLoader.Set(INTAKE_LOAD);
}

void Loader::unload()
{
	floorPickup.Set(INTAKE_UNLOAD);
	shooterLoader.Set(INTAKE_UNLOAD);
}

void Loader::stop()
{
	floorPickup.Set(INTAKE_OFF);
	shooterLoader.Set(INTAKE_OFF);
}

bool Loader::isIntakeOff()
{
	return intakeOff;
}

void Loader::setIntakeOff(bool b)
{
	intakeOff = b;
}

UINT32 Loader::getTop()
{
	return top.Get();
}

UINT32 Loader::getMiddle()
{
	return middle.Get();
}

UINT32 Loader::getShooter()
{
	return shooter.Get();
}
