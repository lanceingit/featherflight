#pragma once

#include "inertialSensor.h"
#include "baro.h"
#include "compass.h"

class SENSORS
{

public:
	SENSORS(InertialSensor* _inertialSensor, Compass* _compass
			)
			:
			inertialSensor(_inertialSensor),
			compass(_compass)
	{
	}
	~SENSORS(){}

	InertialSensor* inertialSensor;
//	Baro baro;
//	GPS gps;
	Compass* compass;

//	Vision vision;
//	Flow flow;
//    GroundDistance groundDistance;


private:

};
