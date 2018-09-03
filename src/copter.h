#pragma once

#include "sensors.h"
#include "estimator.h"
#include "link_mavlink.h"



class COPTER
{
    
public:

	COPTER(SENSORS* _sensors,
			Estimator* _estimator
			):
	   sensors(_sensors),
	   estimator(_estimator)
	{}

//	HAL hal;

	SENSORS* sensors;

	Estimator* estimator;
//
//	ATT_CONTROL att_control;
//
//	POS_CONTROL pos_control;
//
//	NAVIGATOR navgator;
//
//	MOTORS motos;
//
//	RC rc;
//
//	BATTERY battery;
//
	LINK_MAVLINK link_mavlink;
//
//	PARAM param;
//
//	LOG log;


private:    
    
};


extern COPTER copter;

