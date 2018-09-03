#pragma once

//using namespace math;
#include "inertialSensor.h"

class Hil_inertialSensor : public InertialSensor
{
public:
	Hil_inertialSensor() {}

	~Hil_inertialSensor() {}

    bool init(void) {return true;}
    void read() {}


private:
    
};

