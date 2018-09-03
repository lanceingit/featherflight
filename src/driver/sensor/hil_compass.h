#pragma once

#include "compass.h"

class Hil_compass : public Compass
{
public:
	Hil_compass() {}
	~Hil_compass() {}

    bool init() {return true;}
    void read() {}

};
