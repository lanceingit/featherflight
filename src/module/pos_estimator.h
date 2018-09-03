#pragma once

#include "estimator.h"

class Pos_Estimator : virtual public Estimator
{

public:
	Pos_Estimator(SENSORS* sensor):
		Estimator(sensor)
	{}
	~Pos_Estimator() {}

  	virtual bool init(){return true;}
  	virtual void run(){}


protected:

private:

};
