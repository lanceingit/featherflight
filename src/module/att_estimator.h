#pragma once

#include "estimator.h"

class Att_Estimator : virtual public Estimator
{

public:
	Att_Estimator(SENSORS* sensor):
		Estimator(sensor)
	{}

	~Att_Estimator() {}

  	virtual bool init(){return true;}
  	virtual void run(){}

protected:

private:

};
