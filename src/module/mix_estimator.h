#pragma once

#include "att_estimator.h"
#include "pos_estimator.h"
#include "link_mavlink.h"

class Mix_estimator : public Att_Estimator, public Pos_Estimator
{
public:
	Mix_estimator(SENSORS* sensor, Att_Estimator* att_Estimator, Pos_Estimator* pos_Estimator):
		Estimator(sensor),
		Att_Estimator(sensor),
		Pos_Estimator(sensor),
		_att_Estimator(att_Estimator),
		_pos_Estimator(pos_Estimator)
	{}

	~Mix_estimator() {}

	virtual bool init();
	virtual void run();

private:
	Att_Estimator* _att_Estimator;
	Pos_Estimator* _pos_Estimator;
};

