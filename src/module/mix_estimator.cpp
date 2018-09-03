#include "mix_estimator.h"

//#define LINK_DEBUG(a) _link->send_text(a)
#define LINK_DEBUG(a)

bool  Mix_estimator::init()
{
	bool ret = _att_Estimator->init();
	LINK_DEBUG("[mix]init");

	return ret;
}


void Mix_estimator::run()
{
//	LINK_DEBUG("[mix]run");
	_att_Estimator->run();

    _roll = _att_Estimator->get_roll();
    _pitch = _att_Estimator->get_pitch();
    _yaw = _att_Estimator->get_yaw();

    _att_Estimator->get_q(_q);
}

