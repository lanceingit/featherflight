#pragma once

#include "att_estimator.h"
#include "mathlib.h"
#include "LowPassFilter2p.hpp"
#include "link_mavlink.h"


class Att_Est_Q : public Att_Estimator
{
public:
	Att_Est_Q(SENSORS* sensor, LINK_MAVLINK* link_mavlink);
	~Att_Est_Q();

	bool init();
	void run();

//	bool is_init() {return _inited;}

private:
	math::Vector<3>	_gyro;
	math::Vector<3>	_accel;
	math::Vector<3>	_mag;

	math::Vector<3>	_gyro_bias;
	math::Vector<3>	_rates;

	math::Vector<3> _pos_acc;

	math::LowPassFilter2p	_lp_accel_x;
	math::LowPassFilter2p	_lp_accel_y;
	math::LowPassFilter2p	_lp_accel_z;
	math::LowPassFilter2p	_lp_gyro_x;
	math::LowPassFilter2p	_lp_gyro_y;
	math::LowPassFilter2p	_lp_gyro_z;

	bool _use_compass;
	bool _mag_decl_auto;
	float _mag_decl;
	uint64_t _last_time;
	const float _dt_max;
	const float _bias_max;
	const float _w_accel;
	const float _w_mag;
	const float _w_gyro_bias;

	LINK_MAVLINK* _link;

	bool _inited;



	void update_mag_declination(float new_declination);
	bool update(float dt);
};

