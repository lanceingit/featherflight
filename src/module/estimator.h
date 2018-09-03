#pragma once

#include "sensors.h"
#include "mathlib.h"

class Estimator
{

public:
	Estimator(SENSORS* sensor) :
		_sensors(sensor)
	{}

	~Estimator() {}

	virtual bool init() {return true;}
	virtual void run() {}

    float get_roll() { return _roll; }
    float get_pitch() { return _pitch; }
    float get_yaw() { return _yaw; }

    void get_q(math::Quaternion &q) { q = _q; }



protected:
	SENSORS* _sensors;

    float _roll;
    float _pitch;
    float _yaw;

    float _roll_rate;
    float _pitch_rate;
    float _yaw_rate;

    math::Quaternion	_q;

	float x;		//N
	float y;		//E
	float z;		//D
	float vx;
	float vy;
	float vz;

	float eph;
	float epv;
	bool xy_valid;
	bool z_valid;

	double lat;
	double lon;
	float alt;
	float terrain_alt;

	double ref_lat;
	double ref_lon;
	float ref_alt;

private:

};
