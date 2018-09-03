#pragma once

//#include "LowPassFilter2p.hpp"


class InertialSensor
{

public:
	InertialSensor():
	_ready(false)
	{}

	~InertialSensor() {}

	virtual bool init(void)=0;
	virtual void read()=0;

    void get_accel(float *acc) { acc[0] = _acc[0]; acc[1] = _acc[1]; acc[2] = _acc[2];}
    void get_gyro(float *gyro) {gyro[0] = _gyro[0]; gyro[1] = _gyro[1]; gyro[2] = _gyro[2];}

    float get_acc_x() {return _acc[0];}
    float get_acc_y() {return _acc[1];}
    float get_acc_z() {return _acc[2];}

    float get_gyro_x() {return _gyro[0];}
    float get_gyro_y() {return _gyro[1];}
    float get_gyro_z() {return _gyro[2];}

    void set_accel(float *acc) { _acc[0] = acc[0]; _acc[1] = acc[1]; _acc[2] = acc[2];}
    void set_gyro(float *gyro) {_gyro[0] = gyro[0]; _gyro[1] = gyro[1]; _gyro[2] = gyro[2];}

    void set_acc_x(float acc) {_acc[0] = acc;}
    void set_acc_y(float acc) {_acc[1] = acc;}
    void set_acc_z(float acc) {_acc[2] = acc;}

    void set_gyro_x(float gyro) {_gyro[0] = gyro;}
    void set_gyro_y(float gyro) {_gyro[1] = gyro;}
    void set_gyro_z(float gyro) {_gyro[2] = gyro;}

    bool ready() { return _ready;}
    void set_ready() {_ready=true;}


protected:
    float _acc[3];
    float _gyro[3];

//	math::LowPassFilter2p	_accel_filter_x;
//	math::LowPassFilter2p	_accel_filter_y;
//	math::LowPassFilter2p	_accel_filter_z;
//	math::LowPassFilter2p	_gyro_filter_x;
//	math::LowPassFilter2p	_gyro_filter_y;
//	math::LowPassFilter2p	_gyro_filter_z;

private:
    bool _ready;

};
