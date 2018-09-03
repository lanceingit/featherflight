#pragma once

#include "mavlink.h"

class MAVLINK_FUNC
{
public:
    MAVLINK_FUNC();
    ~MAVLINK_FUNC();
    
    void init();
    void send();
    void recv();
    void updata();

private:

    mavlink_status_t _r_mavlink_status;

};


