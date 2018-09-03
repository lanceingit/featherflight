#pragma once

#include "mavlink.h"


class LINK_MAVLINK
{
public:
    LINK_MAVLINK(){}
    ~LINK_MAVLINK(){}

    void init();
    void msg_send(mavlink_message_t &msg);
    void updata();

    char* get_text_buf() { return textbuf; }
    bool text_need_send() { return _text_cnt>0? true: false; }

    void send_text(const char* s);

private:
    uint8_t sendbuf[300];
    mavlink_status_t _r_mavlink_status;
    char textbuf[50];
    uint8_t _text_cnt;

};
