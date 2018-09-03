#include "mavlink_func.h"
#include "debug.h"




MAVLINK_FUNC::MAVLINK_FUNC()
{

}


MAVLINK_FUNC::~MAVLINK_FUNC()
{

}

    
void MAVLINK_FUNC::init()
{

}


void MAVLINK_FUNC::send()
{

}

void MAVLINK_FUNC::recv()
{

}


void MAVLINK_FUNC::updata()
{
    uint8_t c;
    mavlink_message_t msg={};

    
    if(!usb_IsEmpty())
    {
        usb_get(&c);
        if(mavlink_parse_char(0, c, &msg, &_r_mavlink_status))
        {
            DEBUG_PRINTF("msg id:%d  %x %x %x %x \r\n", msg.msgid, msg.payload64[0], msg.payload64[1], msg.payload64[2], msg.payload64[3]);
            switch(msg.msgid)
            {
                case MAVLINK_MSG_ID_COMMAND_LONG:
				{
                    uint16_t cmd = mavlink_msg_command_long_get_command(&msg);
                    DEBUG_PRINTF("cmd:%d \r\n", mavlink_msg_command_long_get_command(&msg));
                    if(cmd == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
                    {
                        NVIC_SystemReset();
                    }
                    break;
				}
                default:break;
            }
        }
    }
}

