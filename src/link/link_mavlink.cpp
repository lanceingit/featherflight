#include "usbd_cdc_vcp.h"

#include "mavlink.h"
#include "link_mavlink.h"
#include "copter.h"


void LINK_MAVLINK::init()
{

//    USART_InitTypeDef USART_InitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//    NVIC_InitTypeDef NVIC_InitStruct;
//    
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

//    GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_10);   
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
//    
//    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
//    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//    
//    NVIC_Init(&NVIC_InitStruct);

//    USART_InitStructure.USART_BaudRate = 57600;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//    USART_DeInit(USART1);
//    USART_Init(USART1, &USART_InitStructure);
//    //USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
//    USART_ClearFlag (USART1,USART_FLAG_TC);
//    USART_Cmd(USART1, ENABLE);    
}    


//void LINK_MAVLINK::send(mavlink_message_t &msg)
//{
//    mavlink_message_t msg;
//    uint16_t len;
//    
//    mavlink_msg_heartbeat_pack(0, 0, &msg,
//                               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 
//                               MAV_MODE_FLAG_TEST_ENABLED, 0, MAV_STATE_STANDBY);
//    
//    len = mavlink_msg_to_send_buffer(sendbuf, &msg);
//    
//    for(uint16_t i=0; i<len; i++)
//    {
//        USART_SendData(USART1, sendbuf[i]);
//        while (!USART_GetFlagStatus(USART1,USART_FLAG_TC));
//    }
//    
//    //vTaskDelay(M2T(200));
//    
//    mavlink_msg_sys_status_pack(0, 0, &msg,
//                               0, 0, 0,10, 0, 0, 0, 0, 0, 0,0, 0, 0)    ;

//    len = mavlink_msg_to_send_buffer(sendbuf, &msg);
//    
//    for(uint16_t i=0; i<len; i++)
//    {
//        USART_SendData(USART1, sendbuf[i]);
//        while (!USART_GetFlagStatus(USART1,USART_FLAG_TC));
//    }    
//    
//}

void LINK_MAVLINK::msg_send(mavlink_message_t &msg)
{
    uint16_t len;
       
    len = mavlink_msg_to_send_buffer(sendbuf, &msg);
    
//    for(uint16_t i=0; i<len; i++)
//    {
//        USART_SendData(USART1, sendbuf[i]);
//        while (!USART_GetFlagStatus(USART1,USART_FLAG_TC));
//    }

    usb_write(sendbuf, len);
}

void LINK_MAVLINK::send_text(const char* s)
{
	mavlink_message_t msg;

	mavlink_msg_statustext_pack(1, 1, &msg,
							    0, s);
    msg_send(msg);
}


void LINK_MAVLINK::updata()
{
    uint8_t c;
    mavlink_message_t msg={};


    if(!usb_IsEmpty())
    {
        usb_get(&c);
    	//send_text("recv usb");
        if(mavlink_parse_char(0, c, &msg, &_r_mavlink_status))
        {
            //DEBUG_PRINTF("msg id:%d  %x %x %x %x \r\n", msg.msgid, msg.payload64[0], msg.payload64[1], msg.payload64[2], msg.payload64[3]);
//        	send_text("recv msg");
            switch(msg.msgid)
            {
                case MAVLINK_MSG_ID_COMMAND_LONG:
				{
                    uint16_t cmd = mavlink_msg_command_long_get_command(&msg);
                    //DEBUG_PRINTF("cmd:%d \r\n", mavlink_msg_command_long_get_command(&msg));
                    if(cmd == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
                    {
                        NVIC_SystemReset();
                    }
                    break;
				}

                case MAVLINK_MSG_ID_HIL_SENSOR:
                {
//                	send_text("recv hil");
                	if(!copter.sensors->inertialSensor->ready())
                	{
                		copter.sensors->inertialSensor->set_ready();
                	}

                	copter.sensors->inertialSensor->set_acc_x(mavlink_msg_hil_sensor_get_xacc(&msg));
                	copter.sensors->inertialSensor->set_acc_y(mavlink_msg_hil_sensor_get_yacc(&msg));
                	copter.sensors->inertialSensor->set_acc_z(mavlink_msg_hil_sensor_get_zacc(&msg));
                	copter.sensors->inertialSensor->set_gyro_x(mavlink_msg_hil_sensor_get_xgyro(&msg));
                	copter.sensors->inertialSensor->set_gyro_y(mavlink_msg_hil_sensor_get_ygyro(&msg));
                	copter.sensors->inertialSensor->set_gyro_z(mavlink_msg_hil_sensor_get_zgyro(&msg));

                	copter.sensors->compass->set_mag_x(mavlink_msg_hil_sensor_get_xmag(&msg));
                	copter.sensors->compass->set_mag_y(mavlink_msg_hil_sensor_get_ymag(&msg));
                	copter.sensors->compass->set_mag_z(mavlink_msg_hil_sensor_get_zmag(&msg));

                	break;

                }


                default:break;
            }
        }
    }
}


//
//extern "C" {
//
//void USART1_IRQHandler(void)
//{
//
//}
//
//}

