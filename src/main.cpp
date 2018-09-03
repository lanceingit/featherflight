#include <stdbool.h>

#include "chip_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "copter.h"

#include "timer.h"
#include "eprintf.h"

#include "usbd_cdc_vcp.h"
#include "usbd_usr.h" 
#include "usbd_desc.h"
#include "usbd_core.h"

#include "mavlink_func.h"
#include "cli.h"
#include "hil_inertialSensor.h"
#include "hil_compass.h"
#include "mix_estimator.h"
#include "att_est_q.h"
#include "matrix/math.hpp"
#include "mathlib/mathlib.h"

USB_OTG_CORE_HANDLE    USB_OTG_dev;


#define SYSTEM_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
#define SYSTEM_TASK_PRI                3

#define LINK_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
#define LINK_TASK_PRI                5

#define LINK_RECV_TASK_STACKSIZE     (2* configMINIMAL_STACK_SIZE)
#define LINK_RECV_TASK_PRI           1

#define SENSOR_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
#define SENSOR_TASK_PRI                2

#define ATT_EST_TASK_STACKSIZE         (30* configMINIMAL_STACK_SIZE)
#define ATT_EST_TASK_PRI                1


static void systemTask(void *arg);
static void linkTask(void *arg);
static void linkRecvTask(void *arg);
//static void sensorTask(void *arg);
static void attEstTask(void *arg);

//static xSemaphoreHandle hil_sensor_ready;

//extern Att_Est_Q att_Est_Q;
//extern COPTER copter;
//extern SENSORS sensor;

MAVLINK_FUNC mavlink;

Hil_compass hil_compass;
Hil_inertialSensor hil_inertialSensor;
SENSORS sensor(&hil_inertialSensor, &hil_compass);

Att_Est_Q att_Est_Q(&sensor, &copter.link_mavlink);
Mix_estimator mix_estimator(&sensor, &att_Est_Q, nullptr);

COPTER copter(&sensor, &mix_estimator);


int main() 
{    
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    RCC_ClearFlag();
        
    Timer_init();
    mavlink.init();
    cli_init();
    USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
    usb_printf("system init ...\n");

    xTaskCreate(systemTask, (const char * const)"sys_task",
                SYSTEM_TASK_STACKSIZE, NULL,
                SYSTEM_TASK_PRI, NULL);  

    xTaskCreate(linkTask, (const char * const)"link_task",
                LINK_TASK_STACKSIZE, NULL,
                LINK_TASK_PRI, NULL);

//
//    xTaskCreate(sensorTask, (const char * const)"sensor_task",
//                SENSOR_TASK_STACKSIZE, NULL,
//                SENSOR_TASK_PRI, NULL);
//
    xTaskCreate(attEstTask, (const char * const)"att_est_task",
                ATT_EST_TASK_STACKSIZE, NULL,
                ATT_EST_TASK_PRI, NULL);

    
    //Start the FreeRTOS scheduler
    vTaskStartScheduler();


  //Should never reach this point!
  while(1)
  {
    //led_on();
    vTaskDelay(M2T(1000));
    
   // Timer::delayUs(1*1000*1000);

			
	//led_off();
    vTaskDelay(M2T(1000));
  //  Timer::delayUs(1*1000*1000);
	  
  }    

    return 0;
}

void systemTask(void *arg)
{


//    led_init();
//    led_off();

//	char buf[100];


  //Should never reach this point!
    while(1)
    {

//    	esprintf(buf, "[main]%d %x %d", 0, mix_estimator.get_attEst(), 1);
	    //copter.link_mavlink.send_text(buf);
        //led_on();
        //vTaskDelay(M2T(500));
        //Timer::delayUs(1*1000*1000);

        //led_off();
    	//cli_updata();
        vTaskDelay(M2T(10));
        //Timer::delayUs(1*1000*1000);
    }
}


void linkTask(void *arg)
{
    mavlink_message_t msg;
    uint8_t system_id=1;
    uint8_t component_id=1;

    copter.link_mavlink.init();

    xTaskCreate(linkRecvTask, (const char * const)"link_recv",
                LINK_RECV_TASK_STACKSIZE, NULL,
                LINK_RECV_TASK_PRI, NULL);

    while(1)
    {
        mavlink_msg_heartbeat_pack(system_id, component_id, &msg,
                                       MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                                       0,
                                       0, MAV_STATE_STANDBY);

        copter.link_mavlink.msg_send(msg);
        vTaskDelay(M2T(10));

        mavlink_msg_attitude_pack(system_id, component_id, &msg,
                               Timer_getTime(),
							   copter.estimator->get_pitch(),//euler.phi(),
							   copter.estimator->get_roll(),//euler.theta(),
							   copter.estimator->get_yaw(),//euler.psi(),
//							   att_Est_Q.get_pitch(),//euler.phi(),
//							   att_Est_Q.get_roll(),//euler.theta(),
//							   att_Est_Q.get_yaw(),//euler.psi(),
                               0,0,0
                                 );
        copter.link_mavlink.msg_send(msg);
        vTaskDelay(M2T(10));

	if(hil_inertialSensor.ready()) {
		mavlink_msg_highres_imu_pack(system_id, component_id, &msg,
                               Timer_getTime(),
							   copter.sensors->inertialSensor->get_acc_x(), copter.sensors->inertialSensor->get_acc_y(), copter.sensors->inertialSensor->get_acc_z(),
							   copter.sensors->inertialSensor->get_gyro_x(), copter.sensors->inertialSensor->get_gyro_y(), copter.sensors->inertialSensor->get_gyro_z(),
							   copter.sensors->compass->get_mag_x(), copter.sensors->compass->get_mag_y(), copter.sensors->compass->get_mag_z(),
                               0,0,0,0,0
                                 );
        copter.link_mavlink.msg_send(msg);
        vTaskDelay(M2T(10));
	}

//        if(copter.link_mavlink.text_need_send())
//        {
//        	mavlink_msg_statustext_pack(system_id, component_id, &msg,
//        							    0, copter.link_mavlink.get_text_buf());
//            copter.link_mavlink.msg_send(msg);
//        }
    }

}

void linkRecvTask(void *arg)
{
	while(1)
	{
	    copter.link_mavlink.updata();
	    //copter.link_mavlink.send_text("recv run");
//        vTaskDelay(M2T(10));
	}
}

//
//
//void sensorTask(void *arg)
//{
//    copter.mpu9250.init();
////    copter.hmc5883.init();
////    copter.ms5611.init();
//
//    uint64_t last_mpu9250_updata_time = 0;
////    uint64_t last_hmc5883_updata_time = 0;
////    uint64_t last_ms5611_updata_time = 0;
//
//    while(1)
//    {
//        if(Timer::getTime() - last_mpu9250_updata_time > 1000)
//        {        
//            copter.mpu9250.read();
//            last_mpu9250_updata_time = Timer::getTime();
//        }
////        if(Timer::getTime() - last_hmc5883_updata_time > (1000000 / 150))
////        {
////            copter.hmc5883.read();
////            last_hmc5883_updata_time = Timer::getTime();
////        }
////        if(Timer::getTime() - last_ms5611_updata_time > 25000)
////        {
////            copter.ms5611.read();
////            last_ms5611_updata_time = Timer::getTime();
////        }
//        vTaskDelay(1);
//    }
//}
//
//
void attEstTask(void *arg)
{
    while(1)
    {
		copter.estimator->run();

		//copter.link_mavlink.send_text("att run");

        vTaskDelay(M2T(1));
    }
}





