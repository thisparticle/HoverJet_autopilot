/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：飞控初始化
**********************************************************************************/
#include "include.h"
#include "Drv_pwm_out.h"
#include "Drv_led.h"
#include "Drv_spi.h"
#include "Drv_icm20602.h"
#include "drv_ak8975.h"
#include "drv_spl06.h"
#include "Drv_w25qxx.h"
#include "Drv_i2c_soft.h"
#include "Drv_laser.h"
#include "Ano_FlightCtrl.h"
#include "Drv_adc.h"
#include "Drv_heating.h"
#include "Ano_RC.h"
#include "Ano_Sensor_Basic.h"
#include "Drv_UP_Flow.h"


u8 of_init_type;
u8 All_Init() ///对整机的传感器和外设进行初始化
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置

	SysTick_Configuration(); 		//滴答时钟
	
	Delay_ms(100);					//延时
	Drv_LED_Init();					//LED功能初始化
	
	Flash_Init();             		//板载FLASH芯片驱动初始化
	
	Para_Data_Init();              		//参数数据初始化
	
	Remote_Control_Init();  ///进行遥控接收配置 ///ppm和sbus接收解码得到的摇杆数据为RC_PPM.Captures和Rc_Sbus_In
	///在带入pid控制前，还要进行两个步骤，一是把取值范围不同的两组数据归一化到同一个范围内，这样在修改接收机模式时不会影响其他参数的选择
	///二是提高数据的可靠性，针对这个问题的解决方案时利用“通道看门狗”
	PWM_Out_Init();					//初始化电调输出功能	
	Delay_ms(100);					//延时
	
	Drv_SPI2_init();          		//spi_2初始化，用于读取飞控板上所有传感器，都用SPI读取
	Drv_Icm20602CSPin_Init(); 		//spi片选初始化//惯性传感器
	Drv_AK8975CSPin_Init();   		//spi片选初始化//磁场传感器

	Drv_SPL06CSPin_Init();    		//spi片选初始化//气压计
	sens_hd_check.gyro_ok = sens_hd_check.acc_ok = 
	Drv_Icm20602Reg_Init();   		//icm陀螺仪加速度计初始化，若初始化成功，则将陀螺仪和加速度的初始化成功标志位赋值///陀螺仪获得载体的角速度，俯仰角和滚转角通过加速度计
	sens_hd_check.mag_ok = 1;       //标记罗盘OK	///偏航角通过磁力计
	sens_hd_check.baro_ok = Drv_Spl0601_Init();       		//气压计初始化

	Usb_Hid_Init();					//飞控usb接口的hid初始化
	Delay_ms(100);					//延时
	
	Usart2_Init(500000);			//串口2初始化，函数参数为波特率
	Delay_ms(10);					//延时	
//	Uart4_Init(115200);				//首先判断是否连接的是激光模块
//	if(!Drv_Laser_Init())			//激光没有有效连接，则配置为光流模式
//		Uart4_Init(500000);
//	Delay_ms(10);					//延时
//	Usart3_Init(500000);			//连接UWB
//	Delay_ms(10);					//延时
	//
	Usart3_Init(500000);			//连接OPENMV
	//	
	Uart4_Init(19200);	//接优像光流
	Uart5_Init(115200);//接大功率激光	
//	MyDelayMs(200);	
	//优像光流初始化
	of_init_type = (Drv_OFInit()==0)?0:2;
	if(of_init_type==2)//优像光流初始化成功
	{
		//大功率激光初始化
		Drv_Laser_Init();	
	}
	else if(of_init_type==0)//优像光流初始化失败
	{
		Uart4_Init(500000);	//接匿名光流
	}
	//

	Drv_AdcInit();
	Delay_ms(100);					//延时

	All_PID_Init();               		//PID初始化
////////////
	Drv_GpsPin_Init();				//GPS初始化 串口1
	Delay_ms(50);					//延时		
	
	Drv_HeatingInit();
//	Drv_HeatingSet(5);
	//
	Sensor_Basic_Init();
	//
	ANO_DT_SendString("SYS init OK!");
	return (1);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
