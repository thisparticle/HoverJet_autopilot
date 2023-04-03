#include "Ano_MotorCtrl.h"
#include "Ano_Math.h"
#include "ANO_RC.h"
#include "Drv_icm20602.h"
#include "Drv_spl06.h"
#include "Ano_Imu.h"
#include "Drv_pwm_out.h"
#include "Ano_MotionCal.h"
#include "Ano_Filter.h"
#include "Ano_Navigate.h"

/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      屁股
*/


/**********修改于2021/3/26，修改为控制四个舵机和两个涵道风扇
*其中通道1-通道4为舵机通道，5-6为电机通道
*


*/



///重要数组
///第一个数组是最终需求，第二个参数起到临时变量的作用，最后把值传给第一个
double motor[MOTORSNUM];
double motor_step[MOTORSNUM];
//float motor_lpf[MOTORSNUM];

///计算器
///static u16 motor_prepara_cnt;
_mc_st mc;
///IDLING 空转
///见名知意，处于空转转态
///如果没猜错，处于这个状态的话只会让螺旋桨轻微转动
u16 IDLING;//10*Ano_Parame.set.idle_speed_pwm  //200

///电机控制任务
///此文件仅此一个函数
void Motor_Ctrl_Task(u8 dT_ms,s16 *CH_N)
{
	u8 i;
	
//	if(flag.taking_off)
//	{
//		flag.motor_preparation = 1;
//		motor_prepara_cnt = 0;			
//	}
	
	///如果处于解锁状态
//	if(flag.unlock_sta)
//	{		
////		IDLING = 10*LIMIT(Ano_Parame.set.idle_speed_pwm,0,30);
//		IDLING = 30;
//		///如果电机还未准备好
//		if(flag.motor_preparation == 0)
//		{
//			///计数开始
//			motor_prepara_cnt += dT_ms;
//			
//			///再次判断，电机尚未准备好
//			if(flag.motor_preparation == 0)
//			{		
//					///计数值0-250
//				if(motor_prepara_cnt<250)
//				{
//					///第一个电机稍微旋转
//					///简单说下
//					///电机稍微转呈现的状态是解锁后四个电机以此启动然后轻微转动
//					///随后抬油门至50%即零界点，过了50%即控制四轴起飞然后累加50%以上
//					///的增量，因此飞机会越飞越高，同时向下打油门至50%以下，保持不变
//					///速度也会越来越低然后油门拉到底，无人机就降落了
//					motor[m1] = IDLING;
//				}
//				else if(motor_prepara_cnt<500)
//				{
//					///计数值250-500
//					///第二个电机稍微转
//					motor[m2] = IDLING;
//				}
//				else if(motor_prepara_cnt<750)
//				{
//					///
//					///计数值500-750
//					motor[m3] = IDLING;
//				}	
//				else if(motor_prepara_cnt<1000)
//				{	
//					motor[m4] = IDLING;
//				}
//#if (MOTORSNUM >= 6)
//				else if(motor_prepara_cnt<1250)
//				{
//					motor[m5] = 15;
//				}
//				else if(motor_prepara_cnt<1500)
//				{
//					motor[m6] = 15;
//				}
//#endif				
//				else
//				{
//					///电机准备好了同时计数清0
//					flag.motor_preparation = 1;
//					motor_prepara_cnt = 0;
//				}
//			}
//			
//		}	
//	}
//	else
//	{
//		///未准备好飞行，电机不启动
//		flag.motor_preparation = 0;
//	}
		flag.motor_preparation = 1;

			
	///电机准备好了可以进行下一步
	///不难发现mc.ct_val_thr/mc.ct_val_rol/mc.ct_val_pit/mc.ct_val_yaw这四个变量
	///分别来自文件Ano_AltCtrl.c和Ano_AttCtrl.c，直接滑倒文件最下面即可分别找到这四个最终输出变量
	///这四个值即最终作用到电机上的值，起执行作用，作为负反馈控制中的一部分，得改变执行机构即电机
	///才能通过测量反馈至输入端与期望不断比较再次输出，起控制作用。很重要！！！
	if(flag.motor_preparation == 1)
	{	
		
		
#if (MOTORSNUM == 4)
/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
*/    

		motor_step[m1] = mc.ct_val_thr  +mc.ct_val_yaw -mc.ct_val_rol +mc.ct_val_pit;
		motor_step[m2] = mc.ct_val_thr  -mc.ct_val_yaw +mc.ct_val_rol +mc.ct_val_pit;
		motor_step[m3] = mc.ct_val_thr  +mc.ct_val_yaw +mc.ct_val_rol -mc.ct_val_pit;
		motor_step[m4] = mc.ct_val_thr  -mc.ct_val_yaw -mc.ct_val_rol -mc.ct_val_pit;
#elif (MOTORSNUM == 6)	
/*
六轴：
			机头
     m2   m1
      \  /
 m3----------m6
      /  \
     m4   m5
*/
//六轴  sqrt(3)/2 = 0.866f

//		motor_step[m1] = mc.ct_val_thr -mc.ct_val_rol *0.866f +mc.ct_val_pit +mc.ct_val_yaw;
//		motor_step[m2] = mc.ct_val_thr +mc.ct_val_rol *0.866f +mc.ct_val_pit -mc.ct_val_yaw;
//		motor_step[m3] = mc.ct_val_thr +mc.ct_val_rol *0.866f                +mc.ct_val_yaw;
//		motor_step[m4] = mc.ct_val_thr +mc.ct_val_rol *0.866f -mc.ct_val_pit -mc.ct_val_yaw;	
//		motor_step[m5] = mc.ct_val_thr -mc.ct_val_rol *0.866f -mc.ct_val_pit +mc.ct_val_yaw;
//		motor_step[m6] = mc.ct_val_thr -mc.ct_val_rol *0.866f                -mc.ct_val_yaw;			
		//mc.ct_val_rol = mc.ct_val_rol * -1.0f;
		motor_step[m1] = mc.ct_val_pit * 0.5f - mc.ct_val_yaw * 0.5f;
		motor_step[m2] = mc.ct_val_rol * 0.5f;
		motor_step[m3] = mc.ct_val_pit * 0.5f + mc.ct_val_yaw * 0.5f;
		motor_step[m4] = mc.ct_val_rol * 0.5f;	
//		motor_step[m5] = mc.ct_val_thr;
//		motor_step[m6] = mc.ct_val_thr;	
		motor_step[m5] = (CH_N[CH_THR]+500)/10;
		motor_step[m6] = (CH_N[CH_THR]+500)/10;	
	//ANO_DT_SendStrVal("motor_step[m5]:",motor_step[m5]);
//	ANO_DT_SendStrVal("motor_step[m2]:",motor_step[m2]);
//	ANO_DT_SendStrVal("motor_step[m4]:",motor_step[m4]);

#endif		
	
//		for(i=0;i<MOTORSNUM;i++)
//		{	
//			///限幅IDLING - 1000
//			motor_step[i] = LIMIT(motor_step[i],IDLING,1000);
////			motor_lpf[i] += 0.5f *(motor_step[i] - motor_lpf[i]) ;		
//			
//		}
		


	}
	
	for(i=0;i<MOTORSNUM;i++)
	{
		if(flag.unlock_sta)
		{
			if(flag.motor_preparation == 1)
			{
				///motor_step最终把值给了motors，起到临时变量的作用
//				motor[i] = LIMIT(motor_step[i],IDLING,999);
					motor[i] = LIMIT(motor_step[i],-100,100);
			}
	
		}
		else
		{		
			motor[i] = 0;
		}	

	}
	
		motor[0] = LIMIT(motor[0],-30,30);
		motor[1] = LIMIT(motor[1],-30,30);
		motor[2] = LIMIT(motor[2],-30,30);
		motor[3] = LIMIT(motor[3],-30,30);

	//配置输出
//	for(u8 i =0;i<4;i++)
//	{
		///设置PWM控制电机旋转
		SetPwm(motor);
//	}

//#define Cali_Set_ESC
#ifdef Cali_Set_ESC
	//配置输出
	for(u8 i =0;i<4;i++)
	{
		motor[i] = CH_N[CH_THR]+500;
		SetPwm(motor);
	}
	
#endif


}



