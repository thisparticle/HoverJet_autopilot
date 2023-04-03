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
���᣺
      ��ͷ
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      ƨ��
*/


/**********�޸���2021/3/26���޸�Ϊ�����ĸ������������������
*����ͨ��1-ͨ��4Ϊ���ͨ����5-6Ϊ���ͨ��
*


*/



///��Ҫ����
///��һ���������������󣬵ڶ�����������ʱ���������ã�����ֵ������һ��
double motor[MOTORSNUM];
double motor_step[MOTORSNUM];
//float motor_lpf[MOTORSNUM];

///������
///static u16 motor_prepara_cnt;
_mc_st mc;
///IDLING ��ת
///����֪�⣬���ڿ�תת̬
///���û�´��������״̬�Ļ�ֻ������������΢ת��
u16 IDLING;//10*Ano_Parame.set.idle_speed_pwm  //200

///�����������
///���ļ�����һ������
void Motor_Ctrl_Task(u8 dT_ms,s16 *CH_N)
{
	u8 i;
	
//	if(flag.taking_off)
//	{
//		flag.motor_preparation = 1;
//		motor_prepara_cnt = 0;			
//	}
	
	///������ڽ���״̬
//	if(flag.unlock_sta)
//	{		
////		IDLING = 10*LIMIT(Ano_Parame.set.idle_speed_pwm,0,30);
//		IDLING = 30;
//		///��������δ׼����
//		if(flag.motor_preparation == 0)
//		{
//			///������ʼ
//			motor_prepara_cnt += dT_ms;
//			
//			///�ٴ��жϣ������δ׼����
//			if(flag.motor_preparation == 0)
//			{		
//					///����ֵ0-250
//				if(motor_prepara_cnt<250)
//				{
//					///��һ�������΢��ת
//					///��˵��
//					///�����΢ת���ֵ�״̬�ǽ������ĸ�����Դ�����Ȼ����΢ת��
//					///���̧������50%�����㣬����50%�������������Ȼ���ۼ�50%����
//					///����������˷ɻ���Խ��Խ�ߣ�ͬʱ���´�������50%���£����ֲ���
//					///�ٶ�Ҳ��Խ��Խ��Ȼ�����������ף����˻��ͽ�����
//					motor[m1] = IDLING;
//				}
//				else if(motor_prepara_cnt<500)
//				{
//					///����ֵ250-500
//					///�ڶ��������΢ת
//					motor[m2] = IDLING;
//				}
//				else if(motor_prepara_cnt<750)
//				{
//					///
//					///����ֵ500-750
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
//					///���׼������ͬʱ������0
//					flag.motor_preparation = 1;
//					motor_prepara_cnt = 0;
//				}
//			}
//			
//		}	
//	}
//	else
//	{
//		///δ׼���÷��У����������
//		flag.motor_preparation = 0;
//	}
		flag.motor_preparation = 1;

			
	///���׼�����˿��Խ�����һ��
	///���ѷ���mc.ct_val_thr/mc.ct_val_rol/mc.ct_val_pit/mc.ct_val_yaw���ĸ�����
	///�ֱ������ļ�Ano_AltCtrl.c��Ano_AttCtrl.c��ֱ�ӻ����ļ������漴�ɷֱ��ҵ����ĸ������������
	///���ĸ�ֵ���������õ�����ϵ�ֵ����ִ�����ã���Ϊ�����������е�һ���֣��øı�ִ�л��������
	///����ͨ��������������������������ϱȽ��ٴ��������������á�����Ҫ������
	if(flag.motor_preparation == 1)
	{	
		
		
#if (MOTORSNUM == 4)
/*
���᣺
      ��ͷ
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
���᣺
			��ͷ
     m2   m1
      \  /
 m3----------m6
      /  \
     m4   m5
*/
//����  sqrt(3)/2 = 0.866f

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
//			///�޷�IDLING - 1000
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
				///motor_step���հ�ֵ����motors������ʱ����������
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

	//�������
//	for(u8 i =0;i<4;i++)
//	{
		///����PWM���Ƶ����ת
		SetPwm(motor);
//	}

//#define Cali_Set_ESC
#ifdef Cali_Set_ESC
	//�������
	for(u8 i =0;i<4;i++)
	{
		motor[i] = CH_N[CH_THR]+500;
		SetPwm(motor);
	}
	
#endif


}



