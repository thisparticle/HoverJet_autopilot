/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
  * 作者   ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：遥控器通道数据处理
**********************************************************************************/
#include "include.h"
#include "Ano_RC.h"
#include "Ano_Math.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_led.h"
#include "Drv_sbus.h"
#include "Ano_Sensor_Basic.h"

//摇杆触发值，摇杆值范围为+-500，超过300属于触发范围
#define UN_YAW_VALUE  300
#define UN_THR_VALUE  300
#define UN_PIT_VALUE  300
#define UN_ROL_VALUE  300

static u8 RC_IN_MODE;
void Remote_Control_Init()
{
	///提供两种ppm和sbus两种遥控器信号的接收，ppm里面好像包括pwm和ppm
	RC_IN_MODE = Ano_Parame.set.pwmInMode;
	///在参数初始化设置中默认设置为ppm模式
	///ano_parame.set结构体中存储了所有将被设置的参数，包括pid参数、起飞降落速度等，其中pwmInmpde为接收机模式
	///ano_Parame是一个共用体，set和一个2048字节的uint8_t数组byte公用空间，上电后程序先从eeprom中读取2048字节数据到共用体ano_parame的byte中，
	///从而对共用内存空间的set结构体赋值。可见遥控模式设定pwmInmode是保存在ROM中的，设置一次后就一直保持
	if(RC_IN_MODE == SBUS)
	{
		Drv_SbusInit(); ///sbus信号就是一种特殊的uart信号，相比于普通uart信号，sbus信号经过硬件取反后可以直接用uart控制器处理，注意必须进行硬件取反，软件取反不行
		///接收串口配置：波特率100k，8位数据位，偶校验，2位停止位，无控流（硬件流控流），25个字节
	}
	else
	{
		PWM_IN_Init(RC_IN_MODE);  ///ppm接收的外设初始化函数
	}
}

static u16 cwd_cnt[10] ;
u8 chn_en_bit = 0;
///所谓通道看门狗，其实和通常指的看门狗功能差不多，就是检查遥控通道数据读取死否正常，如果数据有问题，就拒绝，从而保证错误的数据不会被带入pid控制
///这个通道看门狗并不是真正的看门狗，不需要使用看门狗定时器，也不需要系统复位，只是因为二者功能都是守卫程序正常运行，所以这样称呼而已

/// 给通道看门狗喂狗
void ch_watch_dog_feed(u8 ch_n)
{
	ch_n = LIMIT(ch_n,0,7);
	cwd_cnt[ch_n] = 0;
}

///检查通道看门狗，dT_ms是调用时间间隔(调用周期)
///看门狗的原理，就是每次调用解码任务时把周期时间dT_ms加到每个通道对应的看门狗计时器cwd_cnt中，代表此通道的值多久没更新了
///(注意在解码部分，每个通道解码成狗后都会调用ch_watch_dog_feed喂狗，清空对应通道的看门狗计时器)
/// 如果未更新时间小于500ms，认为值没问题，给chn_en_bit对应位置1；否则认为此通道无值，给chn_en_bit对应位置0
static void ch_watch_dog(u8 dT_ms)//如果是PPM/SBUS模式，也只检测前8通道
{
	for(u8 i = 0;i<8;i++)
	{
		/// 看门狗计数小于500(数据更新周期小于500ms)，认为通道接收正常，计时增加，给chn_en_bit中对应位标记正常
		if(cwd_cnt[i]<500)
		{
			cwd_cnt[i] += dT_ms;
			chn_en_bit |= 0x01<<i;
		}
		/// 否则认定通道不正常，chn_en_bit中对应位标记0
		else
		{
			chn_en_bit &= ~(0x01<<i);
//			Rc_Pwm_In[i] = 0;  //把捕获值复位
//			Rc_Ppm_In[i] = 0;
//			Rc_Sbus_In[i] = 0;
		}
	}
}

u16 signal_intensity;

s16 CH_N[CH_NUM] = {0,0,0,0};

_stick_f_lp_st unlock_f;
u8 stick_fun_0;
u16 unlock_time = 200;

void unlock(u8 dT_ms)
{
	
	if( flag.power_state <=2 && para_sta.save_trig == 0)//只有电池电压非最低并且没有操作flash时，才允许进行解锁
	{
		if(sens_hd_check.acc_ok && sens_hd_check.gyro_ok)
		{
			if(sens_hd_check.baro_ok)
			{
				if(flag.sensor_imu_ok  )//imu传感器正常时，才允许解锁
				{
					flag.unlock_err = 0;	//允许解锁标志位

				}
				else
				{
					flag.unlock_err = 1;//imu异常，不允许解锁

				}
			}
			else
			{
				LED_STA.errBaro = 1;
				flag.unlock_err = 2;//气压计异常，不允许解锁。
			}
		}
		else
		{
			LED_STA.errMpu = 1;
			flag.unlock_err = 3;//惯性传感器异常，不允许解锁。
		}
	}
	else
	{
		flag.unlock_err = 4;//电池电压异常，不允许解锁
	}
	
	//解锁
	if(flag.unlock_sta == 0)
	{
		if(flag.unlock_cmd != 0)
		{		
			if(flag.unlock_err == 0)
			{
				//
				flag.unlock_sta = flag.unlock_cmd;
				//
				ANO_DT_SendString("Unlock OK!");
				
			}
			else 
			{
				//reset
				flag.unlock_cmd = 0;
				//
				if(flag.unlock_err == 1)
				{
					ANO_DT_SendString("Unlock Fail!");
				}
				else if(flag.unlock_err == 2)
				{
					ANO_DT_SendString("Unlock Fail!");
				}
				else if(flag.unlock_err == 3)
				{
					ANO_DT_SendString("Unlock Fail!");
				}
				else if(flag.unlock_err == 4)
				{
					ANO_DT_SendString("Power Low,Unlock Fail!");
				}
				else
				{
				
				}
			}
		}
		else
		{
		
		}
	}
	else
	{
		if(flag.unlock_cmd == 0)
		{
			ANO_DT_SendString(" FC Output Locked! ");
		}		
		flag.unlock_sta = flag.unlock_cmd;
	}
	
	////////////////////////////////////////////
	//所有功能判断，都要油门在低值时才进行
	if(CH_N[CH_THR] < -UN_THR_VALUE  )
	{
		//判断用户是否想要上锁、解锁
		if(ABS(CH_N[CH_YAW])>0.1f*UN_YAW_VALUE && CH_N[CH_PIT]< -0.1f*UN_PIT_VALUE)
		{
			if(flag.locking == 0)
			{
				flag.locking = 1;
			}
		}
		else
		{
			flag.locking = 0;
		}

		//飞控上锁、解锁检测
		if(CH_N[CH_PIT]<-UN_PIT_VALUE && CH_N[CH_ROL]>UN_ROL_VALUE && CH_N[CH_YAW]<-UN_YAW_VALUE)
		{
			stick_fun_0 = 1;
			flag.locking = 2;
		}
		else if(CH_N[CH_PIT]<-UN_PIT_VALUE && CH_N[CH_ROL]<-UN_ROL_VALUE && CH_N[CH_YAW]>UN_YAW_VALUE)
		{
			stick_fun_0 = 1;
			flag.locking = 2;
		}
		else
		{
			stick_fun_0 = 0;
		}
			
		
		u8 f = 0;		
		if(flag.unlock_sta)
		{
			//如果为解锁状态，最终f=0，将f赋值给flag.unlock_sta，飞控完成上锁
			f = 0;
			unlock_time = 300;
		}
		else
		{
			//如果飞控为锁定状态，则f=2，将f赋值给flag.unlock_sta，飞控解锁完成
			f = 2;
			unlock_time = 500;
		}
		//进行最终的时间积分判断，摇杆必须满足条件unlock_time时间后，才会执行锁定和解锁动作
		stick_function_check_longpress(dT_ms,&unlock_f,unlock_time,stick_fun_0,f,&flag.unlock_cmd);
	}
	else
	{
		flag.locking = 0; //油门高
		if(flag.unlock_cmd == 2)
		{
			flag.unlock_cmd = 1;
		}
	}

	
	if(CH_N[CH_THR]>-350)
	{
		flag.thr_low = 0;//油门非低
	}
	else
	{
		flag.thr_low = 1;//油门拉低
	}
}

///读出脉冲宽度后，调用下面的函数，检测遥控状态，是否触发解锁、传感器校准等特殊工作
///所有遥控解码函数的入口，同时也进行归一化处理，RC_PPM.Captures和Rc_Sbus_In都被限制到-500-500之间，处理后的摇杆值存入CH_N数组中，这是将被真正带入pid使用的参数值
/// RC_duty_task()是一个轮询线程，利用系统滴答定时器轮询，每11ms执行一次，参数dT_ms即为此任务执行周期
/// chn_en_bit 是一个bit-pick格式的字节，每位标志一个通道的值是否正常(是否有值)
void RC_duty_task(u8 dT_ms) //建议2ms调用一次
{
	if(flag.start_ok)	
	{
		/////////////获得通道数据////////////////////////
		if(RC_IN_MODE == PWM)
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))//(Rc_Pwm_In[i]!=0)//该通道有值，==0说明该通道未插线（PWM）
				{
					CH_N[i] = 1.25f *((s16)Rc_Pwm_In[i] - 1500); //1100 -- 1900us,处理成大约+-500摇杆量
//					if(i == CH_PIT)
//					{
//						CH_N[i] = -1.2f *((s16)Rc_Pwm_In[i] - 1500);
//					}
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
			}
		}
		else if(RC_IN_MODE == PPM)
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))//(Rc_Ppm_In[i]!=0)//该通道有值
				{
					//CH_N[]+1500为上位机显示通道值
					CH_N[i] = 1.25f *((s16)Rc_Ppm_In[i] - 1100); //700 -- 1500us,处理成大约+-500摇杆量
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
			}		
		}
		else//sbus
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				if(chn_en_bit & (1<<i))//该通道有值
				{
					//CH_N[]+1500为上位机显示通道值
					CH_N[i] = 0.65f *((s16)Rc_Sbus_In[i] - 1024); //248 --1024 --1800,处理成大约+-500摇杆量
				}
				else
				{
					CH_N[i] = 0;
				}
				CH_N[i] = LIMIT(CH_N[i],-500,500);//限制到+—500
			}					
		}

		///////////////////////////////////////////////
		//解锁监测	
		unlock(dT_ms);
		//摇杆触发功能监测
		stick_function(dT_ms);	
		//通道看门狗
		ch_watch_dog(dT_ms); ///这里调用了看门狗函数，参数是遥控解码任务周期时长

		//失控保护检查
		fail_safe_check(dT_ms);//3ms


	}
}

void fail_safe()
{
	for(u8 i = 0;i<4;i++)
	{
		CH_N[i] = 0;
	}

	if(CH_N[CH_THR]>0)
	{
		CH_N[CH_THR] = 0;
	}

	CH_N[CH_ROL] = 0;
	CH_N[CH_PIT] = 0;
	CH_N[CH_YAW] = 0;
	
	//切记不能给 CH_N[AUX1]赋值，否则可能导致死循环。（根据AUX1特殊值判断接收机failsafe信号）
	
	if(flag.unlock_sta)
	{
		if(switchs.gps_on ==0)
		{
			flag.auto_take_off_land = AUTO_LAND; //如果解锁，自动降落标记置位
		}
		else
		{
			flag.rc_loss_back_home = 1;
		}
		
	}
}

u16 test_si_cnt;

void fail_safe_check(u8 dT_ms) //dT秒调用一次
{
	static u16 cnt;
	static s8 cnt2;
	
	cnt += dT_ms;
	if(cnt >= 500) //500*dT 秒
	{
		cnt=0;
		if((chn_en_bit & 0x0F) != 0x0F || flag.chn_failsafe ) //前4通道有任意一通道无信号或者受到接收机失控保护信号
		{
			cnt2 ++;
		}
		else
		{
			cnt2 --;	
		}
		
		if(cnt2>=2)
		{
			cnt2 = 0;
			
			flag.rc_loss = 1; //认为丢失遥控信号
			
			LED_STA.noRc = 1;
			
			fail_safe();


				
		}
		else if(cnt2<=-2) //认为信号正常
		{
			cnt2 = 0;
			
			if(flag.rc_loss)
			{
				flag.rc_loss = 0;
				LED_STA.noRc = 0;
				
					if(flag.taking_off)
					flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH; //解除下降
			}
			
		}
		
		test_si_cnt = signal_intensity;
		signal_intensity=0; //累计接收次数
	}
	
	
}

void stick_function_check(u8 dT_ms,_stick_f_c_st *sv,u8 times_n,u16 reset_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	if(en)
	{
		sv->s_cnt = 0; //清除计时
		if(sv->s_state==0)
		{
			if(sv->s_now_times!=0)
			{
				sv->s_now_times++;
			}
			sv->s_state = 1;
		}
	}
	else
	{
		sv->s_state = 0;
		/////
		sv->s_cnt += dT_ms;
		if(sv->s_cnt>reset_time_ms)
		{
			sv->s_now_times = 1; //清除记录次数
		}
	}

	if(sv->s_now_times> times_n)
	{
		*trig = trig_val;            //触发功能标记
		sv->s_now_times = 0;
	}

}
void stick_function_check_longpress(u8 dT_ms,u16 *time_cnt,u16 longpress_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	//dT_ms：调用间隔时间
	//time_cnt：积分时间
	//longpress_time_ms：阈值时间，超过这个时间则为满足条件
	//en：摇杆状态是否满足
	//trig_val：满足后的触发值
	//trig：指向需要触发的寄存器
	if(en)//如果满足摇杆条件，则进行时间积分
	{
		if(*time_cnt!=0)
		{
			*time_cnt+=dT_ms;
		}
	}
	else//不满足条件，积分恢复1
	{
		*time_cnt=1;
	}
	//时间积分满足时间阈值，则触发标记
	if(*time_cnt>=longpress_time_ms)
	{
		*trig = trig_val;            //触发功能标记
		*time_cnt = 0;
	}

}

_stick_f_lp_st cali_gyro,cali_acc,cali_surface;
_stick_f_c_st cali_mag;

u8 stick_fun_1,stick_fun_2,stick_fun_3,stick_fun_4,stick_fun_5_magcali;
void stick_function(u8 dT_ms)
{
	//////////////状态监测
	//未解锁才允许检测摇杆功能
	if(flag.unlock_sta == 0)
	{
		//油门低，则继续
		if(flag.thr_low)
		{
			if(CH_N[CH_PIT]<-350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]>350)
			{
				stick_fun_1 = stick_fun_2 = 1;
			}
			else
			{
				stick_fun_1 = stick_fun_2 = 0;
			}
			
			if(CH_N[CH_PIT]>350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]<-350)
			{
				stick_fun_3 = 1;
			}
			else
			{
				stick_fun_3 = 0;
			}
			
			if(CH_N[CH_PIT]>350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]>350)
			{
				stick_fun_4 = 1;
			}
			else
			{
				stick_fun_4 = 0;
			}
			
			if(CH_N[CH_PIT]>350)
			{
				stick_fun_5_magcali =1;
			}
			else if(CH_N[CH_PIT]<50)
			{
				stick_fun_5_magcali =0;
			}
		}
		
			///////////////
		//触发陀螺仪校准
		stick_function_check_longpress(dT_ms,&cali_gyro,1000,stick_fun_1,1,&sensor.gyr_CALIBRATE);
		//触发加速度计校准
		stick_function_check_longpress(dT_ms,&cali_acc,1000,stick_fun_2,1,&sensor.acc_CALIBRATE);
		
//		stick_function_check_longpress(dT_ms,&cali_surface,1000,stick_fun_4,1,&sensor_rot.surface_CALIBRATE );
		//触发罗盘校准
		stick_function_check(dT_ms,&cali_mag,5,1000,stick_fun_5_magcali,1,&mag.mag_CALIBRATE);

		
	}

	//////////////
}











/******************* (C) COPYRIGHT 2017 ANO TECH *****END OF FILE************/

