/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：PWM输出
**********************************************************************************/
#include "Drv_pwm_out.h"
#include "include.h"
#include "Ano_Math.h"

//21分频到 84000000/21 = 4M   0.25us

/*初始化高电平时间1000us（4000份）*/
//#define INIT_DUTY 4000 //u16(1000/0.25)
/*频率400hz*/
#define HZ        50
/*精度10000，每份0.25us*/
#define ACCURACY 2000 //u16(2500/0.25) //accuracy
int INIT_DUTY = ACCURACY/ 2 -1;
/*设置飞控控制信号转换比例为4*/
#define PWM_RADIO 4//(8000 - 4000)/1000.0
///1.改变 CCRx 的值，就可以改变 PWM 输出的占空比，改变 ARR 的值，就可以改变 PWM 输出的频率
///2.捕获/比较模式寄存器（TIMx_CCMR1/2）、捕获/比较使能寄存器（TIMx_CCER）、捕获/比较寄存器（TIMx_CCR1~4）
///3.捕获/比较模式寄存器（TIMx_CCMR1/2），该寄存器一般有 2 个：TIMx _CCMR1和 TIMx _CCMR2。TIMx_CCMR1 控制 CH1 和 2，而 TIMx_CCMR2控制 CH3 和 4。
///4.捕获/比较使能寄存器（TIM14_CCER），该寄存器控制着各个输入输出通道的开关。设置为1为输出
///5.捕获/比较寄存器（TIMx_CCR1~4），该寄存器总共有 4 个，对应 4 个通道 CH1~4。在输出模式下，该寄存器的值与 CNT 的值比较，根据比较结果产生相应动作。利用这点，
///我们通过修改这个寄存器的值，就可以控制 PWM 的输出脉宽了。
///6.如果是通用定时器，则配置以上三个寄存器就够了
///其他的参数 TIM_OutputNState，TIM_OCNPolarity，TIM_OCIdleState 和 TIM_OCNIdleState 是高级定时器才用到的。
///7.定时器除了 TIM6 和 7。其他的定时器都可以用来产生 PWM 输出。其中高级定时器 TIM1 和 TIM8 可以同时产生多达 7 路的 PWM 输出
u8 PWM_Out_Init () //400hz
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;
    u32 hz_set = ACCURACY * HZ;///10000*400=4000000

		GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit ( &TIM_TimeBaseStructure );
    TIM_OCStructInit ( &TIM_OCInitStructure );

    hz_set = LIMIT ( hz_set, 1, 84000000 );

		//APB1   42m
		//APB2   84m
		//TIMx clk = APBx *2
		///开启 TIM5\8\1 和 GPIO 时钟
    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM5, ENABLE );   ///TIM5时钟使能,42M
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM8, ENABLE );   ///TIM8时钟使能,84M
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM1, ENABLE );   ///TIM1时钟使能,84M
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE );///GPIO 时钟使能,42M

/////////////////////////////////////////////////////////////////////////////
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIO_Pin_0 | GPIO_Pin_1 |
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              ///复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;        ///速度100M
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            ///推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;             ///上拉
    GPIO_Init ( GPIOA, &GPIO_InitStructure );                 ///初始化PA2&3

//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
		GPIO_PinAFConfig ( GPIOA, GPIO_PinSource2, GPIO_AF_TIM5 );///GPIOA2复用为定时器5,42M
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource3, GPIO_AF_TIM5 );///GPIOA3复用为定时器5,42M

    /* Compute the prescaler value */
    PrescalerValue = ( uint16_t ) ( ( SystemCoreClock / 2 ) / hz_set ) - 1;///SystemCoreClock=168000000;PrescalerValue=21-1;定时器分频=21分频
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY-1;             ///设置自动重装载值;10000
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;    ///定时器分频，设置预分频值;21-1
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;						 ///设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; ///向上计数模式
    TIM_TimeBaseInit ( TIM5, &TIM_TimeBaseStructure );       ///初始化定时器5,根据指定的参数初始化 TIM5

		///初始化TIM   pwm模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        ///选择定时器模式：TIM脉冲宽度调制模式2;参数 TIM_OCMode 设置模式是 PWM 还是输出比较，这里我们是 PWM 模式。
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;///参数 TIM_OCPolarity 用来设置极性是高还是低;输出极性：TIM输出比较极性高；

//  /* PWM1 Mode configuration: Channel1 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel2 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;///比较输出使能;参数 TIM_OutputState 用来设置比较输出使能，也就是使能 PWM 输出到端口。
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;///初始化高电平时间1000us（4000份）INIT_DUTY 4000
    TIM_OC3Init ( TIM5, &TIM_OCInitStructure );/////根据T指定的参数初始化外设TIM5 OC3
    TIM_OC3PreloadConfig ( TIM5, TIM_OCPreload_Enable );/////使能TIM5在CCR3是的预装载寄存器

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init ( TIM5, &TIM_OCInitStructure );
    TIM_OC4PreloadConfig ( TIM5, TIM_OCPreload_Enable );

    TIM_ARRPreloadConfig ( TIM5, ENABLE );/////ARPE使能
    TIM_Cmd ( TIM5, ENABLE );///使能 TIM5
/////////////////////////////////////////////////////////////////////////////
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;///GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13| GPIO_Pin_14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOE, &GPIO_InitStructure );

    GPIO_PinAFConfig ( GPIOE, GPIO_PinSource9, GPIO_AF_TIM1 );///GPIOE9复用为定时器1,84M
    GPIO_PinAFConfig ( GPIOE, GPIO_PinSource11, GPIO_AF_TIM1 );///GPIOE11复用为定时器1,84M
    GPIO_PinAFConfig ( GPIOE, GPIO_PinSource13, GPIO_AF_TIM1 );///GPIOE13复用为定时器1,84M
    GPIO_PinAFConfig ( GPIOE, GPIO_PinSource14, GPIO_AF_TIM1 );///GPIOE14复用为定时器1,84M

    /* Compute the prescaler value */
    PrescalerValue = ( uint16_t ) ( ( SystemCoreClock ) / hz_set ) - 1;///hz_set = 4000000;SystemCoreClock=168000000;PrescalerValue=42-1;定时器分频=42分频
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY-1;									///设置自动重装载值;10000
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;					///定时器分频，设置预分频值;42-1
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;									///设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		///向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit ( TIM1, &TIM_TimeBaseStructure );						///初始化定时器1,根据指定的参数初始化 TIM1


		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;								///选择定时器模式:PWM 模式。
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;/// 互补输出使能。关闭OCN输出
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;				///参数 TIM_OCPolarity 用来设置极性是高还是低;输出极性：TIM输出比较极性高；
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;			///设置互补输出极性
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;			///选择空闲状态下得非工作状态
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;	////选择互补空闲状态下得非工作状态

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		///选择输出比较状态
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;											///设置了待装入捕获比较器的脉冲值
    TIM_OC1Init ( TIM1, &TIM_OCInitStructure );
    //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC2Init ( TIM1, &TIM_OCInitStructure );
    //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC3Init ( TIM1, &TIM_OCInitStructure );
    //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init ( TIM1, &TIM_OCInitStructure );
    //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs ( TIM1, ENABLE );
    TIM_ARRPreloadConfig ( TIM1, ENABLE );
    TIM_Cmd ( TIM1, ENABLE );
    ////////////////////////////////////////////////////////////////////////////////////

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );

    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource8, GPIO_AF_TIM8 );
    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource9, GPIO_AF_TIM8 );

    /* Compute the prescaler value */
    PrescalerValue = ( uint16_t ) ( ( SystemCoreClock ) / hz_set ) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY-1;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit ( TIM8, &TIM_TimeBaseStructure );


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    /* PWM1 Mode configuration: Channel3 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC3Init ( TIM8, &TIM_OCInitStructure );
    //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init ( TIM8, &TIM_OCInitStructure );
    //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs ( TIM8, ENABLE );
    TIM_ARRPreloadConfig ( TIM8, ENABLE );
    TIM_Cmd ( TIM8, ENABLE );


    if ( hz_set > 84000000 )
    {
        return 0;
    }
    else
    {
        return 1;
    }
}





/*
    以90度为基准，输出 = 输入 + 90
    Args:
				angle - 以九十度为基准所需要转的角度，单位为degree
    Returns:
        pwn  -   对应的pwm计数值
*/
int SetAngle(double angle){
	//u32 hz_set = ACCURACY * HZ;
	//uint16_t PrescalerValue = ( uint16_t ) ( ( SystemCoreClock ) / hz_set ) - 1;///计算分频系数
	//uint16_t time_per_count = 1/(SystemCoreClock/PrescalerValue);///
	//double time_per_count = 20/2000;
	double time_per_count = 0.01;///这里直接赋值是因为如果用除法C++编译优化会直接跳过这条语句，这里是20/2000=0.01，20是周期20ms，2000是计数值
//	time_per_count = safe_div(20,2000,0.01);
	double time_per_degree = 0.5/45;
	double Ninety_degrees_counts = 18.5/time_per_count;
	int pwmvalue = 0;
	pwmvalue = Ninety_degrees_counts - angle * time_per_degree / time_per_count;
	return pwmvalue;
}

void Set_Channel1_Angle(double angle){
	///这里应该根据每个不同的舵机判断符号，再将angle输入到下一步
	///1号2号舵机转向一致，3号4号舵机转向相反
	int pwmvalue = SetAngle(angle);
//	double time_per_count = 20/ACCURACY;
//	double time_per_degree = 0.5/45;
//	double Ninety_degrees_counts = 18.5/time_per_count;
//	int pwmvalue = 0;
//	pwmvalue = (int)(Ninety_degrees_counts - angle * time_per_degree / time_per_count);
	//printf("%d",pwmvalue);
	TIM1->CCR4 = pwmvalue;
}
void Set_Channel2_Angle(double angle){
	///这里应该根据每个不同的舵机判断符号，再将angle输入到下一步
	///1号2号舵机转向一致，3号4号舵机转向相反
	angle = -1.0f * angle;
	int pwmvalue = SetAngle(angle);
	TIM1->CCR3 = pwmvalue;
}
void Set_Channel3_Angle(double angle){
	///这里应该根据每个不同的舵机判断符号，再将angle输入到下一步
	///1号2号舵机转向一致，3号4号舵机转向相反
	angle = -1.0f * angle;
	int pwmvalue = SetAngle(angle);
	TIM1->CCR2 = pwmvalue;
}
void Set_Channel4_Angle(double angle){
	///这里应该根据每个不同的舵机判断符号，再将angle输入到下一步
	///1号2号舵机转向一致，3号4号舵机转向相反
	int pwmvalue = SetAngle(angle);
	TIM1->CCR1 = pwmvalue;
}


/*
    输入油门大小，输出对应的pwm计数值,///这里将油门分成1000份
    Args:
				Throttle - 0~1大小范围的油门值
    Returns:
        pwn  -   对应的pwm计数值
*/
int SetThrottle(double Throttle){
	//double time_per_count = 20/ACCURACY;
	double time_per_count = 0.01;///这里直接赋值是因为如果用除法C++编译优化会直接跳过这条语句，这里是20/2000=0.01，20是周期20ms，2000是计数值
	//double time_per_Throttle = 1/100;
	double time_per_Throttle = 0.01;
	int Zero_Throttle_counts = 2000 - 1/time_per_count;
	int pwmvalue = 0;
	pwmvalue = Zero_Throttle_counts - Throttle * time_per_Throttle / time_per_count;
	return pwmvalue;
}
void Set_Channel5_Throttle(double Throttle){
	int pwmvalue = SetThrottle(Throttle);
	TIM_SetCompare4(TIM5,pwmvalue);
}
void Set_Channel6_Throttle(double Throttle){
	int pwmvalue = SetThrottle(Throttle);
	TIM_SetCompare3(TIM5,pwmvalue);
}


void SetPwm ( double pwm[MOTORSNUM] )
{
	/*注意这里输入的不是直接就是pwm值，我们要输入的是四个舵偏角和两个油门，不过懒得改数组名了*/
//    TIM1->CCR4 = PWM_RADIO * ( pwm[0] ) + INIT_DUTY;				//1
//    TIM1->CCR3 = PWM_RADIO * ( pwm[1] ) + INIT_DUTY;				//2
//    TIM1->CCR2 = PWM_RADIO * ( pwm[2] ) + INIT_DUTY;				//3
//    TIM1->CCR1 = PWM_RADIO * ( pwm[3] ) + INIT_DUTY;				//4
//#if (MOTORSNUM >= 6)
// 	TIM5->CCR4 = PWM_RADIO * ( pwm[4] ) + INIT_DUTY;				//5
// 	TIM5->CCR3 = PWM_RADIO * ( pwm[5] ) + INIT_DUTY;				//6
//#endif
//// 	TIM8->CCR4 = PWM_RADIO * ( pwm[6] ) + INIT_DUTY;				//7
//// 	TIM8->CCR3 = PWM_RADIO * ( pwm[7] ) + INIT_DUTY;				//8
		Set_Channel1_Angle(pwm[0]);
		Set_Channel2_Angle(pwm[1]);
		Set_Channel3_Angle(pwm[2]);
		Set_Channel4_Angle(pwm[3]);
		Set_Channel5_Throttle(pwm[4]);
		Set_Channel6_Throttle(pwm[5]);
//		ANO_DT_SendStrVal("PWM[0]:",pwm[0]);
//	ANO_DT_SendStrVal("PWM[1]:",pwm[1]);
//	ANO_DT_SendStrVal("PWM[2]:",pwm[2]);
//	ANO_DT_SendStrVal("PWM[3]:",pwm[3]);
//	ANO_DT_SendStrVal("PWM[4]:",pwm[4]);
//	ANO_DT_SendStrVal("PWM[5]:",pwm[5]);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
