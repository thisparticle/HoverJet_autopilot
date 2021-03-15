 /******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_IMU.c
 * 描述    ：姿态解算函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 /// 总共有三个坐标系：1.地理坐标系，标记为w
 ///2.机体坐标系，标记为a
 ///航向坐标系，标记为h，四旋翼可以认为机体坐标系就是航向坐标系
 ///匿名的坐标系变换，是按照zyx顺序从地理坐标系转到机体坐标系的
 ///程序中用二维数组 att_matrix 表示机体坐标系转到地理坐标系的旋转矩阵
*****************************************************************************/
#include "Ano_Imu.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"
#include "Ano_DT.h"
//#include "ANO_RC.h"



/*参考坐标，定义为ANO坐标*

俯视，机头方向为x正方向
     +x
     |
 +y--|--
     |
		 
*/	

///下面提供了三个坐标系转换函数，可以实现w系和h系xy方向向量互换，以及从a系到w系三轴方向向量转换
///形参ref_ax的实参是imu_data.hx_vec
///物理上，hx_vec[x]和hx_vec[y]来自机体系转地面系的第一列，表示机体系中的x单位向量（机头方向）[1,0,0]T，在θ和Φ=0时，仅绕z轴旋转ψ后，得到向量中的xy项
///这指向了航向坐标的机头方向，所以在注释中把他们称为水平面方向向量
///航向坐标系h就是地理系w只绕z轴旋转(yaw)，忽略pitch和roll旋转的结果

//世界坐标平面XY转平面航向坐标XY
void w2h_2d_trans(float w[VEC_XYZ],float ref_ax[VEC_XYZ],float h[VEC_XYZ])
{
	h[X] =  w[X] *  ref_ax[X]  + w[Y] *ref_ax[Y];
	h[Y] =  w[X] *(-ref_ax[Y]) + w[Y] *ref_ax[X];
	
}
//平面航向坐标XY转世界坐标平面XY
void h2w_2d_trans(float h[VEC_XYZ],float ref_ax[VEC_XYZ],float w[VEC_XYZ])
{
	w[X] = h[X] *ref_ax[X] + h[Y] *(-ref_ax[Y]);
	w[Y] = h[X] *ref_ax[Y] + h[Y] *  ref_ax[X];
	
}

//载体坐标转世界坐标（ANO约定等同与地理坐标）
float att_matrix[3][3]; //必须由姿态解算算出该矩阵
void a2w_3d_trans(float a[VEC_XYZ],float w[VEC_XYZ])
{
		for(u8 i = 0;i<3;i++)
		{
			float temp = 0;
			for(u8 j = 0;j<3;j++)
			{
				
				temp += a[j] *att_matrix[i][j];
			}
			w[i] = temp;
		}
}

//float mag_yaw_calculate(float dT,float mag_val[VEC_XYZ],float g_z_vec[VEC_XYZ],float h_mag_val[VEC_XYZ])//
//{

//	vec_3dh_transition(g_z_vec, mag_val, h_mag_val);

//	return (fast_atan2(h_mag_val[Y], h_mag_val[X]) *57.3f) ;// 	
//}
	


#define USE_MAG
#define USE_LENGTH_LIM



_imu_st imu_data =  {1,0,0,0,
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					 0,0,0};

static float vec_err[VEC_XYZ];
static float vec_err_i[VEC_XYZ];
static float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;//q0q0,				
static float mag_yaw_err,mag_err_dot_prudoct,mag_val_f[VEC_XYZ];					
static float imu_reset_val;		

static u16 reset_cnt;
					 
_imu_state_st imu_state = {1,1,1,1,1,1,1,1};

static float mag_2d_w_vec[2][2] = {{1,0},{1,0}};//地理坐标中，水平面磁场方向恒为南北 (1,0)

///计算周期dT,Imu标志*state，滤波转换后的三轴陀螺仪数据gyr，滤波转换后的三轴加速度计数据acc，滤波转换后的三轴磁力计数据gyr，Imu数据结构体*Imu
///任务：1.利用互补滤波方法，用加速度计和磁力计数据对陀螺仪数据进行补偿
///2.使用一阶龙格库塔法更新四元数和旋转矩阵
///3.利用旋转矩阵计算三个坐标系下的各项数值，存入*Imu
///4.根据*state中的标记，对互补滤波系数进行修正，进一步提高解算准确度
float imu_test[3];
void IMU_update(float dT,_imu_state_st *state,float gyr[VEC_XYZ], s32 acc[VEC_XYZ],s16 mag_val[VEC_XYZ],_imu_st *imu)
{
//	const float kp = 0.2f,ki = 0.001f;
//	const float kmp = 0.1f;
	
	///互补滤波系数 ///加速度计误差比例系数，越大越倾向加速度计 ///加速度计误差积分积分系数 ///磁力计修正系数
	static float kp_use = 0,ki_use = 0,mkp_use = 0;

	///加速度向量的模 ///加速度向量的模的倒数 ///四元数的模
	float acc_norm_l,acc_norm_l_recip,q_norm_l;
		
	float acc_norm[VEC_XYZ]; ///单位化的加速度向量

	float d_angle[VEC_XYZ]; ///微元转角，即滤波修正后的角速度值
	


	///旋转矩阵需要的一些四元数，先算好调用快一些
//		q0q0 = imu->w * imu->w;							
		q0q1 = imu->w * imu->x;
		q0q2 = imu->w * imu->y;
		q1q1 = imu->x * imu->x;
		q1q3 = imu->x * imu->z;
		q2q2 = imu->y * imu->y;
		q2q3 = imu->y * imu->z;
		q3q3 = imu->z * imu->z;
		q1q2 = imu->x * imu->y;
		q0q3 = imu->w * imu->z;
	
	///这里实际执行不到？
		if(state->obs_en)
		{
			//计算机体坐标下的运动加速度观测量。坐标系为北西天
			for(u8 i = 0;i<3;i++)
			{
				s32 temp = 0;
				for(u8 j = 0;j<3;j++)
				{
					
					temp += imu->obs_acc_w[j] *att_matrix[j][i];//t[i][j] 转置为 t[j][i]
				}
				imu->obs_acc_a[i] = temp;
				
				imu->gra_acc[i] = acc[i] - imu->obs_acc_a[i];
			}
		}
		///只执行这？
		else
		{
			for(u8 i = 0;i<3;i++)
			{			
				imu->gra_acc[i] = acc[i];   ///把加速度计测量值传给结构体保存
			}
		}
    //
		///加速度向量模的倒数
		acc_norm_l_recip = my_sqrt_reciprocal(my_pow(imu->gra_acc[X]) + my_pow(imu->gra_acc[Y]) + my_pow(imu->gra_acc[Z]));
		///加速度向量模
		acc_norm_l = safe_div(1,acc_norm_l_recip,0);
		
		// 加速度计的读数，单位化。
		for(u8 i = 0;i<3;i++)
		{
			acc_norm[i] = imu->gra_acc[i] *acc_norm_l_recip;
		}

		
		///匿名的坐标系变换，是按是按ZYX顺序从地理坐标系转到机体坐标系的，
		///这个旋转矩阵是前面提过的 (注意这三个式子中，θ \thetaθ是绕Y轴转角，ϕ \phiϕ是绕X轴转角，ψ \psiψ是绕Z轴转角)
		///att_matrix是从机体系a到地理系w的转换矩阵,二维数组att_matrix表示从机体坐标系转到地理坐标系的旋转矩阵，是上面那个的转置
		// 载体坐标下的x方向向量，单位化。 ///地面系北方x[1 0 0 ]T转到机体系中，w到a系旋转矩阵第一列，可以看作地理x方向在机体系的三个分量
    att_matrix[0][0] = imu->x_vec[X] = 1 - (2*q2q2 + 2*q3q3);
    att_matrix[0][1] = imu->x_vec[Y] = 2*q1q2 - 2*q0q3;
    att_matrix[0][2] = imu->x_vec[Z] = 2*q1q3 + 2*q0q2;
		
		// 载体坐标下的y方向向量，单位化。  ///地面系西方y[0 1 0]T转到机体系中，w到a系旋转矩阵第二列，可以看作地理y方向在机体系的三个分量
    att_matrix[1][0] = imu->y_vec[X] = 2*q1q2 + 2*q0q3;
    att_matrix[1][1] = imu->y_vec[Y] = 1 - (2*q1q1 + 2*q3q3);
    att_matrix[1][2] = imu->y_vec[Z] = 2*q2q3 - 2*q0q1;
		
    // 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化。 ///地面系天空z[0 0 1 ]T转到机体系中，w到a系旋转矩阵第三列，可以看作地理z方向在机体系的三个分量
    att_matrix[2][0] = imu->z_vec[X] = 2*q1q3 - 2*q0q2;
    att_matrix[2][1] = imu->z_vec[Y] = 2*q2q3 + 2*q0q1;
    att_matrix[2][2] = imu->z_vec[Z] = 1 - (2*q1q1 + 2*q2q2);
		
		//水平面方向向量 ///a到w系第一列xy，机体系x[1 0 0]T转到地面系后的xy向量，也就是航向坐标系的机头指向
	float hx_vec_reci = my_sqrt_reciprocal(my_pow(att_matrix[0][0]) + my_pow(att_matrix[1][0]));
	imu->hx_vec[X] = att_matrix[0][0] *hx_vec_reci;
	imu->hx_vec[Y] = att_matrix[1][0] *hx_vec_reci;
	
	
	// 计算载体坐标下的运动加速度。(与姿态解算无关) ///加速度传感器测量值去掉重力加速度成分，得到机体系中的a_acc，与姿态解算无关
		for(u8 i = 0;i<3;i++)
		{
			imu->a_acc[i] = (s32)(acc[i] - 981 *imu->z_vec[i]);
		}
		
    
		//计算世界坐标下的运动加速度。坐标系为北西天 ///利用旋转矩阵变换a_acc，得到地理系中的加速度值w_acc
		for(u8 i = 0;i<3;i++)
		{
			s32 temp = 0;
			for(u8 j = 0;j<3;j++)
			{
				
				temp += imu->a_acc[j] *att_matrix[i][j];
			}
			imu->w_acc[i] = temp;
		}
		///这里通过上面的计算出的单位向量进行绕z轴转动的坐标变化，即将世界系转换到航向系，因为按照我们的操作习惯都是有头模式的遥控
		///再从地理系转换到航向系，得到航向系中加速度值h_acc
		w2h_2d_trans(imu->w_acc,imu_data.hx_vec,imu->h_acc);
		
		
    // 测量值与等效重力向量的叉积（计算向量误差）。
		///这里是把加速度计测量量的三轴向量，和载体坐标系下的（地理）z方向向量imu->z_vec做叉乘，得到和二者垂直的新向量
		///acc_norm表示的是重力方向来自加速度计，imu->z_vec表示的重力方向来自陀螺仪，这里计算向量外积实质上是在算二者夹角，准备进行融合滤波
    vec_err[X] =  (acc_norm[Y] * imu->z_vec[Z] - imu->z_vec[Y] * acc_norm[Z]);
    vec_err[Y] = -(acc_norm[X] * imu->z_vec[Z] - imu->z_vec[X] * acc_norm[Z]);
    vec_err[Z] = -(acc_norm[Y] * imu->z_vec[X] - imu->z_vec[Y] * acc_norm[X]);

///进行磁力计互补融合滤波
#ifdef USE_MAG

		//电子罗盘赋值为float矢量
		for(u8 i = 0;i<3;i++)
		{
			mag_val_f[i] = (float)mag_val[i];
		}	
			
		if(!(mag_val[X] ==0 && mag_val[Y] == 0 && mag_val[Z] == 0))
		{
			//把载体坐标下的罗盘数据转换到地理坐标下
			a2w_3d_trans(mag_val_f,imu->w_mag);
			//计算方向向量归一化系数（模的倒数）
			float l_re_tmp = my_sqrt_reciprocal(my_pow(imu->w_mag[0]) + my_pow(imu->w_mag[1]));
			//计算南北朝向向量
			mag_2d_w_vec[1][0] = imu->w_mag[0] *l_re_tmp;
			mag_2d_w_vec[1][1] = imu->w_mag[1] *l_re_tmp;
			//计算南北朝向误差(叉乘)，地理坐标中，水平面磁场方向向量应恒为南北 (1,0)
			mag_yaw_err = vec_2_cross_product(mag_2d_w_vec[1],mag_2d_w_vec[0]);
			//计算南北朝向向量点乘，判断同向或反向
			mag_err_dot_prudoct = vec_2_dot_product(mag_2d_w_vec[1],mag_2d_w_vec[0]);
			//若反向，直接给最大误差
			if(mag_err_dot_prudoct<0)
			{
				mag_yaw_err = my_sign(mag_yaw_err) *1.0f;
			}			
			//
			
		}
#endif
	
		for(u8 i = 0;i<3;i++)
		{
///这个没用？
#ifdef USE_EST_DEADZONE	
			if(state->G_reset == 0 && state->obs_en == 0)
			{
				vec_err[i] = my_deadzone(vec_err[i],0,imu->gacc_deadzone[i]);
			}
#endif	
			///用了这个？这里是利用加速度模长判断，如果水平方向加速度太大，就不能直接把加速度数据看作重力加速度数据，这一轮计算的偏差清零就不补偿了
#ifdef USE_LENGTH_LIM			
			if(acc_norm_l>1060 || acc_norm_l<900)
			{
				vec_err[X] = vec_err[Y] = vec_err[Z] = 0;
			}
#endif
		//误差积分  ///pi修正中I项 ///用于互补滤波的pi控制器
		vec_err_i[i] +=  LIMIT(vec_err[i],-0.1f,0.1f) *dT *ki_use;

		
	// 构造增量旋转（含融合纠正）。	
	//    d_angle[X] = (gyr[X] + (vec_err[X]  + vec_err_i[X]) * kp_use - mag_yaw_err *imu->z_vec[X] *kmp_use *RAD_PER_DEG) * dT / 2 ;
	//    d_angle[Y] = (gyr[Y] + (vec_err[Y]  + vec_err_i[Y]) * kp_use - mag_yaw_err *imu->z_vec[Y] *kmp_use *RAD_PER_DEG) * dT / 2 ;
	//    d_angle[Z] = (gyr[Z] + (vec_err[Z]  + vec_err_i[Z]) * kp_use - mag_yaw_err *imu->z_vec[Z] *kmp_use *RAD_PER_DEG) * dT / 2 ;
			
			
#ifdef USE_MAG
			///用的这个，在普通互补融合滤波基础上增加了磁力计补偿
			d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * kp_use + mag_yaw_err *imu->z_vec[i] *mkp_use) * dT / 2 ;
#else
			///这个没用，普通的互补融合滤波，用加速度计pi控制器输出补偿陀螺仪
			d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * kp_use ) * dT / 2 ;
#endif
		}
    // 计算姿态。    ///龙格库塔更新四元数，一阶龙格库塔法，wxyz初值为1000

    imu->w = imu->w            - imu->x*d_angle[X] - imu->y*d_angle[Y] - imu->z*d_angle[Z];
    imu->x = imu->w*d_angle[X] + imu->x            + imu->y*d_angle[Z] - imu->z*d_angle[Y];
    imu->y = imu->w*d_angle[Y] - imu->x*d_angle[Z] + imu->y            + imu->z*d_angle[X];
    imu->z = imu->w*d_angle[Z] + imu->x*d_angle[Y] - imu->y*d_angle[X] + imu->z;
		
		q_norm_l = my_sqrt_reciprocal(imu->w*imu->w + imu->x*imu->x + imu->y*imu->y + imu->z*imu->z);
    imu->w *= q_norm_l;
    imu->x *= q_norm_l;
    imu->y *= q_norm_l;
    imu->z *= q_norm_l;
		

  
  /////////////////////修正开关///////////////////////////
#ifdef USE_MAG
		if(state->M_fix_en==0)//磁力
		{
			mkp_use = 0;//不修正
			state->M_reset = 0;//罗盘修正不复位，清除复位标记
		}
		else
		{
			if(state->M_reset)//
			{
				//通过增量进行对准
				mkp_use = 10.0f;
				if(mag_yaw_err != 0 && ABS(mag_yaw_err)<0.01f)
				{
					state->M_reset = 0;//误差小于2的时候，清除复位标记
				}
			}
			else
			{
				mkp_use = state->mkp; //正常修正
			}
		}
#endif
		
		if(state->G_fix_en==0)//重力方向修正
		{
			kp_use = 0;//不修正
		}
		else
		{
			if(state->G_reset == 0)//正常修正
			{			
				kp_use = state->gkp;
				ki_use = state->gki;
			}
			else//快速修正，通过增量进行对准
			{
				kp_use = 10.0f;
				ki_use = 0.0f;
//				imu->est_speed_w[X] = imu->est_speed_w[Y] = 0;
//				imu->est_acc_w[X] = imu->est_acc_w[Y] = 0;
//				imu->est_acc_h[X] = imu->est_acc_h[Y] =0;
				
				//计算静态误差是否缩小
//				imu_reset_val += (ABS(vec_err[X]) + ABS(vec_err[Y])) *1000 *dT;
//				imu_reset_val -= 0.01f;
				imu_reset_val = (ABS(vec_err[X]) + ABS(vec_err[Y]));
				
				imu_reset_val = LIMIT(imu_reset_val,0,1.0f);
				
				if((imu_reset_val < 0.02f) && (state->M_reset == 0))
				{
					//计时
					reset_cnt += 2;
					if(reset_cnt>400)
					{
						reset_cnt = 0;
						state->G_reset = 0;//已经对准，清除复位标记
					}
				}
				else
				{
					reset_cnt = 0;
				}
			}
		}
}

///姿态角输出函数，利用旋转矩阵反解欧拉角，注意避免一下机头垂直向上时的特殊情况就行了（好像是处理陀螺仪死锁问题
static float t_temp;
void calculate_RPY()
{
	///////////////////////输出姿态角///////////////////////////////
	
		t_temp = LIMIT(1 - my_pow(att_matrix[2][0]),0,1);
		
		//imu_data.pit = asin(2*q1q3 - 2*q0q2)*57.30f;
	
		if(ABS(imu_data.z_vec[Z])>0.05f)//避免奇点的运算
		{
			imu_data.pit =  fast_atan2(att_matrix[2][0],my_sqrt(t_temp))*57.30f;
			imu_data.rol =  fast_atan2(att_matrix[2][1], att_matrix[2][2])*57.30f; 
			imu_data.yaw = -fast_atan2(att_matrix[1][0], att_matrix[0][0])*57.30f; 
		}
}


/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

