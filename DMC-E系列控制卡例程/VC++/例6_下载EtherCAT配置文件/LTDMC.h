#ifndef _DMC_LIB_H
#define _DMC_LIB_H

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#ifdef  __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif

typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef float               FLOAT;

typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
#define VC60 //VC6.0  
#ifdef VC60
	typedef unsigned _int64   uint64;           
	typedef signed   _int64   int64;
#else
	typedef unsigned long long   uint64;           
	typedef signed   long long   int64; 
#endif


typedef struct
{
    uint32  m_Time;
	int32   m_CommandPos;
	double  m_CommandVel;
	uint32	m_CommandAcc;
	int32	m_FpgaPos;
	double	m_FpgaVel;
	int32	m_EncoderPos;
	double	m_ErrorPos;
}struct_PidAdjustData;


#define __DMC_EXPORTS

//定义输入和输出
#ifdef __DMC_EXPORTS
	#define DMC_API __declspec(dllexport)
#else
	#define DMC_API __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C" {
#endif

//设置和读取打印模式
DMC_API short __stdcall dmc_set_debug_mode(WORD mode,const char* pFileName);
DMC_API short __stdcall dmc_get_debug_mode(WORD* mode,char* pFileName);

DMC_API short __stdcall dmc_board_init(void); 	//初始化控制卡
DMC_API short __stdcall dmc_board_init_onecard(WORD CardNo); 	////客户定制独立开卡 初始化一张控制卡
DMC_API short __stdcall dmc_board_close_onecard(WORD CardNo); 	////客户定制独立关卡 关闭一个控制卡
DMC_API short __stdcall	dmc_get_CardInfList(WORD* CardNum,DWORD* CardTypeList,WORD* CardIdList);//读取控制卡信息列表
DMC_API short __stdcall dmc_board_close(void);	//关闭控制卡
DMC_API short __stdcall dmc_board_reset(void);   //硬件复位
DMC_API short __stdcall dmc_board_reset_onecard(WORD CardNo);//客户定制单独硬件复位
DMC_API short __stdcall dmc_soft_reset(WORD CardNo);//控制卡软复位（pci卡） 热复位（总线卡）
DMC_API short __stdcall dmc_cool_reset(WORD CardNo);//控制卡冷复位
DMC_API short __stdcall dmc_original_reset(WORD CardNo);//控制卡初始复位
DMC_API short __stdcall dmc_get_card_ID (WORD CardNo,DWORD *CardID);	//读取控制卡卡号
DMC_API short __stdcall dmc_get_release_version(WORD CardNo,char *ReleaseVersion);//读取发布版本号
DMC_API short __stdcall dmc_get_card_version(WORD CardNo,DWORD *CardVersion);	//读取控制卡硬件版本
DMC_API short __stdcall dmc_get_card_soft_version(WORD CardNo,DWORD *FirmID,DWORD *SubFirmID);	//读取控制卡硬件的固件版本
DMC_API short __stdcall dmc_get_card_lib_version(DWORD *LibVer);	//读取控制卡动态库版本
DMC_API short __stdcall dmc_get_total_axes(WORD CardNo,DWORD *TotalAxis); 	//读取指定卡轴数
DMC_API short __stdcall dmc_get_total_liners(WORD CardNo,DWORD *TotalLiner); //读取指定卡插补坐标系数

DMC_API short __stdcall dmc_get_total_ionum(WORD CardNo,WORD *TotalIn,WORD *TotalOut);//获取本地IO点数
DMC_API short __stdcall dmc_get_total_adcnum(WORD CardNo,WORD* TotalIn,WORD* TotalOut);//获取本地ADDA输入输出数

//密码函数
DMC_API short __stdcall dmc_check_sn(WORD CardNo, const char * str_sn);
DMC_API short __stdcall dmc_write_sn(WORD CardNo, const char * str_sn);

/***********轴参数*************/
//脉冲模式		
DMC_API short __stdcall dmc_set_pulse_outmode(WORD CardNo,WORD axis,WORD outmode);	
DMC_API short __stdcall dmc_get_pulse_outmode(WORD CardNo,WORD axis,WORD* outmode);	
//脉冲当量
DMC_API short __stdcall dmc_set_equiv(WORD CardNo,WORD axis, double equiv);
DMC_API short __stdcall dmc_get_equiv(WORD CardNo,WORD axis, double *equiv);
//反向间隙(当量)
DMC_API short __stdcall dmc_set_backlash_unit(WORD CardNo,WORD axis,double backlash);
DMC_API short __stdcall dmc_get_backlash_unit(WORD CardNo,WORD axis,double *backlash);
//反向间隙(脉冲)
DMC_API short __stdcall dmc_set_backlash(WORD CardNo,WORD axis,long backlash);
DMC_API short __stdcall dmc_get_backlash(WORD CardNo,WORD axis,long *backlash);
 
/***********************************文件操作**************************************/
/*********************************************************************************************************
文件操作添加 总线卡
filetype
0-basic
1-gcode
2-setting
3-firewave
4-CAN configfile
100-trace data
*********************************************************************************************************/
DMC_API short __stdcall dmc_download_file(WORD CardNo, const char* pfilename, const char* pfilenameinControl,WORD filetype);
//下载内存文件 总线卡
DMC_API short __stdcall dmc_download_memfile(WORD CardNo, const char* pbuffer, uint32 buffsize, const char* pfilenameinControl,WORD filetype);
//上传文件
DMC_API short __stdcall dmc_upload_file(WORD CardNo, const char* pfilename, const char* pfilenameinControl, WORD filetype);
//上传内存文件
DMC_API short __stdcall dmc_upload_memfile(WORD CardNo, char* pbuffer, uint32 buffsize, const char* pfilenameinControl, uint32* puifilesize,WORD filetype);
//下载参数文件
DMC_API short __stdcall dmc_download_configfile(WORD CardNo,const char *FileName);
//下载固件文件
DMC_API short __stdcall dmc_download_firmware(WORD CardNo,const char *FileName);
//文件进度
DMC_API short __stdcall dmc_get_progress(WORD CardNo,float* process);

//安全参数
DMC_API short __stdcall dmc_set_softlimit(WORD CardNo,WORD axis,WORD enable, WORD source_sel,WORD SL_action, long N_limit,long P_limit);//设置软限位参数
DMC_API short __stdcall dmc_get_softlimit(WORD CardNo,WORD axis,WORD *enable, WORD *source_sel,WORD *SL_action,long *N_limit,long *P_limit);//读取软限位参数
DMC_API short __stdcall dmc_set_el_mode(WORD CardNo,WORD axis,WORD enable,WORD el_logic,WORD el_mode);//设置EL信号
DMC_API short __stdcall dmc_get_el_mode(WORD CardNo,WORD axis,WORD *enable,WORD *el_logic,WORD *el_mode);//读取设置EL信号
DMC_API short __stdcall dmc_set_emg_mode(WORD CardNo,WORD axis,WORD enable,WORD emg_logic);//设置EMG信号
DMC_API short __stdcall dmc_get_emg_mode(WORD CardNo,WORD axis,WORD *enable,WORD *emg_logic);//读取设置EMG信号

/*************************************单轴运动*****************************************/
//速度设置		
DMC_API short __stdcall dmc_set_profile(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double stop_vel);//设定速度曲线参数
DMC_API short __stdcall dmc_get_profile(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Tacc,double *Tdec,double *stop_vel);//读取速度曲线参数
//速度设置(脉冲当量)
DMC_API short __stdcall dmc_set_profile_unit(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_profile_unit(WORD CardNo,WORD axis,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);

//20160105增加新速度曲线以加速度 减速度 减减速度来表示(脉冲)
DMC_API short __stdcall dmc_set_profile_extern(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Ajerk,double Djerk,double stop_vel);
DMC_API short __stdcall dmc_get_profile_extern(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Tacc,double *Tdec,double *Ajerk,double *Djerk,double *stop_vel);
//速度曲线设置，加速度值表示(脉冲)
DMC_API short __stdcall dmc_set_acc_profile(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Acc,double Dec,double stop_vel);//设定速度曲线参数
DMC_API short __stdcall dmc_get_acc_profile(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Acc,double *Dec,double *stop_vel);//读取速度曲线参数
//速度曲线设置，加速度值表示(当量)
DMC_API short __stdcall dmc_set_profile_unit_acc(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_profile_unit_acc(WORD CardNo,WORD axis,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
DMC_API short __stdcall dmc_set_s_profile(WORD CardNo,WORD axis,WORD s_mode,double s_para);//设置平滑速度曲线参数
DMC_API short __stdcall dmc_get_s_profile(WORD CardNo,WORD axis,WORD s_mode,double *s_para);//读取平滑速度曲线参数 兼容DMC5800 s_mode改用指针返回参数值

//点位运动(脉冲)		
DMC_API short __stdcall dmc_pmove(WORD CardNo,WORD axis,long dist,WORD posi_mode);//指定轴做定长位移运动
//点位运动(当量)
DMC_API short __stdcall dmc_pmove_unit(WORD CardNo,WORD axis,double Dist,WORD posi_mode);
//单轴连续速度运动		
DMC_API short __stdcall dmc_vmove(WORD CardNo,WORD axis,WORD dir);
//指定轴做定长位移运动 同时发送速度和S时间(脉冲)
DMC_API short __stdcall dmc_pmove_extern(WORD CardNo, WORD axis, double dist,double Min_Vel, double Max_Vel, double Tacc, double Tdec, double stop_Vel, double s_para, WORD posi_mode);
//在线变位/变速(脉冲)
DMC_API short __stdcall dmc_reset_target_position(WORD CardNo,WORD axis,long dist,WORD posi_mode);//运动中改变目标位置
DMC_API short __stdcall dmc_change_speed(WORD CardNo,WORD axis,double Curr_Vel,double Taccdec);//在线改变指定轴的当前运动速度
DMC_API short __stdcall dmc_update_target_position(WORD CardNo,WORD axis,long dist,WORD posi_mode);//无论运动与否强行改变目标位置
//变速变位(当量)
DMC_API short __stdcall dmc_reset_target_position_unit(WORD CardNo,WORD axis, double New_Pos);
DMC_API short __stdcall dmc_change_speed_unit(WORD CardNo,WORD axis, double New_Vel,double Taccdec);
DMC_API short __stdcall dmc_update_target_position_unit(WORD CardNo,WORD axis, double New_Pos);

/******************************插补运动**********************************/
//3000系列速度设置(脉冲)
DMC_API short __stdcall dmc_set_vector_profile_multicoor(WORD CardNo,WORD Crd, double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_vector_profile_multicoor(WORD CardNo,WORD Crd, double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
DMC_API short __stdcall dmc_set_vector_s_profile_multicoor(WORD CardNo,WORD Crd,WORD s_mode,double s_para);//设置平滑速度曲线参数
DMC_API short __stdcall dmc_get_vector_s_profile_multicoor(WORD CardNo,WORD Crd,WORD s_mode,double *s_para);//读取平滑速度曲线参数

//插补速度参数(当量)
DMC_API short __stdcall dmc_set_vector_profile_unit(WORD CardNo,WORD Crd,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_vector_profile_unit(WORD CardNo,WORD Crd,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
DMC_API short __stdcall dmc_set_vector_s_profile(WORD CardNo,WORD Crd,WORD s_mode,double s_para);//设置平滑速度曲线参数
DMC_API short __stdcall dmc_get_vector_s_profile(WORD CardNo,WORD Crd,WORD s_mode,double *s_para);

//3000系列插补函数(脉冲)
DMC_API short __stdcall dmc_line_multicoor(WORD CardNo,WORD Crd,WORD axisNum,WORD *axisList,long *DistList,WORD posi_mode);	//指定轴直线插补运动
DMC_API short __stdcall dmc_arc_move_multicoor(WORD CardNo,WORD Crd,WORD *AxisList,long *Target_Pos,long *Cen_Pos,WORD Arc_Dir,WORD posi_mode);//圆弧插补运动
//单段插补(当量)
DMC_API short __stdcall dmc_line_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode);
DMC_API short __stdcall dmc_arc_move_center_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_arc_move_radius_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_arc_move_3points_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_rectangle_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mark_Pos,long num,WORD rect_mode,WORD posi_mode);

/********************PVT运动****************************/
//PVT运动 旧版
DMC_API short __stdcall dmc_PvtTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pVel);
DMC_API short __stdcall dmc_PtsTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pPercent);
DMC_API short __stdcall dmc_PvtsTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double velBegin,double velEnd);
DMC_API short __stdcall dmc_PttTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos);
DMC_API short __stdcall dmc_PvtMove(WORD CardNo,WORD AxisNum,WORD* AxisList);
//PVT缓冲区添加
DMC_API short __stdcall dmc_PttTable_add(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos);
DMC_API short __stdcall dmc_PtsTable_add(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pPercent);
DMC_API short __stdcall dmc_pvt_get_remain_space(WORD CardNo,WORD iaxis);//读取pvt剩余空间
/*****************************************************************************
PVT运动 总线卡新规划
******************************************************************************/
DMC_API short __stdcall dmc_pvt_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double *pVel);
DMC_API short __stdcall dmc_pts_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double *pPercent);
DMC_API short __stdcall dmc_pvts_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double velBegin,double velEnd);
DMC_API short __stdcall dmc_ptt_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos);
DMC_API short __stdcall dmc_pvt_move(WORD CardNo,WORD AxisNum,WORD* AxisList);

DMC_API short __stdcall dmc_SetGearProfile(WORD CardNo,WORD axis,WORD MasterType, WORD MasterIndex,long MasterEven,long SlaveEven,DWORD MasterSlope);
DMC_API short __stdcall dmc_GetGearProfile(WORD CardNo,WORD axis,WORD* MasterType, WORD* MasterIndex,long* MasterEven,long* SlaveEven,DWORD* MasterSlope);
DMC_API short __stdcall dmc_GearMove(WORD CardNo,WORD AxisNum,WORD* AxisList);

/************************回零运动*************************/	
DMC_API short __stdcall dmc_set_home_pin_logic(WORD CardNo,WORD axis,WORD org_logic,double filter);//设置HOME信号
DMC_API short __stdcall dmc_get_home_pin_logic(WORD CardNo,WORD axis,WORD *org_logic,double *filter);//读取设置HOME信号
DMC_API short __stdcall dmc_set_homemode(WORD CardNo,WORD axis,WORD home_dir,double vel,WORD mode,WORD EZ_count);//设定指定轴的回原点模式
DMC_API short __stdcall dmc_get_homemode(WORD CardNo,WORD axis,WORD *home_dir, double *vel_mode,WORD *home_mode,WORD *EZ_count);//读取指定轴的回原点模式
DMC_API short __stdcall dmc_home_move(WORD CardNo,WORD axis);//启动回零
DMC_API short __stdcall dmc_set_home_profile_unit(WORD CardNo,WORD axis,double Low_Vel,double High_Vel,double Tacc,double Tdec);//设置回零速度参数
DMC_API short __stdcall dmc_get_home_profile_unit(WORD CardNo,WORD axis,double* Low_Vel,double* High_Vel,double* Tacc,double* Tdec);//读取回零速度参数
DMC_API short __stdcall dmc_get_home_result(WORD CardNo,WORD axis,WORD* state);//读取回零执行状态
DMC_API short __stdcall dmc_set_home_position_unit(WORD CardNo,WORD axis,WORD enable,double position);
DMC_API short __stdcall dmc_get_home_position_unit(WORD CardNo,WORD axis,WORD *enable,double *position);
DMC_API short __stdcall dmc_set_el_home(WORD CardNo,WORD axis,WORD mode);

/***************************原点锁存******************************/
DMC_API short __stdcall dmc_set_homelatch_mode(WORD CardNo,WORD axis,WORD enable,WORD logic,WORD source);
DMC_API short __stdcall dmc_get_homelatch_mode(WORD CardNo,WORD axis,WORD* enable,WORD* logic,WORD* source);
DMC_API long __stdcall dmc_get_homelatch_flag(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_reset_homelatch_flag(WORD CardNo,WORD axis);
DMC_API long __stdcall dmc_get_homelatch_value(WORD CardNo,WORD axis);
/*****************************EZ锁存********************************/
DMC_API short __stdcall dmc_set_ezlatch_mode(WORD CardNo,WORD axis,WORD enable,WORD logic,WORD source);
DMC_API short __stdcall dmc_get_ezlatch_mode(WORD CardNo,WORD axis,WORD* enable,WORD* logic,WORD* source);
DMC_API long __stdcall dmc_get_ezlatch_flag(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_reset_ezlatch_flag(WORD CardNo,WORD axis);
DMC_API long __stdcall dmc_get_ezlatch_value(WORD CardNo,WORD axis);

/************************手轮运动函数*********************************/	
//一对一
DMC_API short __stdcall dmc_set_handwheel_inmode(WORD CardNo,WORD axis,WORD inmode,long multi,double vh);//设置输入手轮脉冲信号的工作方式
DMC_API short __stdcall dmc_get_handwheel_inmode(WORD CardNo,WORD axis,WORD *inmode,long *multi,double *vh);//读取输入手轮脉冲信号的工作方式
//一对一浮点型倍率
DMC_API short __stdcall dmc_set_handwheel_inmode_decimals(WORD CardNo,WORD axis,WORD inmode,double multi,double vh);
DMC_API short __stdcall dmc_get_handwheel_inmode_decimals(WORD CardNo,WORD axis,WORD *inmode,double *multi,double *vh);
DMC_API short __stdcall dmc_handwheel_move(WORD CardNo,WORD axis);
//设置手轮通道
DMC_API short __stdcall dmc_set_handwheel_channel(WORD CardNo,WORD index);
DMC_API short __stdcall dmc_get_handwheel_channel(WORD CardNo,WORD* index);
//一对多
DMC_API short __stdcall dmc_set_handwheel_inmode_extern(WORD CardNo,WORD inmode,WORD AxisNum,WORD* AxisList,long* multi);
DMC_API short __stdcall dmc_get_handwheel_inmode_extern(WORD CardNo,WORD* inmode,WORD* AxisNum,WORD* AxisList,long *multi);	
//一对多浮点型倍率
DMC_API short __stdcall dmc_set_handwheel_inmode_extern_decimals(WORD CardNo,WORD inmode,WORD AxisNum,WORD* AxisList,double* multi);	
DMC_API short __stdcall dmc_get_handwheel_inmode_extern_decimals(WORD CardNo,WORD* inmode,WORD* AxisNum,WORD* AxisList,double *multi);

/*********************************************************************************************************
手轮运动 新增总线的手轮模式  总线
*********************************************************************************************************/
DMC_API short __stdcall dmc_handwheel_set_axislist( WORD CardNo, WORD AxisSelIndex,WORD AxisNum,WORD* AxisList);
DMC_API short __stdcall dmc_handwheel_get_axislist( WORD CardNo,WORD AxisSelIndex, WORD* AxisNum, WORD* AxisList);
DMC_API short __stdcall dmc_handwheel_set_ratiolist( WORD CardNo, WORD AxisSelIndex, WORD StartRatioIndex, WORD RatioSelNum, double* RatioList);
DMC_API short __stdcall dmc_handwheel_get_ratiolist( WORD CardNo,WORD AxisSelIndex, WORD StartRatioIndex, WORD RatioSelNum,double* RatioList );
DMC_API short __stdcall dmc_handwheel_set_mode( WORD CardNo, WORD InMode, WORD IfHardEnable );
DMC_API short __stdcall dmc_handwheel_get_mode ( WORD CardNo, WORD* InMode, WORD*  IfHardEnable );
DMC_API short __stdcall dmc_handwheel_set_index( WORD CardNo, WORD AxisSelIndex,WORD RatioSelIndex );
DMC_API short __stdcall dmc_handwheel_get_index( WORD CardNo, WORD* AxisSelIndex,WORD* RatioSelIndex );
DMC_API short __stdcall dmc_handwheel_move( WORD CardNo, WORD ForceMove );
DMC_API short __stdcall dmc_handwheel_stop ( WORD CardNo );

/**************************高速锁存函数***************************/
/*************************************
LTC1	AXIS0	AXIS1	AXIS2	AXIS3
LTC2	AXIS4	AXIS5	AXIS6	AXIS7
***************************************/
DMC_API short __stdcall dmc_set_ltc_mode(WORD CardNo,WORD axis,WORD ltc_logic,WORD ltc_mode,double filter);//设置LTC信号
DMC_API short __stdcall dmc_get_ltc_mode(WORD CardNo,WORD axis,WORD*ltc_logic,WORD*ltc_mode,double *filter);//读取设置LTC信号
DMC_API short __stdcall dmc_set_latch_mode(WORD CardNo,WORD axis,WORD all_enable,WORD latch_source,WORD triger_chunnel);//设置锁存方式
DMC_API short __stdcall dmc_get_latch_mode(WORD CardNo,WORD axis,WORD *all_enable,WORD* latch_source,WORD* triger_chunnel);
DMC_API short __stdcall dmc_SetLtcOutMode(WORD CardNo,WORD axis,WORD enable,WORD bitno);//反相输出
DMC_API short __stdcall dmc_GetLtcOutMode(WORD CardNo,WORD axis,WORD *enable,WORD* bitno);
DMC_API short __stdcall dmc_get_latch_flag(WORD CardNo,WORD axis);//读取锁存器标志
DMC_API short __stdcall dmc_reset_latch_flag(WORD CardNo,WORD axis);//复位锁存器标志
DMC_API long __stdcall  dmc_get_latch_value(WORD CardNo,WORD axis);//读取编码器锁存器的值
DMC_API short __stdcall dmc_get_latch_value_unit(WORD CardNo,WORD axis,double* pos_by_mm);
DMC_API short __stdcall dmc_get_latch_flag_extern(WORD CardNo,WORD axis);//读取锁存器标志
DMC_API long __stdcall dmc_get_latch_value_extern(WORD CardNo,WORD axis,WORD index);//按索引取值
DMC_API short __stdcall dmc_set_latch_stop_time(WORD CardNo,WORD axis,long time);//触发急停时间
DMC_API short __stdcall dmc_get_latch_stop_time(WORD CardNo,WORD axis,long* time); 

/*********************************************************************************************************
高速锁存 新规划20170308 总线
*********************************************************************************************************/
//配置锁存器：锁存模式0-单次锁存，1-连续锁存；锁存边沿0-下降沿，1-上升沿，2-双边沿；滤波时间，单位us
DMC_API short __stdcall dmc_ltc_set_mode(WORD CardNo,WORD latch,WORD ltc_mode,WORD ltc_logic,double filter);
DMC_API short __stdcall dmc_ltc_get_mode(WORD CardNo,WORD latch,WORD *ltc_mode,WORD *ltc_logic,double *filter);
//配置锁存源：0-指令位置，1-编码器反馈位置
DMC_API short __stdcall dmc_ltc_set_source(WORD CardNo,WORD latch,WORD axis,WORD ltc_source);
DMC_API short __stdcall dmc_ltc_get_source(WORD CardNo,WORD latch,WORD axis,WORD *ltc_source);
//复位锁存器
DMC_API short __stdcall dmc_ltc_reset(WORD CardNo,WORD latch);
//读取锁存个数
DMC_API short __stdcall dmc_ltc_get_number(WORD CardNo,WORD latch,WORD axis,int *number);
//读取锁存值
DMC_API short __stdcall dmc_ltc_get_value_unit(WORD CardNo,WORD latch,WORD axis,double *value);

/*****************************位置比较函数****************************/
//单轴位置比较	
DMC_API short __stdcall dmc_compare_set_config(WORD CardNo,WORD axis,WORD enable, WORD cmp_source);//配置比较器
DMC_API short __stdcall dmc_compare_get_config(WORD CardNo,WORD axis,WORD *enable, WORD *cmp_source);//读取配置比较器
DMC_API short __stdcall dmc_compare_clear_points(WORD CardNo,WORD cmp);//清除所有比较点
DMC_API short __stdcall dmc_compare_add_point(WORD CardNo,WORD cmp,long pos,WORD dir, WORD action,DWORD actpara);//添加比较点
DMC_API short __stdcall dmc_compare_get_current_point(WORD CardNo,WORD cmp,long *pos);//读取当前比较点
DMC_API short __stdcall dmc_compare_get_points_runned(WORD CardNo,WORD cmp,long *pointNum);//查询已经比较过的点
DMC_API short __stdcall dmc_compare_get_points_remained(WORD CardNo,WORD cmp,long *pointNum);//查询可以加入的比较点数量

//二维位置比较
DMC_API short __stdcall dmc_compare_set_config_extern(WORD CardNo,WORD enable, WORD cmp_source);//配置比较器
DMC_API short __stdcall dmc_compare_get_config_extern(WORD CardNo,WORD *enable, WORD *cmp_source);//读取配置比较器
DMC_API short __stdcall dmc_compare_clear_points_extern(WORD CardNo);//清除所有比较点
DMC_API short __stdcall dmc_compare_add_point_extern(WORD CardNo,WORD* axis,long* pos,WORD* dir, WORD action,DWORD actpara);//添加两轴位置比较点
DMC_API short __stdcall dmc_compare_get_current_point_extern(WORD CardNo,long *pos);//读取当前比较点
DMC_API short __stdcall dmc_compare_add_point_extern_unit(WORD CardNo,WORD* axis,double* pos,WORD* dir, WORD action,DWORD actpara);//添加两轴位置比较点
DMC_API short __stdcall dmc_compare_get_current_point_extern_unit(WORD CardNo,double *pos);//读取当前比较点
DMC_API short __stdcall dmc_compare_get_points_runned_extern(WORD CardNo,long *pointNum);//查询已经比较过的点
DMC_API short __stdcall dmc_compare_get_points_remained_extern(WORD CardNo,long *pointNum);//查询可以加入的比较点数量

//多组位置比较
DMC_API short __stdcall dmc_compare_set_config_multi(WORD CardNo,WORD queue,WORD enable, WORD axis, WORD cmp_source);//配置比较器
DMC_API short __stdcall dmc_compare_get_config_multi(WORD CardNo, WORD queue,WORD* enable, WORD* axis, WORD* cmp_source);//读取配置比较器
DMC_API short __stdcall dmc_compare_add_point_multi(WORD CardNo, WORD cmp,int32 pos, WORD dir,  WORD action, DWORD actpara,double times);//添加比较点 增强

//高速位置比较
DMC_API short __stdcall dmc_hcmp_set_mode(WORD CardNo,WORD hcmp, WORD cmp_mode);//设置高速比较模式
DMC_API short __stdcall dmc_hcmp_get_mode(WORD CardNo,WORD hcmp, WORD* cmp_mode);
//高速比较模式扩展
DMC_API short __stdcall dmc_hcmp_set_config_extern(WORD CardNo,WORD hcmp,WORD axis, WORD cmp_source, WORD cmp_logic,WORD cmp_mode,long dist,long time);
DMC_API short __stdcall dmc_hcmp_get_config_extern(WORD CardNo,WORD hcmp,WORD* axis, WORD* cmp_source, WORD* cmp_logic,WORD* cmp_mode,long* dist,long* time);

DMC_API short __stdcall dmc_hcmp_set_config(WORD CardNo,WORD hcmp,WORD axis, WORD cmp_source, WORD cmp_logic,long time);//设置高速比较参数
DMC_API short __stdcall dmc_hcmp_get_config(WORD CardNo,WORD hcmp,WORD* axis, WORD* cmp_source, WORD* cmp_logic,long* time);
DMC_API short __stdcall dmc_hcmp_add_point(WORD CardNo,WORD hcmp, long cmp_pos);
DMC_API short __stdcall dmc_hcmp_set_liner(WORD CardNo,WORD hcmp, long Increment,long Count);//设置线性模式参数
DMC_API short __stdcall dmc_hcmp_get_liner(WORD CardNo,WORD hcmp, long* Increment,long* Count);
DMC_API short __stdcall dmc_hcmp_get_current_state(WORD CardNo,WORD hcmp,long *remained_points,long *current_point,long *runned_points); //读取高速比较状态
DMC_API short __stdcall dmc_hcmp_clear_points(WORD CardNo,WORD hcmp);
DMC_API short __stdcall dmc_read_cmp_pin(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_write_cmp_pin(WORD CardNo,WORD axis, WORD on_off);//控制cmp端口输出

//二维高速位置比较功能
DMC_API short __stdcall dmc_hcmp_2d_set_enable(WORD CardNo,WORD hcmp, WORD cmp_enable);
DMC_API short __stdcall dmc_hcmp_2d_get_enable(WORD CardNo,WORD hcmp, WORD *cmp_enable);
DMC_API short __stdcall dmc_hcmp_2d_set_config(WORD CardNo,WORD hcmp,WORD cmp_mode,WORD x_axis, WORD x_cmp_source, WORD y_axis, WORD y_cmp_source, long error,WORD cmp_logic,long time,WORD pwm_enable,double duty,long freq,WORD port_sel,WORD pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_get_config(WORD CardNo,WORD hcmp,WORD *cmp_mode,WORD *x_axis, WORD *x_cmp_source, WORD *y_axis, WORD *y_cmp_source, long *error,WORD *cmp_logic,long *time,WORD *pwm_enable,double *duty,long *freq,WORD *port_sel,WORD *pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_add_point(WORD CardNo,WORD hcmp, long x_cmp_pos, long y_cmp_pos);
DMC_API short __stdcall dmc_hcmp_2d_get_current_state(WORD CardNo,WORD hcmp,long *remained_points,long *x_current_point,long *y_current_point,long *runned_points,WORD *current_state); 
DMC_API short __stdcall dmc_hcmp_2d_clear_points(WORD CardNo,WORD hcmp);
DMC_API short __stdcall dmc_hcmp_2d_force_output(WORD CardNo,WORD hcmp,WORD enable);

/********************通用IO功能**************************/
//通用IO
DMC_API short __stdcall dmc_read_inbit(WORD CardNo,WORD bitno);//读取输入口的状态
DMC_API short __stdcall dmc_write_outbit(WORD CardNo,WORD bitno,WORD on_off);//设置输出口的状态
DMC_API short __stdcall dmc_read_outbit(WORD CardNo,WORD bitno);//读取输出口的状态
DMC_API DWORD __stdcall dmc_read_inport(WORD CardNo,WORD portno);//读取输入端口的值
DMC_API DWORD __stdcall dmc_read_outport(WORD CardNo,WORD portno);//读取输出端口的值
DMC_API short __stdcall dmc_write_outport(WORD CardNo,WORD portno,DWORD outport_val);//设置所有输出端口的值

DMC_API short __stdcall dmc_write_outport_16X(WORD CardNo,WORD portno,DWORD outport_val);//设置通用输出端口的值

//虚拟IO映射
DMC_API short __stdcall dmc_set_io_map_virtual(WORD CardNo,WORD bitno,WORD MapIoType,WORD MapIoIndex,double Filter);
DMC_API short __stdcall dmc_get_io_map_virtual(WORD CardNo,WORD bitno,WORD* MapIoType,WORD* MapIoIndex,double* Filter);
DMC_API short __stdcall dmc_read_inbit_virtual(WORD CardNo,WORD bitno); //读取输入口的状态

DMC_API short __stdcall dmc_reverse_outbit(WORD CardNo,WORD bitno,double reverse_time);//IO延时翻转
DMC_API short __stdcall dmc_set_io_count_mode(WORD CardNo,WORD bitno,WORD mode,double filter);//设置IO计数模式
DMC_API short __stdcall dmc_get_io_count_mode(WORD CardNo,WORD bitno,WORD *mode,double* filter);
DMC_API short __stdcall dmc_set_io_count_value(WORD CardNo,WORD bitno,DWORD CountValue);//设置IO计数值
DMC_API short __stdcall dmc_get_io_count_value(WORD CardNo,WORD bitno,DWORD* CountValue);

/*********************专用IO信号************************/
DMC_API short __stdcall dmc_set_axis_io_map(WORD CardNo,WORD Axis,WORD IoType,WORD MapIoType,WORD MapIoIndex,double Filter);
DMC_API short __stdcall dmc_get_axis_io_map(WORD CardNo,WORD Axis,WORD IoType,WORD* MapIoType,WORD* MapIoIndex,double* Filter);
DMC_API short __stdcall dmc_set_special_input_filter(WORD CardNo,double Filter);//设置所有专用IO滤波时间

//3410专用 回原点减速信号配置
DMC_API short __stdcall dmc_set_sd_mode(WORD CardNo,WORD axis,WORD enable,WORD sd_logic,WORD sd_mode);//设置SD信号
DMC_API short __stdcall dmc_get_sd_mode(WORD CardNo,WORD axis,WORD* enable,WORD *sd_logic,WORD *sd_mode);//读取设置SD信号
//专用IO
DMC_API short __stdcall dmc_set_inp_mode(WORD CardNo,WORD axis,WORD enable,WORD inp_logic);//设置INP信号
DMC_API short __stdcall dmc_get_inp_mode(WORD CardNo,WORD axis,WORD *enable,WORD *inp_logic);//读取设置INP信号
DMC_API short __stdcall dmc_set_rdy_mode(WORD CardNo,WORD axis,WORD enable,WORD rdy_logic);//设置RDY信号
DMC_API short __stdcall dmc_get_rdy_mode(WORD CardNo,WORD axis,WORD* enable,WORD* rdy_logic);//读取设置RDY信号
DMC_API short __stdcall dmc_set_erc_mode(WORD CardNo,WORD axis,WORD enable,WORD erc_logic,WORD erc_width,WORD erc_off_time);//设置ERC信号
DMC_API short __stdcall dmc_get_erc_mode(WORD CardNo,WORD axis,WORD *enable,WORD *erc_logic, WORD *erc_width,WORD *erc_off_time);//读取设置ERC信号
DMC_API short __stdcall dmc_set_alm_mode(WORD CardNo,WORD axis,WORD enable,WORD alm_logic,WORD alm_action);//设置ALM信号
DMC_API short __stdcall dmc_get_alm_mode(WORD CardNo,WORD axis,WORD *enable,WORD *alm_logic,WORD *alm_action);//读取设置ALM信号
DMC_API short __stdcall dmc_set_ez_mode(WORD CardNo,WORD axis,WORD ez_logic,WORD ez_mode,double filter);//设置EZ信号
DMC_API short __stdcall dmc_get_ez_mode(WORD CardNo,WORD axis,WORD *ez_logic,WORD *ez_mode,double *filter);//读取设置EZ信号

DMC_API short __stdcall dmc_write_sevon_pin(WORD CardNo,WORD axis,WORD on_off);//输出SEVON信号
DMC_API short __stdcall dmc_read_sevon_pin(WORD CardNo,WORD axis);//读取SEVON信号
DMC_API short __stdcall dmc_read_rdy_pin(WORD CardNo,WORD axis);//读取RDY状态
DMC_API short __stdcall dmc_write_erc_pin(WORD CardNo,WORD axis,WORD on_off);//控制ERC信号输出
DMC_API short __stdcall dmc_read_erc_pin(WORD CardNo,WORD axis); 	
DMC_API short __stdcall dmc_write_sevrst_pin(WORD CardNo,WORD axis,WORD on_off);//输出伺服复位信号
DMC_API short __stdcall dmc_read_sevrst_pin(WORD CardNo,WORD axis);//读伺服复位信号

//外部减速停止信号及减速停止时间设置
DMC_API short __stdcall dmc_set_io_dstp_mode(WORD CardNo,WORD axis,WORD enable,WORD logic);//enable:0-禁用，1-按时间减速停止，2-按距离减速停止
DMC_API short __stdcall dmc_get_io_dstp_mode(WORD CardNo,WORD axis,WORD *enable,WORD *logic); 
//减速停止时间
DMC_API short __stdcall dmc_set_dec_stop_time(WORD CardNo,WORD axis,double time);
DMC_API short __stdcall dmc_get_dec_stop_time(WORD CardNo,WORD axis,double *time);
//插补减速停止信号和减速时间设置
DMC_API short __stdcall dmc_set_vector_dec_stop_time(WORD CardNo,WORD Crd,double time);
DMC_API short __stdcall dmc_get_vector_dec_stop_time(WORD CardNo,WORD Crd,double *time);
//减速停止距离
DMC_API short __stdcall dmc_set_dec_stop_dist(WORD CardNo,WORD axis,long dist);
DMC_API short __stdcall dmc_get_dec_stop_dist(WORD CardNo,WORD axis,long *dist);
DMC_API short __stdcall dmc_set_io_dstp_bitno(WORD CardNo,WORD axis,WORD bitno,double filter);//设置通用输入口的一位减速停止IO口
DMC_API short __stdcall dmc_get_io_dstp_bitno(WORD CardNo,WORD axis,WORD *bitno,double* filter);

/************************编码器函数**********************/	
DMC_API short __stdcall dmc_set_counter_inmode(WORD CardNo,WORD axis,WORD mode);//设定编码器的计数方式
DMC_API short __stdcall dmc_get_counter_inmode(WORD CardNo,WORD axis,WORD *mode);//读取编码器的计数方式
//编码器值(脉冲)
DMC_API long __stdcall dmc_get_encoder(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_set_encoder(WORD CardNo,WORD axis,long encoder_value);
//编码器值(当量)
DMC_API short __stdcall dmc_set_encoder_unit(WORD CardNo,WORD axis, double pos);
DMC_API short __stdcall dmc_get_encoder_unit(WORD CardNo,WORD axis, double * pos);
//手轮编码器 总线卡
DMC_API short __stdcall dmc_set_handwheel_encoder(WORD CardNo,WORD channel, long pos);
DMC_API short __stdcall dmc_get_handwheel_encoder(WORD CardNo,WORD channel, long * pos);
//设置辅助编码模式
DMC_API short __stdcall dmc_set_extra_encoder_mode(WORD CardNo,WORD channel,WORD inmode,WORD multi);
DMC_API short __stdcall dmc_get_extra_encoder_mode(WORD CardNo,WORD channel,WORD* inmode,WORD* multi);
//设置辅助编码器值
DMC_API short __stdcall dmc_set_extra_encoder(WORD CardNo,WORD channel, int pos);
DMC_API short __stdcall dmc_get_extra_encoder(WORD CardNo,WORD channel, int * pos);

/*********************位置计数控制***************************/
//当前位置(脉冲)
DMC_API short __stdcall dmc_set_position(WORD CardNo,WORD axis,long current_position);
DMC_API long __stdcall dmc_get_position(WORD CardNo,WORD axis);	
//当前位置(当量)
DMC_API short __stdcall dmc_set_position_unit(WORD CardNo,WORD axis, double pos);
DMC_API short __stdcall dmc_get_position_unit(WORD CardNo,WORD axis, double * pos);

/**************************运动状态********************************/
//轴状态
DMC_API double __stdcall dmc_read_current_speed(WORD CardNo,WORD axis);	//读取指定轴的当前速度(脉冲)
DMC_API short __stdcall dmc_read_current_speed_unit(WORD CardNo,WORD axis, double *current_speed);//读取当前速度(当量)
DMC_API double __stdcall dmc_read_vector_speed(WORD CardNo);	//读取当前卡的插补速度
DMC_API long __stdcall dmc_get_target_position(WORD CardNo,WORD axis);	//读取指定轴的目标位置
DMC_API short __stdcall dmc_get_target_position_unit(WORD CardNo,WORD axis, double * pos);//读取指定轴的目标位置(当量)
DMC_API short __stdcall dmc_check_done(WORD CardNo,WORD axis);	//读取指定轴的运动状态

DMC_API DWORD __stdcall dmc_axis_io_status(WORD CardNo,WORD axis);//读取指定轴有关运动信号的状态
DMC_API short __stdcall dmc_stop(WORD CardNo,WORD axis,WORD stop_mode);//单轴停止
DMC_API short __stdcall dmc_check_done_multicoor(WORD CardNo,WORD Crd);//插补运动状态
DMC_API short __stdcall dmc_stop_multicoor(WORD CardNo,WORD Crd,WORD stop_mode);//停止插补器
DMC_API short __stdcall dmc_emg_stop(WORD CardNo);//紧急停止所有轴
DMC_API short __stdcall dmc_LinkState(WORD CardNo,WORD* State);//连接状态
DMC_API short __stdcall dmc_get_axis_run_mode(WORD CardNo, WORD axis,WORD* run_mode);//读取指定轴的运动模式
DMC_API short __stdcall dmc_get_stop_reason(WORD CardNo,WORD axis,long* StopReason);//读取停止原因
DMC_API short __stdcall dmc_clear_stop_reason(WORD CardNo,WORD axis);//清除停止原因

//trace功能
DMC_API short __stdcall dmc_set_trace(WORD CardNo,WORD axis,WORD enable);
DMC_API short __stdcall dmc_get_trace(WORD CardNo,WORD axis,WORD* enable);
DMC_API short __stdcall dmc_read_trace_data(WORD CardNo,WORD axis,WORD data_option,long* ReceiveSize,double* time,double* data,long* remain_num);
DMC_API short __stdcall dmc_trace_start(WORD CardNo,WORD AxisNum,WORD *AxisList);
DMC_API short __stdcall dmc_trace_stop(WORD CardNo);

//弧长计算
DMC_API short __stdcall dmc_calculate_arclength_center(double* start_pos,double *target_pos,double *cen_pos, WORD arc_dir,double circle,double* ArcLength);

/*********************************连续插补函数**************************************
连续插补的速度函数和单段插补相同
************************************************************************************/
DMC_API short __stdcall dmc_conti_open_list (WORD CardNo,WORD Crd,WORD AxisNum,WORD *AxisList);//打开连续缓存区
DMC_API short __stdcall dmc_conti_close_list(WORD CardNo,WORD Crd);//关闭连续缓存区
DMC_API short __stdcall dmc_conti_reset_list(WORD CardNo,WORD Crd);//复位连续缓存区
DMC_API short __stdcall dmc_conti_stop_list (WORD CardNo,WORD Crd,WORD stop_mode);//连续插补中停止
DMC_API short __stdcall dmc_conti_pause_list(WORD CardNo,WORD Crd);//连续插补中暂停
DMC_API short __stdcall dmc_conti_start_list(WORD CardNo,WORD Crd);//开始连续插补
DMC_API short __stdcall dmc_conti_get_run_state(WORD CardNo,WORD Crd);//0-运行，1-暂停，2-正常停止，3-未启动，4-空闲
DMC_API long __stdcall dmc_conti_remain_space (WORD CardNo,WORD Crd);//查连续插补剩余缓存数
DMC_API long __stdcall dmc_conti_read_current_mark (WORD CardNo,WORD Crd);//读取当前连续插补段的标号

//blend模式
DMC_API short __stdcall dmc_conti_set_blend(WORD CardNo,WORD Crd,WORD enable);
DMC_API short __stdcall dmc_conti_get_blend(WORD CardNo,WORD Crd,WORD* enable);
DMC_API short __stdcall dmc_conti_set_override(WORD CardNo,WORD Crd,double Percent);//设置插补中速度比例
DMC_API short __stdcall dmc_conti_change_speed_ratio (WORD CardNo,WORD Crd,double percent);//设置插补中动态变速
//小线段前瞻
DMC_API short __stdcall dmc_conti_set_lookahead_mode(WORD CardNo,WORD Crd,WORD enable,long LookaheadSegments,double PathError,double LookaheadAcc);
DMC_API short __stdcall dmc_conti_get_lookahead_mode(WORD CardNo,WORD Crd,WORD* enable,long* LookaheadSegments,double* PathError,double* LookaheadAcc);

//连续插补IO功能
DMC_API short __stdcall dmc_conti_wait_input(WORD CardNo,WORD Crd,WORD bitno,WORD on_off,double TimeOut,long mark);
DMC_API short __stdcall dmc_conti_delay_outbit_to_start(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_delay_outbit_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_time,double ReverseTime);
DMC_API short __stdcall dmc_conti_ahead_outbit_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_accurate_outbit_unit(WORD CardNo, WORD Crd, WORD cmp_no,WORD on_off,WORD axis,double abs_pos,WORD pos_source,double ReverseTime);
DMC_API short __stdcall dmc_conti_write_outbit(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ReverseTime);
DMC_API short __stdcall dmc_conti_clear_io_action(WORD CardNo, WORD Crd, DWORD Io_Mask);
DMC_API short __stdcall dmc_conti_set_pause_output(WORD CardNo,WORD Crd,WORD action,long mask,long state);
DMC_API short __stdcall dmc_conti_get_pause_output(WORD CardNo,WORD Crd,WORD* action,long* mask,long* state);
DMC_API short __stdcall dmc_conti_delay(WORD CardNo, WORD Crd,double delay_time,long mark);//延时指令

DMC_API short __stdcall  dmc_conti_reverse_outbit(WORD CardNo, WORD Crd, WORD bitno,double reverse_time);//IO输出延时翻转
DMC_API short __stdcall  dmc_conti_delay_outbit(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_time);//IO延时输出

//连续插补轨迹段
DMC_API short __stdcall dmc_conti_line_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_center_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_radius_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_3points_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_rectangle_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mark_Pos,long num,WORD rect_mode,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_pmove_unit(WORD CardNo,WORD Crd,WORD axis,double dist,WORD posi_mode,WORD mode,long imark);
DMC_API short __stdcall dmc_conti_set_involute_mode(WORD CardNo,WORD Crd,WORD mode);//设置螺旋线插补运动模式
DMC_API short __stdcall dmc_conti_get_involute_mode(WORD CardNo,WORD Crd,WORD* mode);
DMC_API short __stdcall dmc_set_gear_follow_profile(WORD CardNo,WORD axis,WORD enable,WORD master_axis,double ratio);//双Z轴
DMC_API short __stdcall dmc_get_gear_follow_profile(WORD CardNo,WORD axis,WORD* enable,WORD* master_axis,double* ratio);

DMC_API short __stdcall dmc_conti_line_unit_extern(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Cen_Pos,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_center_unit_extern(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Cen_Pos,double Arc_Radius,WORD posi_mode,long mark);

/*********************************PWM功能函数*******************************/
//PWM控制
DMC_API short __stdcall dmc_set_pwm_pin(WORD CardNo,WORD portno,WORD ON_OFF, double dfreqency,double dduty);	
DMC_API short __stdcall dmc_get_pwm_pin(WORD CardNo,WORD portno,WORD *ON_OFF, double *dfreqency,double *dduty);
//PWM功能
DMC_API short __stdcall dmc_set_pwm_enable(WORD CardNo,WORD enable);
DMC_API short __stdcall dmc_get_pwm_enable(WORD CardNo,WORD* enable);
DMC_API short __stdcall dmc_set_pwm_output(WORD CardNo, WORD PwmNo,double fDuty, double fFre);
DMC_API short __stdcall dmc_get_pwm_output(WORD CardNo,WORD PwmNo,double* fDuty, double* fFre);
DMC_API short __stdcall dmc_conti_set_pwm_output(WORD CardNo,WORD Crd, WORD PwmNo,double fDuty, double fFre);

//高速PWM功能
DMC_API short __stdcall dmc_set_pwm_enable_extern(WORD CardNo,WORD channel, WORD enable);
DMC_API short __stdcall dmc_get_pwm_enable_extern(WORD CardNo,WORD channel, WORD* enable);

/**********PWM速度跟随**************
mode:跟随模式0-不跟随 保持状态 1-不跟随 输出低电平2-不跟随 输出高电平3-跟随 占空比自动调整4-跟随 频率自动调整
MaxVel:最大运行速度，单位unit
MaxValue:最大输出占空比或者频率
OutValue：设置输出频率或占空比
*************************************/
DMC_API short __stdcall dmc_conti_set_pwm_follow_speed(WORD CardNo,WORD Crd,WORD pwm_no,WORD mode,double MaxVel,double MaxValue,double OutValue);
DMC_API short __stdcall dmc_conti_get_pwm_follow_speed(WORD CardNo,WORD Crd,WORD pwm_no,WORD* mode,double* MaxVel,double* MaxValue,double* OutValue);
//设置PWM开关对应的占空比
DMC_API short __stdcall dmc_set_pwm_onoff_duty(WORD CardNo, WORD PwmNo,double fOnDuty, double fOffDuty);
DMC_API short __stdcall dmc_get_pwm_onoff_duty(WORD CardNo, WORD PwmNo,double* fOnDuty, double* fOffDuty);
DMC_API short __stdcall dmc_conti_delay_pwm_to_start(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_delay_pwm_to_stop(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double delay_time,double ReverseTime);
DMC_API short __stdcall dmc_conti_ahead_pwm_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_write_pwm(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double ReverseTime);

/*********************ADDA输出******************************/
//PWM转DA输出
DMC_API short __stdcall dmc_set_da_enable(WORD CardNo,WORD enable);
DMC_API short __stdcall dmc_get_da_enable(WORD CardNo,WORD* enable);
DMC_API short __stdcall dmc_set_da_output(WORD CardNo, WORD channel,double Vout);
DMC_API short __stdcall dmc_get_da_output(WORD CardNo,WORD channel,double* Vout);
//读取AD输入
DMC_API short __stdcall dmc_get_ad_input(WORD CardNo,WORD channel,double* Vout);
//设置连续插补DA输出
DMC_API short __stdcall dmc_conti_set_da_output(WORD CardNo, WORD Crd, WORD channel,double Vout);
//设置连续DA使能
DMC_API short __stdcall dmc_conti_set_da_enable(WORD CardNo, WORD Crd, WORD enable,WORD channel,long mark);
/**********DA速度跟随**************
da_no:通道号
MaxVel:最大运行速度，单位unit
MaxValue:最大电压
*************************************/
DMC_API short __stdcall dmc_conti_set_da_follow_speed(WORD CardNo,WORD Crd,WORD da_no,double MaxVel,double MaxValue,double acc_offset,double dec_offset,double acc_dist,double dec_dist);
DMC_API short __stdcall dmc_conti_get_da_follow_speed(WORD CardNo,WORD Crd,WORD da_no,double* MaxVel,double* MaxValue,double* acc_offset,double* dec_offset,double* acc_dist,double* dec_dist);

/******************************CAN IO***********************************/
//baud:0-1M 1-800 2-500 3-250 4-125 5-100
DMC_API short __stdcall dmc_set_can_state(WORD CardNo,WORD NodeNum,WORD state,WORD baud);//0-断开；1-连接；2-复位后自动连接
DMC_API short __stdcall dmc_get_can_state(WORD CardNo,WORD* NodeNum,WORD* state);////0-断开；1-连接；2-异常
DMC_API short __stdcall dmc_get_can_errcode(WORD CardNo,WORD *Errcode);
DMC_API short __stdcall dmc_write_can_outbit(WORD CardNo,WORD Node,WORD bitno,WORD on_off);
DMC_API short __stdcall dmc_read_can_outbit(WORD CardNo,WORD Node,WORD bitno);
DMC_API short __stdcall dmc_read_can_inbit(WORD CardNo,WORD Node,WORD bitno);
DMC_API short __stdcall dmc_write_can_outport(WORD CardNo,WORD Node,WORD PortNo,DWORD outport_val);
DMC_API DWORD __stdcall dmc_read_can_outport(WORD CardNo,WORD Node,WORD PortNo);
DMC_API DWORD __stdcall dmc_read_can_inport(WORD CardNo,WORD Node,WORD PortNo);
//读取CAN通讯错误
DMC_API short __stdcall dmc_get_can_errcode_extern(WORD CardNo,WORD *Errcode,WORD *msg_losed, WORD *emg_msg_num, WORD *lostHeartB, WORD *EmgMsg);

DMC_API long __stdcall  dmc_set_profile_limit(WORD CardNo,WORD axis,double Max_Vel,double Max_Acc,double EvenTime);
DMC_API long __stdcall  dmc_get_profile_limit(WORD CardNo,WORD axis,double* Max_Vel,double* Max_Acc,double* EvenTime);
DMC_API long __stdcall  dmc_set_vector_profile_limit(WORD CardNo,WORD Crd,double Max_Vel,double Max_Acc,double EvenTime);
DMC_API long __stdcall  dmc_get_vector_profile_limit(WORD CardNo,WORD Crd,double* Max_Vel,double* Max_Acc,double* EvenTime);
//小圆限速使能
DMC_API short __stdcall dmc_set_arc_limit(WORD CardNo,WORD Crd,WORD Enable,double MaxCenAcc,double MaxArcError);
DMC_API short __stdcall dmc_get_arc_limit(WORD CardNo,WORD Crd,WORD* Enable,double* MaxCenAcc,double* MaxArcError);

//DMC_API short __stdcall dmc_get_axis_debug_state(WORD CardNo,WORD axis,struct_DebugPara* pack);

//软锁存功能
//配置锁存器：锁存模式0-单次锁存，1-连续锁存；锁存边沿0-下降沿，1-上升沿，2-双边沿；滤波时间，单位us
DMC_API short __stdcall dmc_softltc_set_mode(WORD CardNo,WORD latch,WORD ltc_enable,WORD ltc_mode,WORD ltc_inbit,WORD ltc_logic,double filter);
DMC_API short __stdcall dmc_softltc_get_mode(WORD CardNo,WORD latch,WORD *ltc_enable,WORD *ltc_mode,WORD *ltc_inbit,WORD *ltc_logic,double *filter);
//配置锁存源：0-指令位置，1-编码器反馈位置
DMC_API short __stdcall dmc_softltc_set_source(WORD CardNo,WORD latch,WORD axis,WORD ltc_source);
DMC_API short __stdcall dmc_softltc_get_source(WORD CardNo,WORD latch,WORD axis,WORD *ltc_source);
//复位锁存器
DMC_API short __stdcall dmc_softltc_reset(WORD CardNo,WORD latch);
//读取锁存个数
DMC_API short __stdcall dmc_softltc_get_number(WORD CardNo,WORD latch,WORD axis,int *number);
//读取锁存值
DMC_API short __stdcall dmc_softltc_get_value_unit(WORD CardNo,WORD latch,WORD axis,double *value);

DMC_API short __stdcall dmc_set_IoFilter(WORD CardNo,WORD bitno, double filter);
DMC_API short __stdcall dmc_get_IoFilter(WORD CardNo,WORD bitno, double *filter);

//螺距补偿
DMC_API short __stdcall dmc_set_lsc_index_value (WORD CardNo, WORD axis,WORD IndexID, long IndexValue);
DMC_API short __stdcall dmc_get_lsc_index_value(WORD CardNo, WORD axis,WORD IndexID, long *IndexValue);

DMC_API short __stdcall dmc_set_lsc_config(WORD CardNo, WORD axis,WORD Origin, DWORD Interal,DWORD NegIndex,DWORD PosIndex,double Ratio);
DMC_API short __stdcall dmc_get_lsc_config(WORD CardNo, WORD axis,WORD *Origin, DWORD *Interal,DWORD *NegIndex,DWORD *PosIndex,double *Ratio);

//看门狗
DMC_API short __stdcall dmc_set_watchdog(WORD CardNo,WORD enable,DWORD time);
DMC_API short __stdcall dmc_call_watchdog(WORD CardNo);

DMC_API short __stdcall dmc_read_diagnoseData(WORD CardNo);	
DMC_API short __stdcall dmc_conti_set_cmd_end(WORD CardNo,WORD Crd,WORD enable);

//区域软限位
DMC_API short __stdcall dmc_set_zone_limit_config(WORD CardNo, WORD *axis, WORD *Source, long x_pos_p, long x_pos_n, long y_pos_p, long y_pos_n, WORD action_para);
DMC_API short __stdcall dmc_get_zone_limit_config(WORD CardNo, WORD* axis, WORD* Source, long* x_pos_p, long* x_pos_n, long* y_pos_p, long* y_pos_n, WORD* action_para);
DMC_API short __stdcall dmc_set_zone_limit_enable(WORD CardNo, WORD enable);

//轴互锁功能
DMC_API short __stdcall dmc_set_interlock_config(WORD CardNo, WORD* axis, WORD* Source, long delta_pos, WORD action_para);
DMC_API short __stdcall dmc_get_interlock_config(WORD CardNo, WORD* axis, WORD* Source, long* delta_pos, WORD* action_para);
DMC_API short __stdcall dmc_set_interlock_enable(WORD CardNo, WORD enable);

//龙门模式的误差保护
DMC_API short __stdcall dmc_set_grant_error_protect(WORD CardNo, WORD axis,WORD enable,DWORD dstp_error, DWORD emg_error);
DMC_API short __stdcall dmc_get_grant_error_protect(WORD CardNo, WORD axis,WORD* enable,DWORD* dstp_error, DWORD* emg_error);

DMC_API short __stdcall dmc_set_safety_param(WORD CardNo,WORD axis,WORD enable,long safety_pos);	
DMC_API short __stdcall dmc_get_safety_param(WORD CardNo,WORD axis,WORD* enable,long* safety_pos);	
DMC_API short __stdcall dmc_get_diagnose_param(WORD CardNo,WORD axis,long* tartet_pos,int* mode,long* pulse_pos,long* endcoder_pos);

//物件分拣功能
DMC_API short __stdcall dmc_set_camerablow_config(WORD CardNo,WORD camerablow_en,long cameraPos,WORD piece_num,long piece_distance,WORD axis_sel,long latch_distance_min);	
DMC_API short __stdcall dmc_get_camerablow_config(WORD CardNo,WORD* camerablow_en,long* cameraPos,WORD* piece_num,long* piece_distance,WORD* axis_sel,long* latch_distance_min);	
DMC_API short __stdcall dmc_clear_camerablow_errorcode(WORD CardNo);	
DMC_API short __stdcall dmc_get_camerablow_errorcode(WORD CardNo,WORD* errorcode);

//配置通用输入（0~15）做为轴的限位信号
DMC_API short __stdcall dmc_set_io_limit_config(WORD CardNo,WORD portno,WORD enable,WORD axis_sel,WORD el_mode,WORD el_logic);	
DMC_API short __stdcall dmc_get_io_limit_config(WORD CardNo,WORD portno,WORD* enable,WORD* axis_sel,WORD* el_mode,WORD* el_logic);	

//手轮滤波参数
DMC_API short __stdcall dmc_set_handwheel_filter(WORD CardNo,WORD axis,double filter_factor);
DMC_API short __stdcall dmc_get_handwheel_filter(WORD CardNo,WORD axis,double* filter_factor);

//读取坐标系各轴的当前规划坐标
DMC_API short __stdcall dmc_conti_get_interp_map(WORD CardNo,WORD Crd,WORD* AxisNum ,WORD* AxisList,double *pPosList);
//坐标系错误代码 
DMC_API short __stdcall dmc_conti_get_crd_errcode(WORD CardNo,WORD Crd,WORD* errcode);


DMC_API short __stdcall dmc_line_unit_follow(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode);
DMC_API short __stdcall dmc_conti_line_unit_follow(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark);

//连续插补缓冲区DA操作  
DMC_API short __stdcall dmc_conti_set_da_action(WORD CardNo,WORD Crd,WORD mode,WORD portno,double dvalue);

//读编码器速度
DMC_API short __stdcall dmc_read_encoder_speed(WORD CardNo,WORD Axis,double *current_speed);

DMC_API short __stdcall dmc_axis_follow_line_enable(WORD CardNo,WORD Crd,WORD enable_flag);

//插补轴脉冲补偿
DMC_API short __stdcall dmc_set_interp_compensation(WORD CardNo,WORD axis, double dvalue,double time);
DMC_API short __stdcall dmc_get_interp_compensation(WORD CardNo,WORD axis, double *dvalue,double *time); 

//IO精确停止
DMC_API short __stdcall dmc_set_io_exactstop(WORD CardNo,WORD axis, WORD ioNum,WORD *ioList,WORD enable,WORD valid_logic,WORD action,WORD move_dir); 

//读取相对于起点的距离
DMC_API short __stdcall dmc_get_distance_to_start(WORD CardNo,WORD Crd, double* distance_x, double* distance_y,long imark); 
//设置标志位 表示是否开始计算相对起点
DMC_API short __stdcall dmc_set_start_distance_flag(WORD CardNo,WORD Crd,WORD flag);

/******************设置回零限位距离**********************
	参  数：
	CardNo:卡号
	Axis：轴号
	N_limit:负限位脉冲数
	P_limit:正限位脉冲数
	返回值：错误代码
*******************************************************************/
DMC_API short __stdcall dmc_set_home_soft_limit(WORD CardNo,WORD Axis,int N_limit,int P_limit); 
DMC_API short __stdcall dmc_get_home_soft_limit(WORD CardNo,WORD Axis,int* N_limit,int* P_limit); 

/*********************
	指令说明：实现刀向跟随功能，启动某个轴跟随运动，调用此指令时，跟随轴axis会跟随着此指令后面缓冲区的插补指令合轨迹运动，直到设置的脉冲需求完毕。
	指令类型：插补缓冲区指令
	输入参数：CardNo 坐标系号
	axis 跟随轴
	dist 跟随距离(脉冲当量单位)
        follow_mode //运动模式：0-暂停后启动，1-运动中启动
	imark 段号
	输出参数：无
*******************************/
DMC_API short __stdcall dmc_conti_gear_unit(WORD CardNo,WORD Crd,WORD axis,double dist, WORD follow_mode,long imark); 

//轨迹拟合使能设置
DMC_API short __stdcall dmc_set_path_fitting_enable(WORD CardNo,WORD Crd,WORD enable);
//螺距补偿功能(新)
DMC_API short __stdcall dmc_enable_leadscrew_comp(WORD CardNo, WORD axis,WORD enable);
DMC_API short __stdcall dmc_set_leadscrew_comp_config(WORD CardNo, WORD axis,WORD n, int startpos,int lenpos,int *pCompPos,int *pCompNeg);
//指定轴做定长位移运动 按固定曲线运动
DMC_API short __stdcall dmc_t_pmove_extern(WORD CardNo, WORD axis, double MidPos,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,WORD posi_mode);

/*
功能：设置脉冲计数值和编码器反馈值之间差值的报警阀值
输入参数：CardNo 卡号
axis 轴号
error 差值报警报警阀值
输出参数：无
*/
DMC_API short __stdcall dmc_set_pulse_encoder_count_error(WORD CardNo,WORD axis,WORD error);
DMC_API short __stdcall dmc_get_pulse_encoder_count_error(WORD CardNo,WORD axis,WORD *error);
/*
功能：检查脉冲计数值和编码器反馈值之间差值是否超过报警阀值
	输入参数：CardNo 卡号
	axis 轴号
	输出参数;无
	返回参数：0：差值小于报警阀值
	1：差值大于等于报警阀值
*/
DMC_API short __stdcall dmc_check_pulse_encoder_count_error(WORD CardNo,WORD axis,int* pulse_position, int* enc_position);
/*
强行变位扩展
mid_pos: 中间位置
aim_pos:目标位置
posi_mode: 保留参数，默认为绝对值
*/
DMC_API short __stdcall dmc_update_target_position_extern(WORD CardNo, WORD axis, double mid_pos, double aim_pos, double vel,WORD posi_mode);

//新物件分拣功能
DMC_API short __stdcall dmc_sorting_close(WORD CardNo);
DMC_API short __stdcall dmc_sorting_start(WORD CardNo);
DMC_API short __stdcall dmc_sorting_set_init_config(WORD CardNo ,WORD cameraCount, int *pCameraPos, WORD *pCamIONo, DWORD cameraTime, WORD cameraTrigLevel, WORD blowCount, int*pBlowPos, WORD*pBlowIONo, DWORD blowTime, WORD blowTrigLevel, WORD axis, WORD dir, WORD checkMode);
DMC_API short __stdcall dmc_sorting_set_camera_trig_count(WORD CardNo ,WORD cameraNum, DWORD cameraTrigCnt);
DMC_API short __stdcall dmc_sorting_get_camera_trig_count(WORD CardNo ,WORD cameraNum, DWORD* pCameraTrigCnt, WORD count);
DMC_API short __stdcall dmc_sorting_set_blow_trig_count(WORD CardNo ,WORD blowNum, DWORD blowTrigCnt);
DMC_API short __stdcall dmc_sorting_get_blow_trig_count(WORD CardNo ,WORD blowNum, DWORD* pBlowTrigCnt, WORD count);
DMC_API short __stdcall dmc_sorting_get_camera_config(WORD CardNo ,WORD index,int* pos,DWORD* trigTime, WORD* ioNo, WORD* trigLevel);
DMC_API short __stdcall dmc_sorting_get_blow_config(WORD CardNo ,WORD index, int* pos,DWORD* trigTime, WORD* ioNo, WORD* trigLevel);
DMC_API short __stdcall dmc_sorting_get_blow_status(WORD CardNo ,DWORD* trigCntAll, WORD* trigMore,WORD* trigLess);
DMC_API short __stdcall dmc_sorting_trig_blow(WORD CardNo ,WORD blowNum);
DMC_API short __stdcall dmc_sorting_set_blow_enable(WORD CardNo ,WORD blowNum,WORD enable);
DMC_API short __stdcall dmc_sorting_set_piece_config(WORD CardNo ,DWORD maxWidth,DWORD minWidth,DWORD minDistance, DWORD minTimeIntervel);
DMC_API short __stdcall dmc_sorting_get_piece_status(WORD CardNo ,DWORD* pieceFind,DWORD* piecePassCam, DWORD* dist2next, DWORD*pieceWidth);
DMC_API short __stdcall dmc_sorting_set_cam_trig_phase(WORD CardNo,WORD blowNo,double coef);
DMC_API short __stdcall dmc_sorting_set_blow_trig_phase(WORD CardNo,WORD blowNo,double coef);

DMC_API short __stdcall dmc_set_sevon_enable(WORD CardNo,WORD axis,WORD on_off);
DMC_API short __stdcall dmc_get_sevon_enable(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_compare_add_point_cycle(WORD CardNo,WORD cmp,long pos,WORD dir, DWORD bitno,DWORD cycle,WORD level);//添加比较点

//使能和设置跟踪编码器误差不在范围内时轴的停止模式
DMC_API short __stdcall dmc_set_encoder_count_error_action_config(WORD CardNo,WORD enable,WORD stopmode);
DMC_API short __stdcall dmc_get_encoder_count_error_action_config(WORD CardNo,WORD* enable,WORD* stopmode);

DMC_API short __stdcall dmc_set_home_el_return(WORD CardNo,WORD axis,WORD enable);

//连续编码器da跟随
DMC_API short __stdcall dmc_conti_set_encoder_da_follow_enable(WORD CardNo, WORD Crd,WORD axis,WORD enable);	
DMC_API short __stdcall dmc_conti_get_encoder_da_follow_enable(WORD CardNo, WORD Crd,WORD* axis,WORD* enable);

DMC_API short __stdcall dmc_check_done_pos(WORD CardNo,WORD axis,WORD posi_mode);
DMC_API short __stdcall dmc_set_factor_error(WORD CardNo,WORD axis,double factor,long error);
DMC_API short __stdcall dmc_set_factor(WORD CardNo,WORD axis,double factor);
DMC_API short __stdcall dmc_set_error(WORD CardNo,WORD axis,long error);
DMC_API short __stdcall dmc_get_factor_error(WORD CardNo,WORD axis,double* factor,long* error);
DMC_API short __stdcall dmc_check_success_pulse(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_check_success_encoder(WORD CardNo,WORD axis);

//IO及编码器计数功能
DMC_API short __stdcall dmc_set_io_count_profile(WORD CardNo, WORD chan, WORD bitno,WORD mode,double filter, double count_value, WORD* axis_list, WORD axis_num, WORD stop_mode );
DMC_API short __stdcall dmc_get_io_count_profile(WORD CardNo, WORD chan, WORD* bitno,WORD* mode,double* filter, double* count_value, WORD* axis_list, WORD* axis_num, WORD* stop_mode );
DMC_API short __stdcall dmc_set_io_count_enable(WORD CardNo, WORD chan, WORD ifenable);
DMC_API short __stdcall dmc_clear_io_count(WORD CardNo, WORD chan);
DMC_API short __stdcall dmc_get_io_count_value_extern(WORD CardNo, WORD chan, long* current_value);

//螺距补偿前的脉冲位置，编码器位置
DMC_API short __stdcall dmc_get_position_ex(WORD CardNo,WORD axis, double * pos);
DMC_API short __stdcall dmc_get_encoder_ex(WORD CardNo,WORD axis, double * pos);

//回零偏移模式函数
DMC_API short __stdcall dmc_set_home_shift_param(WORD CardNo, WORD axis, WORD pos_clear_mode, double ShiftValue);
DMC_API short __stdcall dmc_get_home_shift_param(WORD CardNo, WORD axis, WORD *pos_clear_mode, double* ShiftValue);

DMC_API short __stdcall dmc_change_speed_extend(WORD CardNo,WORD axis,double Curr_Vel, double Taccdec, WORD pin_num, WORD trig_mode);

DMC_API short __stdcall dmc_follow_vector_speed_move(WORD CardNo,WORD axis,WORD Follow_AxisNum,WORD* Follow_AxisList,double ratio);
DMC_API short __stdcall dmc_conti_line_unit_extend(WORD CardNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* pPosList, WORD posi_mode, double Extend_Len, WORD enable,long mark); //连续插补直线
DMC_API short __stdcall dmc_hcmp_2d_set_config_unit(WORD CardNo,WORD hcmp,WORD cmp_mode,WORD x_axis, WORD x_cmp_source, double x_cmp_error, WORD y_axis, WORD y_cmp_source, double y_cmp_error,WORD cmp_logic,int time);
DMC_API short __stdcall dmc_hcmp_2d_get_config_unit(WORD CardNo,WORD hcmp,WORD *cmp_mode,WORD *x_axis, WORD *x_cmp_source,double *x_cmp_error,  WORD *y_axis, WORD *y_cmp_source, double *y_cmp_error, WORD *cmp_logic,int *time);

DMC_API short __stdcall dmc_hcmp_2d_set_pwmoutput(WORD CardNo,WORD hcmp,WORD pwm_enable,double duty,double freq,WORD pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_get_pwmoutput(WORD CardNo,WORD hcmp,WORD *pwm_enable,double *duty,double *freq,WORD *pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_add_point_unit(WORD ConnectNo,WORD hcmp, double x_cmp_pos, double y_cmp_pos,WORD cmp_outbit);
DMC_API short __stdcall dmc_hcmp_2d_get_current_state_unit(WORD CardNo,WORD hcmp,int *remained_points,double *x_current_point,double *y_current_point,int *runned_points,WORD *current_state,WORD *current_outbit); 

/*****************************总线相关函数**********************************/
//回零运动
DMC_API short __stdcall nmc_set_home_profile(WORD CardNo ,WORD axis,WORD home_mode,double Low_Vel, double High_Vel,double Tacc,double Tdec ,double offsetpos);//设置回零参数，合并函数
DMC_API short __stdcall nmc_get_home_profile(WORD CardNo ,WORD axis,WORD* home_mode,double* Low_Vel, double* High_Vel,double* Tacc,double* Tdec ,double* offsetpos);
DMC_API short __stdcall nmc_home_move(WORD CardNo,WORD axis);

//-------------------------总线配置-----------------------
/*******************************************************
portnum表示端口号，定义如下
0: 表示canopen的0号端口
1: 表示canopen的1号端口
10:表示EtherCAT的0号端口
11:表示EtherCAT的1号端口
********************************************************/
DMC_API short __stdcall nmc_set_manager_para(WORD CardNo,WORD PortNum,DWORD Baudrate,WORD ManagerID);
DMC_API short __stdcall nmc_get_manager_para(WORD CardNo,WORD PortNum,DWORD *Baudrate,WORD *ManagerID);
DMC_API short __stdcall nmc_set_manager_od(WORD CardNo,WORD PortNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD Value);
DMC_API short __stdcall nmc_get_manager_od(WORD CardNo,WORD PortNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD *Value);

DMC_API short __stdcall nmc_set_node_od(WORD CardNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,long Value);
DMC_API short __stdcall nmc_get_node_od(WORD CardNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,long* Value);

DMC_API short __stdcall nmc_upload_configfile(WORD CardNo,WORD PortNum, const char *FileName);
DMC_API short __stdcall nmc_reset_to_factory(WORD CardNo,WORD PortNum,WORD NodeNum);
DMC_API short __stdcall nmc_write_to_pci(WORD CardNo,WORD PortNum,WORD NodeNum);
DMC_API short __stdcall nmc_download_configfile(WORD CardNo,WORD PortNum,const char *FileName);//总线ENI配置文件
DMC_API short __stdcall nmc_download_mapfile(WORD CardNo,const char *FileName);//总线映射文件

//添加单轴使能函数 255表示全使能
DMC_API short __stdcall nmc_set_axis_enable(WORD CardNo,WORD axis);
DMC_API short __stdcall nmc_set_axis_disable(WORD CardNo,WORD axis);

//清除报警信号
DMC_API short __stdcall nmc_set_alarm_clear(WORD CardNo,WORD PortNum,WORD NodeNum);

DMC_API short __stdcall nmc_get_slave_nodes(WORD CardNo,WORD PortNum,WORD BaudRate,WORD* NodeId,WORD* NodeNum);
//获取总线轴数
DMC_API short __stdcall nmc_get_total_axes(WORD CardNo,DWORD* TotalAxis);
//获取总线ADDA输入输出口数
DMC_API short __stdcall nmc_get_total_adcnum(WORD CardNo,WORD* TotalIn,WORD* TotalOut);
//获取总线IO口数
DMC_API short __stdcall nmc_get_total_ionum(WORD CardNo,WORD *TotalIn,WORD *TotalOut);
//清除端口报警
DMC_API short __stdcall nmc_clear_alarm_fieldbus(WORD CardNo,WORD PortNum);
//获取控制器工作模式  1表示ethercat模式，0表示仿真模式
DMC_API short __stdcall nmc_get_controller_workmode(WORD CardNo,WORD* controller_mode);
//设置控制器工作模式  1表示ethercat模式，0表示仿真模式
DMC_API short __stdcall nmc_set_controller_workmode(WORD CardNo,WORD controller_mode);
//设置ethercat总线循环周期(us)
DMC_API short __stdcall nmc_set_cycletime(WORD CardNo,WORD PortNum,DWORD CycleTime);
//获取ethercat总线循环周期(us)
DMC_API short __stdcall nmc_get_cycletime(WORD CardNo,WORD PortNum,DWORD* CycleTime);

//读取总线线程时间消耗参数
DMC_API short __stdcall dmc_get_perline_time(WORD CardNo,WORD TypeIndex,DWORD *Averagetime,DWORD *Maxtime,uint64 *Cycles ); //TypeIndex:0~6  m_Averagetime ; 平均时间 m_Maxtime;最大时间 uint64  m_Cycles;当前时间
DMC_API short __stdcall nmc_set_axis_run_mode(WORD CardNo,WORD axis,WORD run_mode);//设置轴的运行模式 1为pp模式，6为回零模式，8为csp模式


//获取轴类型
/*
enum fieldbus_type
{
	virtual_type=0,
	ect_type=1,
	can_type=2,
	pulse_type=3, // or local IO 
	unknown=4
};*/
DMC_API short __stdcall nmc_get_axis_type(WORD CardNo,WORD axis, WORD* Axis_Type);
//获取总线时间量，平均时间，最大时间，执行周期数
DMC_API short __stdcall nmc_get_consume_time_fieldbus(WORD CardNo,WORD PortNum,DWORD* Average_time, DWORD* Max_time,uint64* Cycles);
//清除时间量
DMC_API short __stdcall nmc_clear_consume_time_fieldbus(WORD CardNo,WORD PortNum);
//停止ethercat总线,返回0表示成功，其他参数表示不成功
DMC_API short __stdcall nmc_stop_etc(WORD CardNo,WORD* ETCState);

//---------------------------总线轴模式---------------
//获取轴状态字
DMC_API short __stdcall nmc_get_axis_statusword(WORD CardNo,WORD axis,long* statusword);
//获取总线轴控制字
DMC_API short __stdcall nmc_set_axis_contrlword(WORD CardNo,WORD Axis,long Contrlword);
//设置总线轴控制字
DMC_API short __stdcall nmc_get_axis_contrlword(WORD CardNo,WORD Axis,long *Contrlword);
DMC_API short __stdcall nmc_set_axis_contrlmode(WORD CardNo,WORD Axis,long Contrlmode);
DMC_API short __stdcall nmc_get_axis_contrlmode(WORD CardNo,WORD Axis,long *Contrlmode);

// 获取总线端口错误码
DMC_API short __stdcall nmc_get_errcode(WORD CardNo,WORD channel,WORD *Errcode);
// 获取控制卡错误码
DMC_API short __stdcall nmc_get_card_errcode(WORD CardNo,WORD *Errcode);
// 获取总线轴错误码
DMC_API short __stdcall nmc_get_axis_errcode(WORD CardNo,WORD axis,WORD *Errcode);
// 清除总线端口错误码
DMC_API short __stdcall nmc_clear_errcode(WORD CardNo,WORD channel);
// 清除控制卡错误码
DMC_API short __stdcall nmc_clear_card_errcode(WORD CardNo);
// 清除总线轴错误码
DMC_API short __stdcall nmc_clear_axis_errcode(WORD CardNo,WORD iaxis);

DMC_API short __stdcall nmc_get_LostHeartbeat_Nodes(WORD CardNo,WORD PortNum,WORD* NodeID,WORD* NodeNum);
DMC_API short __stdcall nmc_get_EmergeneyMessege_Nodes(WORD CardNo,WORD PortNum,DWORD* NodeMsg,WORD* MsgNum);
DMC_API short __stdcall nmc_SendNmtCommand(WORD CardNo,WORD PortNum,WORD NodeID,WORD NmtCommand);
DMC_API short __stdcall nmc_syn_move(WORD CardNo,WORD AxisNum,WORD* AxisList,long* Position,WORD* PosiMode);
DMC_API short __stdcall nmc_syn_move_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,double* Position,WORD* PosiMode);
//总线多轴同步运动
DMC_API short __stdcall nmc_sync_pmove_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,double* Dist,WORD* PosiMode);
DMC_API short __stdcall nmc_sync_vmove_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* Dir);

//设置主站参数
DMC_API short __stdcall nmc_set_master_para(WORD CardNo,WORD PortNum,WORD Baudrate,DWORD NodeCnt,WORD MasterId);
//读取主站参数
DMC_API short __stdcall nmc_get_master_para(WORD CardNo,WORD PortNum,WORD *Baudrate,DWORD *NodeCnt,WORD *MasterId);
//设置io输出
DMC_API short __stdcall nmc_write_outbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD IoValue);
//读取io输出
DMC_API short __stdcall nmc_read_outbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue);
//读取io输入
DMC_API short __stdcall nmc_read_inbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue);
//设置DA参数	
DMC_API short __stdcall nmc_set_da_output(WORD CardNo,WORD NoteID,WORD channel,double Value);
//读取DA参数
DMC_API short __stdcall nmc_get_da_output(WORD CardNo,WORD NoteID,WORD channel,double *Value);
//读取AD参数
DMC_API short __stdcall nmc_get_ad_input(WORD CardNo,WORD NoteID,WORD channel,double *Value);
//配置AD模式
DMC_API short __stdcall nmc_set_ad_mode(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums);
DMC_API short __stdcall nmc_get_ad_mode(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums);
//配置DA模式
DMC_API short __stdcall nmc_set_da_mode(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums);
DMC_API short __stdcall nmc_get_da_mode(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums);

//参数写入flash
DMC_API short __stdcall nmc_write_to_flash(WORD CardNo,WORD PortNum,WORD NodeNum);

//总线链接
DMC_API short __stdcall nmc_set_connect_state(WORD CardNo,WORD NodeNum,WORD state,WORD baud);//0-断开；1-连接；2-复位后自动连接
DMC_API short __stdcall nmc_get_connect_state(WORD CardNo,WORD* NodeNum,WORD* state);//0-断开；1-连接；2-异常

//设置io输出32位
DMC_API short __stdcall nmc_write_outport(WORD CardNo,WORD NoteID,WORD portno,DWORD outport_val);
//读取io输出32位
DMC_API short __stdcall nmc_read_outport(WORD CardNo,WORD NoteID,WORD portno,DWORD *outport_val);
//读取io输入32位
DMC_API short __stdcall nmc_read_inport(WORD CardNo,WORD NoteID,WORD portno,DWORD *inport_val);

//轴状态机
DMC_API short __stdcall nmc_get_axis_state_machine(WORD CardNo,WORD axis, WORD* Axis_StateMachine);
//获取轴配置控制模式，返回值（6回零模式，8csp模式）
DMC_API short __stdcall nmc_get_axis_setting_contrlmode(WORD CardNo,WORD axis,long* contrlmode);
// 获取从站个数
DMC_API short __stdcall nmc_get_total_slaves(WORD CardNo,WORD PortNum,WORD* TotalSlaves);
// 获取轴的从站信息
DMC_API short __stdcall nmc_get_axis_node_address(WORD CardNo,WORD axis, WORD* SlaveAddr,WORD* Sub_SlaveAddr);
DMC_API short __stdcall nmc_set_axis_io_out(WORD CardNo,WORD axis,DWORD  iostate);
DMC_API DWORD __stdcall nmc_get_axis_io_out(WORD CardNo,WORD axis);
DMC_API DWORD __stdcall nmc_get_axis_io_in(WORD CardNo,WORD axis);


/************************************************************
*
*RTEX卡添加函数
*
*
************************************************************/
DMC_API short __stdcall nmc_start_connect(WORD CardNo,WORD chan,WORD*info,WORD* len);
//DMC_API short __stdcall nmc_get_node_info(WORD CardNo,WORD*info,WORD* len);
DMC_API short __stdcall nmc_get_vendor_info(WORD CardNo,WORD axis,char* info,WORD* len);
DMC_API short __stdcall nmc_get_slave_type_info(WORD CardNo,WORD axis,char* info,WORD* len);
DMC_API short __stdcall nmc_get_slave_name_info(WORD CardNo,WORD axis,char* info,WORD* len);
DMC_API short __stdcall nmc_get_slave_version_info(WORD CardNo,WORD axis,char* info,WORD* len);

DMC_API short __stdcall nmc_write_parameter(WORD CardNo,WORD axis,WORD index, WORD subindex,DWORD para_data);
/**************************************************************
*功能说明：RTEX驱动器写EEPROM操作
*
*
**************************************************************/
DMC_API short __stdcall nmc_write_slave_eeprom(WORD CardNo,WORD axis);
/**************************************************************
*index:rtex驱动器的参数分类
*subindex:rtex驱动器在index类别下的参数编号
*para_data:读出的参数值
**************************************************************/
DMC_API short __stdcall nmc_read_parameter(WORD CardNo,WORD axis,WORD index, WORD subindex,DWORD* para_data);
/**************************************************************
*index:rtex驱动器的参数分类
*subindex:rtex驱动器在index类别下的参数编号
*para_data:读出的参数值
**************************************************************/
DMC_API short __stdcall nmc_read_parameter_attributes(WORD CardNo,WORD axis,WORD index, WORD subindex,DWORD* para_data);
DMC_API short __stdcall nmc_set_cmdcycletime(WORD CardNo,WORD PortNum,DWORD cmdtime);
//设置RTEX总线周期比(us)
DMC_API short __stdcall nmc_get_cmdcycletime(WORD CardNo,WORD PortNum,DWORD* cmdtime);
DMC_API short __stdcall nmc_start_log(WORD CardNo,WORD chan,WORD node, WORD Ifenable);
DMC_API short __stdcall nmc_get_log(WORD CardNo,WORD chan,WORD node, DWORD* data);
DMC_API short __stdcall nmc_config_atuo_log(WORD CardNo,WORD ifenable,WORD dir,WORD byte_index,WORD mask,WORD condition,DWORD counter);

//扩展PDO
DMC_API short __stdcall nmc_write_rxpdo_extra(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,int Value);
DMC_API short __stdcall nmc_read_rxpdo_extra(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,int* Value);
DMC_API short __stdcall nmc_read_txpdo_extra(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,int* Value);
DMC_API short __stdcall nmc_write_rxpdo_extra_uint(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,DWORD Value);
DMC_API short __stdcall nmc_read_rxpdo_extra_uint(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,DWORD* Value);
DMC_API short __stdcall nmc_read_txpdo_extra_uint(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,DWORD* Value);
DMC_API short __stdcall nmc_get_log_state(WORD CardNo,WORD chan, DWORD* state);
DMC_API short __stdcall nmc_driver_reset(WORD CardNo,WORD axis);
DMC_API short __stdcall nmc_set_offset_pos(WORD CardNo,WORD axis, double offset_pos);
DMC_API short __stdcall nmc_get_offset_pos(WORD CardNo,WORD axis, double* offset_pos);

#ifdef __cplusplus
}
#endif

#endif