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
//中断回调函数指针定义
typedef uint32 (__stdcall * DMC3K5K_OPERATE)(PVOID operator_data);

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

//设置和读取打印模式（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_set_debug_mode(WORD mode,const char* pFileName);
DMC_API short __stdcall dmc_get_debug_mode(WORD* mode,char* pFileName);
//初始化控制卡（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_board_init(void); 	
//读取控制卡信息列表（适用于所有脉冲/总线卡）
DMC_API short __stdcall	dmc_get_CardInfList(WORD* CardNum,DWORD* CardTypeList,WORD* CardIdList);
//关闭控制卡（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_board_close(void);	
//硬件复位（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_board_reset(void);   
//控制卡热复位（适用于EtherCAT、RTEX总线卡）
DMC_API short __stdcall dmc_soft_reset(WORD CardNo);
//控制卡冷复位（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_cool_reset(WORD CardNo);
//控制卡初始复位（适用于EtherCAT总线卡）
DMC_API short __stdcall dmc_original_reset(WORD CardNo);
//读取控制卡卡号（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_get_card_ID (WORD CardNo,DWORD *CardID);
//读取发布版本号（适用于DMC3000/DMC5X10系列脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_get_release_version(WORD CardNo,char *ReleaseVersion);
//读取控制卡硬件版本（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_get_card_version(WORD CardNo,DWORD *CardVersion);	
//读取控制卡硬件的固件版本（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_get_card_soft_version(WORD CardNo,DWORD *FirmID,DWORD *SubFirmID);	
//读取控制卡动态库版本（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_get_card_lib_version(DWORD *LibVer);
//读取指定卡轴数（适用于所有脉冲卡）
DMC_API short __stdcall dmc_get_total_axes(WORD CardNo,DWORD *TotalAxis); 	
//获取本地IO点数（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_get_total_ionum(WORD CardNo,WORD *TotalIn,WORD *TotalOut);
//获取本地ADDA输入输出数（适用于所有脉冲卡）
DMC_API short __stdcall dmc_get_total_adcnum(WORD CardNo,WORD* TotalIn,WORD* TotalOut);

//密码函数（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_check_sn(WORD CardNo, const char * str_sn);
DMC_API short __stdcall dmc_write_sn(WORD CardNo, const char * str_sn);
//登入sn20191101（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_enter_password_ex(WORD CardNo, const char * str_pass);

//定制类（保留）
DMC_API short __stdcall dmc_board_init_onecard(WORD CardNo); 	////客户定制独立开卡 初始化一张控制卡
DMC_API short __stdcall dmc_board_close_onecard(WORD CardNo); 	////客户定制独立关卡 关闭一个控制卡
DMC_API short __stdcall dmc_board_reset_onecard(WORD CardNo);//客户定制单独硬件复位
//读取指定卡插补坐标系数（保留）
DMC_API short __stdcall dmc_get_total_liners(WORD CardNo,DWORD *TotalLiner); 

/***********轴参数*************/
//脉冲模式（适用于所有脉冲卡）		
DMC_API short __stdcall dmc_set_pulse_outmode(WORD CardNo,WORD axis,WORD outmode);	
DMC_API short __stdcall dmc_get_pulse_outmode(WORD CardNo,WORD axis,WORD* outmode);	
//脉冲当量（适用于EtherCAT总线卡、RTEX总卡、DMC5000/5X10系列脉冲卡）		
DMC_API short __stdcall dmc_set_equiv(WORD CardNo,WORD axis, double equiv);
DMC_API short __stdcall dmc_get_equiv(WORD CardNo,WORD axis, double *equiv);
//反向间隙(当量)（适用于E5032总线卡、DMC5000/5X10系列脉冲卡）	
DMC_API short __stdcall dmc_set_backlash_unit(WORD CardNo,WORD axis,double backlash);
DMC_API short __stdcall dmc_get_backlash_unit(WORD CardNo,WORD axis,double *backlash);
//反向间隙(脉冲)（适用于DMC5000系列脉冲卡）	
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
//上传文件
DMC_API short __stdcall dmc_upload_file(WORD CardNo, const char* pfilename, const char* pfilenameinControl, WORD filetype);
//下载内存文件 总线卡（适用于EtherCAT总线卡）
DMC_API short __stdcall dmc_download_memfile(WORD CardNo, const char* pbuffer, uint32 buffsize, const char* pfilenameinControl,WORD filetype);
//上传内存文件（适用于EtherCAT总线卡）
DMC_API short __stdcall dmc_upload_memfile(WORD CardNo, char* pbuffer, uint32 buffsize, const char* pfilenameinControl, uint32* puifilesize,WORD filetype);
//下载参数文件（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_download_configfile(WORD CardNo,const char *FileName);
//下载固件文件（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_download_firmware(WORD CardNo,const char *FileName);
//文件进度（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_get_progress(WORD CardNo,float* process);

//安全参数
//设置读取软限位参数（适用于E3032总线卡、R3032总线卡、DMC3000/5000/5X10系列脉冲卡）	
DMC_API short __stdcall dmc_set_softlimit(WORD CardNo,WORD axis,WORD enable, WORD source_sel,WORD SL_action, long N_limit,long P_limit);
DMC_API short __stdcall dmc_get_softlimit(WORD CardNo,WORD axis,WORD *enable, WORD *source_sel,WORD *SL_action,long *N_limit,long *P_limit);
//设置读取软限位参数unit（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_softlimit_unit(WORD CardNo,WORD axis,WORD enable, WORD source_sel,WORD SL_action, double N_limit,double P_limit);
DMC_API short __stdcall dmc_get_softlimit_unit(WORD CardNo,WORD axis,WORD *enable, WORD *source_sel,WORD *SL_action,double *N_limit,double *P_limit);
//设置读取EL信号（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_el_mode(WORD CardNo,WORD axis,WORD enable,WORD el_logic,WORD el_mode);
DMC_API short __stdcall dmc_get_el_mode(WORD CardNo,WORD axis,WORD *enable,WORD *el_logic,WORD *el_mode);
//设置读取EMG信号（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_set_emg_mode(WORD CardNo,WORD axis,WORD enable,WORD emg_logic);
DMC_API short __stdcall dmc_get_emg_mode(WORD CardNo,WORD axis,WORD *enable,WORD *emg_logic);
//外部减速停止信号及减速停止时间设置（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_io_dstp_mode(WORD CardNo,WORD axis,WORD enable,WORD logic);//enable:0-禁用，1-按时间减速停止，2-按距离减速停止
DMC_API short __stdcall dmc_get_io_dstp_mode(WORD CardNo,WORD axis,WORD *enable,WORD *logic); 
//点位运动减速停止时间设置读取（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_dec_stop_time(WORD CardNo,WORD axis,double time);
DMC_API short __stdcall dmc_get_dec_stop_time(WORD CardNo,WORD axis,double *time);
//插补减速停止信号和减速时间设置（适用于DMC5X10系列脉冲卡、EthreCAT总线卡）
DMC_API short __stdcall dmc_set_vector_dec_stop_time(WORD CardNo,WORD Crd,double time);
DMC_API short __stdcall dmc_get_vector_dec_stop_time(WORD CardNo,WORD Crd,double *time);
//IO减速停止距离（适用于DMC3000、DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_dec_stop_dist(WORD CardNo,WORD axis,long dist);
DMC_API short __stdcall dmc_get_dec_stop_dist(WORD CardNo,WORD axis,long *dist);
//IO减速停止，支持pmove/vmove运动（适用于DMC3000、DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_io_exactstop(WORD CardNo,WORD axis, WORD ioNum,WORD *ioList,WORD enable,WORD valid_logic,WORD action,WORD move_dir); 
//设置通用输入口的一位减速停止IO口（保留）
DMC_API short __stdcall dmc_set_io_dstp_bitno(WORD CardNo,WORD axis,WORD bitno,double filter);
DMC_API short __stdcall dmc_get_io_dstp_bitno(WORD CardNo,WORD axis,WORD *bitno,double* filter);

/*************************************单轴运动*****************************************/
//设定读取速度曲线参数	（适用于所有脉冲卡）	
DMC_API short __stdcall dmc_set_profile(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double stop_vel);
DMC_API short __stdcall dmc_get_profile(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Tacc,double *Tdec,double *stop_vel);
//速度设置(脉冲当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）	
DMC_API short __stdcall dmc_set_profile_unit(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_profile_unit(WORD CardNo,WORD axis,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);

//20160105增加新速度曲线以加速度 减速度 减减速度来表示(脉冲)（保留）
DMC_API short __stdcall dmc_set_profile_extern(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Ajerk,double Djerk,double stop_vel);
DMC_API short __stdcall dmc_get_profile_extern(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Tacc,double *Tdec,double *Ajerk,double *Djerk,double *stop_vel);
//速度曲线设置，加速度值表示(脉冲)（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_acc_profile(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Acc,double Dec,double stop_vel);
DMC_API short __stdcall dmc_get_acc_profile(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Acc,double *Dec,double *stop_vel);
//速度曲线设置，加速度值表示(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_profile_unit_acc(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_profile_unit_acc(WORD CardNo,WORD axis,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
//设置读取平滑速度曲线参数（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_set_s_profile(WORD CardNo,WORD axis,WORD s_mode,double s_para);
DMC_API short __stdcall dmc_get_s_profile(WORD CardNo,WORD axis,WORD s_mode,double *s_para);

//点位运动(脉冲)（适用于所有脉冲卡）		
DMC_API short __stdcall dmc_pmove(WORD CardNo,WORD axis,long dist,WORD posi_mode);
//点位运动(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_pmove_unit(WORD CardNo,WORD axis,double Dist,WORD posi_mode);
//单轴连续速度运动（适用于所有脉冲/总线卡）		
DMC_API short __stdcall dmc_vmove(WORD CardNo,WORD axis,WORD dir);
//指定轴做定长位移运动 同时发送速度和S时间(脉冲)（适用于DMC5X10系列脉冲卡）	
DMC_API short __stdcall dmc_pmove_extern(WORD CardNo, WORD axis, double dist,double Min_Vel, double Max_Vel, double Tacc, double Tdec, double stop_Vel, double s_para, WORD posi_mode);
//在线变位(脉冲)，运动中改变目标位置（适用于所有脉冲卡）
DMC_API short __stdcall dmc_reset_target_position(WORD CardNo,WORD axis,long dist,WORD posi_mode);
//在线变速(脉冲)，运动中改变指定轴的当前运动速度（适用于所有脉冲卡）
DMC_API short __stdcall dmc_change_speed(WORD CardNo,WORD axis,double Curr_Vel,double Taccdec);
//无论运动与否强行改变目标位置（适用于所有脉冲卡）
DMC_API short __stdcall dmc_update_target_position(WORD CardNo,WORD axis,long dist,WORD posi_mode);
/*
强行变位扩展
mid_pos: 中间位置
aim_pos:目标位置
posi_mode: 保留参数，默认为绝对值
*///（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_update_target_position_extern(WORD CardNo, WORD axis, double mid_pos, double aim_pos, double vel,WORD posi_mode);
//变速变位(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_reset_target_position_unit(WORD CardNo,WORD axis, double New_Pos);
//在线变速(当量)，运动中改变指定轴的当前运动速度（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_change_speed_unit(WORD CardNo,WORD axis, double New_Vel,double Taccdec);
//在线变速(当量)，运动中改变指定轴的当前运动速度（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_update_target_position_unit(WORD CardNo,WORD axis, double New_Pos);

/******************************插补运动**********************************/
//插补速度设置(脉冲)（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_set_vector_profile_multicoor(WORD CardNo,WORD Crd, double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_vector_profile_multicoor(WORD CardNo,WORD Crd, double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
//设置读取平滑速度曲线参数（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_set_vector_s_profile_multicoor(WORD CardNo,WORD Crd,WORD s_mode,double s_para);
DMC_API short __stdcall dmc_get_vector_s_profile_multicoor(WORD CardNo,WORD Crd,WORD s_mode,double *s_para);

//插补速度参数(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_vector_profile_unit(WORD CardNo,WORD Crd,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_vector_profile_unit(WORD CardNo,WORD Crd,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
//设置平滑速度曲线参数（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_vector_s_profile(WORD CardNo,WORD Crd,WORD s_mode,double s_para);
DMC_API short __stdcall dmc_get_vector_s_profile(WORD CardNo,WORD Crd,WORD s_mode,double *s_para);

//直线插补运动（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_line_multicoor(WORD CardNo,WORD Crd,WORD axisNum,WORD *axisList,long *DistList,WORD posi_mode);	
//圆弧插补运动（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_arc_move_multicoor(WORD CardNo,WORD Crd,WORD *AxisList,long *Target_Pos,long *Cen_Pos,WORD Arc_Dir,WORD posi_mode);
//单段插补(当量)
//直线插补(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_line_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode);
//圆心圆弧插补(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_arc_move_center_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode);
//半径圆弧插补(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_arc_move_radius_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode);
//三点圆弧插补(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_arc_move_3points_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode);
//矩形插补(当量)（适用于EtherCAT总线卡、RTEX总线卡、DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_rectangle_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mark_Pos,long num,WORD rect_mode,WORD posi_mode);

/********************PVT运动****************************/
//PVT运动 旧版,（适用于所有脉冲卡）
DMC_API short __stdcall dmc_PvtTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pVel);
DMC_API short __stdcall dmc_PtsTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pPercent);
DMC_API short __stdcall dmc_PvtsTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double velBegin,double velEnd);
DMC_API short __stdcall dmc_PttTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos);
DMC_API short __stdcall dmc_PvtMove(WORD CardNo,WORD AxisNum,WORD* AxisList);
//PVT缓冲区添加
DMC_API short __stdcall dmc_PttTable_add(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos);
DMC_API short __stdcall dmc_PtsTable_add(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pPercent);
//读取pvt剩余空间
DMC_API short __stdcall dmc_pvt_get_remain_space(WORD CardNo,WORD iaxis);
/*****************************************************************************
PVT运动 总线卡新规划，适用于EtherCAT总线卡
******************************************************************************/
DMC_API short __stdcall dmc_pvt_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double *pVel);
DMC_API short __stdcall dmc_pts_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double *pPercent);
DMC_API short __stdcall dmc_pvts_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double velBegin,double velEnd);
DMC_API short __stdcall dmc_ptt_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos);
DMC_API short __stdcall dmc_pvt_move(WORD CardNo,WORD AxisNum,WORD* AxisList);
//其它类（保留）
DMC_API short __stdcall dmc_SetGearProfile(WORD CardNo,WORD axis,WORD MasterType, WORD MasterIndex,long MasterEven,long SlaveEven,DWORD MasterSlope);
DMC_API short __stdcall dmc_GetGearProfile(WORD CardNo,WORD axis,WORD* MasterType, WORD* MasterIndex,long* MasterEven,long* SlaveEven,DWORD* MasterSlope);
DMC_API short __stdcall dmc_GearMove(WORD CardNo,WORD AxisNum,WORD* AxisList);

/************************回零运动*************************/	
//设置读取HOME信号（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_home_pin_logic(WORD CardNo,WORD axis,WORD org_logic,double filter);
DMC_API short __stdcall dmc_get_home_pin_logic(WORD CardNo,WORD axis,WORD *org_logic,double *filter);
//设定读取指定轴的回原点模式（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_homemode(WORD CardNo,WORD axis,WORD home_dir,double vel,WORD mode,WORD EZ_count);
DMC_API short __stdcall dmc_get_homemode(WORD CardNo,WORD axis,WORD *home_dir, double *vel_mode,WORD *home_mode,WORD *EZ_count);
//启动回零（适用于所有脉冲卡）
DMC_API short __stdcall dmc_home_move(WORD CardNo,WORD axis);
//设置读取回零速度参数（适用于Rtex总线卡）
DMC_API short __stdcall dmc_set_home_profile_unit(WORD CardNo,WORD axis,double Low_Vel,double High_Vel,double Tacc,double Tdec);
DMC_API short __stdcall dmc_get_home_profile_unit(WORD CardNo,WORD axis,double* Low_Vel,double* High_Vel,double* Tacc,double* Tdec);
//读取回零执行状态（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_get_home_result(WORD CardNo,WORD axis,WORD* state);
//设置读取回零偏移量及清零模式（适用于DMC5X10脉冲卡）
DMC_API short __stdcall dmc_set_home_position_unit(WORD CardNo,WORD axis,WORD enable,double position);
DMC_API short __stdcall dmc_get_home_position_unit(WORD CardNo,WORD axis,WORD *enable,double *position);
//（保留）
DMC_API short __stdcall dmc_set_el_home(WORD CardNo,WORD axis,WORD mode);
//设置回零遇限位是否反找（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_home_el_return(WORD CardNo,WORD axis,WORD enable);
//读取参数遇限位反找使能（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_get_home_el_return(WORD CardNo,WORD axis,WORD *enable);
//设置回零偏移量及偏移模式（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_set_home_position(WORD CardNo,WORD axis,WORD enable,double position);
DMC_API short __stdcall dmc_get_home_position(WORD CardNo,WORD axis,WORD *enable,double *position);
//回零偏移模式函数（保留）
DMC_API short __stdcall dmc_set_home_shift_param(WORD CardNo, WORD axis, WORD pos_clear_mode, double ShiftValue);
DMC_API short __stdcall dmc_get_home_shift_param(WORD CardNo, WORD axis, WORD *pos_clear_mode, double* ShiftValue);
/******************设置回零限位距离**********************
	参  数：
	CardNo:卡号
	Axis：轴号
	N_limit:负限位脉冲数
	P_limit:正限位脉冲数
	返回值：错误代码
*******************************************************************/
//保留
DMC_API short __stdcall dmc_set_home_soft_limit(WORD CardNo,WORD Axis,int N_limit,int P_limit); 
DMC_API short __stdcall dmc_get_home_soft_limit(WORD CardNo,WORD Axis,int* N_limit,int* P_limit); 

/***************************原点锁存******************************/
//设置读取原点锁存模式（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_homelatch_mode(WORD CardNo,WORD axis,WORD enable,WORD logic,WORD source);
DMC_API short __stdcall dmc_get_homelatch_mode(WORD CardNo,WORD axis,WORD* enable,WORD* logic,WORD* source);
//读取原点锁存标志（适用于所有脉冲卡）
DMC_API long __stdcall dmc_get_homelatch_flag(WORD CardNo,WORD axis);
//清除原点锁存标志（适用于所有脉冲卡）
DMC_API short __stdcall dmc_reset_homelatch_flag(WORD CardNo,WORD axis);
//读取原点锁存值（适用于所有脉冲卡）
DMC_API long __stdcall dmc_get_homelatch_value(WORD CardNo,WORD axis);
//读取原点锁存值（unit）（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_get_homelatch_value_unit(WORD CardNo,WORD axis, double* pos);
/*****************************EZ锁存********************************/
//设置读取EZ锁存模式（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_ezlatch_mode(WORD CardNo,WORD axis,WORD enable,WORD logic,WORD source);
DMC_API short __stdcall dmc_get_ezlatch_mode(WORD CardNo,WORD axis,WORD* enable,WORD* logic,WORD* source);
//读取EZ锁存标志（适用于所有脉冲卡）
DMC_API long __stdcall dmc_get_ezlatch_flag(WORD CardNo,WORD axis);
//清除EZ锁存标志（适用于所有脉冲卡）
DMC_API short __stdcall dmc_reset_ezlatch_flag(WORD CardNo,WORD axis);
//读取EZ锁存值（适用于所有脉冲卡）
DMC_API long __stdcall dmc_get_ezlatch_value(WORD CardNo,WORD axis);
//读取EZ锁存值（unit）（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_get_ezlatch_value_unit(WORD CardNo,WORD axis, double* pos);
/************************手轮运动函数*********************************/	
//设置读取单轴手轮脉冲信号的工作方式（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_set_handwheel_inmode(WORD CardNo,WORD axis,WORD inmode,long multi,double vh);
DMC_API short __stdcall dmc_get_handwheel_inmode(WORD CardNo,WORD axis,WORD *inmode,long *multi,double *vh);
//设置读取单轴手轮脉冲信号的工作方式，浮点型倍率（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_handwheel_inmode_decimals(WORD CardNo,WORD axis,WORD inmode,double multi,double vh);
DMC_API short __stdcall dmc_get_handwheel_inmode_decimals(WORD CardNo,WORD axis,WORD *inmode,double *multi,double *vh);
//启动手轮运动（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_handwheel_move(WORD CardNo,WORD axis);
//设置读取手轮通道（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_handwheel_channel(WORD CardNo,WORD index);
DMC_API short __stdcall dmc_get_handwheel_channel(WORD CardNo,WORD* index);
//设置读取多轴手轮脉冲信号的工作方式（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_set_handwheel_inmode_extern(WORD CardNo,WORD inmode,WORD AxisNum,WORD* AxisList,long* multi);
DMC_API short __stdcall dmc_get_handwheel_inmode_extern(WORD CardNo,WORD* inmode,WORD* AxisNum,WORD* AxisList,long *multi);	
//设置读取单轴手轮脉冲信号的工作方式，浮点型倍率（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_handwheel_inmode_extern_decimals(WORD CardNo,WORD inmode,WORD AxisNum,WORD* AxisList,double* multi);	
DMC_API short __stdcall dmc_get_handwheel_inmode_extern_decimals(WORD CardNo,WORD* inmode,WORD* AxisNum,WORD* AxisList,double *multi);

/*********************************************************************************************************
手轮运动 新增总线的手轮模式    (保留)
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
//设置读取指定轴的LTC信号（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_ltc_mode(WORD CardNo,WORD axis,WORD ltc_logic,WORD ltc_mode,double filter);
DMC_API short __stdcall dmc_get_ltc_mode(WORD CardNo,WORD axis,WORD*ltc_logic,WORD*ltc_mode,double *filter);
//设置读到锁存方式（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_latch_mode(WORD CardNo,WORD axis,WORD all_enable,WORD latch_source,WORD triger_chunnel);
DMC_API short __stdcall dmc_get_latch_mode(WORD CardNo,WORD axis,WORD *all_enable,WORD* latch_source,WORD* triger_chunnel);
//设置读取LTC反相输出（适用于所有脉冲卡）
DMC_API short __stdcall dmc_SetLtcOutMode(WORD CardNo,WORD axis,WORD enable,WORD bitno);
DMC_API short __stdcall dmc_GetLtcOutMode(WORD CardNo,WORD axis,WORD *enable,WORD* bitno);
//读取锁存器标志（适用于所有脉冲卡）
DMC_API short __stdcall dmc_get_latch_flag(WORD CardNo,WORD axis);
//复位锁存器标志（适用于所有脉冲卡）
DMC_API short __stdcall dmc_reset_latch_flag(WORD CardNo,WORD axis);
//读取编码器锁存器的值（适用于所有脉冲卡）
DMC_API long __stdcall  dmc_get_latch_value(WORD CardNo,WORD axis);
//读取编码器锁存器的值unit（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_get_latch_value_unit(WORD CardNo,WORD axis,double* pos_by_mm);
//读取锁存个数（适用DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_get_latch_flag_extern(WORD CardNo,WORD axis);
//按索引取值（适用DMC3000系列脉冲卡）
DMC_API long __stdcall dmc_get_latch_value_extern(WORD CardNo,WORD axis,WORD index);
//高速锁存（预留）
DMC_API short __stdcall dmc_get_latch_value_extern_unit(WORD CardNo,WORD axis,WORD index,double* pos_by_mm);//按索引取值读取 未完成
//LTC触发延时急停时间（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_latch_stop_time(WORD CardNo,WORD axis,long time);
DMC_API short __stdcall dmc_get_latch_stop_time(WORD CardNo,WORD axis,long* time); 

/*********************************************************************************************************
高速锁存 新规划20170308 总线卡
*********************************************************************************************************/
//配置锁存器：锁存模式0-单次锁存，1-连续锁存；锁存边沿0-下降沿，1-上升沿，2-双边沿；滤波时间，单位us（适用于所有总线卡）
DMC_API short __stdcall dmc_ltc_set_mode(WORD CardNo,WORD latch,WORD ltc_mode,WORD ltc_logic,double filter);
DMC_API short __stdcall dmc_ltc_get_mode(WORD CardNo,WORD latch,WORD *ltc_mode,WORD *ltc_logic,double *filter);
//配置锁存源：0-指令位置，1-编码器反馈位置（适用于所有总线卡）
DMC_API short __stdcall dmc_ltc_set_source(WORD CardNo,WORD latch,WORD axis,WORD ltc_source);
DMC_API short __stdcall dmc_ltc_get_source(WORD CardNo,WORD latch,WORD axis,WORD *ltc_source);
//复位锁存器（适用于所有总线卡）
DMC_API short __stdcall dmc_ltc_reset(WORD CardNo,WORD latch);
//读取锁存个数（适用于所有总线卡）
DMC_API short __stdcall dmc_ltc_get_number(WORD CardNo,WORD latch,WORD axis,int *number);
//读取锁存值（适用于所有总线卡）
DMC_API short __stdcall dmc_ltc_get_value_unit(WORD CardNo,WORD latch,WORD axis,double *value);

/*****************************软件锁存****************************/ 
//配置锁存器：锁存模式0-单次锁存，1-连续锁存；锁存边沿0-下降沿，1-上升沿，2-双边沿；滤波时间，单位us（适用于DMC5X10/3000系列脉冲卡、总线卡）
DMC_API short __stdcall dmc_softltc_set_mode(WORD CardNo,WORD latch,WORD ltc_enable,WORD ltc_mode,WORD ltc_inbit,WORD ltc_logic,double filter);
DMC_API short __stdcall dmc_softltc_get_mode(WORD CardNo,WORD latch,WORD *ltc_enable,WORD *ltc_mode,WORD *ltc_inbit,WORD *ltc_logic,double *filter);
//配置锁存源：0-指令位置，1-编码器反馈位置（适用于DMC5X10/3000系列脉冲卡、总线卡）
DMC_API short __stdcall dmc_softltc_set_source(WORD CardNo,WORD latch,WORD axis,WORD ltc_source);
DMC_API short __stdcall dmc_softltc_get_source(WORD CardNo,WORD latch,WORD axis,WORD *ltc_source);
//复位锁存器（适用于DMC5X10/3000系列脉冲卡、总线卡）
DMC_API short __stdcall dmc_softltc_reset(WORD CardNo,WORD latch);
//读取锁存个数（适用于DMC5X10/3000系列脉冲卡、总线卡）
DMC_API short __stdcall dmc_softltc_get_number(WORD CardNo,WORD latch,WORD axis,int *number);
//读取锁存值（适用于DMC5X10系列脉冲卡、所有总线卡）
DMC_API short __stdcall dmc_softltc_get_value_unit(WORD CardNo,WORD latch,WORD axis,double *value);

/*****************************位置比较函数****************************/
//一维低速位置比较	
//配置读取比较器（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_compare_set_config(WORD CardNo,WORD axis,WORD enable, WORD cmp_source);
DMC_API short __stdcall dmc_compare_get_config(WORD CardNo,WORD axis,WORD *enable, WORD *cmp_source);
//清除所有比较点（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_compare_clear_points(WORD CardNo,WORD cmp);
//添加比较点（适用于所有脉冲卡）
DMC_API short __stdcall dmc_compare_add_point(WORD CardNo,WORD cmp,long pos,WORD dir, WORD action,DWORD actpara);
//添加比较点（适用于E3032/R3032）
DMC_API short __stdcall dmc_compare_add_point_cycle(WORD CardNo,WORD cmp,long pos,WORD dir, DWORD bitno,DWORD cycle,WORD level);
//添加比较点unit（适用于E5032）
DMC_API short __stdcall dmc_compare_add_point_cycle_unit(WORD CardNo,WORD cmp,double pos,WORD dir, DWORD bitno,DWORD cycle,WORD level);
//添加比较点（适用于所有DMC5X10脉冲卡）
DMC_API short __stdcall dmc_compare_add_point_unit(WORD CardNo,WORD cmp,double pos,WORD dir, WORD action,DWORD actpara);
//读取当前比较点（适用于所有脉冲卡、Rtex总线卡、E3032卡）
DMC_API short __stdcall dmc_compare_get_current_point(WORD CardNo,WORD cmp,long *pos);
//读取当前比较点（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_compare_get_current_point_unit(WORD CardNo,WORD cmp,double *pos);
//查询已经比较过的点（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_compare_get_points_runned(WORD CardNo,WORD cmp,long *pointNum);
//查询可以加入的比较点数量（适用于所有脉冲/总线卡）
DMC_API short __stdcall dmc_compare_get_points_remained(WORD CardNo,WORD cmp,long *pointNum);

//二维低速位置比较 
//配置读取比较器（适用于所有脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_compare_set_config_extern(WORD CardNo,WORD enable, WORD cmp_source);
DMC_API short __stdcall dmc_compare_get_config_extern(WORD CardNo,WORD *enable, WORD *cmp_source);
//清除所有比较点（适用于所有脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_compare_clear_points_extern(WORD CardNo);
//添加两轴位置比较点（适用于所有脉冲卡）
DMC_API short __stdcall dmc_compare_add_point_extern(WORD CardNo,WORD* axis,long* pos,WORD* dir, WORD action,DWORD actpara);
//读取当前比较点（适用于所有脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_compare_get_current_point_extern(WORD CardNo,long *pos);
//读取当前比较点unit（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_compare_get_current_point_extern_unit(WORD CardNo,double *pos);
//添加两轴位置比较点（适用于DMC5X10脉冲卡）
DMC_API short __stdcall dmc_compare_add_point_extern_unit(WORD CardNo,WORD* axis,double* pos,WORD* dir, WORD action,DWORD actpara);
//查询已经比较过的点（适用于所有脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_compare_get_points_runned_extern(WORD CardNo,long *pointNum);
//查询可以加入的比较点数量（适用于所有脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_compare_get_points_remained_extern(WORD CardNo,long *pointNum);

//多组位置比较（保留）
DMC_API short __stdcall dmc_compare_set_config_multi(WORD CardNo,WORD queue,WORD enable, WORD axis, WORD cmp_source);//配置比较器
DMC_API short __stdcall dmc_compare_get_config_multi(WORD CardNo, WORD queue,WORD* enable, WORD* axis, WORD* cmp_source);//读取配置比较器
DMC_API short __stdcall dmc_compare_add_point_multi(WORD CardNo, WORD cmp,int32 pos, WORD dir,  WORD action, DWORD actpara,double times);//添加比较点 增强
DMC_API short __stdcall dmc_compare_add_point_multi_unit(WORD CardNo, WORD cmp,double pos, WORD dir,  WORD action, DWORD actpara,double times);//添加比较点 增强

/*****************************一维高速位置比较函数****************************/
//设置读取高速比较模式（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_hcmp_set_mode(WORD CardNo,WORD hcmp, WORD cmp_mode);
DMC_API short __stdcall dmc_hcmp_get_mode(WORD CardNo,WORD hcmp, WORD* cmp_mode);
//高速比较模式扩展（保留）
DMC_API short __stdcall dmc_hcmp_set_config_extern(WORD CardNo,WORD hcmp,WORD axis, WORD cmp_source, WORD cmp_logic,WORD cmp_mode,long dist,long time);
DMC_API short __stdcall dmc_hcmp_get_config_extern(WORD CardNo,WORD hcmp,WORD* axis, WORD* cmp_source, WORD* cmp_logic,WORD* cmp_mode,long* dist,long* time);
//设置高速比较参数（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_hcmp_set_config(WORD CardNo,WORD hcmp,WORD axis, WORD cmp_source, WORD cmp_logic,long time);
DMC_API short __stdcall dmc_hcmp_get_config(WORD CardNo,WORD hcmp,WORD* axis, WORD* cmp_source, WORD* cmp_logic,long* time);
//添加比较点（适用于所有脉冲卡、E3032总线卡、R3032总线卡）
DMC_API short __stdcall dmc_hcmp_add_point(WORD CardNo,WORD hcmp, long cmp_pos);
//添加比较点unit（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_hcmp_add_point_unit(WORD CardNo,WORD hcmp, double cmp_pos);
//设置读取线性模式参数（适用于所有脉冲卡、E3032总线卡、R3032总线卡）
DMC_API short __stdcall dmc_hcmp_set_liner(WORD CardNo,WORD hcmp, long Increment,long Count);
DMC_API short __stdcall dmc_hcmp_get_liner(WORD CardNo,WORD hcmp, long* Increment,long* Count);
//设置线性模式参数（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_hcmp_set_liner_unit(WORD CardNo,WORD hcmp, double Increment,long Count);
DMC_API short __stdcall dmc_hcmp_get_liner_unit(WORD CardNo,WORD hcmp, double* Increment,long* Count);
//读取高速比较状态（适用于所有脉冲卡、E3032总线卡、R3032总线卡）
DMC_API short __stdcall dmc_hcmp_get_current_state(WORD CardNo,WORD hcmp,long *remained_points,long *current_point,long *runned_points); 
//读取高速比较状态（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_hcmp_get_current_state_unit(WORD CardNo,WORD hcmp,long *remained_points,double *current_point,long *runned_points); 
//清除比较点（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_hcmp_clear_points(WORD CardNo,WORD hcmp);
//读取指定CMP端口的电平（保留）
DMC_API short __stdcall dmc_read_cmp_pin(WORD CardNo,WORD axis);
//控制cmp端口输出（保留）
DMC_API short __stdcall dmc_write_cmp_pin(WORD CardNo,WORD axis, WORD on_off);
//1、	启用缓存方式添加比较位置：（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_hcmp_fifo_set_mode(WORD CardNo,WORD hcmp, WORD fifo_mode);
DMC_API short __stdcall dmc_hcmp_fifo_get_mode(WORD CardNo,WORD hcmp, WORD* fifo_mode);
//2、	读取剩余缓存状态，上位机通过此函数判断是否继续添加比较位置（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_hcmp_fifo_get_state(WORD CardNo,WORD hcmp,long *remained_points); 
//3、	按数组的方式批量添加比较位置（适用于DMC5000系列脉冲卡）
DMC_API short __stdcall dmc_hcmp_fifo_add_point_unit(WORD CardNo,WORD hcmp, WORD num,double *cmp_pos);
//4、	清除比较位置,也会把FPGA的位置同步清除掉（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_hcmp_fifo_clear_points(WORD CardNo,WORD hcmp);
//添加大数据，会堵塞一段时间，指导数据添加完成（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_hcmp_fifo_add_table(WORD CardNo,WORD hcmp, WORD num,double *cmp_pos);
/*****************************二维高速位置比较函数****************************/
//设置读取高速比较使能（适用于所有脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_hcmp_2d_set_enable(WORD CardNo,WORD hcmp, WORD cmp_enable);
DMC_API short __stdcall dmc_hcmp_2d_get_enable(WORD CardNo,WORD hcmp, WORD *cmp_enable);
//配置读取二维高速比较器（适用于所有脉冲卡）
DMC_API short __stdcall dmc_hcmp_2d_set_config(WORD CardNo,WORD hcmp,WORD cmp_mode,WORD x_axis, WORD x_cmp_source, WORD y_axis, WORD y_cmp_source, long error,WORD cmp_logic,long time,WORD pwm_enable,double duty,long freq,WORD port_sel,WORD pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_get_config(WORD CardNo,WORD hcmp,WORD *cmp_mode,WORD *x_axis, WORD *x_cmp_source, WORD *y_axis, WORD *y_cmp_source, long *error,WORD *cmp_logic,long *time,WORD *pwm_enable,double *duty,long *freq,WORD *port_sel,WORD *pwm_number);
//配置读取二维高速比较器（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_hcmp_2d_set_config_unit(WORD CardNo,WORD hcmp,WORD cmp_mode,WORD x_axis, WORD x_cmp_source, double x_cmp_error, WORD y_axis, WORD y_cmp_source, double y_cmp_error,WORD cmp_logic,int time);
DMC_API short __stdcall dmc_hcmp_2d_get_config_unit(WORD CardNo,WORD hcmp,WORD *cmp_mode,WORD *x_axis, WORD *x_cmp_source,double *x_cmp_error,  WORD *y_axis, WORD *y_cmp_source, double *y_cmp_error, WORD *cmp_logic,int *time);
//添加二维高速位置比较点（适用于所有脉冲卡）
DMC_API short __stdcall dmc_hcmp_2d_add_point(WORD CardNo,WORD hcmp, long x_cmp_pos, long y_cmp_pos);
//添加二维高速位置比较点unit（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_hcmp_2d_add_point_unit(WORD ConnectNo,WORD hcmp, double x_cmp_pos, double y_cmp_pos,WORD cmp_outbit);
//读取二维高速比较参数（适用于所有脉冲卡）
DMC_API short __stdcall dmc_hcmp_2d_get_current_state(WORD CardNo,WORD hcmp,long *remained_points,long *x_current_point,long *y_current_point,long *runned_points,WORD *current_state); 
//读取二维高速比较参数（适用于DMC5X10系列脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_hcmp_2d_get_current_state_unit(WORD CardNo,WORD hcmp,int *remained_points,double *x_current_point,double *y_current_point,int *runned_points,WORD *current_state,WORD *current_outbit); 
//清除二维高速位置比较点（适用于所有脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_hcmp_2d_clear_points(WORD CardNo,WORD hcmp);
//强制二维高速比较输出（适用于所有脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_hcmp_2d_force_output(WORD CardNo,WORD hcmp,WORD cmp_outbit);
//配置读取二维比较PWM输出模式（适用于DMC5X10系列脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_hcmp_2d_set_pwmoutput(WORD CardNo,WORD hcmp,WORD pwm_enable,double duty,double freq,WORD pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_get_pwmoutput(WORD CardNo,WORD hcmp,WORD *pwm_enable,double *duty,double *freq,WORD *pwm_number);

/********************通用IO功能**************************/
//读取输入口的状态（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_read_inbit(WORD CardNo,WORD bitno);
//设置输出口的状态（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_write_outbit(WORD CardNo,WORD bitno,WORD on_off);
//读取输出口的状态（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_read_outbit(WORD CardNo,WORD bitno);
//读取输入端口的值（适用于所有脉冲卡、总线卡）
DMC_API DWORD __stdcall dmc_read_inport(WORD CardNo,WORD portno);
//读取输出端口的值（适用于所有脉冲卡、总线卡）
DMC_API DWORD __stdcall dmc_read_outport(WORD CardNo,WORD portno);
//设置所有输出端口的值（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_write_outport(WORD CardNo,WORD portno,DWORD outport_val);
//设置通用输出端口的值（保留）
DMC_API short __stdcall dmc_write_outport_16X(WORD CardNo,WORD portno,DWORD outport_val);
/********************通用IO带连接状态返回值**************************/
//读取输入口的状态（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_read_inbit_ex(WORD CardNo,WORD bitno,WORD *state);
//读取输出口的状态（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_read_outbit_ex(WORD CardNo,WORD bitno,WORD *state);
//读取输入端口的值（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_read_inport_ex(WORD CardNo,WORD portno,DWORD *state);
//读取输出端口的值（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_read_outport_ex(WORD CardNo,WORD portno,DWORD *state);
/********************虚拟IO映射**************************/
//设置读取虚拟IO映射关系（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_io_map_virtual(WORD CardNo,WORD bitno,WORD MapIoType,WORD MapIoIndex,double Filter);
DMC_API short __stdcall dmc_get_io_map_virtual(WORD CardNo,WORD bitno,WORD* MapIoType,WORD* MapIoIndex,double* Filter);
//读取虚拟输入口的状态（适用于所有脉冲卡）
DMC_API short __stdcall dmc_read_inbit_virtual(WORD CardNo,WORD bitno); 
//IO延时翻转（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_reverse_outbit(WORD CardNo,WORD bitno,double reverse_time);
//设置读取IO计数模式（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_io_count_mode(WORD CardNo,WORD bitno,WORD mode,double filter);
DMC_API short __stdcall dmc_get_io_count_mode(WORD CardNo,WORD bitno,WORD *mode,double* filter);
//设置IO计数值（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_io_count_value(WORD CardNo,WORD bitno,DWORD CountValue);
DMC_API short __stdcall dmc_get_io_count_value(WORD CardNo,WORD bitno,DWORD* CountValue);

/*********************专用IO信号************************/
//设置读取轴IO映射关系（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_axis_io_map(WORD CardNo,WORD Axis,WORD IoType,WORD MapIoType,WORD MapIoIndex,double Filter);
DMC_API short __stdcall dmc_get_axis_io_map(WORD CardNo,WORD Axis,WORD IoType,WORD* MapIoType,WORD* MapIoIndex,double* Filter);
//设置所有专用IO滤波时间（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_special_input_filter(WORD CardNo,double Filter);

//设置读取SD信号，3410专用 回原点减速信号配置
DMC_API short __stdcall dmc_set_sd_mode(WORD CardNo,WORD axis,WORD enable,WORD sd_logic,WORD sd_mode);
DMC_API short __stdcall dmc_get_sd_mode(WORD CardNo,WORD axis,WORD* enable,WORD *sd_logic,WORD *sd_mode);
//专用IO 脉冲卡指令
//设置读取INP信号（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_inp_mode(WORD CardNo,WORD axis,WORD enable,WORD inp_logic);
DMC_API short __stdcall dmc_get_inp_mode(WORD CardNo,WORD axis,WORD *enable,WORD *inp_logic);
//设置读取RDY信号（保留）
DMC_API short __stdcall dmc_set_rdy_mode(WORD CardNo,WORD axis,WORD enable,WORD rdy_logic);
DMC_API short __stdcall dmc_get_rdy_mode(WORD CardNo,WORD axis,WORD* enable,WORD* rdy_logic);
//设置读取ERC信号（保留）
DMC_API short __stdcall dmc_set_erc_mode(WORD CardNo,WORD axis,WORD enable,WORD erc_logic,WORD erc_width,WORD erc_off_time);
DMC_API short __stdcall dmc_get_erc_mode(WORD CardNo,WORD axis,WORD *enable,WORD *erc_logic, WORD *erc_width,WORD *erc_off_time);
//设置读取ALM信号（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_alm_mode(WORD CardNo,WORD axis,WORD enable,WORD alm_logic,WORD alm_action);
DMC_API short __stdcall dmc_get_alm_mode(WORD CardNo,WORD axis,WORD *enable,WORD *alm_logic,WORD *alm_action);
//设置读取EZ信号（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_ez_mode(WORD CardNo,WORD axis,WORD ez_logic,WORD ez_mode,double filter);
DMC_API short __stdcall dmc_get_ez_mode(WORD CardNo,WORD axis,WORD *ez_logic,WORD *ez_mode,double *filter);
//输出读取SEVON信号（适用于所有脉冲卡）
DMC_API short __stdcall dmc_write_sevon_pin(WORD CardNo,WORD axis,WORD on_off);
DMC_API short __stdcall dmc_read_sevon_pin(WORD CardNo,WORD axis);
//读取RDY状态（适用于所有脉冲卡）
DMC_API short __stdcall dmc_read_rdy_pin(WORD CardNo,WORD axis);
//控制ERC信号输出（适用于所有脉冲卡）
DMC_API short __stdcall dmc_write_erc_pin(WORD CardNo,WORD axis,WORD on_off);
//读取ERC信号状态（适用于所有脉冲卡）
DMC_API short __stdcall dmc_read_erc_pin(WORD CardNo,WORD axis); 	
//输出伺服复位信号（保留）
DMC_API short __stdcall dmc_write_sevrst_pin(WORD CardNo,WORD axis,WORD on_off);
//读伺服复位信号（保留）
DMC_API short __stdcall dmc_read_sevrst_pin(WORD CardNo,WORD axis);

/************************编码器函数**********************/	
//设定读取编码器的计数方式（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_counter_inmode(WORD CardNo,WORD axis,WORD mode);
DMC_API short __stdcall dmc_get_counter_inmode(WORD CardNo,WORD axis,WORD *mode);
//编码器值（适用于所有脉冲卡）
DMC_API long __stdcall dmc_get_encoder(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_set_encoder(WORD CardNo,WORD axis,long encoder_value);
//编码器值(当量)（适用于DMC5000/5X10系列脉冲卡、所有总线卡）
DMC_API short __stdcall dmc_set_encoder_unit(WORD CardNo,WORD axis, double pos);
DMC_API short __stdcall dmc_get_encoder_unit(WORD CardNo,WORD axis, double * pos);
//手轮编码器（备用，同dmc_set_extra_encoder）
DMC_API short __stdcall dmc_set_handwheel_encoder(WORD CardNo,WORD channel, long pos);
DMC_API short __stdcall dmc_get_handwheel_encoder(WORD CardNo,WORD channel, long * pos);
//设置辅助编码模式（适用于所有总线卡）
DMC_API short __stdcall dmc_set_extra_encoder_mode(WORD CardNo,WORD channel,WORD inmode,WORD multi);
DMC_API short __stdcall dmc_get_extra_encoder_mode(WORD CardNo,WORD channel,WORD* inmode,WORD* multi);
//设置辅助编码器值（适用于所有总线卡）
DMC_API short __stdcall dmc_set_extra_encoder(WORD CardNo,WORD channel, int pos);
DMC_API short __stdcall dmc_get_extra_encoder(WORD CardNo,WORD channel, int * pos);

/*********************位置计数控制***************************/
//当前位置(脉冲)（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_position(WORD CardNo,WORD axis,long current_position);
DMC_API long __stdcall dmc_get_position(WORD CardNo,WORD axis);	
//当前位置(当量)（适用于DMC5000/5X10系列脉冲卡、所有总线卡）
DMC_API short __stdcall dmc_set_position_unit(WORD CardNo,WORD axis, double pos);
DMC_API short __stdcall dmc_get_position_unit(WORD CardNo,WORD axis, double * pos);

/**************************运动状态********************************/
//读取指定轴的当前速度（适用于所有脉冲卡）
DMC_API double __stdcall dmc_read_current_speed(WORD CardNo,WORD axis);	
//读取当前速度(当量)（适用于DMC5000/5X10系列脉冲卡、所有总线卡）
DMC_API short __stdcall dmc_read_current_speed_unit(WORD CardNo,WORD axis, double *current_speed);
//读取当前卡的插补速度（保留）
DMC_API double __stdcall dmc_read_vector_speed(WORD CardNo);	
//读取当前卡的插补速度（适用于DMC5X10系列脉冲卡、所有总线卡）
DMC_API short __stdcall dmc_read_vector_speed_unit(WORD CardNo,WORD Crd,double *current_speed);	
//读取指定轴的目标位置（适用于所有脉冲卡、R3032总线卡）
DMC_API long __stdcall dmc_get_target_position(WORD CardNo,WORD axis);	
//读取指定轴的目标位置(当量)（适用于DMC5X10系列脉冲卡、所有EtherCAT总线系列卡）
DMC_API short __stdcall dmc_get_target_position_unit(WORD CardNo,WORD axis, double * pos);
//读取指定轴的运动状态（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_check_done(WORD CardNo,WORD axis);	
//读取指定轴有关运动信号的状态（适用于所有脉冲卡、总线卡）
DMC_API DWORD __stdcall dmc_axis_io_status(WORD CardNo,WORD axis);
//单轴停止（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_stop(WORD CardNo,WORD axis,WORD stop_mode);
//插补运动状态（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_check_done_multicoor(WORD CardNo,WORD Crd);
//停止插补器（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_stop_multicoor(WORD CardNo,WORD Crd,WORD stop_mode);
//紧急停止所有轴（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_emg_stop(WORD CardNo);
//主卡与接线盒通讯状态（适用于所有脉冲卡）
DMC_API short __stdcall dmc_LinkState(WORD CardNo,WORD* State);
//读取指定轴的运动模式（适用于DMC5000/5X10系列脉冲卡、所有总线卡）
DMC_API short __stdcall dmc_get_axis_run_mode(WORD CardNo, WORD axis,WORD* run_mode);
//读取停止原因（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_get_stop_reason(WORD CardNo,WORD axis,long* StopReason);
//清除停止原因（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_clear_stop_reason(WORD CardNo,WORD axis);

//trace功能（内部使用函数）
DMC_API short __stdcall dmc_set_trace(WORD CardNo,WORD axis,WORD enable);
DMC_API short __stdcall dmc_get_trace(WORD CardNo,WORD axis,WORD* enable);
DMC_API short __stdcall dmc_read_trace_data(WORD CardNo,WORD axis,WORD data_option,long* ReceiveSize,double* time,double* data,long* remain_num);
DMC_API short __stdcall dmc_trace_start(WORD CardNo,WORD AxisNum,WORD *AxisList);
DMC_API short __stdcall dmc_trace_stop(WORD CardNo);

//弧长计算（备用）
DMC_API short __stdcall dmc_calculate_arclength_center(double* start_pos,double *target_pos,double *cen_pos, WORD arc_dir,double circle,double* ArcLength);

/*********************************连续插补函数**************************************
连续插补的速度函数和单段插补相同
************************************************************************************/
//打开连续缓存区（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_open_list (WORD CardNo,WORD Crd,WORD AxisNum,WORD *AxisList);
//关闭连续缓存区（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_close_list(WORD CardNo,WORD Crd);
//复位连续缓存区（预留）
DMC_API short __stdcall dmc_conti_reset_list(WORD CardNo,WORD Crd);
//连续插补中停止（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_stop_list (WORD CardNo,WORD Crd,WORD stop_mode);
//连续插补中暂停（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_pause_list(WORD CardNo,WORD Crd);
//开始连续插补（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_start_list(WORD CardNo,WORD Crd);
//检测连续插补运动状态：0-运行，1-暂停，2-正常停止（DMC5X10不支持），3-未启动，4-空闲（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_get_run_state(WORD CardNo,WORD Crd);
//查连续插补剩余缓存数（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API long __stdcall dmc_conti_remain_space (WORD CardNo,WORD Crd);
//读取当前连续插补段的标号（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API long __stdcall dmc_conti_read_current_mark (WORD CardNo,WORD Crd);

//blend模式（适用于DMC5000系列脉冲卡）
DMC_API short __stdcall dmc_conti_set_blend(WORD CardNo,WORD Crd,WORD enable);
DMC_API short __stdcall dmc_conti_get_blend(WORD CardNo,WORD Crd,WORD* enable);
//设置插补中速度比例（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_set_override(WORD CardNo,WORD Crd,double Percent);
//设置插补中动态变速（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_conti_change_speed_ratio (WORD CardNo,WORD Crd,double percent);
//小线段前瞻（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_set_lookahead_mode(WORD CardNo,WORD Crd,WORD enable,long LookaheadSegments,double PathError,double LookaheadAcc);
DMC_API short __stdcall dmc_conti_get_lookahead_mode(WORD CardNo,WORD Crd,WORD* enable,long* LookaheadSegments,double* PathError,double* LookaheadAcc);

//连续插补IO功能
//等待IO输入（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_wait_input(WORD CardNo,WORD Crd,WORD bitno,WORD on_off,double TimeOut,long mark);
//相对于轨迹起点IO滞后输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_delay_outbit_to_start(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
//相对于轨迹终点IO滞后输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_delay_outbit_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_time,double ReverseTime);
//相对于轨迹终点IO提前输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_ahead_outbit_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
//连续插补精确位置CMP输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_accurate_outbit_unit(WORD CardNo, WORD Crd, WORD cmp_no,WORD on_off,WORD axis,double abs_pos,WORD pos_source,double ReverseTime);
//连续插补立即IO输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_write_outbit(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ReverseTime);
//清除段内未执行完的IO（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_clear_io_action(WORD CardNo, WORD Crd, DWORD Io_Mask);
//连续插补暂停及异常时IO输出状态（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_set_pause_output(WORD CardNo,WORD Crd,WORD action,long mask,long state);
DMC_API short __stdcall dmc_conti_get_pause_output(WORD CardNo,WORD Crd,WORD* action,long* mask,long* state);
//延时指令（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_delay(WORD CardNo, WORD Crd,double delay_time,long mark);
//IO输出延时翻转（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall  dmc_conti_reverse_outbit(WORD CardNo, WORD Crd, WORD bitno,double reverse_time);
//IO延时输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall  dmc_conti_delay_outbit(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_time);

//连续插补轨迹段
//连续插补直线插补（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_line_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark);
//连续插补圆心圆弧插补（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_arc_move_center_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
//连续插补半径圆弧插补（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_arc_move_radius_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
//连续插补3点圆弧插补（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_arc_move_3points_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode,long mark);
//连续插补矩形插补（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_rectangle_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mark_Pos,long num,WORD rect_mode,WORD posi_mode,long mark);
//连续插补单轴运动（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_pmove_unit(WORD CardNo,WORD Crd,WORD axis,double dist,WORD posi_mode,WORD mode,long imark);
//设置螺旋线插补运动模式（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_set_involute_mode(WORD CardNo,WORD Crd,WORD mode);
DMC_API short __stdcall dmc_conti_get_involute_mode(WORD CardNo,WORD Crd,WORD* mode);
//（备用）
DMC_API short __stdcall dmc_conti_line_unit_extern(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Cen_Pos,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_center_unit_extern(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Cen_Pos,double Arc_Radius,WORD posi_mode,long mark);

//设置读取龙门跟随模式（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_gear_follow_profile(WORD CardNo,WORD axis,WORD enable,WORD master_axis,double ratio);
DMC_API short __stdcall dmc_get_gear_follow_profile(WORD CardNo,WORD axis,WORD* enable,WORD* master_axis,double* ratio);

/*********************************PWM功能函数*******************************/
//PWM控制（备用）
DMC_API short __stdcall dmc_set_pwm_pin(WORD CardNo,WORD portno,WORD ON_OFF, double dfreqency,double dduty);	
DMC_API short __stdcall dmc_get_pwm_pin(WORD CardNo,WORD portno,WORD *ON_OFF, double *dfreqency,double *dduty);
//设置读取PWM使能（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_pwm_enable(WORD CardNo,WORD enable);
DMC_API short __stdcall dmc_get_pwm_enable(WORD CardNo,WORD* enable);
//设置读取PWM立即输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_pwm_output(WORD CardNo, WORD PwmNo,double fDuty, double fFre);
DMC_API short __stdcall dmc_get_pwm_output(WORD CardNo,WORD PwmNo,double* fDuty, double* fFre);
//连续插补PWM输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_set_pwm_output(WORD CardNo,WORD Crd, WORD PwmNo,double fDuty, double fFre);

//高速PWM功能（备用）
DMC_API short __stdcall dmc_set_pwm_enable_extern(WORD CardNo,WORD channel, WORD enable);
DMC_API short __stdcall dmc_get_pwm_enable_extern(WORD CardNo,WORD channel, WORD* enable);

/**********PWM速度跟随**************
mode:跟随模式0-不跟随 保持状态 1-不跟随 输出低电平2-不跟随 输出高电平3-跟随 占空比自动调整4-跟随 频率自动调整
MaxVel:最大运行速度，单位unit
MaxValue:最大输出占空比或者频率
OutValue：设置输出频率或占空比
*************************************/
//连续插补PWM速度跟随（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_set_pwm_follow_speed(WORD CardNo,WORD Crd,WORD pwm_no,WORD mode,double MaxVel,double MaxValue,double OutValue);
DMC_API short __stdcall dmc_conti_get_pwm_follow_speed(WORD CardNo,WORD Crd,WORD pwm_no,WORD* mode,double* MaxVel,double* MaxValue,double* OutValue);
//设置PWM开关对应的占空比（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_pwm_onoff_duty(WORD CardNo, WORD PwmNo,double fOnDuty, double fOffDuty);
DMC_API short __stdcall dmc_get_pwm_onoff_duty(WORD CardNo, WORD PwmNo,double* fOnDuty, double* fOffDuty);
//连续插补相对轨迹起点PWM滞后输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_delay_pwm_to_start(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
//（备用）
DMC_API short __stdcall dmc_conti_delay_pwm_to_stop(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double delay_time,double ReverseTime);
//连续插补相对轨迹终点PWM提前输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_ahead_pwm_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
//连续插补PWM立即输出（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_write_pwm(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double ReverseTime);

/*********************ADDA输出******************************/
//PWM转DA输出
//设置DA输出使能（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_da_enable(WORD CardNo,WORD enable);
DMC_API short __stdcall dmc_get_da_enable(WORD CardNo,WORD* enable);
//设置DA输出（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_da_output(WORD CardNo, WORD channel,double Vout);
DMC_API short __stdcall dmc_get_da_output(WORD CardNo,WORD channel,double* Vout);
//读取AD输入（适用于所有脉冲卡）
DMC_API short __stdcall dmc_get_ad_input(WORD CardNo,WORD channel,double* Vout);
//设置连续插补DA输出 5000脉冲卡
DMC_API short __stdcall dmc_conti_set_da_output(WORD CardNo, WORD Crd, WORD channel,double Vout);
//设置连续DA使能（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_conti_set_da_enable(WORD CardNo, WORD Crd, WORD enable,WORD channel,long mark);
//编码器da跟随（预留）
DMC_API short __stdcall dmc_set_encoder_da_follow_enable(WORD CardNo,WORD axis,WORD enable);	
DMC_API short __stdcall dmc_get_encoder_da_follow_enable(WORD CardNo,WORD axis,WORD* enable);
/**********DA速度跟随**************
da_no:通道号
MaxVel:最大运行速度，单位unit
MaxValue:最大电压
*************************************/
//连续插补DA速度跟随（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_conti_set_da_follow_speed(WORD CardNo,WORD Crd,WORD da_no,double MaxVel,double MaxValue,double acc_offset,double dec_offset,double acc_dist,double dec_dist);
DMC_API short __stdcall dmc_conti_get_da_follow_speed(WORD CardNo,WORD Crd,WORD da_no,double* MaxVel,double* MaxValue,double* acc_offset,double* dec_offset,double* acc_dist,double* dec_dist);

/******************************CAN IO（旧接口函数）***********************************/
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
//设置主站参数
DMC_API short __stdcall nmc_set_master_para(WORD CardNo,WORD PortNum,WORD Baudrate,DWORD NodeCnt,WORD MasterId);
//读取主站参数
DMC_API short __stdcall nmc_get_master_para(WORD CardNo,WORD PortNum,WORD *Baudrate,DWORD *NodeCnt,WORD *MasterId);
/********************CAN IO**************************/
//设置CAN io输出（适用于所有脉冲卡）
DMC_API short __stdcall nmc_write_outbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD IoValue);
//读取CAN io输出（适用于所有脉冲卡）
DMC_API short __stdcall nmc_read_outbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue);
//读取CAN io输入（适用于所有脉冲卡）
DMC_API short __stdcall nmc_read_inbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue);
//设置CAN io输出32位（适用于所有脉冲卡）
DMC_API short __stdcall nmc_write_outport(WORD CardNo,WORD NoteID,WORD portno,DWORD outport_val);
//读取CAN io输出32位（适用于所有脉冲卡）
DMC_API short __stdcall nmc_read_outport(WORD CardNo,WORD NoteID,WORD portno,DWORD *outport_val);
//读取CAN io输入32位（适用于所有脉冲卡）
DMC_API short __stdcall nmc_read_inport(WORD CardNo,WORD NoteID,WORD portno,DWORD *inport_val);
/********************CAN IO带连接状态返回值**************************/
//设置CAN io输出（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_write_outbit_ex(WORD CardNo,WORD NoteID,WORD IoBit,WORD IoValue,WORD* state);
//读取CAN io输出（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_read_outbit_ex(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue,WORD* state);
//读取CAN io输入（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_read_inbit_ex(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue,WORD* state);
//设置CAN io输出32位（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_write_outport_ex(WORD CardNo,WORD NoteID,WORD portno,DWORD outport_val,WORD* state);
//读取CAN io输出32位（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_read_outport_ex(WORD CardNo,WORD NoteID,WORD portno,DWORD *outport_val,WORD* state);
//读取CAN io输入32位（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_read_inport_ex(WORD CardNo,WORD NoteID,WORD portno,DWORD *inport_val,WORD* state);
/********************CAN ADDA**************************/
//CAN ADDA指令 设置DA参数 （适用于所有脉冲卡）
DMC_API short __stdcall nmc_set_da_output(WORD CardNo,WORD NoteID,WORD channel,double Value);
//读取CAN DA参数（适用于所有脉冲卡）
DMC_API short __stdcall nmc_get_da_output(WORD CardNo,WORD NoteID,WORD channel,double *Value);
//读取CAN AD参数（适用于所有脉冲卡）
DMC_API short __stdcall nmc_get_ad_input(WORD CardNo,WORD NoteID,WORD channel,double *Value);
//配置CAN AD模式（适用于所有脉冲卡）
DMC_API short __stdcall nmc_set_ad_mode(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums);
DMC_API short __stdcall nmc_get_ad_mode(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums);
//配置CAN DA模式（适用于所有脉冲卡）
DMC_API short __stdcall nmc_set_da_mode(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums);
DMC_API short __stdcall nmc_get_da_mode(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums);
//CAN参数写入flash（适用于所有脉冲卡）
DMC_API short __stdcall nmc_write_to_flash(WORD CardNo,WORD PortNum,WORD NodeNum);
//CAN总线链接（适用于所有脉冲卡）
DMC_API short __stdcall nmc_set_connect_state(WORD CardNo,WORD NodeNum,WORD state,WORD baud);//0-断开；1-连接；2-复位后自动连接
DMC_API short __stdcall nmc_get_connect_state(WORD CardNo,WORD* NodeNum,WORD* state);//0-断开；1-连接；2-异常
/********************CAN ADDA带连接状态返回值**************************/
//设置CAN DA参数（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_set_da_output_ex(WORD CardNo,WORD NoteID,WORD channel,double Value,WORD* state);
//读取CAN DA参数（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_get_da_output_ex(WORD CardNo,WORD NoteID,WORD channel,double *Value,WORD* state);
//读取CAN AD参数（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_get_ad_input_ex(WORD CardNo,WORD NoteID,WORD channel,double *Value,WORD* state);
//配置CAN AD模式（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_set_ad_mode_ex(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums,WORD* state);
DMC_API short __stdcall nmc_get_ad_mode_ex(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums,WORD* state);
//配置CAN DA模式（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_set_da_mode_ex(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums,WORD* state);
DMC_API short __stdcall nmc_get_da_mode_ex(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums,WORD* state);
//参数写入flash（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall nmc_write_to_flash_ex(WORD CardNo,WORD PortNum,WORD NodeNum,WORD* state);

//预留
DMC_API long __stdcall  dmc_set_profile_limit(WORD CardNo,WORD axis,double Max_Vel,double Max_Acc,double EvenTime);
DMC_API long __stdcall  dmc_get_profile_limit(WORD CardNo,WORD axis,double* Max_Vel,double* Max_Acc,double* EvenTime);
DMC_API long __stdcall  dmc_set_vector_profile_limit(WORD CardNo,WORD Crd,double Max_Vel,double Max_Acc,double EvenTime);
DMC_API long __stdcall  dmc_get_vector_profile_limit(WORD CardNo,WORD Crd,double* Max_Vel,double* Max_Acc,double* EvenTime);
//小圆限速使能（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_arc_limit(WORD CardNo,WORD Crd,WORD Enable,double MaxCenAcc,double MaxArcError);
DMC_API short __stdcall dmc_get_arc_limit(WORD CardNo,WORD Crd,WORD* Enable,double* MaxCenAcc,double* MaxArcError);

//DMC_API short __stdcall dmc_get_axis_debug_state(WORD CardNo,WORD axis,struct_DebugPara* pack);


//（预留）
DMC_API short __stdcall dmc_set_IoFilter(WORD CardNo,WORD bitno, double filter);
DMC_API short __stdcall dmc_get_IoFilter(WORD CardNo,WORD bitno, double *filter);

//螺距补偿（旧指令，不使用）
DMC_API short __stdcall dmc_set_lsc_index_value (WORD CardNo, WORD axis,WORD IndexID, long IndexValue);
DMC_API short __stdcall dmc_get_lsc_index_value(WORD CardNo, WORD axis,WORD IndexID, long *IndexValue);
DMC_API short __stdcall dmc_set_lsc_config(WORD CardNo, WORD axis,WORD Origin, DWORD Interal,DWORD NegIndex,DWORD PosIndex,double Ratio);
DMC_API short __stdcall dmc_get_lsc_config(WORD CardNo, WORD axis,WORD *Origin, DWORD *Interal,DWORD *NegIndex,DWORD *PosIndex,double *Ratio);
//看门狗（旧指令，不使用）
DMC_API short __stdcall dmc_set_watchdog(WORD CardNo,WORD enable,DWORD time);
DMC_API short __stdcall dmc_call_watchdog(WORD CardNo);
DMC_API short __stdcall dmc_read_diagnoseData(WORD CardNo);	
DMC_API short __stdcall dmc_conti_set_cmd_end(WORD CardNo,WORD Crd,WORD enable);
//区域软限位（保留）
DMC_API short __stdcall dmc_set_zone_limit_config(WORD CardNo, WORD *axis, WORD *Source, long x_pos_p, long x_pos_n, long y_pos_p, long y_pos_n, WORD action_para);
DMC_API short __stdcall dmc_get_zone_limit_config(WORD CardNo, WORD* axis, WORD* Source, long* x_pos_p, long* x_pos_n, long* y_pos_p, long* y_pos_n, WORD* action_para);
DMC_API short __stdcall dmc_set_zone_limit_enable(WORD CardNo, WORD enable);
//轴互锁功能（保留）
DMC_API short __stdcall dmc_set_interlock_config(WORD CardNo, WORD* axis, WORD* Source, long delta_pos, WORD action_para);
DMC_API short __stdcall dmc_get_interlock_config(WORD CardNo, WORD* axis, WORD* Source, long* delta_pos, WORD* action_para);
DMC_API short __stdcall dmc_set_interlock_enable(WORD CardNo, WORD enable);
//龙门模式的误差保护（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_grant_error_protect(WORD CardNo, WORD axis,WORD enable,DWORD dstp_error, DWORD emg_error);
DMC_API short __stdcall dmc_get_grant_error_protect(WORD CardNo, WORD axis,WORD* enable,DWORD* dstp_error, DWORD* emg_error);
//龙门模式的误差保护unit（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_grant_error_protect_unit(WORD CardNo, WORD axis,WORD enable,double dstp_error, double emg_error);
DMC_API short __stdcall dmc_get_grant_error_protect_unit(WORD CardNo, WORD axis,WORD* enable,double* dstp_error, double* emg_error);
//（保留）
DMC_API short __stdcall dmc_set_safety_param(WORD CardNo,WORD axis,WORD enable,long safety_pos);	
DMC_API short __stdcall dmc_get_safety_param(WORD CardNo,WORD axis,WORD* enable,long* safety_pos);	
DMC_API short __stdcall dmc_get_diagnose_param(WORD CardNo,WORD axis,long* tartet_pos,int* mode,long* pulse_pos,long* endcoder_pos);

//物件分拣功能（分拣固件专用）
DMC_API short __stdcall dmc_set_camerablow_config(WORD CardNo,WORD camerablow_en,long cameraPos,WORD piece_num,long piece_distance,WORD axis_sel,long latch_distance_min);	
DMC_API short __stdcall dmc_get_camerablow_config(WORD CardNo,WORD* camerablow_en,long* cameraPos,WORD* piece_num,long* piece_distance,WORD* axis_sel,long* latch_distance_min);	
DMC_API short __stdcall dmc_clear_camerablow_errorcode(WORD CardNo);	
DMC_API short __stdcall dmc_get_camerablow_errorcode(WORD CardNo,WORD* errorcode);

//配置通用输入（0~15）做为轴的限位信号（保留）
DMC_API short __stdcall dmc_set_io_limit_config(WORD CardNo,WORD portno,WORD enable,WORD axis_sel,WORD el_mode,WORD el_logic);	
DMC_API short __stdcall dmc_get_io_limit_config(WORD CardNo,WORD portno,WORD* enable,WORD* axis_sel,WORD* el_mode,WORD* el_logic);	
//手轮滤波参数（保留）
DMC_API short __stdcall dmc_set_handwheel_filter(WORD CardNo,WORD axis,double filter_factor);
DMC_API short __stdcall dmc_get_handwheel_filter(WORD CardNo,WORD axis,double* filter_factor);
//读取坐标系各轴的当前规划坐标（保留）
DMC_API short __stdcall dmc_conti_get_interp_map(WORD CardNo,WORD Crd,WORD* AxisNum ,WORD* AxisList,double *pPosList);
//坐标系错误代码 （保留）
DMC_API short __stdcall dmc_conti_get_crd_errcode(WORD CardNo,WORD Crd,WORD* errcode);
//保留
DMC_API short __stdcall dmc_line_unit_follow(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode);
DMC_API short __stdcall dmc_conti_line_unit_follow(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark);
//连续插补缓冲区DA操作（保留）
DMC_API short __stdcall dmc_conti_set_da_action(WORD CardNo,WORD Crd,WORD mode,WORD portno,double dvalue);
//读编码器速度（保留）
DMC_API short __stdcall dmc_read_encoder_speed(WORD CardNo,WORD Axis,double *current_speed);
//保留
DMC_API short __stdcall dmc_axis_follow_line_enable(WORD CardNo,WORD Crd,WORD enable_flag);
//插补轴脉冲补偿（保留）
DMC_API short __stdcall dmc_set_interp_compensation(WORD CardNo,WORD axis, double dvalue,double time);
DMC_API short __stdcall dmc_get_interp_compensation(WORD CardNo,WORD axis, double *dvalue,double *time); 
//读取相对于起点的距离（保留）
DMC_API short __stdcall dmc_get_distance_to_start(WORD CardNo,WORD Crd, double* distance_x, double* distance_y,long imark); 
//设置标志位 表示是否开始计算相对起点（保留）
DMC_API short __stdcall dmc_set_start_distance_flag(WORD CardNo,WORD Crd,WORD flag);



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
//（适用于DMC5000/5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_gear_unit(WORD CardNo,WORD Crd,WORD axis,double dist, WORD follow_mode,long imark); 

//轨迹拟合使能设置（保留）
DMC_API short __stdcall dmc_set_path_fitting_enable(WORD CardNo,WORD Crd,WORD enable);

//螺距补偿相关
//螺距补偿功能（适用于所有脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_enable_leadscrew_comp(WORD CardNo, WORD axis,WORD enable);
//配置逻辑补偿参数（适用于所有脉冲卡）
DMC_API short __stdcall dmc_set_leadscrew_comp_config(WORD CardNo, WORD axis,WORD n, int startpos,int lenpos,int *pCompPos,int *pCompNeg);
DMC_API short __stdcall dmc_get_leadscrew_comp_config(WORD CardNo, WORD axis,WORD *n, int *startpos,int *lenpos,int *pCompPos,int *pCompNeg);
//配置逻辑补偿参数unit（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_leadscrew_comp_config_unit(WORD CardNo, WORD axis,WORD n, double startpos,double lenpos,double *pCompPos,double *pCompNeg);
DMC_API short __stdcall dmc_get_leadscrew_comp_config_unit(WORD CardNo, WORD axis,WORD *n, double *startpos,double *lenpos,double *pCompPos,double *pCompNeg);
//螺距补偿前的脉冲位置，编码器位置//20191025（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_get_position_ex(WORD CardNo,WORD axis, double * pos);
DMC_API short __stdcall dmc_get_encoder_ex(WORD CardNo,WORD axis, double * pos);
//螺距补偿前的脉冲位置，编码器位置 当量（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_get_position_ex_unit(WORD CardNo,WORD axis, double * pos);
DMC_API short __stdcall dmc_get_encoder_ex_unit(WORD CardNo,WORD axis, double * pos);

//指定轴做定长位移运动 按固定曲线运动（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_t_pmove_extern(WORD CardNo, WORD axis, double MidPos,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,WORD posi_mode);
//
DMC_API short __stdcall dmc_t_pmove_extern_unit(WORD CardNo, WORD axis, double MidPos,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,WORD posi_mode);
/*
功能：设置脉冲计数值和编码器反馈值之间差值的报警阀值
输入参数：CardNo 卡号
axis 轴号
error 差值报警报警阀值
输出参数：无
*///（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_pulse_encoder_count_error(WORD CardNo,WORD axis,WORD error);
DMC_API short __stdcall dmc_get_pulse_encoder_count_error(WORD CardNo,WORD axis,WORD *error);
/*
功能：检查脉冲计数值和编码器反馈值之间差值是否超过报警阀值
	输入参数：CardNo 卡号
	axis 轴号
	输出参数;无
	返回参数：0：差值小于报警阀值
	1：差值大于等于报警阀值
*///（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_check_pulse_encoder_count_error(WORD CardNo,WORD axis,int* pulse_position, int* enc_position);
//使能和设置跟踪编码器误差不在范围内时轴的停止模式（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_encoder_count_error_action_config(WORD CardNo,WORD enable,WORD stopmode);
DMC_API short __stdcall dmc_get_encoder_count_error_action_config(WORD CardNo,WORD* enable,WORD* stopmode);

//新物件分拣功能 分拣固件专用
//固定
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
//内部使用（保留）
DMC_API short __stdcall dmc_set_sevon_enable(WORD CardNo,WORD axis,WORD on_off);
DMC_API short __stdcall dmc_get_sevon_enable(WORD CardNo,WORD axis);

//连续编码器da跟随（适用于DMC5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_conti_set_encoder_da_follow_enable(WORD CardNo, WORD Crd,WORD axis,WORD enable);	
DMC_API short __stdcall dmc_conti_get_encoder_da_follow_enable(WORD CardNo, WORD Crd,WORD* axis,WORD* enable);
//保留
DMC_API short __stdcall dmc_check_done_pos(WORD CardNo,WORD axis,WORD posi_mode);
//设置位置误差带（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_factor_error(WORD CardNo,WORD axis,double factor,long error);
DMC_API short __stdcall dmc_get_factor_error(WORD CardNo,WORD axis,double* factor,long* error);
//保留
DMC_API short __stdcall dmc_set_factor(WORD CardNo,WORD axis,double factor);
DMC_API short __stdcall dmc_set_error(WORD CardNo,WORD axis,long error);
//检测指令到位（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_check_success_pulse(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_check_success_encoder(WORD CardNo,WORD axis);

//IO及编码器计数功能（保留）
DMC_API short __stdcall dmc_set_io_count_profile(WORD CardNo, WORD chan, WORD bitno,WORD mode,double filter, double count_value, WORD* axis_list, WORD axis_num, WORD stop_mode );
DMC_API short __stdcall dmc_get_io_count_profile(WORD CardNo, WORD chan, WORD* bitno,WORD* mode,double* filter, double* count_value, WORD* axis_list, WORD* axis_num, WORD* stop_mode );
DMC_API short __stdcall dmc_set_io_count_enable(WORD CardNo, WORD chan, WORD ifenable);
DMC_API short __stdcall dmc_clear_io_count(WORD CardNo, WORD chan);
DMC_API short __stdcall dmc_get_io_count_value_extern(WORD CardNo, WORD chan, long* current_value);
//保留
DMC_API short __stdcall dmc_change_speed_extend(WORD CardNo,WORD axis,double Curr_Vel, double Taccdec, WORD pin_num, WORD trig_mode);
DMC_API short __stdcall dmc_follow_vector_speed_move(WORD CardNo,WORD axis,WORD Follow_AxisNum,WORD* Follow_AxisList,double ratio);
DMC_API short __stdcall dmc_conti_line_unit_extend(WORD CardNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* pPosList, WORD posi_mode, double Extend_Len, WORD enable,long mark); //连续插补直线

//保留
DMC_API short __stdcall dmc_conti_line_io_union(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,WORD bitno,WORD on_off,double io_value,WORD io_mode,WORD MapAxis,WORD pos_source,double ReverseTime,long mark);
//设置编码器方向（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_set_encoder_dir(WORD CardNo, WORD axis,WORD dir);

//圆形区域软限位（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_arc_zone_limit_config(WORD CardNo, WORD* AxisList, WORD AxisNum, double *Center, double Radius, WORD Source,WORD StopMode);
DMC_API short __stdcall dmc_get_arc_zone_limit_config(WORD CardNo, WORD* AxisList, WORD* AxisNum, double *Center, double* Radius, WORD* Source,WORD* StopMode);
//查询相应轴的状态（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_get_arc_zone_limit_axis_status(WORD CardNo, WORD axis);
//圆形限位使能（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_arc_zone_limit_enable(WORD CardNo, WORD enable);
DMC_API short __stdcall dmc_get_arc_zone_limit_enable(WORD CardNo, WORD* enable);

//控制卡接线盒断线后是否初始化输出电平（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_output_status_repower(WORD CardNo, WORD enable);
//旧接口（软启动），不使用
DMC_API short __stdcall dmc_t_pmove_extern_softlanding(WORD CardNo, WORD axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, DWORD delay_ms, double Max_Vel2,double stop_vel2, double acc_time, double dec_time, WORD posi_mode);
//保留
DMC_API short __stdcall dmc_compare_add_point_XD(WORD CardNo,WORD cmp,long pos,WORD dir, WORD action,DWORD actpara, long startPos);//硒电定制比较函数
//配置ORG输入触发在线变速变位（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_pmove_change_pos_speed_config(WORD CardNo,WORD axis,double tar_vel, double tar_rel_pos, WORD trig_mode, WORD source);
DMC_API short __stdcall dmc_get_pmove_change_pos_speed_config(WORD CardNo,WORD axis,double* tar_vel, double* tar_rel_pos, WORD* trig_mode, WORD* source);
//ORG输入触发在线变速变位使能（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_pmove_change_pos_speed_enable(WORD CardNo,WORD axis, WORD enable);
DMC_API short __stdcall dmc_get_pmove_change_pos_speed_enable(WORD CardNo,WORD axis, WORD* enable);
//读取ORG输入触发在线变速变位的状态  trig_num 触发次数，trig_pos 触发位置（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_get_pmove_change_pos_speed_state(WORD CardNo,WORD axis, WORD * trig_num, double * trig_pos);

//保留
DMC_API short __stdcall dmc_compare_add_point_extend(WORD CardNo,WORD axis, long pos, WORD dir, WORD action, WORD para_num, DWORD* actpara,DWORD compare_time);
DMC_API short __stdcall dmc_get_cmd_position(WORD CardNo,WORD axis, double * pos);
//逻辑采样配置（内部使用）
DMC_API short __stdcall dmc_set_logic_analyzer_config(WORD CardNo,WORD channel, DWORD SampleFre, DWORD SampleDepth, WORD SampleMode);
DMC_API short __stdcall dmc_start_logic_analyzer(WORD CardNo,WORD channel, WORD enable);
DMC_API short __stdcall dmc_get_logic_analyzer_counter(WORD CardNo, WORD channel, DWORD *counter);
//20190923修改kg定制函数接口（客户定制）
DMC_API short __stdcall dmc_read_inbit_append(WORD CardNo,WORD bitno);//读取输入口的状态
DMC_API short __stdcall dmc_write_outbit_append(WORD CardNo,WORD bitno,WORD on_off);//设置输出口的状态
DMC_API short __stdcall dmc_read_outbit_append(WORD CardNo,WORD bitno);//读取输出口的状态
DMC_API DWORD __stdcall dmc_read_inport_append(WORD CardNo,WORD portno);//读取输入端口的值
DMC_API DWORD __stdcall dmc_read_outport_append(WORD CardNo,WORD portno);//读取输出端口的值
DMC_API short __stdcall dmc_write_outport_append(WORD CardNo,WORD portno,DWORD port_value);//设置所有输出端口的值
//M运动测试（不使用）
DMC_API short __stdcall	dmc_m_move_unit(WORD CardNo,WORD Crd, WORD axis_num, WORD* axis_list,double* mid_pos, double* target_pos, double* saftpos, WORD pos_mode);
DMC_API short __stdcall	dmc_get_m_move_config(WORD CardNo,WORD Crd, WORD *axis_num, WORD* axis_list,double* mid_pos, double* target_pos, double* saftpos,WORD* pos_mode);

/***************椭圆插补及切向跟随****************/
// 设置坐标系切向跟随（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_set_tangent_follow(WORD CardNo, WORD Crd, WORD axis, WORD follow_curve, WORD rotate_dir, double degree_equivalent);
// 获取指定坐标系切向跟随参数（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API	short __stdcall dmc_get_tangent_follow_param(WORD CardNo, WORD Crd, WORD* axis, WORD* follow_curve, WORD* rotate_dir, double* degree_equivalent);
// 取消坐标系跟随（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_disable_follow_move(WORD CardNo, WORD Crd);
// 椭圆插补（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_ellipse_move(WORD CardNo, WORD Crd,WORD axisNum, WORD* Axis_List, double* Target_Pos, double* Cen_Pos, double A_Axis_Len, double B_Axis_Len, WORD Dir, WORD Pos_Mode);

//新看门狗功能
//设置看门口触发响应事件（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_watchdog_action_event(WORD CardNo, WORD event_mask);
DMC_API short __stdcall dmc_get_watchdog_action_event(WORD CardNo, WORD* event_mask);
//使能看门口保护机制（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_watchdog_enable (WORD CardNo, double timer_period, WORD enable);
DMC_API short __stdcall dmc_get_watchdog_enable (WORD CardNo, double * timer_period, WORD* enable);
//复位看门狗定时器（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_reset_watchdog_timer (WORD CardNo);

//io定制功能（定制类）
DMC_API short __stdcall dmc_set_io_check_control(WORD CardNo, WORD sensor_in_no, WORD check_mode, WORD A_out_no, WORD B_out_no, WORD C_out_no, WORD output_mode);
DMC_API short __stdcall dmc_get_io_check_control(WORD CardNo, WORD* sensor_in_no, WORD* check_mode, WORD* A_out_no, WORD* B_out_no, WORD* C_out_no, WORD* output_mode);
DMC_API short __stdcall dmc_stop_io_check_control(WORD CardNo);

//设置限位反找偏移距离（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_el_ret_deviation(WORD CardNo, WORD axis, WORD enable,double deviation);
DMC_API short __stdcall  dmc_get_el_ret_deviation(WORD CardNo, WORD axis, WORD* enable, double* deviation);

/*****************************总线相关函数**********************************/
//设置回零参数（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_set_home_profile(WORD CardNo ,WORD axis,WORD home_mode,double Low_Vel, double High_Vel,double Tacc,double Tdec ,double offsetpos);//设置回零参数，合并函数
DMC_API short __stdcall nmc_get_home_profile(WORD CardNo ,WORD axis,WORD* home_mode,double* Low_Vel, double* High_Vel,double* Tacc,double* Tdec ,double* offsetpos);
//启动回零（适用于EtherCAT总线卡）
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
//设置从站对象字典参数（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_set_node_od(WORD CardNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,long Value);
DMC_API short __stdcall nmc_get_node_od(WORD CardNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,long* Value);
//总线ENI配置文件
DMC_API short __stdcall nmc_download_configfile(WORD CardNo,WORD PortNum,const char *FileName);
DMC_API short __stdcall nmc_upload_configfile(WORD CardNo,WORD PortNum, const char *FileName);
DMC_API short __stdcall nmc_reset_to_factory(WORD CardNo,WORD PortNum,WORD NodeNum);
DMC_API short __stdcall nmc_write_to_pci(WORD CardNo,WORD PortNum,WORD NodeNum);
//总线映射文件
DMC_API short __stdcall nmc_download_mapfile(WORD CardNo,const char *FileName);

//总线单轴使能函数 255表示全使能（适用于所有总线卡）
DMC_API short __stdcall nmc_set_axis_enable(WORD CardNo,WORD axis);
DMC_API short __stdcall nmc_set_axis_disable(WORD CardNo,WORD axis);
//清除报警信号(内部使用)
DMC_API short __stdcall nmc_set_alarm_clear(WORD CardNo,WORD PortNum,WORD NodeNum);
DMC_API short __stdcall nmc_get_slave_nodes(WORD CardNo,WORD PortNum,WORD BaudRate,WORD* NodeId,WORD* NodeNum);
//获取总线轴数（适用于所有总线卡）
DMC_API short __stdcall nmc_get_total_axes(WORD CardNo,DWORD* TotalAxis);
//获取总线ADDA输入输出口数（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_total_adcnum(WORD CardNo,WORD* TotalIn,WORD* TotalOut);
//获取总线IO口数（适用于所有总线卡）
DMC_API short __stdcall nmc_get_total_ionum(WORD CardNo,WORD *TotalIn,WORD *TotalOut);
//清除端口报警(内部使用)
DMC_API short __stdcall nmc_clear_alarm_fieldbus(WORD CardNo,WORD PortNum);
//获取控制器工作模式  1表示ethercat模式，0表示仿真模式（适用于所有总线卡）
DMC_API short __stdcall nmc_get_controller_workmode(WORD CardNo,WORD* controller_mode);
//设置控制器工作模式  1表示ethercat模式，0表示仿真模式（适用于所有总线卡）
DMC_API short __stdcall nmc_set_controller_workmode(WORD CardNo,WORD controller_mode);
//设置ethercat总线循环周期(us)（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_set_cycletime(WORD CardNo,WORD PortNum,DWORD CycleTime);
//获取ethercat总线循环周期(us)（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_cycletime(WORD CardNo,WORD PortNum,DWORD* CycleTime);
//读取总线线程时间消耗参数(内部使用)
DMC_API short __stdcall dmc_get_perline_time(WORD CardNo,WORD TypeIndex,DWORD *Averagetime,DWORD *Maxtime,uint64 *Cycles ); //TypeIndex:0~6  m_Averagetime ; 平均时间 m_Maxtime;最大时间 uint64  m_Cycles;当前时间
//设置轴的运行模式 1为pp模式，6为回零模式，8为csp模式(内部使用)
DMC_API short __stdcall nmc_set_axis_run_mode(WORD CardNo,WORD axis,WORD run_mode);

//获取轴类型
/*
enum fieldbus_type
{
	virtual_type=0,
	ect_type=1,
	can_type=2,
	pulse_type=3, // or local IO 
	unknown=4
};*///（适用于所有总线卡）
DMC_API short __stdcall nmc_get_axis_type(WORD CardNo,WORD axis, WORD* Axis_Type);
//获取总线时间量，平均时间，最大时间，执行周期数（适用于所有总线卡）
DMC_API short __stdcall nmc_get_consume_time_fieldbus(WORD CardNo,WORD PortNum,DWORD* Average_time, DWORD* Max_time,uint64* Cycles);
//清除时间量（适用于所有总线卡）
DMC_API short __stdcall nmc_clear_consume_time_fieldbus(WORD CardNo,WORD PortNum);
//停止ethercat总线,返回0表示成功，其他参数表示不成功（适用于所有总线卡）
DMC_API short __stdcall nmc_stop_etc(WORD CardNo,WORD* ETCState);

//---------------------------总线轴模式---------------
//获取轴状态字（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_axis_statusword(WORD CardNo,WORD axis,long* statusword);
//设置总线轴控制字（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_set_axis_contrlword(WORD CardNo,WORD Axis,long Contrlword);
//获取总线轴控制字（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_axis_contrlword(WORD CardNo,WORD Axis,long *Contrlword);

DMC_API short __stdcall nmc_set_axis_contrlmode(WORD CardNo,WORD Axis,long Contrlmode);
DMC_API short __stdcall nmc_get_axis_contrlmode(WORD CardNo,WORD Axis,long *Contrlmode);

// 获取总线端口错误码（适用于所有总线卡）
DMC_API short __stdcall nmc_get_errcode(WORD CardNo,WORD channel,WORD *Errcode);
// 获取控制卡错误码（受限使用）
DMC_API short __stdcall nmc_get_card_errcode(WORD CardNo,WORD *Errcode);
// 获取总线轴错误码（适用于所有总线卡）
DMC_API short __stdcall nmc_get_axis_errcode(WORD CardNo,WORD axis,WORD *Errcode);
// 清除总线端口错误码（适用于所有总线卡）
DMC_API short __stdcall nmc_clear_errcode(WORD CardNo,WORD channel);
// 清除控制卡错误码（受限使用）
DMC_API short __stdcall nmc_clear_card_errcode(WORD CardNo);
// 清除总线轴错误码（适用于所有总线卡）
DMC_API short __stdcall nmc_clear_axis_errcode(WORD CardNo,WORD iaxis);
//（受限使用）
DMC_API short __stdcall nmc_get_LostHeartbeat_Nodes(WORD CardNo,WORD PortNum,WORD* NodeID,WORD* NodeNum);
DMC_API short __stdcall nmc_get_EmergeneyMessege_Nodes(WORD CardNo,WORD PortNum,DWORD* NodeMsg,WORD* MsgNum);
DMC_API short __stdcall nmc_SendNmtCommand(WORD CardNo,WORD PortNum,WORD NodeID,WORD NmtCommand);
DMC_API short __stdcall nmc_syn_move(WORD CardNo,WORD AxisNum,WORD* AxisList,long* Position,WORD* PosiMode);
DMC_API short __stdcall nmc_syn_move_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,double* Position,WORD* PosiMode);
//总线多轴同步运动（受限使用）
DMC_API short __stdcall nmc_sync_pmove_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,double* Dist,WORD* PosiMode);
DMC_API short __stdcall nmc_sync_vmove_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* Dir);

//轴状态机（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_axis_state_machine(WORD CardNo,WORD axis, WORD* Axis_StateMachine);
//获取轴配置控制模式，返回值（6回零模式，8csp模式）（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_axis_setting_contrlmode(WORD CardNo,WORD axis,long* contrlmode);
// 获取从站个数（适用于所有总线卡）
DMC_API short __stdcall nmc_get_total_slaves(WORD CardNo,WORD PortNum,WORD* TotalSlaves);
// 获取轴的从站信息（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_axis_node_address(WORD CardNo,WORD axis, WORD* SlaveAddr,WORD* Sub_SlaveAddr);
//设置驱动器上IO输出（适用于所有总线卡）
DMC_API short __stdcall nmc_set_axis_io_out(WORD CardNo,WORD axis,DWORD  iostate);
DMC_API DWORD __stdcall nmc_get_axis_io_out(WORD CardNo,WORD axis);
//读取驱动器上IO输入（适用于所有总线卡）
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
//写入Rtex驱动参数（适用于Rtex总线卡）
DMC_API short __stdcall nmc_write_parameter(WORD CardNo,WORD axis,WORD index, WORD subindex,DWORD para_data);
/**************************************************************
*功能说明：RTEX驱动器写EEPROM操作（适用于Rtex总线卡）
*
*
**************************************************************/
DMC_API short __stdcall nmc_write_slave_eeprom(WORD CardNo,WORD axis);
/**************************************************************
*index:rtex驱动器的参数分类
*subindex:rtex驱动器在index类别下的参数编号（适用于Rtex总线卡）
*para_data:读出的参数值
**************************************************************/
DMC_API short __stdcall nmc_read_parameter(WORD CardNo,WORD axis,WORD index, WORD subindex,DWORD* para_data);
/**************************************************************
*index:rtex驱动器的参数分类
*subindex:rtex驱动器在index类别下的参数编号（适用于Rtex总线卡）
*para_data:读出的参数值
**************************************************************/
DMC_API short __stdcall nmc_read_parameter_attributes(WORD CardNo,WORD axis,WORD index, WORD subindex,DWORD* para_data);
DMC_API short __stdcall nmc_set_cmdcycletime(WORD CardNo,WORD PortNum,DWORD cmdtime);
//设置RTEX总线周期比(us)（适用于Rtex总线卡）
DMC_API short __stdcall nmc_get_cmdcycletime(WORD CardNo,WORD PortNum,DWORD* cmdtime);
DMC_API short __stdcall nmc_start_log(WORD CardNo,WORD chan,WORD node, WORD Ifenable);
DMC_API short __stdcall nmc_get_log(WORD CardNo,WORD chan,WORD node, DWORD* data);
DMC_API short __stdcall nmc_config_atuo_log(WORD CardNo,WORD ifenable,WORD dir,WORD byte_index,WORD mask,WORD condition,DWORD counter);

//扩展PDO
//设置读取从站扩展有符号RxPDO的值（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_write_rxpdo_extra(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,int Value);
DMC_API short __stdcall nmc_read_rxpdo_extra(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,int* Value);
//设置读取从站扩展有符号TxPDO的值（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_read_txpdo_extra(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,int* Value);
DMC_API short __stdcall nmc_write_rxpdo_extra_uint(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,DWORD Value);
//读取从站扩展无符号RxPDO的值（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_read_rxpdo_extra_uint(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,DWORD* Value);
//读取从站扩展无符号TxPDO的值（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_read_txpdo_extra_uint(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,DWORD* Value);
//（受限使用）
DMC_API short __stdcall nmc_get_log_state(WORD CardNo,WORD chan, DWORD* state);
DMC_API short __stdcall nmc_driver_reset(WORD CardNo,WORD axis);
//设置偏移量值，使用驱动器实际位置（适用于所有总线卡）
DMC_API short __stdcall nmc_set_offset_pos(WORD CardNo,WORD axis, double offset_pos);
DMC_API short __stdcall nmc_get_offset_pos(WORD CardNo,WORD axis, double* offset_pos);
//清除rtex绝对值编码器的多圈值（适用于Rtex总线卡）
DMC_API short __stdcall nmc_clear_abs_driver_multi_cycle(WORD CardNo,WORD axis);

//---------------------------EtherCAT IO扩展模块操作指令----------------------
//设置io输出32位总线扩展（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_write_outport_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD portno,DWORD outport_val);
//读取io输出32位总线扩展（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_read_outport_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD portno,DWORD *outport_val);
//读取io输入32位总线扩展（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_read_inport_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD portno,DWORD *inport_val);
//设置io输出（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_write_outbit_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD IoBit,WORD IoValue);
//读取io输出（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_read_outbit_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD IoBit,WORD *IoValue);
//读取io输入（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_read_inbit_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD IoBit,WORD *IoValue);

//返回最近错误码（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_current_fieldbus_state_info(WORD CardNo,WORD Channel,WORD *Axis,WORD *ErrorType,WORD *SlaveAddr,DWORD *ErrorFieldbusCode);
// 返回历史错误码（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_detail_fieldbus_state_info(WORD CardNo,WORD Channel,DWORD ReadErrorNum,DWORD *TotalNum, DWORD *ActualNum, WORD *Axis,WORD *ErrorType,WORD *SlaveAddr,DWORD *ErrorFieldbusCode);

//启动采集（受限使用）
DMC_API short __stdcall nmc_start_pdo_trace(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD Index_Num,DWORD Trace_Len,WORD *Index,WORD *Sub_Index);
//获取采集参数（受限使用）
DMC_API short __stdcall nmc_get_pdo_trace(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD *Index_Num,DWORD *Trace_Len,WORD *Index,WORD *Sub_Index);
//设置触发采集参数（受限使用）
DMC_API short __stdcall nmc_set_pdo_trace_trig_para(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD Trig_Index,WORD Trig_Sub_Index,int Trig_Value,WORD Trig_Mode);
//获取触发采集参数（受限使用）
DMC_API short __stdcall nmc_get_pdo_trace_trig_para(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD *Trig_Index,WORD *Trig_Sub_Index,int *Trig_Value,WORD *Trig_Mode);
//采集清除（受限使用）
DMC_API short __stdcall nmc_clear_pdo_trace_data(WORD CardNo,WORD Channel,WORD SlaveAddr);
//采集停止（受限使用）
DMC_API short __stdcall nmc_stop_pdo_trace(WORD CardNo,WORD Channel,WORD SlaveAddr);
//采集数据读取（受限使用）
DMC_API short __stdcall nmc_read_pdo_trace_data(WORD CardNo,WORD Channel,WORD SlaveAddr,DWORD StartAddr,DWORD Readlen,DWORD *ActReadlen,BYTE *Data);
//已采集个数（受限使用）
DMC_API short __stdcall nmc_get_pdo_trace_num(WORD CardNo,WORD Channel,WORD SlaveAddr,DWORD *Data_num, DWORD *Size_of_each_bag);
//采集状态（受限使用）
DMC_API short __stdcall nmc_get_pdo_trace_state(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD *Trace_state);
//总线专用
//canopen总线复位（适用于所有脉冲卡）
DMC_API short __stdcall nmc_reset_canopen(WORD CardNo);
//Rtex总线复位（适用于Rtex总线卡）
DMC_API short __stdcall nmc_reset_rtex(WORD CardNo);
//EtherCAT总线复位（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_reset_etc(WORD CardNo);
//总线错误处理配置（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_set_fieldbus_error_switch(WORD CardNo, WORD channel,WORD data);
DMC_API short __stdcall nmc_get_fieldbus_error_switch(WORD CardNo, WORD channel,WORD* data);
//启动转矩运动（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_torque_move(WORD CardNo,WORD axis,int Torque,WORD PosLimitValid,double PosLimitValue,WORD PosMode);
//在线调整转矩值（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_change_torque(WORD CardNo,WORD axis,int Torque);
//读取转矩大小（适用于EtherCAT总线卡）
DMC_API short __stdcall nmc_get_torque(WORD CardNo,WORD axis,int* Torque);
//modbus函数（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_modbus_active_COM1(WORD id,const char* COMID,int speed, int bits, int check, int stop);
DMC_API short __stdcall dmc_modbus_active_COM2(WORD id,const char* COMID,int speed, int bits, int check, int stop);
DMC_API short __stdcall dmc_modbus_active_ETH(WORD id, WORD port);
//读写位寄存器（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_modbus_0x(WORD CardNo, WORD start, WORD inum, const char* pdata);
DMC_API short __stdcall dmc_get_modbus_0x(WORD CardNo, WORD start, WORD inum, char* pdata);
//读写字寄存器（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_modbus_4x(WORD CardNo, WORD start, WORD inum, const WORD* pdata);
DMC_API short __stdcall dmc_get_modbus_4x(WORD CardNo, WORD start, WORD inum, WORD* pdata);
//读写字寄存器float（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_modbus_4x_float(WORD CardNo, WORD start, WORD inum, const float* pdata);
DMC_API short __stdcall dmc_get_modbus_4x_float(WORD CardNo, WORD start, WORD inum, float* pdata);
//读写字寄存器int（适用于所有脉冲卡、总线卡）
DMC_API short __stdcall dmc_set_modbus_4x_int(WORD CardNo, WORD start, WORD inum, const int* pdata);
DMC_API short __stdcall dmc_get_modbus_4x_int(WORD CardNo, WORD start, WORD inum, int* pdata);

//两轴位置叠加，高速比较功能，定制试用中（测试使用）
DMC_API short __stdcall dmc_hcmp_set_config_overlap(WORD CardNo, WORD hcmp, WORD axis, WORD cmp_source, WORD cmp_logic, long time, WORD axis_num, WORD aux_axis, WORD aux_source);
DMC_API short __stdcall dmc_hcmp_get_config_overlap(WORD CardNo, WORD hcmp, WORD* axis, WORD* cmp_source, WORD* cmp_logic, long* time, WORD* axis_num, WORD* aux_axis, WORD* aux_source);

//M运动，测试指令（测试使用）
DMC_API short __stdcall	dmc_m_move_set_coodinate(WORD card,WORD liner, WORD axis_num, WORD* axis_list,uint32 in_io_num, WORD trig_flag, WORD pos_mode);
DMC_API short __stdcall	dmc_m_move_get_coodinate_para(WORD card,WORD liner, WORD* axis_num, WORD* axis_list,uint32* in_io_num, WORD* trig_flag, WORD* pos_mode);
DMC_API short __stdcall	dmc_m_move_set_z_para(WORD card,WORD liner, double z_up_safe_pos, double z_up_target_pos,double z_down_safe_pos, double z_down_target_pos);
DMC_API short __stdcall	dmc_m_move_get_z_para(WORD card,WORD liner, double* z_up_safe_pos, double* z_up_target_pos,double* z_down_safe_pos, double* z_down_target_pos);
DMC_API short __stdcall	dmc_m_move_set_xy_para(WORD card,WORD liner, double x_first_safe_pos,double x_second_safe_pos, double x_target_pos, WORD y_num ,double* y_target_pos);
DMC_API short __stdcall	dmc_m_move_get_xy_para(WORD card,WORD liner, double* x_first_safe_pos,double* x_second_safe_pos, double* x_target_pos, WORD y_num ,double* y_target_pos);
DMC_API short __stdcall	dmc_m_move_axes(WORD card,WORD liner);
DMC_API short __stdcall	dmc_m_move_get_run_phase(WORD card,WORD liner, WORD* run_phase);
DMC_API short __stdcall	dmc_m_move_stop(WORD card,WORD liner, WORD stop_mode);

/**************************五轴相关运动************************/
//（测试使用）
DMC_API short __stdcall dmc_rtcp_set_kinematic_type(WORD CardNo,WORD Crd, WORD machine_type);
DMC_API short __stdcall dmc_rtcp_get_kinematic_type(WORD CardNo,WORD Crd, WORD* machine_type);
//启动或者关闭RTCP功能（测试使用）
DMC_API short __stdcall dmc_rtcp_set_enable(WORD CardNo,WORD Crd, WORD enable);
DMC_API short __stdcall dmc_rtcp_get_enable(WORD CardNo,WORD Crd, WORD* enable);
//位置输入数据类型：0-工件坐标系位置，1-机械坐标系位置（测试使用）
DMC_API short __stdcall dmc_rtcp_set_position_type(WORD CardNo,WORD Crd, WORD position_type);
DMC_API short __stdcall dmc_rtcp_get_position_type(WORD CardNo,WORD Crd, WORD* position_type);
//设置A轴的坐标原点相对于前一个坐标系的偏移, xyz的偏移a_offset[3]（测试使用）
DMC_API short __stdcall dmc_rtcp_set_vector_a(WORD CardNo,WORD Crd, double* a_offset);
DMC_API short __stdcall dmc_rtcp_get_vector_a(WORD CardNo,WORD Crd, double* a_offset);
//设置B轴的坐标原点相对于前一个坐标系的偏移, xyz的偏移b_offset[3]（测试使用）
DMC_API short __stdcall dmc_rtcp_set_vector_b(WORD CardNo,WORD Crd, double* b_offset);
DMC_API short __stdcall dmc_rtcp_get_vector_b(WORD CardNo,WORD Crd, double* b_offset);
//设置C轴的坐标原点相对于前一个坐标系的偏移, xyz的偏移c_offset[3]（测试使用）
DMC_API short __stdcall dmc_rtcp_set_vector_c(WORD CardNo,WORD Crd, double* c_offset);
DMC_API short __stdcall dmc_rtcp_get_vector_c(WORD CardNo,WORD Crd, double* c_offset);
//设置A，B，C轴的偏移位置,
//A,B,C在回0后，再移动到初始姿态的位置，这个时候的A/B/C的偏移角度，
//如果到了初始姿态的位置不进行清0，这个时候就要设置偏移角度      （测试使用）
DMC_API short __stdcall dmc_rtcp_set_rotate_joint_offset(WORD CardNo,WORD Crd, double A, double B, double C);
DMC_API short __stdcall dmc_rtcp_get_rotate_joint_offset(WORD CardNo,WORD Crd, double* A, double* B, double* C);
//设置各轴的方向与工件坐标系的关系
//轴相对于工件的方向，如果轴正向运动的时候，驱动刀具相对于工件也是正向运动的，设定为1
//否则设为-1，初始设置为1
//dir[5],对应的是X,Y,Z,旋转轴1，旋转轴2                          （测试使用）
DMC_API short __stdcall dmc_rtcp_set_joints_direction(WORD CardNo,WORD Crd, int* dir);
DMC_API short __stdcall dmc_rtcp_get_joints_direction(WORD CardNo,WORD Crd, int* dir);
//设置刀具长度，在双摆头中有作用（测试使用）
DMC_API short __stdcall dmc_rtcp_set_tool_length(WORD CardNo,WORD Crd, double tool);
DMC_API short __stdcall dmc_rtcp_get_tool_length(WORD CardNo,WORD Crd, double* tool);
//（测试使用）
DMC_API short __stdcall dmc_rtcp_get_actual_pos(WORD CardNo,WORD Crd, WORD AxisNum,WORD *AxisList,double* actual_pos);
DMC_API short __stdcall dmc_rtcp_get_command_pos(WORD CardNo,WORD Crd, WORD AxisNum,WORD *AxisList,double* command_pos);
//（测试使用）
DMC_API short __stdcall dmc_rtcp_kinematics_forward(WORD CardNo,WORD Crd, double* joint_pos, double* axis_pos);
DMC_API short __stdcall dmc_rtcp_kinematics_inverse(WORD CardNo,WORD Crd, double* axis_pos, double* joint_pos);
//设置纯旋转轴的速度（测试使用）
DMC_API short __stdcall dmc_rtcp_set_max_rotate_param(WORD CardNo,WORD Crd, double rotate_speed, double rotate_acc);
DMC_API short __stdcall dmc_rtcp_get_max_rotate_param(WORD CardNo,WORD Crd, double* rotate_speed, double* rotate_acc);
//设置工件原点偏置值（测试使用）
DMC_API short __stdcall dmc_rtcp_set_workpiece_offset(WORD CardNo,WORD Crd, WORD workpiece_index, double* offset);
DMC_API short __stdcall dmc_rtcp_get_workpiece_offset(WORD CardNo,WORD Crd, WORD workpiece_index, double* offset);
//设置工件原点偏置方式（测试使用）
DMC_API short __stdcall dmc_rtcp_set_workpiece_mode(WORD CardNo,WORD Crd, WORD enable,WORD workpiece_index);
DMC_API short __stdcall dmc_rtcp_get_workpiece_mode(WORD CardNo,WORD Crd, WORD* enable,WORD* workpiece_index);

//螺旋插补(测试使用，DMC5000/5X10系列脉冲卡、E5032总线卡)
DMC_API short __stdcall dmc_conti_helix_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD * AixsList,double * StartPos,double * TargetPos,WORD Arc_Dir,int Circle,WORD mode,int mark);
DMC_API short __stdcall dmc_helix_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double * StartPos,double *TargetPos,WORD Arc_Dir,int Circle,WORD mode);

//PDO缓存20190715（内部使用）
DMC_API short __stdcall dmc_pdo_buffer_enter(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_pdo_buffer_stop(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_pdo_buffer_clear(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_pdo_buffer_run_state(WORD CardNo,WORD axis,int * RunState,int * Remain,int * NotRunned,int* Runned);
DMC_API short __stdcall dmc_pdo_buffer_add_data(WORD CardNo,WORD axis, int size, int* data_table);
DMC_API short __stdcall dmc_pdo_buffer_start_multi(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* ResultList);
DMC_API short __stdcall dmc_pdo_buffer_pause_multi(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* ResultList);
DMC_API short __stdcall dmc_pdo_buffer_stop_multi(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* ResultList);
DMC_API short __stdcall dmc_pdo_buffer_add_data_multi(WORD CardNo, WORD AxisNum,WORD* AxisList, int size, int** data_table);
//保留
DMC_API short __stdcall dmc_calculate_arccenter_3point(double *start_pos,double *mid_pos, double *target_pos,double* cen_pos);
//内部指令（内部使用）
DMC_API short __stdcall	dmc_cmd_buf_open(WORD card,WORD group, WORD axis_num, WORD* axis_list);
DMC_API short __stdcall	dmc_cmd_buf_close(WORD card,WORD group);
DMC_API short __stdcall	dmc_cmd_buf_start(WORD card,WORD group);
DMC_API short __stdcall	dmc_cmd_buf_stop(WORD card,WORD group, WORD stop_mode);
DMC_API short __stdcall	dmc_cmd_buf_temp_stop(WORD CardNo,WORD group,WORD stop_mode);
DMC_API short __stdcall dmc_cmd_buf_set_allow_error(WORD CardNo, WORD group, double allow_error);
DMC_API short __stdcall dmc_cmd_buf_get_allow_error(WORD CardNo, WORD group, double* allow_error);
DMC_API short __stdcall	dmc_cmd_buf_get_group_config(WORD card,WORD group, WORD* enable ,WORD* axis_num, WORD* axis_list);
DMC_API short __stdcall	dmc_cmd_buf_get_group_run_info(WORD card,WORD group, WORD* enable ,DWORD* array_id, DWORD* stop_reason,WORD* trig_phase ,WORD* phase);
DMC_API short __stdcall	dmc_cmd_buf_add_cmd(WORD card, WORD group, DWORD index, WORD  cmd_type, WORD  ProcessMode, WORD Dim,
	WORD* AxisList, double* TargetPositionFirst, double* m_TargetPositionSecond,WORD*  m_SoftlandFlag,double* SoftLandVel,double* SoftLandEndVel, WORD*  m_PosMode,double* m_TrigAheadPos,
	WORD  m_TrigFlag,  WORD m_TrigAxisNum,WORD* m_TrigAxislist,WORD*  m_TrigPosType,WORD*  m_TrigAxisPosRelationgship,double* m_TrigAxisPos,
	WORD m_TrigIONum, WORD*  m_TrigIOState,WORD* m_TrigINIOList,
	DWORD m_DelayCmdTime, WORD m_IOList, WORD  m_IOState,WORD m_TrigError
	);
DMC_API short __stdcall	dmc_cmd_buf_set_axis_profile(WORD card,WORD group, WORD axis_num, WORD* axis_list, double* start_vel, double* max_vel, double* stop_vel, double* tacc, double* tdec);
/**************************指令缓存门型运动************************/
//门型运动指令（适用于DMC3000/5000系列脉冲卡）
DMC_API short __stdcall dmc_m_set_muti_profile_unit(WORD CardNo, WORD group, WORD axis_num, WORD* axis_list, double* start_vel, double* max_vel, double* tacc, double* tdec, double* stop_vel);
DMC_API short __stdcall dmc_m_set_profile_unit(WORD CardNo, WORD group, WORD axis, double start_vel, double max_vel, double tacc, double tdec, double stop_vel);
DMC_API short __stdcall dmc_m_add_sigaxis_moveseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, WORD process_mode, DWORD mark);
DMC_API short __stdcall dmc_m_add_sigaxis_move_twoseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD process_mode, DWORD mark);
DMC_API short __stdcall dmc_m_add_mutiaxis_moveseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axis_list, double* Target_pos, WORD process_mode, DWORD mark);
DMC_API short __stdcall dmc_m_add_mutiaxis_move_twoseg_data(WORD CardNo, WORD group, WORD axisnum,  WORD* axis_list, double* Target_pos, double* softland_pos, double* softland_vel, double* softland_endvel, WORD process_mode, DWORD mark);
DMC_API short __stdcall dmc_m_add_ioTrig_movseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, WORD process_mode, WORD trigINbit, WORD trigINstate, DWORD mark);//io触发移动
DMC_API short __stdcall dmc_m_add_mutiposTrig_movseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, WORD process_mode, WORD trigaxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigPosMode, DWORD mark);//位置触发移动
DMC_API short __stdcall dmc_m_add_mutiposTrig_mov_twoseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD process_mode, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigPosMode, DWORD mark);//多轴位置触发移动
DMC_API short __stdcall dmc_m_add_upseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, DWORD mark);
DMC_API short __stdcall dmc_m_add_up_twoseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, DWORD mark);
DMC_API short __stdcall dmc_m_add_ioPosTrig_movseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, WORD process_mode, WORD trigAxis,double trigPos, WORD trigPosType, WORD trigMode, WORD TrigINNum, WORD* trigINList, WORD* trigINstate, DWORD mark);//位置+io触发移动
DMC_API short __stdcall dmc_m_add_ioPosTrig_mov_twoseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, double* softland_pos, double* softland_vel, double* softland_endvel,WORD process_mode, WORD trigAxis, double trigPos, WORD trigPosType, WORD trigMode, WORD TrigINNum, WORD* trigINList, WORD* trigINstate, DWORD mark);//位置+io触发移动
DMC_API short __stdcall dmc_m_add_posTrig_movseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, WORD process_mode, WORD trigAxis, double trigPos, WORD trigPosType, WORD trigMode, DWORD mark);//位置触发移动
DMC_API short __stdcall dmc_m_add_posTrig_mov_twoseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, double* softland_pos, double* softland_vel, double* softland_endvel, WORD process_mode, WORD trigAxis, double trigPos, WORD trigPosType, WORD trigMode, DWORD mark);//位置触发移动
DMC_API short __stdcall dmc_m_add_ioPosTrig_down_seg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, WORD trigIN, WORD trigINstate, DWORD mark);
DMC_API short __stdcall dmc_m_add_ioPosTrig_down_twoseg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, WORD trigIN, WORD trigINstate, DWORD mark);
DMC_API short __stdcall dmc_m_add_posTrig_down_seg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);
DMC_API short __stdcall dmc_m_add_posTrig_down_twoseg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);
DMC_API short __stdcall dmc_m_posTrig_outbit(WORD CardNo, WORD group, WORD bitno, WORD on_off, WORD ahead_axis, double ahead_value, WORD ahead_PosType, WORD ahead_Mode, DWORD mark);
DMC_API short __stdcall dmc_m_immediate_write_outbit(WORD CardNo, WORD group, WORD bitno, WORD on_off, DWORD mark);
DMC_API short __stdcall dmc_m_wait_input(WORD CardNo, WORD group, WORD bitno, WORD on_off, double time_out, DWORD mark);
DMC_API short __stdcall dmc_m_delay_time(WORD CardNo, WORD group, double delay_time, DWORD mark);
DMC_API short __stdcall dmc_m_get_run_state(WORD CardNo, WORD group, WORD* state, WORD* enable, DWORD* stop_reason, WORD* trig_phase, DWORD* mark);
DMC_API short __stdcall dmc_m_open_list(WORD CardNo, WORD group, WORD axis_num, WORD* axis_list);
DMC_API short __stdcall dmc_m_close_list(WORD CardNo, WORD group);
DMC_API short __stdcall dmc_m_start_list(WORD CardNo, WORD group);
DMC_API short __stdcall dmc_m_stop_list(WORD CardNo, WORD group,WORD stopMode);
DMC_API short __stdcall dmc_m_add_posTrig_down_seg_cmd_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, WORD trigAxisNum, WORD* trigAxisList, DWORD mark);
DMC_API short __stdcall dmc_m_add_posTrig_down_twoseg_cmd_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD trigAxisNum, WORD* trigAxisList, DWORD mark);
DMC_API short __stdcall dmc_m_pause_list(WORD CardNo, WORD group, WORD stop_mode);
//反馈位置误差允许范围设置，门型使用
DMC_API short __stdcall dmc_m_set_encoder_error_allow(WORD CardNo, WORD group, double allow_error);
DMC_API short __stdcall dmc_m_get_encoder_error_allow(WORD CardNo, WORD group, double* allow_error);

//读取所有AD输入（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_get_ad_input_all(WORD CardNo, double* Vout);
//连续插补暂停后使用pmove（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_conti_pmove_unit_pausemode(WORD CardNo, WORD axis,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,double smooth_time,WORD posi_mode);
//连续插补暂停使用pmove后，回到暂停位置（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_conti_return_pausemode(WORD CardNo, WORD Crd, WORD axis);
//检验接线盒是否支持通讯校验（适用于DMC3000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_check_if_crc_support(WORD CardNo);

//两轴碰撞检测功能接口（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_set_axis_conflict_config(WORD CardNo, WORD*  axis_list, WORD* axis_depart_dir, double home_dist, double conflict_dist, WORD stop_mode); 
DMC_API short __stdcall dmc_get_axis_conflict_config (WORD CardNo, WORD*  axis_list, WORD* axis_depart_dir, double* home_dist, double* conflict_dist, WORD* stop_mode);
DMC_API short __stdcall dmc_axis_conflict_config_en(WORD CardNo, WORD enable);
DMC_API short __stdcall dmc_get_axis_conflict_config_en(WORD CardNo, WORD* enable);

//物件分拣加通道,分拣固件专用
DMC_API short __stdcall dmc_sorting_close_ex(WORD CardNo,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_start_ex(WORD CardNo,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_set_init_config_ex(WORD CardNo ,WORD cameraCount, int *pCameraPos, WORD *pCamIONo, DWORD cameraTime, WORD cameraTrigLevel, WORD blowCount, int*pBlowPos, WORD*pBlowIONo, DWORD blowTime, WORD blowTrigLevel, WORD axis, WORD dir, WORD checkMode,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_set_camera_trig_count_ex(WORD CardNo ,WORD cameraNum, DWORD cameraTrigCnt,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_get_camera_trig_count_ex(WORD CardNo ,WORD cameraNum, DWORD* pCameraTrigCnt, WORD count,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_set_blow_trig_count_ex(WORD CardNo ,WORD blowNum, DWORD blowTrigCnt,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_get_blow_trig_count_ex(WORD CardNo ,WORD blowNum, DWORD* pBlowTrigCnt, WORD count,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_get_camera_config_ex(WORD CardNo ,WORD index,int* pos,DWORD* trigTime, WORD* ioNo, WORD* trigLevel,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_get_blow_config_ex(WORD CardNo ,WORD index, int* pos,DWORD* trigTime, WORD* ioNo, WORD* trigLevel,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_get_blow_status_ex(WORD CardNo ,DWORD* trigCntAll, WORD* trigMore,WORD* trigLess,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_trig_blow_ex(WORD CardNo ,WORD blowNum,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_set_blow_enable_ex(WORD CardNo ,WORD blowNum,WORD enable,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_set_piece_config_ex(WORD CardNo ,DWORD maxWidth,DWORD minWidth,DWORD minDistance, DWORD minTimeIntervel,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_get_piece_status_ex(WORD CardNo ,DWORD* pieceFind,DWORD* piecePassCam, DWORD* dist2next, DWORD*pieceWidth,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_set_cam_trig_phase_ex(WORD CardNo,WORD blowNo,double coef,WORD sortModuleNo);
DMC_API short __stdcall dmc_sorting_set_blow_trig_phase_ex(WORD CardNo,WORD blowNo,double coef,WORD sortModuleNo);
//获取分拣指令数量函数
DMC_API short __stdcall dmc_get_sortdev_blow_cmd_cnt(WORD CardNo, WORD blowDevNum, long* cnt);
//获取未处理指令数量函数函数
DMC_API short __stdcall dmc_get_sortdev_blow_cmderr_cnt(WORD CardNo, WORD blowDevNum, long* errCnt);
//分拣队列状态
DMC_API short __stdcall dmc_get_sortqueue_status(WORD CardNo, long * curSorQueueLen, long* passCamWithNoCmd);

// 椭圆连续插补（适用于DMC5X10系列脉冲卡、E5032总线卡）
DMC_API short __stdcall dmc_conti_ellipse_move_unit(WORD CardNo, WORD Crd,WORD AxisNum, WORD* AxisList, double* Target_Pos, double* Cen_Pos, double A_Axis_Len, double B_Axis_Len, WORD Dir, WORD Pos_Mode,long mark);
//获取轴状态函数（预留）
DMC_API	short  __stdcall dmc_get_axis_status_advance(WORD CardNo, WORD axis_no, WORD motion_no, WORD* axis_plan_state, DWORD* ErrPlulseCnt, WORD* fpga_busy);
//连续插补vmove（DMC5000系列卡受限使用）
DMC_API	short __stdcall dmc_conti_vmove_unit(WORD CardNo, WORD Crd, WORD axis, double vel, double acc ,WORD dir, long imark);
DMC_API	short __stdcall dmc_conti_vmove_stop(WORD CardNo, WORD Crd, WORD axis, double dec, long imark);

/**************************读写掉电保持区************************/
//写入字符数据到断电保持区（DMC5000系列卡受限使用）
DMC_API short __stdcall dmc_set_persistent_reg_byte(WORD CardNo, DWORD start, DWORD inum, const char* pdata);
//从断电保持区读取写入的字符（DMC5000系列卡受限使用）
DMC_API short __stdcall dmc_get_persistent_reg_byte(WORD CardNo, DWORD start, DWORD inum, char* pdata);
//写入浮点型数据到断电保持区（DMC5000系列卡受限使用）
DMC_API short __stdcall dmc_set_persistent_reg_float(WORD CardNo, DWORD start, DWORD inum, const float* pdata);
//从断电保持区读取写入的浮点型数据（DMC5000系列卡受限使用）
DMC_API short __stdcall dmc_get_persistent_reg_float(WORD CardNo, DWORD start, DWORD inum, float* pdata);
//写入整型数据到断电保持区（DMC5000系列卡受限使用）
DMC_API short __stdcall dmc_set_persistent_reg_int(WORD CardNo, DWORD start, DWORD inum, const int* pdata);
//从断电保持区读取写入的整型数据（DMC5000系列卡受限使用）
DMC_API short __stdcall dmc_get_persistent_reg_int(WORD CardNo, DWORD start, DWORD inum, int* pdata);

//EtherCAT总线复位IO模块输出保持开关设置（适用于所有EtherCAT总线卡）
DMC_API short __stdcall nmc_set_slave_output_retain(WORD CardNo,WORD Enable);
DMC_API short __stdcall nmc_get_slave_output_retain(WORD CardNo,WORD * Enable);

DMC_API short __stdcall dmc_m_add_mutiposTrig_singledown_seg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, WORD process_mode, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);
DMC_API short __stdcall dmc_m_add_mutiposTrig_mutidown_seg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axis_list, double* safePos, double* Target_pos, WORD process_mode, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);

//轴参数配置写flash，实现断电保存急停信号配置（适用于DMC3000系列脉冲卡）
DMC_API short __stdcall dmc_set_persistent_param_config(WORD CardNo, WORD axis, DWORD item);
DMC_API short __stdcall dmc_get_persistent_param_config(WORD CardNo, WORD axis, DWORD* item);

//一维高速比较，添加的比较点关联运动方向，添加少量数据（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_hcmp_fifo_add_point_dir_unit(WORD CardNo,WORD hcmp, WORD num,double *cmp_pos,DWORD dir);
//一维高速比较，添加的比较点关联运动方向，添加大量数据（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_hcmp_fifo_add_table_dir(WORD CardNo,WORD hcmp, WORD num,double *cmp_pos,DWORD dir);

//读取指定轴有关运动信号的状态（适用于所有卡）
DMC_API short __stdcall dmc_axis_io_status_ex(WORD CardNo,WORD axis,DWORD *state);
//读取指定轴的运动状态（适用于所有卡）
DMC_API short __stdcall dmc_check_done_ex(WORD CardNo,WORD axis,WORD * state);

//读取运行时是启动正常固件还是备份固件（适用于DMC3000/5000/5X10系列脉冲卡）
DMC_API short __stdcall dmc_get_firmware_boot_type(WORD CardNo,WORD* boot_type);

/**************************中断功能 （适用于DMC5X10系列脉冲卡）************************/
//开启控制卡中断功能
DMC_API DWORD __stdcall dmc_int_enable(WORD CardNo, DMC3K5K_OPERATE funcIntHandler, PVOID operate_data);
//禁止控制卡的中断
DMC_API DWORD __stdcall dmc_int_disable(WORD CardNo);
//设置/读取指定控制卡中断通道使能
DMC_API short __stdcall dmc_set_intmode_enable(WORD Cardno,WORD Intno,WORD Enable);
DMC_API short __stdcall dmc_get_intmode_enable(WORD Cardno,WORD Intno,WORD *Status);
//设置/读取指定控制卡中断配置
DMC_API short __stdcall dmc_set_intmode_config(WORD Cardno,WORD Intno,WORD IntItem,WORD IntIndex,WORD IntSubIndex,WORD Logic);
DMC_API	short __stdcall dmc_get_intmode_config(WORD Cardno,WORD Intno,WORD *IntItem,WORD *IntIndex,WORD *IntSubIndex,WORD*Logic);
//读取指定控制卡中断通道的中断状态
DMC_API short __stdcall dmc_get_int_status(WORD Cardno,DWORD *IntStatus);
//复位指定控制卡输入口的中断
DMC_API short __stdcall dmc_reset_int_status(WORD Cardno, WORD Intno);
/**************************************************************************************/
//IO变速变位，配置io输入口（适用于EtherCAT总线系列卡）
DMC_API short __stdcall dmc_pmove_change_pos_speed_inbit(WORD CardNo,WORD axis, WORD inbit, WORD enable);
DMC_API short __stdcall dmc_get_pmove_change_pos_speed_inbit(WORD CardNo,WORD axis, WORD * inbit, WORD* enable);
//圆形区域软限位unit（适用于DMC5X10系列脉冲卡）
DMC_API short __stdcall dmc_set_arc_zone_limit_config_unit(WORD CardNo, WORD* AxisList, WORD AxisNum, double *Center, double Radius,WORD Source, WORD StopMode);
DMC_API short __stdcall dmc_get_arc_zone_limit_config_unit(WORD CardNo, WORD* AxisList, WORD* AxisNum, double *Center, double* Radius,WORD* Source, WORD* StopMode);
//设置/回读LTC端口触发延时急停轴配置（适用于EtherCAT总线系列卡）
DMC_API short __stdcall dmc_set_latch_stop_axis(WORD CardNo, WORD latch, WORD  num,  WORD * axislist);
DMC_API short __stdcall dmc_get_latch_stop_axis(WORD CardNo, WORD latch, WORD * num,  WORD * axislist);
//添加二维低速位置比较点（适用于EtherCAT总线系列卡）
DMC_API short __stdcall dmc_compare_add_point_cycle_2d(WORD CardNo,WORD* axis,double* pos,WORD* dir, DWORD bitno,DWORD cycle,WORD level);
//设置/回读位置误差带unit（适用于DMC5X10脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_set_factor_error_unit(WORD CardNo,WORD axis,double factor,double error);
DMC_API short __stdcall dmc_get_factor_error_unit(WORD CardNo,WORD axis,double* factor,double* error);
//设置/回读脉冲计数值和编码器反馈值之间差值的报警阈值unit（适用于DMC5X10脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_set_pulse_encoder_count_error_unit(WORD CardNo,WORD axis,double error);
DMC_API short __stdcall dmc_get_pulse_encoder_count_error_unit(WORD CardNo,WORD axis,double *error);
//检查脉冲计数值和编码器反馈值之间差值是否超过报警阈值unit（适用于DMC5X10脉冲卡、EtherCAT总线卡）
DMC_API short __stdcall dmc_check_pulse_encoder_count_error_unit(WORD CardNo,WORD axis,double* pulse_position, double* enc_position);
#ifdef __cplusplus
}
#endif

#endif