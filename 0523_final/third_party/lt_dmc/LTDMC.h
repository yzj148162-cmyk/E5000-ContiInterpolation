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

typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	?????8¦Ë???????  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		?§Ù???8¦Ë???????  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	?????16¦Ë??????? */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		?§Ù???16¦Ë??????? */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	?????32¦Ë??????? */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		?§Ù???32¦Ë??????? */
typedef unsigned long long   uint64;
typedef signed   long long   int64;

typedef uint32 (__stdcall * DMC3K5K_OPERATE)(void* operator_data);

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

typedef struct{
    double start_pos;   //???????????¦Ë??.
    double interval;    //????.
    int count;//????
} struct_hs_cmp_info;


#define __DMC_EXPORTS

//??????????????
#ifdef __DMC_EXPORTS
    #define DMC_API __declspec(dllexport)
#else
    #define DMC_API __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C" {
#endif

//???¨²????????
DMC_API short __stdcall dmc_set_debug_mode(WORD mode,const char* pFileName);
DMC_API short __stdcall dmc_get_debug_mode(WORD* mode,char* pFileName);

DMC_API short __stdcall dmc_board_init(void); 	//??????????
DMC_API short __stdcall dmc_board_init_onecard(WORD CardNo); 	////?????????????? ????????????
DMC_API short __stdcall dmc_board_close_onecard(WORD CardNo); 	////????????????? ???????????
DMC_API short __stdcall	dmc_get_CardInfList(WORD* CardNum,DWORD* CardTypeList,WORD* CardIdList);//????????????§Ò?
DMC_API short __stdcall dmc_board_close(void);	//???????
DMC_API short __stdcall dmc_board_reset(void);   //?????¦Ë
DMC_API short __stdcall dmc_board_reset_onecard(WORD CardNo);//???????????????¦Ë
DMC_API short __stdcall dmc_soft_reset(WORD CardNo);//?????????¦Ë??pci???? ???¦Ë?????????
DMC_API short __stdcall dmc_cool_reset(WORD CardNo);//??????ÁÙ¦Ë
DMC_API short __stdcall dmc_original_reset(WORD CardNo);//??????????¦Ë
DMC_API short __stdcall dmc_get_card_ID (WORD CardNo,DWORD *CardID);	//????????????
DMC_API short __stdcall dmc_get_release_version(WORD CardNo,char *ReleaseVersion);//????????·Ú??
DMC_API short __stdcall dmc_get_card_version(WORD CardNo,DWORD *CardVersion);	//????????????·Ú
DMC_API short __stdcall dmc_get_card_soft_version(WORD CardNo,DWORD *FirmID,DWORD *SubFirmID);	//????????????????·Ú
DMC_API short __stdcall dmc_get_card_lib_version(DWORD *LibVer);	//??????????????·Ú
DMC_API short __stdcall dmc_get_total_axes(WORD CardNo,DWORD *TotalAxis); 	//????????????
DMC_API short __stdcall dmc_get_total_liners(WORD CardNo,DWORD *TotalLiner); //?????????????????

DMC_API short __stdcall dmc_get_total_ionum(WORD CardNo,WORD *TotalIn,WORD *TotalOut);//???????IO????
DMC_API short __stdcall dmc_get_total_adcnum(WORD CardNo,WORD* TotalIn,WORD* TotalOut);//???????ADDA??????????

//??????
DMC_API short __stdcall dmc_check_sn(WORD CardNo, const char * str_sn);
DMC_API short __stdcall dmc_write_sn(WORD CardNo, const char * str_sn);

/***********??????*************/
//??????
DMC_API short __stdcall dmc_set_pulse_outmode(WORD CardNo,WORD axis,WORD outmode);
DMC_API short __stdcall dmc_get_pulse_outmode(WORD CardNo,WORD axis,WORD* outmode);
//??????
DMC_API short __stdcall dmc_set_equiv(WORD CardNo,WORD axis, double equiv);
DMC_API short __stdcall dmc_get_equiv(WORD CardNo,WORD axis, double *equiv);
//???????(????)
DMC_API short __stdcall dmc_set_backlash_unit(WORD CardNo,WORD axis,double backlash);
DMC_API short __stdcall dmc_get_backlash_unit(WORD CardNo,WORD axis,double *backlash);
//???????(????)
DMC_API short __stdcall dmc_set_backlash(WORD CardNo,WORD axis,long backlash);
DMC_API short __stdcall dmc_get_backlash(WORD CardNo,WORD axis,long *backlash);

//???????
DMC_API short __stdcall dmc_set_profile_unit_acc(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_profile_unit_acc(WORD CardNo,WORD axis,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
DMC_API short __stdcall dmc_set_vector_profile_multicoor_acc(WORD CardNo,WORD iDMC_ILINER, double Min_Vel,double Max_Vel,double Acc,double Dec,double Stop_Vel);
DMC_API short __stdcall dmc_get_vector_profile_multicoor_acc(WORD CardNo,WORD iDMC_ILINER, double* Min_Vel,double* Max_Vel,double* Acc,double* Dec,double* Stop_Vel);
DMC_API short __stdcall dmc_set_vector_profile_unit_acc(WORD CardNo,WORD Crd,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_vector_profile_unit_acc(WORD CardNo,WORD Crd,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
DMC_API short __stdcall dmc_change_speed_unit_acc(WORD CardNo,WORD axis, double New_Vel,double Taccdec);
DMC_API short __stdcall dmc_change_speed_acc(WORD CardNo,WORD axis,double Curr_Vel,double Taccdec);
DMC_API short __stdcall dmc_pmove_extern_acc(WORD CardNo, WORD axis, double dist,double Min_Vel, double Max_Vel, double Tacc, double Tdec, double stop_Vel, double s_para, WORD posi_mode);
DMC_API short __stdcall nmc_set_home_profile_acc(WORD CardNo ,WORD axis,WORD home_mode,double Low_Vel, double High_Vel,double Tacc,double Tdec,double offsetpos );
DMC_API short __stdcall nmc_get_home_profile_acc(WORD CardNo,WORD axis,WORD* home_mode,double* Low_Vel,double* High_Vel,double* Tacc,double* Tdec,double* Offsetpos);
DMC_API short __stdcall dmc_set_home_profile_unit_acc(WORD CardNo,WORD axis,double Low_Vel,double High_Vel,double Tacc,double Tdec);//?????????????
DMC_API short __stdcall dmc_get_home_profile_unit_acc(WORD CardNo,WORD axis,double* Low_Vel,double* High_Vel,double* Tacc,double* Tdec);//?????????????
DMC_API short __stdcall dmc_t_pmove_extern_softstart_acc(WORD CardNo, WORD axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, DWORD delay_ms, double Max_Vel2,double stop_vel2, double acc_time, double dec_time, WORD posi_mode);
/***********************************???????**************************************/
/*********************************************************************************************************
??????????? ?????
filetype
0-basic
1-gcode
2-setting
3-firewave
4-CAN configfile
100-trace data
*********************************************************************************************************/
DMC_API short __stdcall dmc_download_file(WORD CardNo, const char* pfilename, const char* pfilenameinControl,WORD filetype);
//?????????? ?????
DMC_API short __stdcall dmc_download_memfile(WORD CardNo, const char* pbuffer, uint32 buffsize, const char* pfilenameinControl,WORD filetype);
//??????
DMC_API short __stdcall dmc_upload_file(WORD CardNo, const char* pfilename, const char* pfilenameinControl, WORD filetype);
//?????????
DMC_API short __stdcall dmc_upload_memfile(WORD CardNo, char* pbuffer, uint32 buffsize, const char* pfilenameinControl, uint32* puifilesize,WORD filetype);
//??????????
DMC_API short __stdcall dmc_download_configfile(WORD CardNo,const char *FileName);
//?????????
DMC_API short __stdcall dmc_download_firmware(WORD CardNo,const char *FileName);
//???????
DMC_API short __stdcall dmc_get_progress(WORD CardNo,float* process);

//???????
DMC_API short __stdcall dmc_set_softlimit(WORD CardNo,WORD axis,WORD enable, WORD source_sel,WORD SL_action, long N_limit,long P_limit);//????????¦Ë????
DMC_API short __stdcall dmc_get_softlimit(WORD CardNo,WORD axis,WORD *enable, WORD *source_sel,WORD *SL_action,long *N_limit,long *P_limit);//???????¦Ë????
DMC_API short __stdcall dmc_set_el_mode(WORD CardNo,WORD axis,WORD enable,WORD el_logic,WORD el_mode);//????EL???
DMC_API short __stdcall dmc_get_el_mode(WORD CardNo,WORD axis,WORD *enable,WORD *el_logic,WORD *el_mode);//???????EL???
DMC_API short __stdcall dmc_set_emg_mode(WORD CardNo,WORD axis,WORD enable,WORD emg_logic);//????EMG???
DMC_API short __stdcall dmc_get_emg_mode(WORD CardNo,WORD axis,WORD *enable,WORD *emg_logic);//???????EMG???

/*************************************???????*****************************************/
//???????
DMC_API short __stdcall dmc_set_profile(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double stop_vel);//????????????
DMC_API short __stdcall dmc_get_profile(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Tacc,double *Tdec,double *stop_vel);//?????????????
//???????(??????)
DMC_API short __stdcall dmc_set_profile_unit(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_profile_unit(WORD CardNo,WORD axis,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);

//20160105??????????????????? ????? ????????????(????)
DMC_API short __stdcall dmc_set_profile_extern(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Acc,double Dec,double Ajerk,double Djerk,double stop_vel);
DMC_API short __stdcall dmc_get_profile_extern(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Acc,double *Dec,double *Ajerk,double *Djerk,double *stop_vel);
//?????????????????????(????)
DMC_API short __stdcall dmc_set_acc_profile(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Acc,double Dec,double stop_vel);//????????????
DMC_API short __stdcall dmc_get_acc_profile(WORD CardNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Acc,double *Dec,double *stop_vel);//?????????????
//?????????????????????(????)
DMC_API short __stdcall dmc_set_profile_unit_acc(WORD CardNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_profile_unit_acc(WORD CardNo,WORD axis,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
DMC_API short __stdcall dmc_set_s_profile(WORD CardNo,WORD axis,WORD s_mode,double s_para);//?????????????????
DMC_API short __stdcall dmc_get_s_profile(WORD CardNo,WORD axis,WORD s_mode,double *s_para);//???????????????? ????DMC5800 s_mode?????????????

//??¦Ë???(????)
DMC_API short __stdcall dmc_pmove(WORD CardNo,WORD axis,long dist,WORD posi_mode);//???????????¦Ë?????
//??¦Ë???(????)
DMC_API short __stdcall dmc_pmove_unit(WORD CardNo,WORD axis,double Dist,WORD posi_mode);
//??????????????
DMC_API short __stdcall dmc_vmove(WORD CardNo,WORD axis,WORD dir);
//???????????¦Ë????? ??????????S???(????)
DMC_API short __stdcall dmc_pmove_extern(WORD CardNo, WORD axis, double dist,double Min_Vel, double Max_Vel, double Tacc, double Tdec, double stop_Vel, double s_para, WORD posi_mode);
//?????¦Ë/????(????)
DMC_API short __stdcall dmc_reset_target_position(WORD CardNo,WORD axis,long dist,WORD posi_mode);//????§Ú?????¦Ë??
DMC_API short __stdcall dmc_change_speed(WORD CardNo,WORD axis,double Curr_Vel,double Taccdec);//?????????????????????
DMC_API short __stdcall dmc_update_target_position(WORD CardNo,WORD axis,long dist,WORD posi_mode);//?????????????§Ú?????¦Ë??
//?????¦Ë(????)
DMC_API short __stdcall dmc_reset_target_position_unit(WORD CardNo,WORD axis, double New_Pos);
DMC_API short __stdcall dmc_change_speed_unit(WORD CardNo,WORD axis, double New_Vel,double Taccdec);
DMC_API short __stdcall dmc_update_target_position_unit(WORD CardNo,WORD axis, double New_Pos);

/******************************?????**********************************/
//3000??????????(????)
DMC_API short __stdcall dmc_set_vector_profile_multicoor(WORD CardNo,WORD Crd, double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_vector_profile_multicoor(WORD CardNo,WORD Crd, double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
DMC_API short __stdcall dmc_set_vector_s_profile_multicoor(WORD CardNo,WORD Crd,WORD s_mode,double s_para);//?????????????????
DMC_API short __stdcall dmc_get_vector_s_profile_multicoor(WORD CardNo,WORD Crd,WORD s_mode,double *s_para);//????????????????

//????????(????)
DMC_API short __stdcall dmc_set_vector_profile_unit(WORD CardNo,WORD Crd,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_get_vector_profile_unit(WORD CardNo,WORD Crd,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
DMC_API short __stdcall dmc_set_vector_s_profile(WORD CardNo,WORD Crd,WORD s_mode,double s_para);//?????????????????
DMC_API short __stdcall dmc_get_vector_s_profile(WORD CardNo,WORD Crd,WORD s_mode,double *s_para);

//3000??§Ó?????(????)
DMC_API short __stdcall dmc_line_multicoor(WORD CardNo,WORD Crd,WORD axisNum,WORD *axisList,long *DistList,WORD posi_mode);	//????????????
DMC_API short __stdcall dmc_arc_move_multicoor(WORD CardNo,WORD Crd,WORD *AxisList,long *Target_Pos,long *Cen_Pos,WORD Arc_Dir,WORD posi_mode);//????????
//???¦Â?(????)
DMC_API short __stdcall dmc_line_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode);
DMC_API short __stdcall dmc_arc_move_center_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_arc_move_radius_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_arc_move_3points_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_rectangle_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mark_Pos,long num,WORD rect_mode,WORD posi_mode);

/********************PVT???****************************/
//PVT??? ???
DMC_API short __stdcall dmc_PvtTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pVel);
DMC_API short __stdcall dmc_PtsTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pPercent);
DMC_API short __stdcall dmc_PvtsTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double velBegin,double velEnd);
DMC_API short __stdcall dmc_PttTable(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos);
DMC_API short __stdcall dmc_PvtMove(WORD CardNo,WORD AxisNum,WORD* AxisList);
//PVT??????????
DMC_API short __stdcall dmc_PttTable_add(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos);
DMC_API short __stdcall dmc_PtsTable_add(WORD CardNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pPercent);
DMC_API short __stdcall dmc_pvt_get_remain_space(WORD CardNo,WORD iaxis);//???pvt??????
/*****************************************************************************
PVT??? ????????
******************************************************************************/
DMC_API short __stdcall dmc_pvt_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double *pVel);
DMC_API short __stdcall dmc_pts_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double *pPercent);
DMC_API short __stdcall dmc_pvts_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double velBegin,double velEnd);
DMC_API short __stdcall dmc_ptt_table_unit(WORD CardNo,WORD iaxis,DWORD count,double *pTime,double *pPos);
DMC_API short __stdcall dmc_pvt_move(WORD CardNo,WORD AxisNum,WORD* AxisList);

DMC_API short __stdcall dmc_SetGearProfile(WORD CardNo,WORD axis,WORD MasterType, WORD MasterIndex,long MasterEven,long SlaveEven,DWORD MasterSlope);
DMC_API short __stdcall dmc_GetGearProfile(WORD CardNo,WORD axis,WORD* MasterType, WORD* MasterIndex,long* MasterEven,long* SlaveEven,DWORD* MasterSlope);
DMC_API short __stdcall dmc_GearMove(WORD CardNo,WORD AxisNum,WORD* AxisList);

/************************???????*************************/
DMC_API short __stdcall dmc_set_home_pin_logic(WORD CardNo,WORD axis,WORD org_logic,double filter);//????HOME???
DMC_API short __stdcall dmc_get_home_pin_logic(WORD CardNo,WORD axis,WORD *org_logic,double *filter);//???????HOME???
DMC_API short __stdcall dmc_set_homemode(WORD CardNo,WORD axis,WORD home_dir,double vel,WORD mode,WORD EZ_count);//???????????????
DMC_API short __stdcall dmc_get_homemode(WORD CardNo,WORD axis,WORD *home_dir, double *vel_mode,WORD *home_mode,WORD *EZ_count);//????????????????
DMC_API short __stdcall dmc_home_move(WORD CardNo,WORD axis);//????????
DMC_API short __stdcall dmc_set_home_profile_unit(WORD CardNo,WORD axis,double Low_Vel,double High_Vel,double Tacc,double Tdec);//?????????????
DMC_API short __stdcall dmc_get_home_profile_unit(WORD CardNo,WORD axis,double* Low_Vel,double* High_Vel,double* Tacc,double* Tdec);//?????????????
DMC_API short __stdcall dmc_get_home_result(WORD CardNo,WORD axis,WORD* state);//????????????
DMC_API short __stdcall dmc_set_home_position_unit(WORD CardNo,WORD axis,WORD enable,double position);
DMC_API short __stdcall dmc_get_home_position_unit(WORD CardNo,WORD axis,WORD *enable,double *position);
DMC_API short __stdcall dmc_set_el_home(WORD CardNo,WORD axis,WORD mode);

/***************************???????******************************/
DMC_API short __stdcall dmc_set_homelatch_mode(WORD CardNo,WORD axis,WORD enable,WORD logic,WORD source);
DMC_API short __stdcall dmc_get_homelatch_mode(WORD CardNo,WORD axis,WORD* enable,WORD* logic,WORD* source);
DMC_API long __stdcall dmc_get_homelatch_flag(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_reset_homelatch_flag(WORD CardNo,WORD axis);
DMC_API long __stdcall dmc_get_homelatch_value(WORD CardNo,WORD axis);
/*****************************EZ????********************************/
DMC_API short __stdcall dmc_set_ezlatch_mode(WORD CardNo,WORD axis,WORD enable,WORD logic,WORD source);
DMC_API short __stdcall dmc_get_ezlatch_mode(WORD CardNo,WORD axis,WORD* enable,WORD* logic,WORD* source);
DMC_API long __stdcall dmc_get_ezlatch_flag(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_reset_ezlatch_flag(WORD CardNo,WORD axis);
DMC_API long __stdcall dmc_get_ezlatch_value(WORD CardNo,WORD axis);

/************************???????????*********************************/
//????
DMC_API short __stdcall dmc_set_handwheel_inmode(WORD CardNo,WORD axis,WORD inmode,long multi,double vh);//??????????????????????????
DMC_API short __stdcall dmc_get_handwheel_inmode(WORD CardNo,WORD axis,WORD *inmode,long *multi,double *vh);//?????????????????????????
//?????????????
DMC_API short __stdcall dmc_set_handwheel_inmode_decimals(WORD CardNo,WORD axis,WORD inmode,double multi,double vh);
DMC_API short __stdcall dmc_get_handwheel_inmode_decimals(WORD CardNo,WORD axis,WORD *inmode,double *multi,double *vh);
DMC_API short __stdcall dmc_handwheel_move(WORD CardNo,WORD axis);
//???????????
DMC_API short __stdcall dmc_set_handwheel_channel(WORD CardNo,WORD index);
DMC_API short __stdcall dmc_get_handwheel_channel(WORD CardNo,WORD* index);
//????
DMC_API short __stdcall dmc_set_handwheel_inmode_extern(WORD CardNo,WORD inmode,WORD AxisNum,WORD* AxisList,long* multi);
DMC_API short __stdcall dmc_get_handwheel_inmode_extern(WORD CardNo,WORD* inmode,WORD* AxisNum,WORD* AxisList,long *multi);
//???????????
DMC_API short __stdcall dmc_set_handwheel_inmode_extern_decimals(WORD CardNo,WORD inmode,WORD AxisNum,WORD* AxisList,double* multi);
DMC_API short __stdcall dmc_get_handwheel_inmode_extern_decimals(WORD CardNo,WORD* inmode,WORD* AxisNum,WORD* AxisList,double *multi);

/*********************************************************************************************************
??????? ???????????????  ????
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

/**************************???????ËÝ??***************************/
/*************************************
LTC1	AXIS0	AXIS1	AXIS2	AXIS3
LTC2	AXIS4	AXIS5	AXIS6	AXIS7
***************************************/
DMC_API short __stdcall dmc_set_ltc_mode(WORD CardNo,WORD axis,WORD ltc_logic,WORD ltc_mode,double filter);//????LTC???
DMC_API short __stdcall dmc_get_ltc_mode(WORD CardNo,WORD axis,WORD*ltc_logic,WORD*ltc_mode,double *filter);//???????LTC???
DMC_API short __stdcall dmc_set_latch_mode(WORD CardNo,WORD axis,WORD all_enable,WORD latch_source,WORD triger_chunnel);//?????????
DMC_API short __stdcall dmc_get_latch_mode(WORD CardNo,WORD axis,WORD *all_enable,WORD* latch_source,WORD* triger_chunnel);
DMC_API short __stdcall dmc_SetLtcOutMode(WORD CardNo,WORD axis,WORD enable,WORD bitno);//????????
DMC_API short __stdcall dmc_GetLtcOutMode(WORD CardNo,WORD axis,WORD *enable,WORD* bitno);
DMC_API short __stdcall dmc_get_latch_flag(WORD CardNo,WORD axis);//????????????
DMC_API short __stdcall dmc_reset_latch_flag(WORD CardNo,WORD axis);//??¦Ë?????????
DMC_API long __stdcall  dmc_get_latch_value(WORD CardNo,WORD axis);//??????????????????
DMC_API short __stdcall dmc_get_latch_value_unit(WORD CardNo,WORD axis,double* pos_by_mm);
DMC_API short __stdcall dmc_get_latch_flag_extern(WORD CardNo,WORD axis);//????????????
DMC_API long __stdcall dmc_get_latch_value_extern(WORD CardNo,WORD axis,WORD index);//????????
DMC_API short __stdcall dmc_set_latch_stop_time(WORD CardNo,WORD axis,long time);//??????????
DMC_API short __stdcall dmc_get_latch_stop_time(WORD CardNo,WORD axis,long* time);

/*********************************************************************************************************
???????? ???20170308 ????
*********************************************************************************************************/
//??????????????????0-????????1-????????????????0-??????1-???????2-??????????????¦Ëus
DMC_API short __stdcall dmc_ltc_set_mode(WORD CardNo,WORD latch,WORD ltc_mode,WORD ltc_logic,double filter);
DMC_API short __stdcall dmc_ltc_get_mode(WORD CardNo,WORD latch,WORD *ltc_mode,WORD *ltc_logic,double *filter);
//???????????0-???¦Ë???1-??????????¦Ë??
DMC_API short __stdcall dmc_ltc_set_source(WORD CardNo,WORD latch,WORD axis,WORD ltc_source);
DMC_API short __stdcall dmc_ltc_get_source(WORD CardNo,WORD latch,WORD axis,WORD *ltc_source);
//??¦Ë??????
DMC_API short __stdcall dmc_ltc_reset(WORD CardNo,WORD latch);
//???????????
DMC_API short __stdcall dmc_ltc_get_number(WORD CardNo,WORD latch,WORD axis,int *number);
//????????
DMC_API short __stdcall dmc_ltc_get_value_unit(WORD CardNo,WORD latch,WORD axis,double *value);

/*****************************¦Ë???????****************************/
//????¦Ë????
DMC_API short __stdcall dmc_compare_set_config(WORD CardNo,WORD axis,WORD enable, WORD cmp_source);//????????
DMC_API short __stdcall dmc_compare_get_config(WORD CardNo,WORD axis,WORD *enable, WORD *cmp_source);//???????????
DMC_API short __stdcall dmc_compare_clear_points(WORD CardNo,WORD cmp);//???????§Ò???
DMC_API short __stdcall dmc_compare_add_point(WORD CardNo,WORD cmp,long pos,WORD dir, WORD action,DWORD actpara);//???????
DMC_API short __stdcall dmc_compare_get_current_point(WORD CardNo,WORD cmp,long *pos);//??????????
DMC_API short __stdcall dmc_compare_get_points_runned(WORD CardNo,WORD cmp,long *pointNum);//?????????????
DMC_API short __stdcall dmc_compare_get_points_remained(WORD CardNo,WORD cmp,long *pointNum);//???????????????????

//???¦Ë????
DMC_API short __stdcall dmc_compare_set_config_extern(WORD CardNo,WORD enable, WORD cmp_source);//????????
DMC_API short __stdcall dmc_compare_get_config_extern(WORD CardNo,WORD *enable, WORD *cmp_source);//???????????
DMC_API short __stdcall dmc_compare_clear_points_extern(WORD CardNo);//???????§Ò???
DMC_API short __stdcall dmc_compare_add_point_extern(WORD CardNo,WORD* axis,long* pos,WORD* dir, WORD action,DWORD actpara);//????????¦Ë?????
DMC_API short __stdcall dmc_compare_get_current_point_extern(WORD CardNo,long *pos);//??????????
DMC_API short __stdcall dmc_compare_add_point_extern_unit(WORD CardNo,WORD* axis,double* pos,WORD* dir, WORD action,DWORD actpara);//????????¦Ë?????
DMC_API short __stdcall dmc_compare_get_current_point_extern_unit(WORD CardNo,double *pos);//??????????
DMC_API short __stdcall dmc_compare_get_points_runned_extern(WORD CardNo,long *pointNum);//?????????????
DMC_API short __stdcall dmc_compare_get_points_remained_extern(WORD CardNo,long *pointNum);//???????????????????

//????¦Ë????
DMC_API short __stdcall dmc_compare_set_config_multi(WORD CardNo,WORD queue,WORD enable, WORD axis, WORD cmp_source);//????????
DMC_API short __stdcall dmc_compare_get_config_multi(WORD CardNo, WORD queue,WORD* enable, WORD* axis, WORD* cmp_source);//???????????
DMC_API short __stdcall dmc_compare_add_point_multi(WORD CardNo, WORD cmp,int32 pos, WORD dir,  WORD action, DWORD actpara,double times);//??????? ???

//????¦Ë????
DMC_API short __stdcall dmc_hcmp_set_mode(WORD CardNo,WORD hcmp, WORD cmp_mode);//???????????
DMC_API short __stdcall dmc_hcmp_get_mode(WORD CardNo,WORD hcmp, WORD* cmp_mode);
//???????????
DMC_API short __stdcall dmc_hcmp_set_config_extern(WORD CardNo,WORD hcmp,WORD axis, WORD cmp_source, WORD cmp_logic,WORD cmp_mode,long dist,long time);
DMC_API short __stdcall dmc_hcmp_get_config_extern(WORD CardNo,WORD hcmp,WORD* axis, WORD* cmp_source, WORD* cmp_logic,WORD* cmp_mode,long* dist,long* time);

DMC_API short __stdcall dmc_hcmp_set_config(WORD CardNo,WORD hcmp,WORD axis, WORD cmp_source, WORD cmp_logic,long time);//????????????
DMC_API short __stdcall dmc_hcmp_get_config(WORD CardNo,WORD hcmp,WORD* axis, WORD* cmp_source, WORD* cmp_logic,long* time);
DMC_API short __stdcall dmc_hcmp_add_point(WORD CardNo,WORD hcmp, long cmp_pos);
DMC_API short __stdcall dmc_hcmp_set_liner(WORD CardNo,WORD hcmp, long Increment,long Count);//??????????????
DMC_API short __stdcall dmc_hcmp_get_liner(WORD CardNo,WORD hcmp, long* Increment,long* Count);
DMC_API short __stdcall dmc_hcmp_get_current_state(WORD CardNo,WORD hcmp,long *remained_points,long *current_point,long *runned_points); //???????????
DMC_API short __stdcall dmc_hcmp_clear_points(WORD CardNo,WORD hcmp);
DMC_API short __stdcall dmc_read_cmp_pin(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_write_cmp_pin(WORD CardNo,WORD axis, WORD on_off);//????cmp???????

//???????¦Ë???????
DMC_API short __stdcall dmc_hcmp_2d_set_enable(WORD CardNo,WORD hcmp, WORD cmp_enable);
DMC_API short __stdcall dmc_hcmp_2d_get_enable(WORD CardNo,WORD hcmp, WORD *cmp_enable);
DMC_API short __stdcall dmc_hcmp_2d_set_config(WORD CardNo,WORD hcmp,WORD cmp_mode,WORD x_axis, WORD x_cmp_source, WORD y_axis, WORD y_cmp_source, long error,WORD cmp_logic,long time,WORD pwm_enable,double duty,long freq,WORD port_sel,WORD pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_get_config(WORD CardNo,WORD hcmp,WORD *cmp_mode,WORD *x_axis, WORD *x_cmp_source, WORD *y_axis, WORD *y_cmp_source, long *error,WORD *cmp_logic,long *time,WORD *pwm_enable,double *duty,long *freq,WORD *port_sel,WORD *pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_add_point(WORD CardNo,WORD hcmp, long x_cmp_pos, long y_cmp_pos);
DMC_API short __stdcall dmc_hcmp_2d_get_current_state(WORD CardNo,WORD hcmp,long *remained_points,long *x_current_point,long *y_current_point,long *runned_points,WORD *current_state);
DMC_API short __stdcall dmc_hcmp_2d_clear_points(WORD CardNo,WORD hcmp);
DMC_API short __stdcall dmc_hcmp_2d_force_output(WORD CardNo,WORD hcmp,WORD enable);

/********************???IO????**************************/
//???IO
DMC_API short __stdcall dmc_read_inbit(WORD CardNo,WORD bitno);//????????????
DMC_API short __stdcall dmc_write_outbit(WORD CardNo,WORD bitno,WORD on_off);//?????????????
DMC_API short __stdcall dmc_read_outbit(WORD CardNo,WORD bitno);//????????????
DMC_API DWORD __stdcall dmc_read_inport(WORD CardNo,WORD portno);//????????????
DMC_API DWORD __stdcall dmc_read_outport(WORD CardNo,WORD portno);//????????????
DMC_API short __stdcall dmc_write_outport(WORD CardNo,WORD portno,DWORD outport_val);//?????????????????

DMC_API short __stdcall dmc_write_outport_16X(WORD CardNo,WORD portno,DWORD outport_val);//????????????????

//????IO???
DMC_API short __stdcall dmc_set_io_map_virtual(WORD CardNo,WORD bitno,WORD MapIoType,WORD MapIoIndex,double Filter);
DMC_API short __stdcall dmc_get_io_map_virtual(WORD CardNo,WORD bitno,WORD* MapIoType,WORD* MapIoIndex,double* Filter);
DMC_API short __stdcall dmc_read_inbit_virtual(WORD CardNo,WORD bitno); //????????????

DMC_API short __stdcall dmc_reverse_outbit(WORD CardNo,WORD bitno,double reverse_time);//IO??????
DMC_API short __stdcall dmc_set_io_count_mode(WORD CardNo,WORD bitno,WORD mode,double filter);//????IO??????
DMC_API short __stdcall dmc_get_io_count_mode(WORD CardNo,WORD bitno,WORD *mode,double* filter);
DMC_API short __stdcall dmc_set_io_count_value(WORD CardNo,WORD bitno,DWORD CountValue);//????IO?????
DMC_API short __stdcall dmc_get_io_count_value(WORD CardNo,WORD bitno,DWORD* CountValue);

/*********************???IO???************************/
DMC_API short __stdcall dmc_set_axis_io_map(WORD CardNo,WORD Axis,WORD IoType,WORD MapIoType,WORD MapIoIndex,double Filter);
DMC_API short __stdcall dmc_get_axis_io_map(WORD CardNo,WORD Axis,WORD IoType,WORD* MapIoType,WORD* MapIoIndex,double* Filter);
DMC_API short __stdcall dmc_set_special_input_filter(WORD CardNo,double Filter);//???????????IO??????

//3410??? ????????????????
DMC_API short __stdcall dmc_set_sd_mode(WORD CardNo,WORD axis,WORD enable,WORD sd_logic,WORD sd_mode);//????SD???
DMC_API short __stdcall dmc_get_sd_mode(WORD CardNo,WORD axis,WORD* enable,WORD *sd_logic,WORD *sd_mode);//???????SD???
//???IO
DMC_API short __stdcall dmc_set_inp_mode(WORD CardNo,WORD axis,WORD enable,WORD inp_logic);//????INP???
DMC_API short __stdcall dmc_get_inp_mode(WORD CardNo,WORD axis,WORD *enable,WORD *inp_logic);//???????INP???
DMC_API short __stdcall dmc_set_rdy_mode(WORD CardNo,WORD axis,WORD enable,WORD rdy_logic);//????RDY???
DMC_API short __stdcall dmc_get_rdy_mode(WORD CardNo,WORD axis,WORD* enable,WORD* rdy_logic);//???????RDY???
DMC_API short __stdcall dmc_set_erc_mode(WORD CardNo,WORD axis,WORD enable,WORD erc_logic,WORD erc_width,WORD erc_off_time);//????ERC???
DMC_API short __stdcall dmc_get_erc_mode(WORD CardNo,WORD axis,WORD *enable,WORD *erc_logic, WORD *erc_width,WORD *erc_off_time);//???????ERC???
DMC_API short __stdcall dmc_set_alm_mode(WORD CardNo,WORD axis,WORD enable,WORD alm_logic,WORD alm_action);//????ALM???
DMC_API short __stdcall dmc_get_alm_mode(WORD CardNo,WORD axis,WORD *enable,WORD *alm_logic,WORD *alm_action);//???????ALM???
DMC_API short __stdcall dmc_set_ez_mode(WORD CardNo,WORD axis,WORD ez_logic,WORD ez_mode,double filter);//????EZ???
DMC_API short __stdcall dmc_get_ez_mode(WORD CardNo,WORD axis,WORD *ez_logic,WORD *ez_mode,double *filter);//???????EZ???

DMC_API short __stdcall dmc_write_sevon_pin(WORD CardNo,WORD axis,WORD on_off);//????SEVON???
DMC_API short __stdcall dmc_read_sevon_pin(WORD CardNo,WORD axis);//???SEVON???
DMC_API short __stdcall dmc_read_rdy_pin(WORD CardNo,WORD axis);//???RDY??
DMC_API short __stdcall dmc_write_erc_pin(WORD CardNo,WORD axis,WORD on_off);//????ERC???????
DMC_API short __stdcall dmc_read_erc_pin(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_write_sevrst_pin(WORD CardNo,WORD axis,WORD on_off);//?????????¦Ë???
DMC_API short __stdcall dmc_read_sevrst_pin(WORD CardNo,WORD axis);//???????¦Ë???

//?????????????????????????
DMC_API short __stdcall dmc_set_io_dstp_mode(WORD CardNo,WORD axis,WORD enable,WORD logic);//enable:0-?????1-?????????????2-????????????
DMC_API short __stdcall dmc_get_io_dstp_mode(WORD CardNo,WORD axis,WORD *enable,WORD *logic);
//?????????
DMC_API short __stdcall dmc_set_dec_stop_time(WORD CardNo,WORD axis,double time);
DMC_API short __stdcall dmc_get_dec_stop_time(WORD CardNo,WORD axis,double *time);
//??????????????????????
DMC_API short __stdcall dmc_set_vector_dec_stop_time(WORD CardNo,WORD Crd,double time);
DMC_API short __stdcall dmc_get_vector_dec_stop_time(WORD CardNo,WORD Crd,double *time);
//??????????
DMC_API short __stdcall dmc_set_dec_stop_dist(WORD CardNo,WORD axis,long dist);
DMC_API short __stdcall dmc_get_dec_stop_dist(WORD CardNo,WORD axis,long *dist);
DMC_API short __stdcall dmc_set_io_dstp_bitno(WORD CardNo,WORD axis,WORD bitno,double filter);//???????????????¦Ë??????IO??
DMC_API short __stdcall dmc_get_io_dstp_bitno(WORD CardNo,WORD axis,WORD *bitno,double* filter);

/************************??????????**********************/
DMC_API short __stdcall dmc_set_counter_inmode(WORD CardNo,WORD axis,WORD mode);//????????????????
DMC_API short __stdcall dmc_get_counter_inmode(WORD CardNo,WORD axis,WORD *mode);//?????????????????
//???????(????)
DMC_API long __stdcall dmc_get_encoder(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_set_encoder(WORD CardNo,WORD axis,long encoder_value);
//???????(????)
DMC_API short __stdcall dmc_set_encoder_unit(WORD CardNo,WORD axis, double pos);
DMC_API short __stdcall dmc_get_encoder_unit(WORD CardNo,WORD axis, double * pos);
//????????? ?????
DMC_API short __stdcall dmc_set_handwheel_encoder(WORD CardNo,WORD channel, long pos);
DMC_API short __stdcall dmc_get_handwheel_encoder(WORD CardNo,WORD channel, long * pos);
//?????????????
DMC_API short __stdcall dmc_set_extra_encoder_mode(WORD CardNo,WORD channel,WORD inmode,WORD multi);
DMC_API short __stdcall dmc_get_extra_encoder_mode(WORD CardNo,WORD channel,WORD* inmode,WORD* multi);
//??????????????
DMC_API short __stdcall dmc_set_extra_encoder(WORD CardNo,WORD channel, int pos);
DMC_API short __stdcall dmc_get_extra_encoder(WORD CardNo,WORD channel, int * pos);

/*********************¦Ë?¨¹???????***************************/
//???¦Ë??(????)
DMC_API short __stdcall dmc_set_position(WORD CardNo,WORD axis,long current_position);
DMC_API long __stdcall dmc_get_position(WORD CardNo,WORD axis);
//???¦Ë??(????)
DMC_API short __stdcall dmc_set_position_unit(WORD CardNo,WORD axis, double pos);
DMC_API short __stdcall dmc_get_position_unit(WORD CardNo,WORD axis, double * pos);

/**************************?????********************************/
//????
DMC_API double __stdcall dmc_read_current_speed(WORD CardNo,WORD axis);	//???????????????(????)
DMC_API short __stdcall dmc_read_current_speed_unit(WORD CardNo,WORD axis, double *current_speed);//?????????(????)
DMC_API double __stdcall dmc_read_vector_speed(WORD CardNo);	//??????????????
DMC_API long __stdcall dmc_get_target_position(WORD CardNo,WORD axis);	//?????????????¦Ë??
DMC_API short __stdcall dmc_get_target_position_unit(WORD CardNo,WORD axis, double * pos);//?????????????¦Ë??(????)
DMC_API short __stdcall dmc_check_done(WORD CardNo,WORD axis);	//???????????????

DMC_API DWORD __stdcall dmc_axis_io_status(WORD CardNo,WORD axis);//?????????§Û??????????
DMC_API short __stdcall dmc_stop(WORD CardNo,WORD axis,WORD stop_mode);//??????
DMC_API short __stdcall dmc_check_done_multicoor(WORD CardNo,WORD Crd);//???????
DMC_API short __stdcall dmc_stop_multicoor(WORD CardNo,WORD Crd,WORD stop_mode);//??????
DMC_API short __stdcall dmc_emg_stop(WORD CardNo);//????????????
DMC_API short __stdcall dmc_LinkState(WORD CardNo,WORD* State);//??????
DMC_API short __stdcall dmc_get_axis_run_mode(WORD CardNo, WORD axis,WORD* run_mode);//???????????????
DMC_API short __stdcall dmc_get_stop_reason(WORD CardNo,WORD axis,long* StopReason);//????????
DMC_API short __stdcall dmc_clear_stop_reason(WORD CardNo,WORD axis);//?????????

//trace????
DMC_API short __stdcall dmc_set_trace(WORD CardNo,WORD axis,WORD enable);
DMC_API short __stdcall dmc_get_trace(WORD CardNo,WORD axis,WORD* enable);
DMC_API short __stdcall dmc_read_trace_data(WORD CardNo,WORD axis,WORD data_option,long* ReceiveSize,double* time,double* data,long* remain_num);
DMC_API short __stdcall dmc_trace_start(WORD CardNo,WORD AxisNum,WORD *AxisList);
DMC_API short __stdcall dmc_trace_stop(WORD CardNo);

//????????
DMC_API short __stdcall dmc_calculate_arclength_center(double* start_pos,double *target_pos,double *cen_pos, WORD arc_dir,double circle,double* ArcLength);

/*********************************??????????**************************************
??????????????????¦Â????
************************************************************************************/
DMC_API short __stdcall dmc_conti_open_list (WORD CardNo,WORD Crd,WORD AxisNum,WORD *AxisList);//??????????????
DMC_API short __stdcall dmc_conti_close_list(WORD CardNo,WORD Crd);//?????????????
DMC_API short __stdcall dmc_conti_reset_list(WORD CardNo,WORD Crd);//??¦Ë??????????
DMC_API short __stdcall dmc_conti_stop_list (WORD CardNo,WORD Crd,WORD stop_mode);//??????????
DMC_API short __stdcall dmc_conti_pause_list(WORD CardNo,WORD Crd);//???????????
DMC_API short __stdcall dmc_conti_start_list(WORD CardNo,WORD Crd);//?????????
DMC_API short __stdcall dmc_conti_get_run_state(WORD CardNo,WORD Crd);//0-???§µ?1-?????2-????????3-¦Ä??????4-????
DMC_API long __stdcall dmc_conti_remain_space (WORD CardNo,WORD Crd);//???????????????
DMC_API long __stdcall dmc_conti_read_current_mark (WORD CardNo,WORD Crd);//?????????????¦Å????

//blend??
DMC_API short __stdcall dmc_conti_set_blend(WORD CardNo,WORD Crd,WORD enable);
DMC_API short __stdcall dmc_conti_get_blend(WORD CardNo,WORD Crd,WORD* enable);
DMC_API short __stdcall dmc_conti_set_override(WORD CardNo,WORD Crd,double Percent);//???¨°?????????
DMC_API short __stdcall dmc_conti_change_speed_ratio (WORD CardNo,WORD Crd,double percent);//???¨°??§Ø??????
//§³?????
DMC_API short __stdcall dmc_conti_set_lookahead_mode(WORD CardNo,WORD Crd,WORD enable,long LookaheadSegments,double PathError,double LookaheadAcc);
DMC_API short __stdcall dmc_conti_get_lookahead_mode(WORD CardNo,WORD Crd,WORD* enable,long* LookaheadSegments,double* PathError,double* LookaheadAcc);

//??????IO????
DMC_API short __stdcall dmc_conti_wait_input(WORD CardNo,WORD Crd,WORD bitno,WORD on_off,double TimeOut,long mark);
DMC_API short __stdcall dmc_conti_delay_outbit_to_start(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_delay_outbit_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_time,double ReverseTime);
DMC_API short __stdcall dmc_conti_ahead_outbit_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_accurate_outbit_unit(WORD CardNo, WORD Crd, WORD cmp_no,WORD on_off,WORD axis,double abs_pos,WORD pos_source,double ReverseTime);
DMC_API short __stdcall dmc_conti_write_outbit(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ReverseTime);
DMC_API short __stdcall dmc_conti_clear_io_action(WORD CardNo, WORD Crd, DWORD Io_Mask);
DMC_API short __stdcall dmc_conti_set_pause_output(WORD CardNo,WORD Crd,WORD action,long mask,long state);
DMC_API short __stdcall dmc_conti_get_pause_output(WORD CardNo,WORD Crd,WORD* action,long* mask,long* state);
DMC_API short __stdcall dmc_conti_delay(WORD CardNo, WORD Crd,double delay_time,long mark);//??????

DMC_API short __stdcall  dmc_conti_reverse_outbit(WORD CardNo, WORD Crd, WORD bitno,double reverse_time);//IO??????????
DMC_API short __stdcall  dmc_conti_delay_outbit(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_time);//IO???????

//??????????
DMC_API short __stdcall dmc_conti_line_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_center_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_radius_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_3points_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_rectangle_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mark_Pos,long num,WORD rect_mode,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_pmove_unit(WORD CardNo,WORD Crd,WORD axis,double dist,WORD posi_mode,WORD mode,long imark);
DMC_API short __stdcall dmc_conti_set_involute_mode(WORD CardNo,WORD Crd,WORD mode);//????????????????
DMC_API short __stdcall dmc_conti_get_involute_mode(WORD CardNo,WORD Crd,WORD* mode);
DMC_API short __stdcall dmc_set_gear_follow_profile(WORD CardNo,WORD axis,WORD enable,WORD master_axis,double ratio);//?Z??
DMC_API short __stdcall dmc_get_gear_follow_profile(WORD CardNo,WORD axis,WORD* enable,WORD* master_axis,double* ratio);

DMC_API short __stdcall dmc_conti_line_unit_extern(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Cen_Pos,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_center_unit_extern(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Cen_Pos,double Arc_Radius,WORD posi_mode,long mark);

/*********************************PWM???????*******************************/
//PWM????
DMC_API short __stdcall dmc_set_pwm_pin(WORD CardNo,WORD portno,WORD ON_OFF, double dfreqency,double dduty);
DMC_API short __stdcall dmc_get_pwm_pin(WORD CardNo,WORD portno,WORD *ON_OFF, double *dfreqency,double *dduty);
//PWM????
DMC_API short __stdcall dmc_set_pwm_enable(WORD CardNo,WORD enable);
DMC_API short __stdcall dmc_get_pwm_enable(WORD CardNo,WORD* enable);
DMC_API short __stdcall dmc_set_pwm_output(WORD CardNo, WORD PwmNo,double fDuty, double fFre);
DMC_API short __stdcall dmc_get_pwm_output(WORD CardNo,WORD PwmNo,double* fDuty, double* fFre);
DMC_API short __stdcall dmc_conti_set_pwm_output(WORD CardNo,WORD Crd, WORD PwmNo,double fDuty, double fFre);

//????PWM????
DMC_API short __stdcall dmc_set_pwm_enable_extern(WORD CardNo,WORD channel, WORD enable);
DMC_API short __stdcall dmc_get_pwm_enable_extern(WORD CardNo,WORD channel, WORD* enable);

/**********PWM??????**************
mode:??????0-?????? ?????? 1-?????? ????????2-?????? ????????3-???? ???????????4-???? ??????????
MaxVel:??????????????¦Ëunit
MaxValue:??????????????????
OutValue??????????????????
*************************************/
DMC_API short __stdcall dmc_conti_set_pwm_follow_speed(WORD CardNo,WORD Crd,WORD pwm_no,WORD mode,double MaxVel,double MaxValue,double OutValue);
DMC_API short __stdcall dmc_conti_get_pwm_follow_speed(WORD CardNo,WORD Crd,WORD pwm_no,WORD* mode,double* MaxVel,double* MaxValue,double* OutValue);
//????PWM????????????
DMC_API short __stdcall dmc_set_pwm_onoff_duty(WORD CardNo, WORD PwmNo,double fOnDuty, double fOffDuty);
DMC_API short __stdcall dmc_get_pwm_onoff_duty(WORD CardNo, WORD PwmNo,double* fOnDuty, double* fOffDuty);
DMC_API short __stdcall dmc_conti_delay_pwm_to_start(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_delay_pwm_to_stop(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double delay_time,double ReverseTime);
DMC_API short __stdcall dmc_conti_ahead_pwm_to_stop(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_write_pwm(WORD CardNo, WORD Crd, WORD pwmno,WORD on_off,double ReverseTime);

/*********************ADDA????******************************/
//PWM?DA????
DMC_API short __stdcall dmc_set_da_enable(WORD CardNo,WORD enable);
DMC_API short __stdcall dmc_get_da_enable(WORD CardNo,WORD* enable);
DMC_API short __stdcall dmc_set_da_output(WORD CardNo, WORD channel,double Vout);
DMC_API short __stdcall dmc_get_da_output(WORD CardNo,WORD channel,double* Vout);
//???AD????
DMC_API short __stdcall dmc_get_ad_input(WORD CardNo,WORD channel,double* Vout);
//??????????DA????
DMC_API short __stdcall dmc_conti_set_da_output(WORD CardNo, WORD Crd, WORD channel,double Vout);
//????????DA???
DMC_API short __stdcall dmc_conti_set_da_enable(WORD CardNo, WORD Crd, WORD enable,WORD channel,long mark);
/**********DA??????**************
da_no:?????
MaxVel:??????????????¦Ëunit
MaxValue:???????
*************************************/
DMC_API short __stdcall dmc_conti_set_da_follow_speed(WORD CardNo,WORD Crd,WORD da_no,double MaxVel,double MaxValue,double acc_offset,double dec_offset,double acc_dist,double dec_dist);
DMC_API short __stdcall dmc_conti_get_da_follow_speed(WORD CardNo,WORD Crd,WORD da_no,double* MaxVel,double* MaxValue,double* acc_offset,double* dec_offset,double* acc_dist,double* dec_dist);

/******************************CAN IO***********************************/
//baud:0-1M 1-800 2-500 3-250 4-125 5-100
DMC_API short __stdcall dmc_set_can_state(WORD CardNo,WORD NodeNum,WORD state,WORD baud);//0-?????1-?????2-??¦Ë?????????
DMC_API short __stdcall dmc_get_can_state(WORD CardNo,WORD* NodeNum,WORD* state);////0-?????1-?????2-??
DMC_API short __stdcall dmc_get_can_errcode(WORD CardNo,WORD *Errcode);
DMC_API short __stdcall dmc_write_can_outbit(WORD CardNo,WORD Node,WORD bitno,WORD on_off);
DMC_API short __stdcall dmc_read_can_outbit(WORD CardNo,WORD Node,WORD bitno);
DMC_API short __stdcall dmc_read_can_inbit(WORD CardNo,WORD Node,WORD bitno);
DMC_API short __stdcall dmc_write_can_outport(WORD CardNo,WORD Node,WORD PortNo,DWORD outport_val);
DMC_API DWORD __stdcall dmc_read_can_outport(WORD CardNo,WORD Node,WORD PortNo);
DMC_API DWORD __stdcall dmc_read_can_inport(WORD CardNo,WORD Node,WORD PortNo);
//???CAN??????
DMC_API short __stdcall dmc_get_can_errcode_extern(WORD CardNo,WORD *Errcode,WORD *msg_losed, WORD *emg_msg_num, WORD *lostHeartB, WORD *EmgMsg);

DMC_API long __stdcall  dmc_set_profile_limit(WORD CardNo,WORD axis,double Max_Vel,double Max_Acc,double EvenTime);
DMC_API long __stdcall  dmc_get_profile_limit(WORD CardNo,WORD axis,double* Max_Vel,double* Max_Acc,double* EvenTime);
DMC_API long __stdcall  dmc_set_vector_profile_limit(WORD CardNo,WORD Crd,double Max_Vel,double Max_Acc,double EvenTime);
DMC_API long __stdcall  dmc_get_vector_profile_limit(WORD CardNo,WORD Crd,double* Max_Vel,double* Max_Acc,double* EvenTime);
//§³????????
DMC_API short __stdcall dmc_set_arc_limit(WORD CardNo,WORD Crd,WORD Enable,double MaxCenAcc,double MaxArcError);
DMC_API short __stdcall dmc_get_arc_limit(WORD CardNo,WORD Crd,WORD* Enable,double* MaxCenAcc,double* MaxArcError);

//DMC_API short __stdcall dmc_get_axis_debug_state(WORD CardNo,WORD axis,struct_DebugPara* pack);

//????????
//??????????????????0-????????1-????????????????0-??????1-???????2-??????????????¦Ëus
DMC_API short __stdcall dmc_softltc_set_mode(WORD CardNo,WORD latch,WORD ltc_enable,WORD ltc_mode,WORD ltc_inbit,WORD ltc_logic,double filter);
DMC_API short __stdcall dmc_softltc_get_mode(WORD CardNo,WORD latch,WORD *ltc_enable,WORD *ltc_mode,WORD *ltc_inbit,WORD *ltc_logic,double *filter);
//???????????0-???¦Ë???1-??????????¦Ë??
DMC_API short __stdcall dmc_softltc_set_source(WORD CardNo,WORD latch,WORD axis,WORD ltc_source);
DMC_API short __stdcall dmc_softltc_get_source(WORD CardNo,WORD latch,WORD axis,WORD *ltc_source);
//??¦Ë??????
DMC_API short __stdcall dmc_softltc_reset(WORD CardNo,WORD latch);
//???????????
DMC_API short __stdcall dmc_softltc_get_number(WORD CardNo,WORD latch,WORD axis,int *number);
//????????
DMC_API short __stdcall dmc_softltc_get_value_unit(WORD CardNo,WORD latch,WORD axis,double *value);

DMC_API short __stdcall dmc_set_IoFilter(WORD CardNo,WORD bitno, double filter);
DMC_API short __stdcall dmc_get_IoFilter(WORD CardNo,WORD bitno, double *filter);

//?????
DMC_API short __stdcall dmc_set_lsc_index_value (WORD CardNo, WORD axis,WORD IndexID, long IndexValue);
DMC_API short __stdcall dmc_get_lsc_index_value(WORD CardNo, WORD axis,WORD IndexID, long *IndexValue);

DMC_API short __stdcall dmc_set_lsc_config(WORD CardNo, WORD axis,WORD Origin, DWORD Interal,DWORD NegIndex,DWORD PosIndex,double Ratio);
DMC_API short __stdcall dmc_get_lsc_config(WORD CardNo, WORD axis,WORD *Origin, DWORD *Interal,DWORD *NegIndex,DWORD *PosIndex,double *Ratio);

//?????
DMC_API short __stdcall dmc_set_watchdog(WORD CardNo,WORD enable,DWORD time);
DMC_API short __stdcall dmc_call_watchdog(WORD CardNo);

DMC_API short __stdcall dmc_read_diagnoseData(WORD CardNo);
DMC_API short __stdcall dmc_conti_set_cmd_end(WORD CardNo,WORD Crd,WORD enable);

//????????¦Ë
DMC_API short __stdcall dmc_set_zone_limit_config(WORD CardNo, WORD *axis, WORD *Source, long x_pos_p, long x_pos_n, long y_pos_p, long y_pos_n, WORD action_para);
DMC_API short __stdcall dmc_get_zone_limit_config(WORD CardNo, WORD* axis, WORD* Source, long* x_pos_p, long* x_pos_n, long* y_pos_p, long* y_pos_n, WORD* action_para);
DMC_API short __stdcall dmc_set_zone_limit_enable(WORD CardNo, WORD enable);

//????????
DMC_API short __stdcall dmc_set_interlock_config(WORD CardNo, WORD* axis, WORD* Source, long delta_pos, WORD action_para);
DMC_API short __stdcall dmc_get_interlock_config(WORD CardNo, WORD* axis, WORD* Source, long* delta_pos, WORD* action_para);
DMC_API short __stdcall dmc_set_interlock_enable(WORD CardNo, WORD enable);

//??????????????
DMC_API short __stdcall dmc_set_grant_error_protect(WORD CardNo, WORD axis,WORD enable,DWORD dstp_error, DWORD emg_error);
DMC_API short __stdcall dmc_get_grant_error_protect(WORD CardNo, WORD axis,WORD* enable,DWORD* dstp_error, DWORD* emg_error);

DMC_API short __stdcall dmc_set_safety_param(WORD CardNo,WORD axis,WORD enable,long safety_pos);
DMC_API short __stdcall dmc_get_safety_param(WORD CardNo,WORD axis,WORD* enable,long* safety_pos);
DMC_API short __stdcall dmc_get_diagnose_param(WORD CardNo,WORD axis,long* tartet_pos,int* mode,long* pulse_pos,long* endcoder_pos);

//???????????
DMC_API short __stdcall dmc_set_camerablow_config(WORD CardNo,WORD camerablow_en,long cameraPos,WORD piece_num,long piece_distance,WORD axis_sel,long latch_distance_min);
DMC_API short __stdcall dmc_get_camerablow_config(WORD CardNo,WORD* camerablow_en,long* cameraPos,WORD* piece_num,long* piece_distance,WORD* axis_sel,long* latch_distance_min);
DMC_API short __stdcall dmc_clear_camerablow_errorcode(WORD CardNo);
DMC_API short __stdcall dmc_get_camerablow_errorcode(WORD CardNo,WORD* errorcode);

//???????????0~15???????????¦Ë???
DMC_API short __stdcall dmc_set_io_limit_config(WORD CardNo,WORD portno,WORD enable,WORD axis_sel,WORD el_mode,WORD el_logic);
DMC_API short __stdcall dmc_get_io_limit_config(WORD CardNo,WORD portno,WORD* enable,WORD* axis_sel,WORD* el_mode,WORD* el_logic);

//???????????
DMC_API short __stdcall dmc_set_handwheel_filter(WORD CardNo,WORD axis,double filter_factor);
DMC_API short __stdcall dmc_get_handwheel_filter(WORD CardNo,WORD axis,double* filter_factor);

//??????????????????????
DMC_API short __stdcall dmc_conti_get_interp_map(WORD CardNo,WORD Crd,WORD* AxisNum ,WORD* AxisList,double *pPosList);
//?????????????
DMC_API short __stdcall dmc_conti_get_crd_errcode(WORD CardNo,WORD Crd,WORD* errcode);


DMC_API short __stdcall dmc_line_unit_follow(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode);
DMC_API short __stdcall dmc_conti_line_unit_follow(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark);

//????????????DA????
DMC_API short __stdcall dmc_conti_set_da_action(WORD CardNo,WORD Crd,WORD mode,WORD portno,double dvalue);

//???????????
DMC_API short __stdcall dmc_read_encoder_speed(WORD CardNo,WORD Axis,double *current_speed);

DMC_API short __stdcall dmc_axis_follow_line_enable(WORD CardNo,WORD Crd,WORD enable_flag);

//??????????
DMC_API short __stdcall dmc_set_interp_compensation(WORD CardNo,WORD axis, double dvalue,double time);
DMC_API short __stdcall dmc_get_interp_compensation(WORD CardNo,WORD axis, double *dvalue,double *time);

//IO?????
DMC_API short __stdcall dmc_set_io_exactstop(WORD CardNo,WORD axis, WORD ioNum,WORD *ioList,WORD enable,WORD valid_logic,WORD action,WORD move_dir);

//??????????????????
DMC_API short __stdcall dmc_get_distance_to_start(WORD CardNo,WORD Crd, double* distance_x, double* distance_y,long imark);
//??????¦Ë ?????????????????????
DMC_API short __stdcall dmc_set_start_distance_flag(WORD CardNo,WORD Crd,WORD flag);

/******************?????????¦Ë????**********************
    ??  ????
    CardNo:????
    Axis??????
    N_limit:????¦Ë??????
    P_limit:????¦Ë??????
    ???????????????
*******************************************************************/
DMC_API short __stdcall dmc_set_home_soft_limit(WORD CardNo,WORD Axis,int N_limit,int P_limit);
DMC_API short __stdcall dmc_get_home_soft_limit(WORD CardNo,WORD Axis,int* N_limit,int* P_limit);

/*********************
    ????????????????????????????????????????????????????????axis???????????????????????????????????????????????????????
    ???????????????????
    ??????????CardNo ???????
    axis ??????
    dist ????????(????????¦Ë)
        follow_mode //???????0-???????????1-?????????
    imark ?¦Ê?
    ????????????
*******************************/
DMC_API short __stdcall dmc_conti_gear_unit(WORD CardNo,WORD Crd,WORD axis,double dist, WORD follow_mode,long imark);

//?????????????
DMC_API short __stdcall dmc_set_path_fitting_enable(WORD CardNo,WORD Crd,WORD enable);
//?????????(??)
DMC_API short __stdcall dmc_enable_leadscrew_comp(WORD CardNo, WORD axis,WORD enable);
DMC_API short __stdcall dmc_set_leadscrew_comp_config(WORD CardNo, WORD axis,WORD n, int startpos,int lenpos,int *pCompPos,int *pCompNeg);
//???????????¦Ë????? ????????????
DMC_API short __stdcall dmc_t_pmove_extern(WORD CardNo, WORD axis, double MidPos,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,WORD posi_mode);

/*
????????????????????????????????????????????
??????????CardNo ????
axis ????
error ??????????????
????????????
*/
DMC_API short __stdcall dmc_set_pulse_encoder_count_error(WORD CardNo,WORD axis,WORD error);
DMC_API short __stdcall dmc_get_pulse_encoder_count_error(WORD CardNo,WORD axis,WORD *error);
/*
????????????????????????????????????????????????
    ??????????CardNo ????
    axis ????
    ????????;??
    ?????????0?????§³????????
    1??????????????????
*/
DMC_API short __stdcall dmc_check_pulse_encoder_count_error(WORD CardNo,WORD axis,int* pulse_position, int* enc_position);
/*
??§Ò?¦Ë???
mid_pos: ?§Þ?¦Ë??
aim_pos:???¦Ë??
posi_mode: ???????????????????
*/
DMC_API short __stdcall dmc_update_target_position_extern(WORD CardNo, WORD axis, double mid_pos, double aim_pos, double vel,WORD posi_mode);

//?????????????
//???
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
DMC_API short __stdcall dmc_compare_add_point_cycle(WORD CardNo,WORD cmp,long pos,WORD dir, DWORD bitno,DWORD cycle,WORD level);//???????

//???????????????????????¦¶???????????
DMC_API short __stdcall dmc_set_encoder_count_error_action_config(WORD CardNo,WORD enable,WORD stopmode);
DMC_API short __stdcall dmc_get_encoder_count_error_action_config(WORD CardNo,WORD* enable,WORD* stopmode);

DMC_API short __stdcall dmc_set_home_el_return(WORD CardNo,WORD axis,WORD enable);

//??????????da????
DMC_API short __stdcall dmc_conti_set_encoder_da_follow_enable(WORD CardNo, WORD Crd,WORD axis,WORD enable);
DMC_API short __stdcall dmc_conti_get_encoder_da_follow_enable(WORD CardNo, WORD Crd,WORD* axis,WORD* enable);

DMC_API short __stdcall dmc_check_done_pos(WORD CardNo,WORD axis,WORD posi_mode);
DMC_API short __stdcall dmc_set_factor_error(WORD CardNo,WORD axis,double factor,long error);
DMC_API short __stdcall dmc_set_factor(WORD CardNo,WORD axis,double factor);
DMC_API short __stdcall dmc_set_error(WORD CardNo,WORD axis,long error);
DMC_API short __stdcall dmc_get_factor_error(WORD CardNo,WORD axis,double* factor,long* error);
DMC_API short __stdcall dmc_check_success_pulse(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_check_success_encoder(WORD CardNo,WORD axis);

//IO????????????????
DMC_API short __stdcall dmc_set_io_count_profile(WORD CardNo, WORD chan, WORD bitno,WORD mode,double filter, double count_value, WORD* axis_list, WORD axis_num, WORD stop_mode );
DMC_API short __stdcall dmc_get_io_count_profile(WORD CardNo, WORD chan, WORD* bitno,WORD* mode,double* filter, double* count_value, WORD* axis_list, WORD* axis_num, WORD* stop_mode );
DMC_API short __stdcall dmc_set_io_count_enable(WORD CardNo, WORD chan, WORD ifenable);
DMC_API short __stdcall dmc_clear_io_count(WORD CardNo, WORD chan);
DMC_API short __stdcall dmc_get_io_count_value_extern(WORD CardNo, WORD chan, long* current_value);

//????????????¦Ë?????????¦Ë??//20191025
DMC_API short __stdcall dmc_get_position_ex(WORD CardNo,WORD axis, double * pos);
DMC_API short __stdcall dmc_get_encoder_ex(WORD CardNo,WORD axis, double * pos);
//????????????¦Ë?????????¦Ë?? ????
DMC_API short __stdcall dmc_get_position_ex_unit(WORD CardNo,WORD axis, double * pos);
DMC_API short __stdcall dmc_get_encoder_ex_unit(WORD CardNo,WORD axis, double * pos);

//?????????????
DMC_API short __stdcall dmc_set_home_shift_param(WORD CardNo, WORD axis, WORD pos_clear_mode, double ShiftValue);
DMC_API short __stdcall dmc_get_home_shift_param(WORD CardNo, WORD axis, WORD *pos_clear_mode, double* ShiftValue);

DMC_API short __stdcall dmc_change_speed_extend(WORD CardNo,WORD axis,double Curr_Vel, double Taccdec, WORD pin_num, WORD trig_mode);

DMC_API short __stdcall dmc_follow_vector_speed_move(WORD CardNo,WORD axis,WORD Follow_AxisNum,WORD* Follow_AxisList,double ratio);
DMC_API short __stdcall dmc_conti_line_unit_extend(WORD CardNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* pPosList, WORD posi_mode, double Extend_Len, WORD enable,long mark); //?????????
DMC_API short __stdcall dmc_hcmp_2d_set_config_unit(WORD CardNo,WORD hcmp,WORD cmp_mode,WORD x_axis, WORD x_cmp_source, double x_cmp_error, WORD y_axis, WORD y_cmp_source, double y_cmp_error,WORD cmp_logic,int time);
DMC_API short __stdcall dmc_hcmp_2d_get_config_unit(WORD CardNo,WORD hcmp,WORD *cmp_mode,WORD *x_axis, WORD *x_cmp_source,double *x_cmp_error,  WORD *y_axis, WORD *y_cmp_source, double *y_cmp_error, WORD *cmp_logic,int *time);

DMC_API short __stdcall dmc_hcmp_2d_set_pwmoutput(WORD CardNo,WORD hcmp,WORD pwm_enable,double duty,double freq,WORD pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_get_pwmoutput(WORD CardNo,WORD hcmp,WORD *pwm_enable,double *duty,double *freq,WORD *pwm_number);
DMC_API short __stdcall dmc_hcmp_2d_add_point_unit(WORD ConnectNo,WORD hcmp, double x_cmp_pos, double y_cmp_pos,WORD cmp_outbit);
DMC_API short __stdcall dmc_hcmp_2d_get_current_state_unit(WORD CardNo,WORD hcmp,int *remained_points,double *x_current_point,double *y_current_point,int *runned_points,WORD *current_state,WORD *current_outbit);

DMC_API short __stdcall dmc_set_home_position(WORD CardNo,WORD axis,WORD enable,double position);
DMC_API short __stdcall dmc_get_home_position(WORD CardNo,WORD axis,WORD *enable,double *position);
DMC_API short __stdcall dmc_conti_line_io_union(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,WORD bitno,WORD on_off,double io_value,WORD io_mode,WORD MapAxis,WORD pos_source,double ReverseTime,long mark);
//?????????????
DMC_API short __stdcall dmc_set_encoder_dir(WORD CardNo, WORD axis,WORD dir);

//???????????¦Ë
DMC_API short __stdcall dmc_set_arc_zone_limit_config(WORD CardNo, WORD* AxisList, WORD AxisNum, double *Center, double Radius, WORD Source,WORD StopMode);
DMC_API short __stdcall dmc_get_arc_zone_limit_config(WORD CardNo, WORD* AxisList, WORD* AxisNum, double *Center, double* Radius, WORD* Source,WORD* StopMode);
DMC_API short __stdcall dmc_get_arc_zone_limit_axis_status(WORD CardNo, WORD axis);
DMC_API short __stdcall dmc_set_arc_zone_limit_enable(WORD CardNo, WORD enable);
DMC_API short __stdcall dmc_get_arc_zone_limit_enable(WORD CardNo, WORD* enable);

//1??	??????????????¦Ë???
DMC_API short __stdcall dmc_hcmp_fifo_set_mode(WORD CardNo,WORD hcmp, WORD fifo_mode);
DMC_API short __stdcall dmc_hcmp_fifo_get_mode(WORD CardNo,WORD hcmp, WORD* fifo_mode);
//2??	??????????????¦Ë???????????§Ø??????????????¦Ë??
DMC_API short __stdcall dmc_hcmp_fifo_get_state(WORD CardNo,WORD hcmp,long *remained_points);
//3??	????????????????????¦Ë??
DMC_API short __stdcall dmc_hcmp_fifo_add_point_unit(WORD CardNo,WORD hcmp, WORD num,double *cmp_pos);
//4??	???????¦Ë??,?????FPGA??¦Ë???????????
DMC_API short __stdcall dmc_hcmp_fifo_clear_points(WORD CardNo,WORD hcmp);
//?????????????????????????????????????
DMC_API short __stdcall dmc_hcmp_fifo_add_table(WORD CardNo,WORD hcmp, WORD num,double *cmp_pos);

//???????¦Ë???????
//1??	??????????????¦Ë???
DMC_API short __stdcall dmc_hcmp_2d_fifo_set_mode(WORD CardNo,WORD hcmp, WORD fifo_mode);
DMC_API short __stdcall dmc_hcmp_2d_fifo_get_mode(WORD CardNo,WORD hcmp, WORD* fifo_mode);
//2??	??????????????¦Ë???????????§Ø??????????????¦Ë??
DMC_API short __stdcall dmc_hcmp_2d_fifo_get_state(WORD CardNo,WORD hcmp,long *remained_points);
//3??	????????????????????¦Ë??
DMC_API short __stdcall dmc_hcmp_2d_fifo_add_point_unit(WORD CardNo,WORD hcmp, WORD num,double *x_cmp_pos,double *y_cmp_pos,WORD *cmp_outbit);
//4??	???????¦Ë??,?????FPGA??¦Ë???????????
DMC_API short __stdcall dmc_hcmp_2d_fifo_clear_points(WORD CardNo,WORD hcmp);
//?????????????????????????????????????
DMC_API short __stdcall dmc_hcmp_2d_fifo_add_table(WORD CardNo,WORD hcmp, WORD num,double *x_cmp_pos,double *y_cmp_pos);
//?????????????????????????????????????,????????
DMC_API short __stdcall dmc_hcmp_2d_fifo_add_table_unit(WORD CardNo,WORD hcmp, WORD num,double *x_cmp_pos,double *y_cmp_pos,WORD *cmp_outbit);

//?????????§Ø???????????????????
DMC_API short __stdcall dmc_set_output_status_repower(WORD CardNo, WORD enable);

DMC_API short __stdcall dmc_t_pmove_extern_softlanding(WORD CardNo, WORD axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, DWORD delay_ms, double Max_Vel2,double stop_vel2, double acc_time, double dec_time, WORD posi_mode);
DMC_API short __stdcall dmc_compare_add_point_XD(WORD CardNo,WORD cmp,long pos,WORD dir, WORD action,DWORD actpara, long startPos);//???????????

DMC_API short __stdcall dmc_pmove_change_pos_speed_config(WORD CardNo,WORD axis,double tar_vel, double tar_rel_pos, WORD trig_mode, WORD source);
DMC_API short __stdcall dmc_get_pmove_change_pos_speed_config(WORD CardNo,WORD axis,double* tar_vel, double* tar_rel_pos, WORD* trig_mode, WORD* source);
DMC_API short __stdcall dmc_pmove_change_pos_speed_enable(WORD CardNo,WORD axis, WORD enable);
DMC_API short __stdcall dmc_get_pmove_change_pos_speed_enable(WORD CardNo,WORD axis, WORD* enable);
DMC_API short __stdcall dmc_compare_add_point_extend(WORD CardNo,WORD axis, long pos, WORD dir, WORD action, WORD para_num, DWORD* actpara,DWORD compare_time);
DMC_API short __stdcall dmc_get_cmd_position(WORD CardNo,WORD axis, double * pos);
//???????????
DMC_API short __stdcall dmc_set_logic_analyzer_config(WORD CardNo,WORD channel, DWORD SampleFre, DWORD SampleDepth, WORD SampleMode);
DMC_API short __stdcall dmc_start_logic_analyzer(WORD CardNo,WORD channel, WORD enable);
DMC_API short __stdcall dmc_get_logic_analyzer_counter(WORD CardNo, WORD channel, DWORD *counter);
//????????	 20190923??????????????
DMC_API short __stdcall dmc_read_inbit_append(WORD CardNo,WORD bitno);//????????????
DMC_API short __stdcall dmc_write_outbit_append(WORD CardNo,WORD bitno,WORD on_off);//?????????????
DMC_API short __stdcall dmc_read_outbit_append(WORD CardNo,WORD bitno);//????????????
DMC_API DWORD __stdcall dmc_read_inport_append(WORD CardNo,WORD portno);//????????????
DMC_API DWORD __stdcall dmc_read_outport_append(WORD CardNo,WORD portno);//????????????
DMC_API short __stdcall dmc_write_outport_append(WORD CardNo,WORD portno,DWORD port_value);//?????????????????
//?????
DMC_API short __stdcall	dmc_m_move_unit(WORD CardNo,WORD Crd, WORD axis_num, WORD* axis_list,double* mid_pos, double* target_pos, double* saftpos, WORD pos_mode);
DMC_API short __stdcall	dmc_get_m_move_config(WORD CardNo,WORD Crd, WORD *axis_num, WORD* axis_list,double* mid_pos, double* target_pos, double* saftpos,WORD* pos_mode);
// ?????????????????
DMC_API short __stdcall dmc_set_tangent_follow(WORD CardNo, WORD Crd, WORD axis, WORD follow_curve, WORD rotate_dir, double degree_equivalent);
// ???????????????????????
DMC_API	short __stdcall dmc_get_tangent_follow_param(WORD CardNo, WORD Crd, WORD* axis, WORD* follow_curve, WORD* rotate_dir, double* degree_equivalent);
// ????????????
DMC_API short __stdcall dmc_disable_follow_move(WORD CardNo, WORD Crd);
// ?????
DMC_API short __stdcall dmc_ellipse_move(WORD CardNo, WORD Crd,WORD axisNum, WORD* Axis_List, double* Target_Pos, double* Cen_Pos, double A_Axis_Len, double B_Axis_Len, WORD Dir, WORD Pos_Mode);
DMC_API short __stdcall dmc_read_vector_speed_unit(WORD CardNo,WORD Crd,double *current_speed);	//??????????????
//???????????¦Ë???????
DMC_API short __stdcall dmc_get_home_el_return(WORD CardNo,WORD axis,WORD *enable);

//??????????
DMC_API short __stdcall dmc_set_watchdog_action_event(WORD CardNo, WORD event_mask);
DMC_API short __stdcall dmc_get_watchdog_action_event(WORD CardNo, WORD* event_mask);
DMC_API short __stdcall dmc_set_watchdog_enable (WORD CardNo, double timer_period, WORD enable);
DMC_API short __stdcall dmc_get_watchdog_enable (WORD CardNo, double * timer_period, WORD* enable);
DMC_API short __stdcall dmc_reset_watchdog_timer (WORD CardNo);
//io???????
DMC_API short __stdcall dmc_set_io_check_control(WORD CardNo, WORD sensor_in_no, WORD check_mode, WORD A_out_no, WORD B_out_no, WORD C_out_no, WORD output_mode);
DMC_API short __stdcall dmc_get_io_check_control(WORD CardNo, WORD* sensor_in_no, WORD* check_mode, WORD* A_out_no, WORD* B_out_no, WORD* C_out_no, WORD* output_mode);
DMC_API short __stdcall dmc_stop_io_check_control(WORD CardNo);

//??????¦Ë??????????
DMC_API short __stdcall dmc_set_el_ret_deviation(WORD CardNo, WORD axis, WORD enable,double deviation);
DMC_API short __stdcall  dmc_get_el_ret_deviation(WORD CardNo, WORD axis, WORD* enable, double* deviation);

/*****************************???????????**********************************/
//???????
DMC_API short __stdcall nmc_set_home_profile(WORD CardNo ,WORD axis,WORD home_mode,double Low_Vel, double High_Vel,double Tacc,double Tdec ,double offsetpos);//????????????????????
DMC_API short __stdcall nmc_get_home_profile(WORD CardNo ,WORD axis,WORD* home_mode,double* Low_Vel, double* High_Vel,double* Tacc,double* Tdec ,double* offsetpos);
DMC_API short __stdcall nmc_home_move(WORD CardNo,WORD axis);

//-------------------------????????-----------------------
/*******************************************************
portnum????????????????
0: ???canopen??0????
1: ???canopen??1????
10:???EtherCAT??0????
11:???EtherCAT??1????
********************************************************/
DMC_API short __stdcall nmc_set_manager_para(WORD CardNo,WORD PortNum,DWORD Baudrate,WORD ManagerID);
DMC_API short __stdcall nmc_get_manager_para(WORD CardNo,WORD PortNum,DWORD *Baudrate,WORD *ManagerID);
DMC_API short __stdcall nmc_set_manager_od(WORD CardNo,WORD PortNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD Value);
DMC_API short __stdcall nmc_get_manager_od(WORD CardNo,WORD PortNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD *Value);

DMC_API short __stdcall nmc_set_node_od(WORD CardNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,long Value);
DMC_API short __stdcall nmc_get_node_od(WORD CardNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,long* Value);

DMC_API short __stdcall nmc_set_node_od_pbyte(WORD CardNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD Bytes,unsigned char* Value);
DMC_API short __stdcall nmc_get_node_od_pbyte(WORD CardNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD Bytes,unsigned char*  Value);

DMC_API short __stdcall nmc_upload_configfile(WORD CardNo,WORD PortNum, const char *FileName);
DMC_API short __stdcall nmc_reset_to_factory(WORD CardNo,WORD PortNum,WORD NodeNum);
DMC_API short __stdcall nmc_write_to_pci(WORD CardNo,WORD PortNum,WORD NodeNum);
DMC_API short __stdcall nmc_download_configfile(WORD CardNo,WORD PortNum,const char *FileName);//????ENI???????
DMC_API short __stdcall nmc_download_mapfile(WORD CardNo,const char *FileName);//??????????

//????????????? 255???????
DMC_API short __stdcall nmc_set_axis_enable(WORD CardNo,WORD axis);
DMC_API short __stdcall nmc_set_axis_disable(WORD CardNo,WORD axis);

//???????????
DMC_API short __stdcall nmc_set_alarm_clear(WORD CardNo,WORD PortNum,WORD NodeNum);

DMC_API short __stdcall nmc_get_slave_nodes(WORD CardNo,WORD PortNum,WORD BaudRate,WORD* NodeId,WORD* NodeNum);
//???????????
DMC_API short __stdcall nmc_get_total_axes(WORD CardNo,DWORD* TotalAxis);
//???????ADDA????????????
DMC_API short __stdcall nmc_get_total_adcnum(WORD CardNo,WORD* TotalIn,WORD* TotalOut);
//???????IO????
DMC_API short __stdcall nmc_get_total_ionum(WORD CardNo,WORD *TotalIn,WORD *TotalOut);
//??????????
DMC_API short __stdcall nmc_clear_alarm_fieldbus(WORD CardNo,WORD PortNum);
//???????????????  1???ethercat????0?????????
DMC_API short __stdcall nmc_get_controller_workmode(WORD CardNo,WORD* controller_mode);
//???????????????  1???ethercat????0?????????
DMC_API short __stdcall nmc_set_controller_workmode(WORD CardNo,WORD controller_mode);
//????ethercat???????????(us)
DMC_API short __stdcall nmc_set_cycletime(WORD CardNo,WORD PortNum,DWORD CycleTime);
//???ethercat???????????(us)
DMC_API short __stdcall nmc_get_cycletime(WORD CardNo,WORD PortNum,DWORD* CycleTime);

//????????????????????
DMC_API short __stdcall dmc_get_perline_time(WORD CardNo,WORD TypeIndex,DWORD *Averagetime,DWORD *Maxtime,uint64 *Cycles ); //TypeIndex:0~6  m_Averagetime ; ?????? m_Maxtime;??????? uint64  m_Cycles;??????
DMC_API short __stdcall nmc_set_axis_run_mode(WORD CardNo,WORD axis,WORD run_mode);//?????????????? 1?pp????6?????????8?csp??


//?????????
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
//????????????????????????????????????
DMC_API short __stdcall nmc_get_consume_time_fieldbus(WORD CardNo,WORD PortNum,DWORD* Average_time, DWORD* Max_time,uint64* Cycles);
//?????????
DMC_API short __stdcall nmc_clear_consume_time_fieldbus(WORD CardNo,WORD PortNum);
//??ethercat????,????0????????????????????????
DMC_API short __stdcall nmc_stop_etc(WORD CardNo,WORD* ETCState);

//---------------------------????????---------------
//?????????
DMC_API short __stdcall nmc_get_axis_statusword(WORD CardNo,WORD axis,long* statusword);
//???????????????
DMC_API short __stdcall nmc_set_axis_contrlword(WORD CardNo,WORD Axis,long Contrlword);
//????????????????
DMC_API short __stdcall nmc_get_axis_contrlword(WORD CardNo,WORD Axis,long *Contrlword);
DMC_API short __stdcall nmc_set_axis_contrlmode(WORD CardNo,WORD Axis,long Contrlmode);
DMC_API short __stdcall nmc_get_axis_contrlmode(WORD CardNo,WORD Axis,long *Contrlmode);

// ??????????????
DMC_API short __stdcall nmc_get_errcode(WORD CardNo,WORD channel,WORD *Errcode);
// ??????????????
DMC_API short __stdcall nmc_get_card_errcode(WORD CardNo,WORD *Errcode);
// ???????????????
DMC_API short __stdcall nmc_get_axis_errcode(WORD CardNo,WORD axis,WORD *Errcode);
// ???????????????
DMC_API short __stdcall nmc_clear_errcode(WORD CardNo,WORD channel);
// ???????????????
DMC_API short __stdcall nmc_clear_card_errcode(WORD CardNo);
// ????????????????
DMC_API short __stdcall nmc_clear_axis_errcode(WORD CardNo,WORD iaxis);

DMC_API short __stdcall nmc_get_LostHeartbeat_Nodes(WORD CardNo,WORD PortNum,WORD* NodeID,WORD* NodeNum);
DMC_API short __stdcall nmc_get_EmergeneyMessege_Nodes(WORD CardNo,WORD PortNum,DWORD* NodeMsg,WORD* MsgNum);
DMC_API short __stdcall nmc_SendNmtCommand(WORD CardNo,WORD PortNum,WORD NodeID,WORD NmtCommand);
DMC_API short __stdcall nmc_syn_move(WORD CardNo,WORD AxisNum,WORD* AxisList,long* Position,WORD* PosiMode);
DMC_API short __stdcall nmc_syn_move_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,double* Position,WORD* PosiMode);
//?????????????
DMC_API short __stdcall nmc_sync_pmove_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,double* Dist,WORD* PosiMode);
DMC_API short __stdcall nmc_sync_vmove_unit(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* Dir);

//???????????
DMC_API short __stdcall nmc_set_master_para(WORD CardNo,WORD PortNum,WORD Baudrate,DWORD NodeCnt,WORD MasterId);
//??????????
DMC_API short __stdcall nmc_get_master_para(WORD CardNo,WORD PortNum,WORD *Baudrate,DWORD *NodeCnt,WORD *MasterId);
//????io????
DMC_API short __stdcall nmc_write_outbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD IoValue);
//???io????
DMC_API short __stdcall nmc_read_outbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue);
//???io????
DMC_API short __stdcall nmc_read_inbit(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue);
//????DA????
DMC_API short __stdcall nmc_set_da_output(WORD CardNo,WORD NoteID,WORD channel,double Value);
//???DA????
DMC_API short __stdcall nmc_get_da_output(WORD CardNo,WORD NoteID,WORD channel,double *Value);
//???AD????
DMC_API short __stdcall nmc_get_ad_input(WORD CardNo,WORD NoteID,WORD channel,double *Value);
//????AD??
DMC_API short __stdcall nmc_set_ad_mode(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums);
DMC_API short __stdcall nmc_get_ad_mode(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums);
//????DA??
DMC_API short __stdcall nmc_set_da_mode(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums);
DMC_API short __stdcall nmc_get_da_mode(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums);

//????§Õ??flash
DMC_API short __stdcall nmc_write_to_flash(WORD CardNo,WORD PortNum,WORD NodeNum);

//????????
DMC_API short __stdcall nmc_set_connect_state(WORD CardNo,WORD NodeNum,WORD state,WORD baud);//0-?????1-?????2-??¦Ë?????????
DMC_API short __stdcall nmc_get_connect_state(WORD CardNo,WORD* NodeNum,WORD* state);//0-?????1-?????2-??

//????io????32¦Ë
DMC_API short __stdcall nmc_write_outport(WORD CardNo,WORD NoteID,WORD portno,DWORD outport_val);
//???io????32¦Ë
DMC_API short __stdcall nmc_read_outport(WORD CardNo,WORD NoteID,WORD portno,DWORD *outport_val);
//???io????32¦Ë
DMC_API short __stdcall nmc_read_inport(WORD CardNo,WORD NoteID,WORD portno,DWORD *inport_val);

//??????
DMC_API short __stdcall nmc_get_axis_state_machine(WORD CardNo,WORD axis, WORD* Axis_StateMachine);
//???????????????????????6????????8csp????
DMC_API short __stdcall nmc_get_axis_setting_contrlmode(WORD CardNo,WORD axis,long* contrlmode);
// ??????????
DMC_API short __stdcall nmc_get_total_slaves(WORD CardNo,WORD PortNum,WORD* TotalSlaves);
// ????????????
DMC_API short __stdcall nmc_get_axis_node_address(WORD CardNo,WORD axis, WORD* SlaveAddr,WORD* Sub_SlaveAddr);
DMC_API short __stdcall nmc_set_axis_io_out(WORD CardNo,WORD axis,DWORD  iostate);
DMC_API DWORD __stdcall nmc_get_axis_io_out(WORD CardNo,WORD axis);
DMC_API DWORD __stdcall nmc_get_axis_io_in(WORD CardNo,WORD axis);


/************************************************************
*
*RTEX?????????
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
*?????????RTEX??????§ÕEEPROM????
*
*
**************************************************************/
DMC_API short __stdcall nmc_write_slave_eeprom(WORD CardNo,WORD axis);
/**************************************************************
*index:rtex???????????????
*subindex:rtex????????index??????????????
*para_data:??????????
**************************************************************/
DMC_API short __stdcall nmc_read_parameter(WORD CardNo,WORD axis,WORD index, WORD subindex,DWORD* para_data);
/**************************************************************
*index:rtex???????????????
*subindex:rtex????????index??????????????
*para_data:??????????
**************************************************************/
DMC_API short __stdcall nmc_read_parameter_attributes(WORD CardNo,WORD axis,WORD index, WORD subindex,DWORD* para_data);
DMC_API short __stdcall nmc_set_cmdcycletime(WORD CardNo,WORD PortNum,DWORD cmdtime);
//????RTEX?????????(us)
DMC_API short __stdcall nmc_get_cmdcycletime(WORD CardNo,WORD PortNum,DWORD* cmdtime);
DMC_API short __stdcall nmc_start_log(WORD CardNo,WORD chan,WORD node, WORD Ifenable);
DMC_API short __stdcall nmc_get_log(WORD CardNo,WORD chan,WORD node, DWORD* data);
DMC_API short __stdcall nmc_config_atuo_log(WORD CardNo,WORD ifenable,WORD dir,WORD byte_index,WORD mask,WORD condition,DWORD counter);

//???PDO
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
//????rtex????????????????
DMC_API short __stdcall nmc_clear_abs_driver_multi_cycle(WORD CardNo,WORD axis);
//????io????32¦Ë???????
DMC_API short __stdcall nmc_write_outport_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD portno,DWORD outport_val);
//???io????32¦Ë???????
DMC_API short __stdcall nmc_read_outport_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD portno,DWORD *outport_val);
//???io????32¦Ë???????
DMC_API short __stdcall nmc_read_inport_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD portno,DWORD *inport_val);
//????io????
DMC_API short __stdcall nmc_write_outbit_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD IoBit,WORD IoValue);
//???io????
DMC_API short __stdcall nmc_read_outbit_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD IoBit,WORD *IoValue);
//???io????
DMC_API short __stdcall nmc_read_inbit_extern(WORD CardNo,WORD Channel,WORD NoteID,WORD IoBit,WORD *IoValue);
//??????????????
DMC_API short __stdcall nmc_get_current_fieldbus_state_info(WORD CardNo,WORD Channel,WORD *Axis,WORD *ErrorType,WORD *SlaveAddr,DWORD *ErrorFieldbusCode);
// ?????????????
DMC_API short __stdcall nmc_get_detail_fieldbus_state_info(WORD CardNo,WORD Channel,DWORD ReadErrorNum,DWORD *TotalNum, DWORD *ActualNum, WORD *Axis,WORD *ErrorType,WORD *SlaveAddr,DWORD *ErrorFieldbusCode);

//???????
DMC_API short __stdcall nmc_start_pdo_trace(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD Index_Num,DWORD Trace_Len,WORD *Index,WORD *Sub_Index);
//??????????
DMC_API short __stdcall nmc_get_pdo_trace(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD *Index_Num,DWORD *Trace_Len,WORD *Index,WORD *Sub_Index);
//??????????????
DMC_API short __stdcall nmc_set_pdo_trace_trig_para(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD Trig_Index,WORD Trig_Sub_Index,int Trig_Value,WORD Trig_Mode);
//??????????????
DMC_API short __stdcall nmc_get_pdo_trace_trig_para(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD *Trig_Index,WORD *Trig_Sub_Index,int *Trig_Value,WORD *Trig_Mode);
//???????
DMC_API short __stdcall nmc_clear_pdo_trace_data(WORD CardNo,WORD Channel,WORD SlaveAddr);
//?????
DMC_API short __stdcall nmc_stop_pdo_trace(WORD CardNo,WORD Channel,WORD SlaveAddr);
//?????????
DMC_API short __stdcall nmc_read_pdo_trace_data(WORD CardNo,WORD Channel,WORD SlaveAddr,DWORD StartAddr,DWORD Readlen,DWORD *ActReadlen,BYTE *Data);
//????????
DMC_API short __stdcall nmc_get_pdo_trace_num(WORD CardNo,WORD Channel,WORD SlaveAddr,DWORD *Data_num, DWORD *Size_of_each_bag);
//?????
DMC_API short __stdcall nmc_get_pdo_trace_state(WORD CardNo,WORD Channel,WORD SlaveAddr,WORD *Trace_state);
//???????
DMC_API short __stdcall nmc_reset_canopen(WORD CardNo);
DMC_API short __stdcall nmc_reset_rtex(WORD CardNo);
DMC_API short __stdcall nmc_reset_etc(WORD CardNo);
//???????????????
DMC_API short __stdcall nmc_set_fieldbus_error_switch(WORD CardNo, WORD channel,WORD data);
DMC_API short __stdcall nmc_get_fieldbus_error_switch(WORD CardNo, WORD channel,WORD* data);

DMC_API short __stdcall nmc_torque_move(WORD CardNo,WORD axis,int Torque,WORD PosLimitValid,double PosLimitValue,WORD PosMode);
DMC_API short __stdcall nmc_change_torque(WORD CardNo,WORD axis,int Torque);
//???????§³
DMC_API short __stdcall nmc_get_torque(WORD CardNo,WORD axis,int* Torque);
//modbus????
DMC_API short __stdcall dmc_modbus_active_COM1(WORD id,const char* COMID,int speed, int bits, int check, int stop);
DMC_API short __stdcall dmc_modbus_active_COM2(WORD id,const char* COMID,int speed, int bits, int check, int stop);
DMC_API short __stdcall dmc_modbus_active_ETH(WORD id, WORD port);

DMC_API short __stdcall dmc_set_modbus_0x(WORD CardNo, WORD start, WORD inum, const char* pdata);
DMC_API short __stdcall dmc_get_modbus_0x(WORD CardNo, WORD start, WORD inum, char* pdata);
DMC_API short __stdcall dmc_set_modbus_4x(WORD CardNo, WORD start, WORD inum, const WORD* pdata);
DMC_API short __stdcall dmc_get_modbus_4x(WORD CardNo, WORD start, WORD inum, WORD* pdata);

DMC_API short __stdcall dmc_set_modbus_4x_float(WORD CardNo, WORD start, WORD inum, const float* pdata);
DMC_API short __stdcall dmc_get_modbus_4x_float(WORD CardNo, WORD start, WORD inum, float* pdata);
DMC_API short __stdcall dmc_set_modbus_4x_int(WORD CardNo, WORD start, WORD inum, const int* pdata);
DMC_API short __stdcall dmc_get_modbus_4x_int(WORD CardNo, WORD start, WORD inum, int* pdata);

DMC_API short __stdcall dmc_set_grant_error_protect_unit(WORD CardNo, WORD axis,WORD enable,double dstp_error, double emg_error);
DMC_API short __stdcall dmc_get_grant_error_protect_unit(WORD CardNo, WORD axis,WORD* enable,double* dstp_error, double* emg_error);
//?????????
DMC_API short __stdcall dmc_get_leadscrew_comp_config(WORD CardNo, WORD axis,WORD *n, int *startpos,int *lenpos,int *pCompPos,int *pCompNeg);
DMC_API short __stdcall dmc_set_leadscrew_comp_config_unit(WORD CardNo, WORD axis,WORD n, double startpos,double lenpos,double *pCompPos,double *pCompNeg);
DMC_API short __stdcall dmc_get_leadscrew_comp_config_unit(WORD CardNo, WORD axis,WORD *n, double *startpos,double *lenpos,double *pCompPos,double *pCompNeg);
//EZ???? ?????????????????
DMC_API short __stdcall dmc_get_homelatch_value_unit(WORD CardNo,WORD axis, double* pos);
DMC_API short __stdcall dmc_get_ezlatch_value_unit(WORD CardNo,WORD axis, double* pos);
//????????
DMC_API short __stdcall dmc_get_latch_value_extern_unit(WORD CardNo,WORD axis,WORD index,double* pos_by_mm);//??????????? ¦Ä????
//?????
DMC_API short __stdcall dmc_compare_add_point_unit(WORD CardNo,WORD cmp,double pos,WORD dir, WORD action,DWORD actpara);//???????
DMC_API short __stdcall dmc_compare_get_current_point_unit(WORD CardNo,WORD cmp,double *pos);//??????????
//????¦Ë????
DMC_API short __stdcall dmc_compare_add_point_multi_unit(WORD CardNo, WORD cmp,double pos, WORD dir,  WORD action, DWORD actpara,double times);//??????? ???
//????¦Ë????
DMC_API short __stdcall dmc_hcmp_add_point_unit(WORD CardNo,WORD hcmp, double cmp_pos);
DMC_API short __stdcall dmc_hcmp_set_liner_unit(WORD CardNo,WORD hcmp, double Increment,long Count);//??????????????
DMC_API short __stdcall dmc_hcmp_get_liner_unit(WORD CardNo,WORD hcmp, double* Increment,long* Count);
DMC_API short __stdcall dmc_hcmp_get_current_state_unit(WORD CardNo,WORD hcmp,long *remained_points,double *current_point,long *runned_points); //???????????
DMC_API short __stdcall dmc_set_softlimit_unit(WORD CardNo,WORD axis,WORD enable, WORD source_sel,WORD SL_action, double N_limit,double P_limit);//????????¦Ë????
DMC_API short __stdcall dmc_get_softlimit_unit(WORD CardNo,WORD axis,WORD *enable, WORD *source_sel,WORD *SL_action,double *N_limit,double *P_limit);//???????¦Ë????

//????¦Ë???????????????
DMC_API short __stdcall dmc_hcmp_set_config_overlap(WORD CardNo, WORD hcmp, WORD axis, WORD cmp_source, WORD cmp_logic, long time, WORD axis_num, WORD aux_axis, WORD aux_source);
DMC_API short __stdcall dmc_hcmp_get_config_overlap(WORD CardNo, WORD hcmp, WORD* axis, WORD* cmp_source, WORD* cmp_logic, long* time, WORD* axis_num, WORD* aux_axis, WORD* aux_source);

//?¡ã??????
DMC_API short __stdcall	dmc_m_move_set_coodinate(WORD card,WORD liner, WORD axis_num, WORD* axis_list,uint32 in_io_num, WORD trig_flag, WORD pos_mode);
DMC_API short __stdcall	dmc_m_move_get_coodinate_para(WORD card,WORD liner, WORD* axis_num, WORD* axis_list,uint32* in_io_num, WORD* trig_flag, WORD* pos_mode);
DMC_API short __stdcall	dmc_m_move_set_z_para(WORD card,WORD liner, double z_up_safe_pos, double z_up_target_pos,double z_down_safe_pos, double z_down_target_pos);
DMC_API short __stdcall	dmc_m_move_get_z_para(WORD card,WORD liner, double* z_up_safe_pos, double* z_up_target_pos,double* z_down_safe_pos, double* z_down_target_pos);
DMC_API short __stdcall	dmc_m_move_set_xy_para(WORD card,WORD liner, double x_first_safe_pos,double x_second_safe_pos, double x_target_pos, WORD y_num ,double* y_target_pos);
DMC_API short __stdcall	dmc_m_move_get_xy_para(WORD card,WORD liner, double* x_first_safe_pos,double* x_second_safe_pos, double* x_target_pos, WORD y_num ,double* y_target_pos);
DMC_API short __stdcall	dmc_m_move_axes(WORD card,WORD liner);
DMC_API short __stdcall	dmc_m_move_get_run_phase(WORD card,WORD liner, WORD* run_phase);
DMC_API short __stdcall	dmc_m_move_stop(WORD card,WORD liner, WORD stop_mode);
DMC_API short __stdcall dmc_t_pmove_extern_unit(WORD CardNo, WORD axis, double MidPos,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,WORD posi_mode);

DMC_API short __stdcall dmc_rtcp_set_kinematic_type(WORD CardNo,WORD Crd, WORD machine_type);
DMC_API short __stdcall dmc_rtcp_get_kinematic_type(WORD CardNo,WORD Crd, WORD* machine_type);
//??????????RTCP????
DMC_API short __stdcall dmc_rtcp_set_enable(WORD CardNo,WORD Crd, WORD enable);
DMC_API short __stdcall dmc_rtcp_get_enable(WORD CardNo,WORD Crd, WORD* enable);
//¦Ë???????????????0-?????????¦Ë???1-??§Ö?????¦Ë??
DMC_API short __stdcall dmc_rtcp_set_position_type(WORD CardNo,WORD Crd, WORD position_type);
DMC_API short __stdcall dmc_rtcp_get_position_type(WORD CardNo,WORD Crd, WORD* position_type);
//????A???????????????????????????????, xyz?????a_offset[3]
DMC_API short __stdcall dmc_rtcp_set_vector_a(WORD CardNo,WORD Crd, double* a_offset);
DMC_API short __stdcall dmc_rtcp_get_vector_a(WORD CardNo,WORD Crd, double* a_offset);
//????B???????????????????????????????, xyz?????b_offset[3]
DMC_API short __stdcall dmc_rtcp_set_vector_b(WORD CardNo,WORD Crd, double* b_offset);
DMC_API short __stdcall dmc_rtcp_get_vector_b(WORD CardNo,WORD Crd, double* b_offset);
//????C???????????????????????????????, xyz?????c_offset[3]
DMC_API short __stdcall dmc_rtcp_set_vector_c(WORD CardNo,WORD Crd, double* c_offset);
DMC_API short __stdcall dmc_rtcp_get_vector_c(WORD CardNo,WORD Crd, double* c_offset);
//????A??B??C???????¦Ë??,
//A,B,C???0???????????????????¦Ë????????????A/B/C????????
//???????????????¦Ë?¨°???????0?????????????????????
DMC_API short __stdcall dmc_rtcp_set_rotate_joint_offset(WORD CardNo,WORD Crd, double A, double B, double C);
DMC_API short __stdcall dmc_rtcp_get_rotate_joint_offset(WORD CardNo,WORD Crd, double* A, double* B, double* C);
//?????????????????????????
//???????????????????????????????????????????????????????????????????????1
//???????-1??????????1
//dir[5],???????X,Y,Z,?????1???????2
DMC_API short __stdcall dmc_rtcp_set_joints_direction(WORD CardNo,WORD Crd, int* dir);
DMC_API short __stdcall dmc_rtcp_get_joints_direction(WORD CardNo,WORD Crd, int* dir);
//?????????????????????????
DMC_API short __stdcall dmc_rtcp_set_tool_length(WORD CardNo,WORD Crd, double tool);
DMC_API short __stdcall dmc_rtcp_get_tool_length(WORD CardNo,WORD Crd, double* tool);

DMC_API short __stdcall dmc_rtcp_get_actual_pos(WORD CardNo,WORD Crd, WORD AxisNum,WORD *AxisList,double* actual_pos);
DMC_API short __stdcall dmc_rtcp_get_command_pos(WORD CardNo,WORD Crd, WORD AxisNum,WORD *AxisList,double* command_pos);

DMC_API short __stdcall dmc_rtcp_kinematics_forward(WORD CardNo,WORD Crd, double* joint_pos, double* axis_pos);
DMC_API short __stdcall dmc_rtcp_kinematics_inverse(WORD CardNo,WORD Crd, double* axis_pos, double* joint_pos);
//???????????????
DMC_API short __stdcall dmc_rtcp_set_max_rotate_param(WORD CardNo,WORD Crd, double rotate_speed, double rotate_acc);
DMC_API short __stdcall dmc_rtcp_get_max_rotate_param(WORD CardNo,WORD Crd, double* rotate_speed, double* rotate_acc);
//???¨´??????????
DMC_API short __stdcall dmc_rtcp_set_workpiece_offset(WORD CardNo,WORD Crd, WORD workpiece_index, double* offset);
DMC_API short __stdcall dmc_rtcp_get_workpiece_offset(WORD CardNo,WORD Crd, WORD workpiece_index, double* offset);
//???¨´????????¡Â??
DMC_API short __stdcall dmc_rtcp_set_workpiece_mode(WORD CardNo,WORD Crd, WORD enable,WORD workpiece_index);
DMC_API short __stdcall dmc_rtcp_get_workpiece_mode(WORD CardNo,WORD Crd, WORD* enable,WORD* workpiece_index);

//??????
DMC_API short __stdcall dmc_conti_helix_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD * AixsList,double * StartPos,double * TargetPos,WORD Arc_Dir,int Circle,WORD mode,int mark);
DMC_API short __stdcall dmc_helix_move_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double * StartPos,double *TargetPos,WORD Arc_Dir,int Circle,WORD mode);

DMC_API short __stdcall dmc_compare_add_point_cycle_unit(WORD CardNo,WORD cmp,double pos,WORD dir, DWORD bitno,DWORD cycle,WORD level);//???????
//PDO????20190715
DMC_API short __stdcall dmc_pdo_buffer_enter(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_pdo_buffer_stop(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_pdo_buffer_clear(WORD CardNo,WORD axis);

DMC_API short __stdcall dmc_pdo_buffer_run_state(WORD CardNo,WORD axis,int * RunState,int * Remain,int * NotRunned,int* Runned);
DMC_API short __stdcall dmc_pdo_buffer_add_data(WORD CardNo,WORD axis, int size, int* data_table);

DMC_API short __stdcall dmc_pdo_buffer_start_multi(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* ResultList);
DMC_API short __stdcall dmc_pdo_buffer_pause_multi(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* ResultList);
DMC_API short __stdcall dmc_pdo_buffer_stop_multi(WORD CardNo,WORD AxisNum,WORD* AxisList,WORD* ResultList);
DMC_API short __stdcall dmc_pdo_buffer_add_data_multi(WORD CardNo, WORD AxisNum,WORD* AxisList, int size, int** data_table);
DMC_API short __stdcall dmc_calculate_arccenter_3point(double *start_pos,double *mid_pos, double *target_pos,double* cen_pos);

DMC_API short __stdcall	dmc_cmd_buf_open(WORD card,WORD group, WORD axis_num, WORD* axis_list);
DMC_API short __stdcall	dmc_cmd_buf_close(WORD card,WORD group);
DMC_API short __stdcall	dmc_cmd_buf_start(WORD card,WORD group);
DMC_API short __stdcall	dmc_cmd_buf_stop(WORD card,WORD group, WORD stop_mode);
DMC_API short __stdcall	dmc_cmd_buf_get_group_config(WORD card,WORD group, WORD* enable ,WORD* axis_num, WORD* axis_list);
DMC_API short __stdcall	dmc_cmd_buf_get_group_run_info(WORD card,WORD group, WORD* enable ,DWORD* array_id, DWORD* stop_reason,WORD* trig_phase ,WORD* phase);
DMC_API short __stdcall	dmc_cmd_buf_add_cmd(WORD card, WORD group, DWORD index, WORD  cmd_type, WORD  ProcessMode, WORD Dim,
    WORD* AxisList, double* TargetPositionFirst, double* m_TargetPositionSecond,WORD*  m_SoftlandFlag,double* SoftLandVel,double* SoftLandEndVel, WORD*  m_PosMode,double* m_TrigAheadPos,
    WORD  m_TrigFlag,  WORD m_TrigAxisNum,WORD* m_TrigAxislist,WORD*  m_TrigPosType,WORD*  m_TrigAxisPosRelationgship,double* m_TrigAxisPos,
    WORD m_TrigIONum, WORD*  m_TrigIOState,WORD* m_TrigINIOList,
    DWORD m_DelayCmdTime, WORD m_IOList, WORD  m_IOState,WORD m_TrigError
    );
DMC_API short __stdcall	dmc_cmd_buf_set_axis_profile(WORD card,WORD group, WORD axis_num, WORD* axis_list, double* start_vel, double* max_vel, double* stop_vel, double* tacc, double* tdec);

DMC_API short __stdcall dmc_m_set_muti_profile_unit(WORD CardNo, WORD group, WORD axis_num, WORD* axis_list, double* start_vel, double* max_vel, double* tacc, double* tdec, double* stop_vel);
DMC_API short __stdcall dmc_m_set_profile_unit(WORD CardNo, WORD group, WORD axis, double start_vel, double max_vel, double tacc, double tdec, double stop_vel);
DMC_API short __stdcall dmc_m_add_sigaxis_moveseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, WORD process_mode, DWORD mark);
DMC_API short __stdcall dmc_m_add_sigaxis_move_twoseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD process_mode, DWORD mark);
DMC_API short __stdcall dmc_m_add_mutiaxis_moveseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axis_list, double* Target_pos, WORD process_mode, DWORD mark);
DMC_API short __stdcall dmc_m_add_mutiaxis_move_twoseg_data(WORD CardNo, WORD group, WORD axisnum,  WORD* axis_list, double* Target_pos, double* softland_pos, double* softland_vel, double* softland_endvel, WORD process_mode, DWORD mark);
DMC_API short __stdcall dmc_m_add_ioTrig_movseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, WORD process_mode, WORD trigINbit, WORD trigINstate, DWORD mark);//io???????
DMC_API short __stdcall dmc_m_add_mutiposTrig_movseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, WORD process_mode, WORD trigaxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigPosMode, DWORD mark);//¦Ë????????
DMC_API short __stdcall dmc_m_add_mutiposTrig_mov_twoseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD process_mode, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigPosMode, DWORD mark);//????¦Ë????????
DMC_API short __stdcall dmc_m_add_upseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, DWORD mark);
DMC_API short __stdcall dmc_m_add_up_twoseg_data(WORD CardNo, WORD group, WORD axis, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, DWORD mark);
DMC_API short __stdcall dmc_m_add_ioPosTrig_movseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, WORD process_mode, WORD trigAxis,double trigPos, WORD trigPosType, WORD trigMode, WORD TrigINNum, WORD* trigINList, WORD* trigINstate, DWORD mark);//¦Ë??+io???????
DMC_API short __stdcall dmc_m_add_ioPosTrig_mov_twoseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, double* softland_pos, double* softland_vel, double* softland_endvel,WORD process_mode, WORD trigAxis, double trigPos, WORD trigPosType, WORD trigMode, WORD TrigINNum, WORD* trigINList, WORD* trigINstate, DWORD mark);//¦Ë??+io???????
DMC_API short __stdcall dmc_m_add_posTrig_movseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, WORD process_mode, WORD trigAxis, double trigPos, WORD trigPosType, WORD trigMode, DWORD mark);//¦Ë????????
DMC_API short __stdcall dmc_m_add_posTrig_mov_twoseg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axisList, double* Target_pos, double* softland_pos, double* softland_vel, double* softland_endvel, WORD process_mode, WORD trigAxis, double trigPos, WORD trigPosType, WORD trigMode, DWORD mark);//¦Ë????????
DMC_API short __stdcall dmc_m_add_ioPosTrig_down_seg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, WORD trigIN, WORD trigINstate, DWORD mark);
DMC_API short __stdcall dmc_m_add_ioPosTrig_down_twoseg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, WORD trigIN, WORD trigINstate, DWORD mark);
DMC_API short __stdcall dmc_m_add_posTrig_down_seg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);
DMC_API short __stdcall dmc_m_add_posTrig_down_twoseg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, double softland_pos, double softland_vel, double softland_endvel, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);
DMC_API short __stdcall dmc_m_add_posTrig_torque_movseg_data(WORD CardNo, WORD group, WORD axis, double torque, double PosLimitValue, WORD PosLimitValid, WORD PosMode, WORD trigAxis, double trigPos, WORD trigPosType, WORD trigMode, DWORD mark);//¦Ë???????????

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

DMC_API short __stdcall dmc_get_ad_input_all(WORD CardNo, double* Vout);
DMC_API short __stdcall dmc_conti_pmove_unit_pausemode(WORD CardNo, WORD axis,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,double smooth_time,WORD posi_mode);
DMC_API short __stdcall dmc_conti_return_pausemode(WORD CardNo, WORD Crd, WORD axis);
DMC_API short __stdcall	dmc_cmd_buf_temp_stop(WORD CardNo,WORD group,WORD stop_mode);
DMC_API short __stdcall dmc_check_if_crc_support(WORD CardNo);
//??????da????
DMC_API short __stdcall dmc_set_encoder_da_follow_enable(WORD CardNo,WORD axis,WORD enable);
DMC_API short __stdcall dmc_get_encoder_da_follow_enable(WORD CardNo,WORD axis,WORD* enable);

//?????????????
DMC_API short __stdcall dmc_set_axis_conflict_config(WORD CardNo, WORD*  axis_list, WORD* axis_depart_dir, double home_dist, double conflict_dist, WORD stop_mode);
DMC_API short __stdcall dmc_get_axis_conflict_config (WORD CardNo, WORD*  axis_list, WORD* axis_depart_dir, double* home_dist, double* conflict_dist, WORD* stop_mode);
DMC_API short __stdcall dmc_axis_conflict_config_en(WORD CardNo, WORD enable);
DMC_API short __stdcall dmc_get_axis_conflict_config_en(WORD CardNo, WORD* enable);

//trig_num ??????????trig_pos ????¦Ë??
DMC_API short __stdcall dmc_get_pmove_change_pos_speed_state(WORD CardNo,WORD axis, WORD * trig_num, double * trig_pos);

//????????????????????????
DMC_API short __stdcall dmc_read_inbit_ex(WORD CardNo,WORD bitno,WORD *state);//????????????
DMC_API short __stdcall dmc_read_outbit_ex(WORD CardNo,WORD bitno,WORD *state);//????????????
DMC_API short __stdcall dmc_read_inport_ex(WORD CardNo,WORD portno,DWORD *state);//????????????
DMC_API short __stdcall dmc_read_outport_ex(WORD CardNo,WORD portno,DWORD *state);//????????????
//???????????
//????io????
DMC_API short __stdcall nmc_write_outbit_ex(WORD CardNo,WORD NoteID,WORD IoBit,WORD IoValue,WORD* state);
//???io????
DMC_API short __stdcall nmc_read_outbit_ex(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue,WORD* state);
//???io????
DMC_API short __stdcall nmc_read_inbit_ex(WORD CardNo,WORD NoteID,WORD IoBit,WORD *IoValue,WORD* state);
//????io????32¦Ë
DMC_API short __stdcall nmc_write_outport_ex(WORD CardNo,WORD NoteID,WORD portno,DWORD outport_val,WORD* state);
//???io????32¦Ë
DMC_API short __stdcall nmc_read_outport_ex(WORD CardNo,WORD NoteID,WORD portno,DWORD *outport_val,WORD* state);
//???io????32¦Ë
DMC_API short __stdcall nmc_read_inport_ex(WORD CardNo,WORD NoteID,WORD portno,DWORD *inport_val,WORD* state);

//????DA????
DMC_API short __stdcall nmc_set_da_output_ex(WORD CardNo,WORD NoteID,WORD channel,double Value,WORD* state);
//???DA????
DMC_API short __stdcall nmc_get_da_output_ex(WORD CardNo,WORD NoteID,WORD channel,double *Value,WORD* state);
//???AD????
DMC_API short __stdcall nmc_get_ad_input_ex(WORD CardNo,WORD NoteID,WORD channel,double *Value,WORD* state);
//????AD??
DMC_API short __stdcall nmc_set_ad_mode_ex(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums,WORD* state);
DMC_API short __stdcall nmc_get_ad_mode_ex(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums,WORD* state);
//????DA??
DMC_API short __stdcall nmc_set_da_mode_ex(WORD CardNo,WORD NoteID,WORD channel,WORD mode,DWORD buffer_nums,WORD* state);
DMC_API short __stdcall nmc_get_da_mode_ex(WORD CardNo,WORD NoteID,WORD channel,WORD* mode,DWORD buffer_nums,WORD* state);
//????§Õ??flash
DMC_API short __stdcall nmc_write_to_flash_ex(WORD CardNo,WORD PortNum,WORD NodeNum,WORD* state);

//????????????
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
//?????????????????
DMC_API short __stdcall dmc_get_sortdev_blow_cmd_cnt(WORD CardNo, WORD blowDevNum, long* cnt);
//???¦Ä???????????????????
DMC_API short __stdcall dmc_get_sortdev_blow_cmderr_cnt(WORD CardNo, WORD blowDevNum, long* errCnt);
//?????????
DMC_API short __stdcall dmc_get_sortqueue_status(WORD CardNo, long * curSorQueueLen, long* passCamWithNoCmd);

// ?????????
DMC_API short __stdcall dmc_conti_ellipse_move_unit(WORD CardNo, WORD Crd,WORD AxisNum, WORD* AxisList, double* Target_Pos, double* Cen_Pos, double A_Axis_Len, double B_Axis_Len, WORD Dir, WORD Pos_Mode,long mark);
//???????????
DMC_API	short dmc_get_axis_status_advance(WORD CardNo, WORD axis_no, WORD motion_no, WORD* axis_plan_state, DWORD* ErrPlulseCnt, WORD* fpga_busy);
//??????vmove
DMC_API	short __stdcall dmc_conti_vmove_unit(WORD CardNo, WORD Crd, WORD axis, double vel, double acc ,WORD dir, long imark);
DMC_API	short __stdcall dmc_conti_vmove_stop(WORD CardNo, WORD Crd, WORD axis, double dec, long imark);

//????sn20191101
DMC_API short __stdcall dmc_enter_password_ex(WORD CardNo, const char * str_pass);

//??????????
DMC_API	short __stdcall dmc_gear_in(WORD CardNo, WORD master_axis, WORD slave_axis, WORD follow_source, double ratio_numerator, double ratio_denominator, double acc, double dec, double s_time, WORD buffer_mode);
DMC_API	short __stdcall dmc_get_gear_in(WORD CardNo, WORD* master_axis, WORD slave_axis, WORD* follow_source, double* ratio_numerator, double* ratio_denominator, double* acc, double* dec, double* s_time, WORD* buffer_mode);
DMC_API	short __stdcall dmc_update_gear_scale(WORD CardNo, WORD slave_axis, double ratio_numerator, double ratio_denominator, double acc, double dec,double s_time);
DMC_API	short __stdcall dmc_gear_in_pos(WORD CardNo, WORD master_axis,WORD slave_axis,WORD follow_source,double ratio_numerator,double ratio_denominator,double master_sync_pos,double slave_sync_pos,double master_start_dist,double velocity,double acc,double dec,double s_time, WORD buffer_mode);
DMC_API	short __stdcall dmc_get_gear_in_pos(WORD CardNo, WORD* master_axis,WORD slave_axis,WORD* follow_source,double* ratio_numerator,double* ratio_denominator,double* master_sync_pos,double* slave_sync_pos,double* master_start_dist,double* velocity,double* acc,double* dec,double* s_time, WORD* buffer_mode);
DMC_API	short __stdcall dmc_get_in_gear_state(WORD CardNo, WORD slave_axis,WORD* in_gear);
DMC_API	short __stdcall dmc_get_gear_aborted_state(WORD CardNo, WORD slave_axis,WORD* aborted_state);
DMC_API	short __stdcall dmc_gear_out(WORD CardNo, WORD slave_axis);
DMC_API short __stdcall dmc_trace_set_config(WORD CardNo, short trace_cycle, short lost_handle, short trace_type, short trigger_object_index, short trigger_type, int mask, long long condition);
DMC_API short __stdcall dmc_trace_get_config(WORD CardNo, short * trace_cycle, short * lost_handle, short * trace_type, short * trigger_object_index, short * trigger_type, int * mask, long long * condition);

/***********************************************************
 * ???¨°??????????¦Ï???????500?????????
 * data_type 	????????????????????????
 * data_index 	????????????????????????????????????????????IO??????IO?????????????
 * data_sub_index ?????????????????????????IO???????IO???????????
 * data_bytes 	??????????????§Ó?????????????????0????????????????
 **********************************************************/
DMC_API short __stdcall dmc_trace_reset_config_object(WORD CardNo);
DMC_API short __stdcall dmc_trace_add_config_object(WORD CardNo, short data_type, int data_index, int data_sub_index, short slave_id, short data_bytes);
DMC_API short __stdcall dmc_trace_get_config_object(WORD CardNo,short object_index, short * data_type, int * data_index, int * data_sub_index, short * slave_id, short * data_bytes);

//????trace
DMC_API short __stdcall dmc_trace_data_start(WORD CardNo);

//??trace
DMC_API short __stdcall dmc_trace_data_stop(WORD CardNo);

//??¦Ëtrace????????????????????????????????????????????????¦Ë
DMC_API short __stdcall dmc_trace_data_reset(WORD CardNo);


//trace??????????
DMC_API short __stdcall dmc_trace_get_flag(WORD CardNo,short * start_flag,short * triggered_flag,short * lost_flag);

/***********************************************************
 *????????
 * valid_num 	??????¦Ä??????????????
 * free_num 	???????????????????????
 * object_total_bytes   ??????????????
 * object_total_num 	????????????
 **********************************************************/
DMC_API short __stdcall dmc_trace_get_state(WORD CardNo,int* valid_num,int* free_num,int* object_total_bytes,int* object_total_num);

/***********************************************************
 *??????????
 * bufsize 	??????????????
 * data[1024] 	???????????
 * byte_size   ???????????????
 **********************************************************/
DMC_API short __stdcall dmc_trace_get_data(WORD CardNo,int bufsize,unsigned char* data, int* byte_size);

//trace??¦Ë????????????¦Ë???¦Ë
DMC_API short __stdcall dmc_trace_reset_lost_flag(WORD CardNo);
DMC_API short __stdcall dmc_message_buffer_enable(WORD CardNo,WORD index, DWORD buffer_size,  BYTE console_enable);
DMC_API short __stdcall dmc_message_buffer_disable(WORD CardNo,WORD index);
DMC_API short __stdcall dmc_message_buffer_get_data (WORD CardNo,WORD index, long bufsize, BYTE* data,DWORD *pbufsize);

DMC_API short __stdcall dmc_t_pmove_extern_softstart(WORD CardNo, WORD axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, DWORD delay_ms, double Max_Vel2,double stop_vel2, double acc_time, double dec_time, WORD posi_mode);

DMC_API short __stdcall dmc_t_pmove_extern_softstart_unit(WORD CardNo, WORD axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, DWORD delay_ms, double Max_Vel2,double stop_vel2, double acc_time, double dec_time, WORD posi_mode);


//PVT_continuous???.
//?¡¤?PVT continuous?????????
DMC_API short __stdcall dmc_pvt_table_continuous(WORD CardNo, WORD axis, DWORD count, double* pos, double* vel, double* percent, double* vel_max, double* acc, double* dec);
//?????????????????????¦Ë?????????
DMC_API short __stdcall dmc_pvt_continuous_calculate(WORD CardNo, WORD axis, double* time);
//????PVT continuous ???
DMC_API short __stdcall dmc_pvt_continuous_start(WORD CardNo, WORD axis_num, WORD* axis_list,double* start_delay_time);

//?????¦Ë???????????????
DMC_API short __stdcall nmc_set_slave_output_retain(WORD CardNo,WORD Enable);
DMC_API short __stdcall nmc_get_slave_output_retain(WORD CardNo,WORD * Enable);

DMC_API short __stdcall dmc_m_add_mutiposTrig_singledown_seg_data(WORD CardNo, WORD group, WORD axis, double safePos, double Target_pos, WORD process_mode, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);
DMC_API short __stdcall dmc_m_add_mutiposTrig_mutidown_seg_data(WORD CardNo, WORD group, WORD axisnum, WORD* axis_list, double* safePos, double* Target_pos, WORD process_mode, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);

//????¦Ë????????????¦¶????
DMC_API short __stdcall dmc_cmd_buf_set_allow_error(WORD CardNo, WORD group, double allow_error);
DMC_API short __stdcall dmc_cmd_buf_get_allow_error(WORD CardNo, WORD group, double* allow_error);
//??????????
DMC_API short __stdcall dmc_arc_move_radius_multicoor(WORD CardNo, WORD Crd, WORD* AxisList, double *Target_Pos, double Arc_Radius, WORD Arc_Dir, long Circle, WORD posi_mode);
//???????
DMC_API short __stdcall dmc_arc_move_3points_multicoor(WORD CardNo,WORD Crd,WORD* AxisList, double *Target_Pos, double *Mid_Pos, long Circle, WORD posi_mode);

//????¦Ë????????????¦¶????
DMC_API short __stdcall dmc_m_set_encoder_error_allow(WORD CardNo, WORD group, double allow_error);
DMC_API short __stdcall dmc_m_get_encoder_error_allow(WORD CardNo, WORD group, double* allow_error);

//??????
DMC_API short __stdcall dmc_set_persistent_reg_byte(WORD CardNo, DWORD start, DWORD inum, const char* pdata);
DMC_API short __stdcall dmc_get_persistent_reg_byte(WORD CardNo, DWORD start, DWORD inum, char* pdata);
DMC_API short __stdcall dmc_set_persistent_reg_float(WORD CardNo, DWORD start, DWORD inum, const float* pdata);
DMC_API short __stdcall dmc_get_persistent_reg_float(WORD CardNo, DWORD start, DWORD inum, float* pdata);
DMC_API short __stdcall dmc_set_persistent_reg_int(WORD CardNo, DWORD start, DWORD inum, const int* pdata);
DMC_API short __stdcall dmc_get_persistent_reg_int(WORD CardNo, DWORD start, DWORD inum, int* pdata);

DMC_API short __stdcall nmc_torque_set_delay_cycle(WORD CardNo,WORD axis,int delay_cycle);

DMC_API short __stdcall dmc_conti_delay_outbit_to_start_path(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_delay_outbit_to_stop_path(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double delay_time,double ReverseTime);
DMC_API short __stdcall dmc_conti_ahead_outbit_to_stop_path(WORD CardNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);

//??????????§Õflash???
DMC_API short __stdcall dmc_set_persistent_param_config(WORD CardNo, WORD axis, DWORD item);
DMC_API short __stdcall dmc_get_persistent_param_config(WORD CardNo, WORD axis, DWORD* item);

DMC_API short __stdcall dmc_hcmp_fifo_add_point_dir_unit(WORD CardNo,WORD hcmp, WORD num,double *cmp_pos,DWORD dir);
DMC_API short __stdcall dmc_hcmp_fifo_add_table_dir(WORD CardNo,WORD hcmp, WORD num,double *cmp_pos,DWORD dir);

DMC_API short __stdcall dmc_axis_io_status_ex(WORD CardNo,WORD axis,DWORD *state);//?????????§Û??????????
DMC_API short __stdcall dmc_check_done_ex(WORD CardNo,WORD axis,WORD * state);//???????????????

DMC_API short __stdcall dmc_conti_line_section_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,double Section_Dist,WORD Bitno,WORD On_Off,WORD Io_Mode,double Time_Dist_Value,double ReverseTime,WORD posi_mode,WORD mark);

DMC_API short __stdcall dmc_conti_arc_move_center_section_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Cen_Pos,WORD Arc_Dir,WORD Circle,double Section_Dist,WORD Bitno,WORD On_Off,WORD Io_Mode,double Time_Dist_Value,double ReverseTime,WORD posi_mode,WORD mark);

DMC_API short __stdcall dmc_conti_arc_move_radius_section_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double Arc_Radius,WORD Arc_Dir,WORD Circle,double Section_Dist,WORD Bitno,WORD On_Off,WORD Io_Mode,double Time_Dist_Value,double ReverseTime,WORD posi_mode,WORD mark);

DMC_API short __stdcall dmc_conti_arc_move_3points_section_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mid_Pos,WORD Circle,double Section_Dist,WORD Bitno,WORD On_Off,WORD Io_Mode,double Time_Dist_Value,double ReverseTime,WORD posi_mode,WORD mark);

DMC_API short __stdcall dmc_get_firmware_boot_type(WORD CardNo,WORD* boot_type);



//?§Ø????
DMC_API DWORD __stdcall dmc_int_enable(WORD CardNo, DMC3K5K_OPERATE funcIntHandler, void* operate_data);
DMC_API DWORD __stdcall dmc_int_disable(WORD CardNo);
DMC_API short __stdcall dmc_set_intmode_enable(WORD Cardno,WORD Intno,WORD Enable);
DMC_API short __stdcall dmc_get_intmode_enable(WORD Cardno,WORD Intno,WORD *Status);
DMC_API short __stdcall dmc_set_intmode_config(WORD Cardno,WORD Intno,WORD IntItem,WORD IntIndex,WORD IntSubIndex,WORD Logic);
DMC_API	short __stdcall dmc_get_intmode_config(WORD Cardno,WORD Intno,WORD *IntItem,WORD *IntIndex,WORD *IntSubIndex,WORD*Logic);
DMC_API short __stdcall dmc_get_int_status(WORD Cardno,DWORD *IntStatus);
DMC_API short __stdcall dmc_reset_int_status(WORD Cardno, WORD Intno);

DMC_API short __stdcall dmc_set_pci_int(WORD CardNo);
DMC_API short __stdcall dmc_pmove_change_pos_speed_inbit(WORD CardNo,WORD axis, WORD inbit, WORD enable);
DMC_API short __stdcall dmc_get_pmove_change_pos_speed_inbit(WORD CardNo,WORD axis, WORD * inbit, WORD* enable);

DMC_API short __stdcall dmc_set_arc_zone_limit_config_unit(WORD CardNo, WORD* AxisList, WORD AxisNum, double *Center, double Radius,WORD Source, WORD StopMode);
DMC_API short __stdcall dmc_get_arc_zone_limit_config_unit(WORD CardNo, WORD* AxisList, WORD* AxisNum, double *Center, double* Radius,WORD* Source, WORD* StopMode);

DMC_API short __stdcall dmc_set_latch_stop_axis(WORD CardNo, WORD latch, WORD  num,  WORD * axislist);
DMC_API short __stdcall dmc_get_latch_stop_axis(WORD CardNo, WORD latch, WORD * num,  WORD * axislist);
DMC_API short __stdcall dmc_compare_add_point_cycle_2d(WORD ConnectNo,WORD* axis,double* pos,WORD* dir, DWORD bitno,DWORD cycle,WORD level);

DMC_API short __stdcall dmc_set_factor_error_unit(WORD CardNo,WORD axis,double factor,double error);
DMC_API short __stdcall dmc_get_factor_error_unit(WORD CardNo,WORD axis,double* factor,double* error);
DMC_API short __stdcall dmc_set_pulse_encoder_count_error_unit(WORD CardNo,WORD axis,double error);
DMC_API short __stdcall dmc_get_pulse_encoder_count_error_unit(WORD CardNo,WORD axis,double *error);
DMC_API short __stdcall dmc_check_pulse_encoder_count_error_unit(WORD CardNo,WORD axis,double* pulse_position, double* enc_position);

DMC_API short __stdcall dmc_set_ad_monitor_config(WORD CardNo, WORD Crd, WORD CANid, WORD channel, WORD ADEn, double ADValDown, double ADValUp);
DMC_API short __stdcall dmc_get_ad_monitor_config(WORD CardNo, WORD Crd, WORD* CANid, WORD* channel, WORD* ADEn, double* ADValDown, double* ADValUp);
DMC_API short __stdcall dmc_get_ad_monitor_result(WORD CardNo, WORD Crd, WORD *ch, double* ADRet, WORD* num,double *pos);
DMC_API short __stdcall dmc_clear_ad_monitor_result(WORD CardNo,WORD Crd);


DMC_API short __stdcall dmc_update_target_position_extern_unit(WORD CardNo, WORD axis, double mid_pos, double aim_pos, double vel,WORD posi_mode);
//???????????¦Ë????? ??????????S???(????)
DMC_API short __stdcall dmc_pmove_extern_unit(WORD CardNo, WORD axis, double dist,double Min_Vel, double Max_Vel, double Tacc, double Tdec, double stop_Vel, double s_para, WORD posi_mode);
DMC_API short __stdcall dmc_pmove_extern_acc_unit(WORD CardNo, WORD axis, double dist,double Min_Vel, double Max_Vel, double Tacc, double Tdec, double stop_Vel, double s_para, WORD posi_mode);
DMC_API short __stdcall dmc_m_mutiposTrig_outbit(WORD CardNo, WORD group, WORD bitno, WORD on_off, WORD process_mode, WORD trigAxisNum, WORD* trigAxisList, double* trigPos, WORD* trigPosType, WORD* trigMode, DWORD mark);//¦Ë?????IO????

DMC_API short __stdcall dmc_cmp_fifo_set_enable(WORD CardNo, WORD Crd, WORD enable);
DMC_API short __stdcall dmc_cmp_fifo_get_enable(WORD CardNo, WORD Crd, WORD* enable);
DMC_API short __stdcall dmc_cmp_fifo_get_state(WORD CardNo, WORD Crd, long* remained_space);
DMC_API short __stdcall dmc_cmp_fifo_clear_points(WORD CardNo, WORD Crd);
DMC_API short __stdcall dmc_cmp_fifo_set_config_params(WORD CardNo, WORD Crd, WORD Bitno, WORD On_Off, WORD Io_Mode, double Time_Dist_Value, double ReverseTime);
DMC_API short __stdcall dmc_cmp_fifo_get_config_params(WORD CardNo, WORD Crd, WORD* Bitno, WORD* On_Off, WORD* Io_Mode, double* Time_Dist_Value, double* ReverseTime);
DMC_API short __stdcall dmc_conti_line_add_cmp_fifo_unit(WORD CardNo, WORD Crd, WORD AxisNum,WORD* AxisList,double* Target_Pos, double* cmp_pos, WORD num, WORD posi_mode, long mark);
DMC_API short __stdcall dmc_conti_arc_move_3points_add_cmp_fifo_unit(WORD CardNo, WORD Crd, WORD AxisNum, WORD* AxisList,double* Target_Pos, double* Mid_Pos,WORD Circle, double *cmp_pos,WORD num, WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_center_add_cmp_fifo_unit(WORD CardNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* Target_Pos, double* Cen_Pos,WORD Arc_Dir,WORD Circle, double *cmp_pos,WORD num, WORD posi_mode,long mark);
DMC_API short __stdcall dmc_conti_arc_move_radius_add_cmp_fifo_unit(WORD CardNo, WORD Crd, WORD AxisNum, WORD* AxisList, double* Target_Pos, double Arc_Radius,WORD Arc_Dir,WORD Circle, double *cmp_pos,WORD num, WORD posi_mode,long mark);
DMC_API short __stdcall dmc_cmp_fifo_get_total_point(WORD CardNo,WORD  Crd,long *total_point);
DMC_API short __stdcall dmc_cmp_fifo_get_remain_point(WORD CardNo,WORD  Crd,long *remain_point);
DMC_API short __stdcall dmc_cmp_fifo_get_trig_point(WORD CardNo,WORD  Crd,long *trig_point);
DMC_API short __stdcall dmc_cmp_fifo_get_force_trig_point(WORD CardNo,WORD  Crd,long *force_trig_point);

DMC_API short __stdcall dmc_conti_wait_node_input_delay_to_start(WORD CardNo,WORD Crd,WORD node_ID, WORD bitno,WORD on_off, double delay_value,WORD delay_mode ,double TimeOut);
DMC_API short __stdcall dmc_conti_wait_node_input_ahead_to_stop(WORD CardNo,WORD Crd,WORD node_ID, WORD bitno,WORD on_off, double ahead_value,WORD ahead_mode ,double TimeOut);
DMC_API short __stdcall dmc_conti_delay_node_outbit_to_start(WORD CardNo, WORD Crd,WORD node_ID, WORD bitno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_delay_node_outbit_to_stop(WORD CardNo, WORD Crd,WORD node_ID, WORD bitno,WORD on_off,double delay_time,double ReverseTime);
DMC_API short __stdcall dmc_conti_ahead_node_outbit_to_stop(WORD CardNo, WORD Crd,WORD node_ID, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
DMC_API short __stdcall dmc_conti_write_node_outbit(WORD CardNo, WORD Crd,WORD node_ID, WORD bitno,WORD on_off,double ReverseTime);
DMC_API short __stdcall dmc_conti_clear_node_io_action(WORD CardNo, WORD Crd, WORD node_ID,DWORD Io_Mask);


DMC_API short __stdcall dmc_get_connect_type(WORD CardNo, WORD * ConnectType);
DMC_API short __stdcall dmc_board_init_eth(const char* IpAddr);
//??????????
DMC_API short __stdcall dmc_set_dec_stop_dist_unit(WORD CardNo,WORD axis,double dist);
DMC_API short __stdcall dmc_get_dec_stop_dist_unit(WORD CardNo,WORD axis,double *dist);
//???????????????(??????)
DMC_API short __stdcall dmc_set_profile_limit_unit(WORD CardNo,WORD axis,double Limit_Max_Vel,double Limit_Max_Acc,double EvenTime);
DMC_API short __stdcall dmc_get_profile_limit_unit(WORD CardNo,WORD axis,double* Limit_Max_Vel,double* Limit_MAX_Acc,double* EvenTime);
DMC_API short __stdcall dmc_set_vector_profile_limit_unit(WORD CardNo,WORD Crd,double Limit_Max_Vel,double Limit_Max_Acc,double EvenTime);
DMC_API short __stdcall dmc_get_vector_profile_limit_unit(WORD CardNo,WORD Crd,double* Limit_Max_Vel,double* Limit_MAX_Acc,double* EvenTime);
//??????????????
DMC_API short __stdcall dmc_set_vector_profile_limit_by_axis(WORD CardNo,WORD Crd,WORD Enable);
DMC_API short __stdcall dmc_get_vector_profile_limit_by_axis(WORD CardNo,WORD Crd,WORD* Enable);
DMC_API short __stdcall dmc_get_axis_follow_line_enable(WORD CardNo,WORD Crd,WORD * enable_flag);

DMC_API short __stdcall dmc_set_counter_reverse(WORD CardNo,WORD axis,WORD reverse);
DMC_API short __stdcall dmc_get_counter_reverse(WORD CardNo,WORD axis,WORD *reverse);
DMC_API short __stdcall dmc_set_extra_counter_reverse(WORD CardNo,WORD axis,WORD reverse);
DMC_API short __stdcall dmc_get_extra_counter_reverse(WORD CardNo,WORD axis,WORD *reverse);

DMC_API	short __stdcall dmc_conti_stop_axis(WORD CardNo, WORD Crd, WORD axis, double dec, int imark);

//?????????
DMC_API short __stdcall dmc_read_vector_length_unit(WORD CardNo,WORD Crd, double* total_length, double* left_length);
/*********************************************************************************************************
?????????????
*********************************************************************************************************/
DMC_API short __stdcall dmc_cam_table_unit(WORD CardNo,WORD MasterAxisNo,WORD SlaveAxisNo,DWORD Count,double *pMasterPos,double *pSlavePos,WORD SrcMode);
DMC_API short __stdcall dmc_cam_move(WORD CardNo,WORD axis);
DMC_API short __stdcall dmc_cam_move_cycle(WORD CardNo,WORD axis);

DMC_API short __stdcall dmc_conti_set_fairing_enable(WORD CardNo,WORD Crd, WORD enable, double fairing_length);
DMC_API short __stdcall dmc_conti_get_fairing_enable(WORD CardNo,WORD Crd, WORD * enable, double * fairing_length);

DMC_API short __stdcall dmc_set_eth_timeout(int timems);

DMC_API short __stdcall dmc_set_extra_encoder_extern(WORD CardNo,WORD channel, int pos);
DMC_API short __stdcall dmc_get_extra_encoder_extern(WORD CardNo,WORD channel, int * pos);

DMC_API short __stdcall dmc_conti_smooth_contour_unit(WORD CardNo, WORD Crd, WORD AxisNum,WORD*AxisList, WORD point_num, double*x,double*y,double*z, double vel_coef, double eps,long mark);
DMC_API short __stdcall dmc_get_conti_smooth_contour_curve(WORD point_num, double*x,double*y,double*z, double eps,double* curve_x,double* curve_y,double* curve_z, double* length);


//????????????????????
DMC_API short __stdcall nmc_hcmp_set_mode(WORD CardNo, WORD PortNum, WORD nodenum,  WORD hcmp, WORD cmp_mode);
DMC_API short __stdcall nmc_hcmp_get_mode(WORD CardNo, WORD PortNum, WORD nodenum, WORD hcmp, WORD* cmp_mode);
DMC_API short __stdcall nmc_hcmp_set_config(WORD CardNo, WORD PortNum, WORD nodenum, WORD hcmp, WORD channel,WORD cmp_source, WORD cmp_logic, long time);
DMC_API short __stdcall nmc_hcmp_get_config(WORD CardNo, WORD PortNum, WORD nodenum, WORD hcmp, WORD* channel, WORD* cmp_source, WORD*cmp_logic, long* time);
DMC_API short __stdcall nmc_hcmp_clear_points(WORD CardNo, WORD PortNum, WORD nodenum, WORD hcmp,WORD enable);
DMC_API short __stdcall nmc_hcmp_add_point(WORD CardNo, WORD PortNum, WORD nodenum, WORD hcmp, long cmp_pos);
DMC_API short __stdcall nmc_hcmp_set_liner(WORD CardNo, WORD PortNum, WORD nodenum, WORD hcmp, long Increment, long Count);
DMC_API short __stdcall nmc_hcmp_get_liner(WORD CardNo, WORD PortNum, WORD nodenum, WORD hcmp, long* Increment, long* Count);
DMC_API short __stdcall nmc_hcmp_get_current_state(WORD CardNo, WORD PortNum, WORD nodenum, WORD hcmp, long *remained_points, long* current_point, long *runned_points);

//????????????
DMC_API short __stdcall dmc_set_axis_follow_trajectory_displacement(WORD CardNo, WORD crd, WORD num, WORD* Axis_list);
DMC_API short __stdcall dmc_get_axis_follow_trajectory_displacement(WORD CardNo, WORD crd, WORD*num, WORD* Axis_list);
DMC_API short __stdcall dmc_set_tool_length_compensation_param(WORD CardNo, WORD axis, double length);
DMC_API short __stdcall dmc_get_tool_length_compensation_param(WORD CardNo, WORD axis, double* length);
DMC_API short __stdcall dmc_set_tool_length_compensation_enable(WORD CardNo, WORD axis, WORD enable);
DMC_API short __stdcall dmc_get_tool_length_compensation_enable(WORD CardNo, WORD axis, WORD* enable);
DMC_API short __stdcall dmc_set_normal_direction_control(WORD CardNo, WORD crd, WORD axis, WORD mode);
DMC_API short __stdcall dmc_get_normal_direction_control(WORD CardNo, WORD crd, WORD* axis, WORD* mode);
DMC_API short __stdcall dmc_set_gap_cmp_param(WORD CardNo, WORD crd, WORD pin, WORD logic, WORD mode, WORD auxi_axis, WORD source, long  rev_time, double* gap);
DMC_API short __stdcall dmc_get_gap_cmp_param(WORD CardNo, WORD crd, WORD* pin, WORD* logic, WORD* mode, WORD* auxi_axis, WORD* source, long* rev_time, double* gap);
DMC_API short __stdcall dmc_set_gap_cmp_enable(WORD CardNo, WORD crd, WORD enable);
DMC_API short __stdcall dmc_get_gap_cmp_enable(WORD CardNo, WORD crd, WORD* enable);
DMC_API short __stdcall dmc_set_normal_direction_control_enable(WORD CardNo, WORD crd, WORD enable);
DMC_API short __stdcall dmc_get_normal_direction_control_enable(WORD CardNo, WORD crd, WORD* enable);

DMC_API short __stdcall dmc_mc_gear_in(WORD CardNo, WORD slave_axis, WORD master_axis, WORD execute, WORD conti_update, WORD master_source, double ratio_numerator, double ratio_denominator, double acc, double dec, double jerk, WORD buffer_mode);
DMC_API short __stdcall dmc_get_mc_gear_in(WORD CardNo, WORD slave_axis, WORD* master_axis, WORD * execute, WORD * conti_update, WORD * master_source, double * ratio_numerator, double * ratio_denominator, double * acc, double * dec, double * jerk, WORD * buffer_mode);
DMC_API short __stdcall dmc_get_mc_gearin_status(WORD CardNo, WORD slave_axis, WORD * in_gear, WORD * busy, WORD * active, WORD * cmd_aborted, WORD * error,DWORD* error_id);
DMC_API short __stdcall dmc_mc_gear_out(WORD CardNo, WORD slave_axis, WORD * execute);
DMC_API short __stdcall dmc_get_mc_gear_out_status(WORD CardNo, WORD slave_axis,WORD* done,WORD* busy,WORD* cmd_aborted,WORD* error,DWORD* error_id);
DMC_API short __stdcall dmc_mc_combine_axes (WORD CardNo, WORD slave_axis, WORD master_axis1, WORD master_axis2, WORD execute, WORD conti_update, WORD master_source1, WORD master_source2, WORD combine_mode, double ratio_numerator1, double ratio_denominator1, double ratio_numerator2, double ratio_denominator2, double acc, double dec, double jerk, WORD buffer_mode);
DMC_API short __stdcall dmc_get_mc_combine_axes (WORD CardNo, WORD slave_axis, WORD* master_axis1, WORD* master_axis2, WORD * execute, WORD * conti_update, WORD * master_source1, WORD * master_source2, WORD * combine_mode, double * ratio_numerator1, double * ratio_denominator1, double * ratio_numerator2,double* ratio_denominator2, double * acc, double * dec, double * jerk, WORD * buffer_mode);
DMC_API short __stdcall dmc_get_mc_combine_axes_status(WORD CardNo, WORD slave_axis, WORD * in_sync, WORD * busy, WORD * active, WORD * cmd_aborted, WORD * error,DWORD* error_id);

DMC_API short __stdcall dmc_set_space_collision_zone_param(WORD CardNo, WORD axis_num,WORD* axis_list, WORD zone_num, double* neg_limit, double* pos_limit, WORD stop_mode, WORD pos_source);
DMC_API short __stdcall dmc_get_space_collision_zone_param(WORD CardNo, WORD* axis_num,WORD* axis_list, WORD* zone_num, double* neg_limit, double* pos_limit, WORD* stop_mode, WORD* pos_source);
DMC_API short __stdcall dmc_set_space_collision_zone_enable(WORD CardNo, WORD enable);
DMC_API short __stdcall dmc_get_space_collision_zone_enable(WORD CardNo, WORD* enable);

//??????? ????????20201016
DMC_API short __stdcall dmc_get_position_extern(WORD CardNo,WORD axis, double * pos);
DMC_API short __stdcall dmc_get_encoder_extern(WORD CardNo,WORD axis, double * pos);
DMC_API short __stdcall dmc_read_current_speed_extern(WORD CardNo,WORD axis, double *current_speed);

//???????
DMC_API short __stdcall dmc_cam_in(WORD CardNo, WORD slave_axis,WORD master_axis, WORD execute, WORD conti_update,WORD cam_table, WORD periodic,  WORD master_abs, WORD slave_abs, double master_offset, double slave_offset,double master_scaling, double slave_scaling,double master_start_dist,double master_sync_pos,double active_pos, WORD active_mode ,WORD start_mode, double velocity,double acc, double dec, double jerk, WORD master_source, WORD buffer_mode);
DMC_API short __stdcall dmc_get_cam_in_status(WORD CardNo, WORD slave_axis, WORD* in_sync, WORD* end_of_profile, WORD* busy, WORD* active, WORD* cmd_aborted, WORD* error , DWORD* error_id);
DMC_API short __stdcall dmc_cam_out(WORD CardNo, WORD slave_axis,WORD execute);
DMC_API short __stdcall dmc_get_cam_out_status(WORD CardNo, WORD slave_axis,WORD* done,WORD* busy,WORD* cmd_aborted,WORD* error,DWORD* error_id);
DMC_API short __stdcall dmc_cam_read_point(WORD CardNo, WORD execute,WORD cam_table, WORD cam_chg_point, DWORD cam_point_num, WORD* done,WORD*busy, WORD*error, DWORD *error_id,double* master_pos,double* slave_pos,double* slave_vel,double* slave_acc, double* slave_jerk, WORD* type);
DMC_API short __stdcall dmc_cam_write_point(WORD CardNo, WORD execute,WORD cam_table, DWORD cam_point_num, double master_pos,double slave_pos,double slave_vel,double slave_acc, double  slave_jerk,WORD type ,WORD* done,WORD*busy, WORD*error,DWORD*error_id);
DMC_API short __stdcall dmc_cam_set(WORD CardNo, WORD execute,WORD cam_table, WORD* done,WORD* busy, WORD* error,DWORD* error_id);
DMC_API short __stdcall dmc_cam_read_tappet_status(WORD CardNo, WORD execute,WORD cam_table, DWORD tappet_num1, DWORD tappet_num2, DWORD tappet_num3, DWORD tappet_num4, DWORD tappet_num5, DWORD tappet_num6, DWORD tappet_num7, DWORD tappet_num8, WORD* valid, WORD* busy, WORD* error,DWORD* error_id, WORD * status1, WORD * status2, WORD * status3, WORD * status4, WORD * status5, WORD * status6, WORD * status7, WORD * status8);
DMC_API short __stdcall dmc_cam_read_tappet_value(WORD CardNo, WORD execute,WORD cam_table, DWORD tappet_num, WORD* valid, WORD* busy, WORD* error,DWORD* error_id, double * master_pos, WORD * positive_mode, WORD * negative_mode);
DMC_API short __stdcall dmc_cam_write_tappet_value(WORD CardNo, WORD execute,WORD cam_table, DWORD tappet_num, double  master_pos, WORD  positive_mode, WORD  negative_mode, WORD* done, WORD* busy, WORD* error,DWORD* error_id);
DMC_API short __stdcall dmc_cam_add_tappet (WORD CardNo, WORD execute,WORD cam_table, double  master_pos, WORD positive_mode, WORD negative_mode ,WORD* done, WORD* busy, WORD* error,DWORD* error_id, DWORD* tappet_num);
DMC_API short __stdcall dmc_cam_delete_tappet (WORD CardNo, WORD execute,WORD cam_table, WORD* done, WORD* busy, WORD* error,DWORD* error_id);

DMC_API short __stdcall	dmc_cmd_buf_temp_delete(WORD CardNo,WORD group,WORD addr,WORD num,WORD delete_mode);

DMC_API short __stdcall dmc_conti_set_wait_mode(WORD CardNo, WORD Crd, WORD wait_mode);
DMC_API short __stdcall dmc_conti_get_wait_mode(WORD CardNo, WORD Crd, WORD* wait_mode);

DMC_API short __stdcall dmc_set_peak_config(WORD CardNo, WORD axis, WORD enable,double u_time);
DMC_API short __stdcall dmc_get_peak_config(WORD CardNo, WORD axis, WORD* enable,double* u_time);

DMC_API short __stdcall dmc_set_axis_err_band(WORD CardNo, WORD axis,double err_band, WORD set_cycle);
DMC_API short __stdcall dmc_get_axis_err_band(WORD CardNo, WORD axis,double* err_band, WORD* set_cycle);
DMC_API short __stdcall dmc_set_axis_err_band_unit(WORD CardNo, WORD axis,double err_band, WORD set_cycle);
DMC_API short __stdcall dmc_get_axis_err_band_unit(WORD CardNo, WORD axis,double* err_band, WORD* set_cycle);

//???pmove?????????¹À??????
DMC_API short __stdcall dmc_get_axis_plan_time_info(WORD CardNo,WORD axis,double *sum_time, double *remain_time);
//???¨°?????????????????
DMC_API short __stdcall dmc_set_axis_max_interpo_speed(WORD CardNo,WORD axis,double max_speed);
//?????????????????????
DMC_API short __stdcall dmc_get_axis_max_interpo_speed(WORD CardNo,WORD axis, double* max_speed);

DMC_API short __stdcall dmc_set_axis_max_interpo_speed_enable(WORD CardNo,WORD axis, WORD enable);
DMC_API short __stdcall dmc_get_axis_max_interpo_speed_enable(WORD CardNo,WORD axis, WORD* enable);

DMC_API short __stdcall dmc_set_diagnosis_log_enable(WORD CardNo,WORD Crd, WORD enable);
DMC_API short __stdcall dmc_get_diagnosis_log_enable(WORD CardNo,WORD Crd, WORD* enable);
DMC_API short __stdcall dmc_get_diagnosis_log_data(WORD CardNo,WORD Crd);

DMC_API short __stdcall nmc_reverse_outbit(WORD CardNo, WORD Channel, WORD NoteID, WORD IoBit,double reverse_time);

DMC_API short __stdcall dmc_sine_oscillate(WORD CardNo,WORD Axis,double Amplitude,double Frequency);
DMC_API short __stdcall dmc_sine_oscillate_stop(WORD CardNo,WORD Axis);

DMC_API short __stdcall dmc_set_apf_rotary_cut_init(WORD CardNo, DWORD rotary_cut_id, WORD execute, double rotary_axis_radius, DWORD rotary_axis_knife_num, double feed_axis_radius, double cutlength,double sync_start_pos, double sync_stop_pos, double rot_start_pos, double fed_stop_pos);
DMC_API short __stdcall dmc_get_apf_rotary_cut_init(WORD CardNo, DWORD rotary_cut_id, WORD* execute, double* rotary_axis_radius, DWORD* rotary_axis_knife_num, double* feed_axis_radius, double* cutlength, double* sync_start_pos, double* sync_stop_pos, double* rot_start_pos, double* fed_stop_pos);
DMC_API short __stdcall dmc_get_apf_rotary_cut_init_status(WORD CardNo, DWORD  rotary_cut_id, WORD* done, WORD* busy, WORD* error , DWORD* error_id);
DMC_API short __stdcall dmc_apf_rotary_cut_in(WORD CardNo, DWORD rotary_cut_id, WORD execute, WORD rotary_axis, WORD feed_axis);
DMC_API short __stdcall dmc_get_apf_rotary_cut_in_status(WORD CardNo, DWORD rotary_cut_id, WORD* done, WORD* busy, WORD*error, DWORD* error_id);
DMC_API short __stdcall dmc_get_apf_rotary_cut_in(WORD CardNo, DWORD rotary_cut_id, WORD* execute,WORD* rotary_axis, WORD* feed_axis);
DMC_API short __stdcall dmc_apf_rotary_cut_out(WORD CardNo, DWORD rotary_cut_id, WORD execute, WORD rotary_axis);
DMC_API short __stdcall dmc_get_apf_rotary_cut_out_status(WORD CardNo, DWORD rotary_cut_id, WORD* done, WORD* busy, WORD* error, DWORD* error_id);

DMC_API short __stdcall dmc_conti_set_clear_current_mark_mode(WORD CardNo,WORD Crd, WORD mode);
DMC_API short __stdcall dmc_conti_get_clear_current_mark_mode(WORD CardNo,WORD Crd, WORD* mode);
DMC_API short __stdcall dmc_conti_clear_current_mark(WORD CardNo,WORD Crd);

DMC_API short __stdcall dmc_conti_set_arc_translate_mode(WORD CardNo,WORD Crd, WORD mode);
DMC_API short __stdcall dmc_conti_get_arc_translate_mode(WORD CardNo,WORD Crd, WORD* mode);
DMC_API short __stdcall dmc_trace_set_source(WORD CardNo,WORD source);

DMC_API short __stdcall nmc_sync_set_profile_unit(WORD CardNo, WORD AxisNum, WORD* AxisList,double* Min_Vel,double* Max_Vel, double* Tacc, double* Tdec, double* Stop_Vel);

DMC_API short __stdcall nmc_write_rxpdo_extra_short(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,WORD* Value);
DMC_API short __stdcall nmc_read_rxpdo_extra_short(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,WORD* Value);
DMC_API short __stdcall nmc_read_txpdo_extra_short(WORD CardNo,WORD PortNum,WORD address,WORD DataLen,WORD* Value);

DMC_API short __stdcall dmc_set_timeout(WORD CardNo, DWORD timems);
DMC_API short __stdcall nmc_sync_pos_change_mode (WORD CardNo, WORD portno, WORD axis);

DMC_API short __stdcall dmc_mc_gear_in_pos(WORD CardNo, WORD slave_axis, WORD master_axis, WORD execute, WORD conti_update, WORD master_source, double ratio_numerator, double ratio_denominator, double master_sync_pos, double slave_sync_pos, double master_start_dist, WORD buffer_mode);
DMC_API short __stdcall dmc_get_mc_gear_in_pos(WORD CardNo, WORD slave_axis, WORD* master_axis, WORD * execute, WORD * conti_update, WORD * master_source, double * ratio_numerator, double * ratio_denominator, double* master_sync_pos, double* slave_sync_pos, double* master_start_dist, WORD * buffer_mode);
DMC_API short __stdcall dmc_get_mc_gear_in_pos_status(WORD CardNo, WORD slave_axis, WORD * start_sync, WORD * in_sync, WORD * busy, WORD * active, WORD * cmd_aborted, WORD * error,DWORD* error_id);

DMC_API short __stdcall dmc_get_watchdog_trig_status (WORD CardNo, WORD* status);
DMC_API short __stdcall dmc_reset_watchdog_trig_status (WORD CardNo);
DMC_API short __stdcall dmc_conti_set_transarc_io_insert_mode (WORD CardNo,WORD Crd, WORD mode);
DMC_API short __stdcall dmc_conti_get_transarc_io_insert_mode(WORD CardNo,WORD Crd, WORD* mode);

DMC_API short __stdcall dmc_multi_axes_motion_sync_pmove_unit(WORD CardNo,WORD axis_num, WORD* axis_list, double* dist_list,double* Min_Vel_list, double* Max_Vel_list, double* Tacc_list, double* Tdec_list, double* stop_Vel_list, double* s_para_list, WORD* posi_mode_list, WORD mode);

DMC_API short __stdcall dmc_set_ez_map_input(WORD CardNo,WORD axis,WORD enable,WORD mode,WORD index,WORD sub_index);
DMC_API short __stdcall dmc_get_ez_map_input(WORD CardNo,WORD axis,WORD *enable,WORD *mode,WORD *index,WORD *sub_index);
DMC_API short __stdcall nmc_set_etc_el_stop_mode(WORD CardNo,WORD axis,WORD el_control_mode, double diff_pos,DWORD filter);

DMC_API short __stdcall dmc_circle_move_center_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_conti_circle_move_center_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
DMC_API short __stdcall dmc_set_acuate_angle_config_params(WORD CardNo, WORD Crd, double acuate_angle, double angle_trans_speed, WORD enable);
DMC_API short __stdcall dmc_get_acuate_angle_config_params(WORD CardNo, WORD Crd, double* acuate_angle, double* angle_trans_speed, WORD* enable);
DMC_API short __stdcall dmc_set_axes_link_params(WORD CardNo, WORD master, WORD slave);
DMC_API short __stdcall dmc_get_axes_link_params(WORD CardNo, WORD master, WORD* slave);
DMC_API short __stdcall dmc_remove_axes_link_params(WORD CardNo, WORD master);

DMC_API short __stdcall dmc_set_alm_mode_ex(WORD CardNo,WORD axis,WORD enable,WORD alm_logic,WORD alm_action,WORD alm_all); //????????
DMC_API short __stdcall dmc_get_alm_mode_ex(WORD CardNo,WORD axis,WORD* enable,WORD* alm_logic,WORD* alm_action,WORD* alm_all); //????????
//?????????????
DMC_API short __stdcall dmc_get_encoder_dir(WORD CardNo, WORD axis,WORD* dir);

DMC_API short __stdcall dmc_download_configfile_ex(WORD CardNo,const char *FileName);

DMC_API short __stdcall dmc_set_s_profile_config(WORD CardNo, WORD axis, double acc_s_time, double dec_s_time);
DMC_API short __stdcall dmc_get_s_profile_config(WORD CardNo, WORD axis, double* acc_s_time, double* dec_s_time);

DMC_API short __stdcall nmc_set_slave_state(WORD CardNo, WORD SlaveId, WORD SlaveState);
DMC_API short __stdcall nmc_get_slave_state(WORD CardNo, WORD SlaveId, WORD *SlaveState);

DMC_API short __stdcall dmc_sync_pmove_unit(WORD CardNo,WORD axis_num, WORD* axis_list, double* dist_list, WORD* posi_mode_list, WORD mode);
DMC_API short __stdcall dmc_set_axis_abnormal_mode(WORD CardNo,WORD enable);
DMC_API short __stdcall dmc_get_axis_abnormal_mode(WORD CardNo,WORD* enable);
DMC_API short __stdcall dmc_clear_axis_abnormal_state(WORD CardNo,WORD axis,WORD count);
DMC_API short __stdcall dmc_set_coordinate_abnormal_mode(WORD CardNo,WORD enable);
DMC_API short __stdcall dmc_get_coordinate_abnormal_mode(WORD CardNo,WORD* enable);
DMC_API short __stdcall dmc_clear_crd_abnormal_state(WORD CardNo,WORD Crd,WORD count);
DMC_API short __stdcall dmc_set_coordinate_remainspace_mode(WORD CardNo,WORD Crd,WORD enable);
DMC_API short __stdcall dmc_get_coordinate_remainspace_mode(WORD CardNo,WORD Crd,WORD* enable);
DMC_API short __stdcall dmc_hcmp_add_linear_unit(WORD CardNo,WORD hcmp,int count, struct_hs_cmp_info* cmp_str);
DMC_API short __stdcall dmc_set_axis_handwheel_encoder_filter_frequancy(WORD CardNo, WORD axis,double frequancy);
DMC_API short __stdcall dmc_get_axis_handwheel_encoder_filter_frequancy(WORD CardNo, WORD axis,double *frequancy);

DMC_API short __stdcall nmc_set_slave_alias(WORD CardNo,  WORD portnum, WORD auto_address, WORD alias_address);
DMC_API short __stdcall nmc_get_slave_alias(WORD CardNo,  WORD portnum, WORD auto_address, WORD* alias_address);

DMC_API short __stdcall dmc_set_pwm_first_pulse_mode(WORD CardNo, WORD pwm_no, WORD enable);
DMC_API short __stdcall	dmc_get_pwm_first_pulse_mode(WORD CardNo, WORD pwm_no, WORD* enable);
DMC_API short __stdcall dmc_set_pwm_first_pulse_duty(WORD CardNo, WORD pwm_no, double duty);
DMC_API short __stdcall dmc_get_pwm_first_pulse_duty(WORD CardNo, WORD pwm_no, double* duty);

DMC_API short __stdcall dmc_cmp_fifo_set_hcmp2d_pos_ratio(WORD CardNo,WORD Crd, WORD hcmp2d, double xpos_ratio, double ypos_ratio);
DMC_API short __stdcall dmc_cmp_fifo_get_hcmp2d_pos_ratio(WORD CardNo,WORD Crd, WORD hcmp2d, double * xpos_ratio, double * ypos_ratio);

DMC_API short __stdcall dmc_set_leadscrew_comp_datasheet_enable (WORD CardNo, WORD axis,WORD enable, int point_num);
DMC_API short __stdcall dmc_get_leadscrew_comp_datasheet_enable (WORD CardNo, WORD axis,WORD* enable, int* point_num);
DMC_API short __stdcall dmc_set_pos_calibrate_config(WORD CardNo, WORD axis,WORD settle_time, double err_band, WORD enable);
DMC_API short __stdcall dmc_get_pos_calibrate_config(WORD CardNo, WORD axis,WORD* settle_time, double* err_band, WORD* enable);

//???????
DMC_API short __stdcall dmc_userlib_loadlibrary(WORD CardNo,const char *pLibname);
DMC_API short __stdcall dmc_userlib_set_parameter(WORD CardNo, int type, const unsigned char *pParameter,int length);
DMC_API short __stdcall dmc_userlib_get_parameter(WORD CardNo, int type, unsigned char *pParameter,int length);
DMC_API short __stdcall dmc_userlib_imd_stop(WORD CardNo,WORD axis);

DMC_API short __stdcall dmc_cmp_fifo_get_fpga_receive_point(WORD CardNo,WORD cmp_no, long *receive_point);
DMC_API short __stdcall dmc_cmp_fifo_check_fpga_clear_status(WORD CardNo,WORD cmp_no, WORD *clr_status, long *clr_point);
DMC_API short __stdcall nmc_modify_slaveid(WORD CardNo,WORD index,WORD subindex,WORD newindex,const char *FileName);

DMC_API short __stdcall dmc_set_home_finish_map(WORD CardNo,WORD axis,WORD enable,WORD mode,WORD index,WORD sub_index,WORD bit_index,WORD bit_logic);

DMC_API short __stdcall dmc_get_home_finish_map(WORD CardNo,WORD axis,WORD *enable,WORD *mode,WORD *index,WORD *sub_index,WORD *bit_index,WORD *bit_logic);


DMC_API short __stdcall dmc_get_config_error_info(WORD CardNo,int* axis, int* liner,int* type, int* errorcode);

DMC_API short __stdcall dmc_set_t_pmove_extern_dectime(WORD CardNo, WORD axis, DWORD dec_time);
DMC_API short __stdcall dmc_get_t_pmove_extern_dectime(WORD CardNo, WORD axis, DWORD* dec_time);
DMC_API short __stdcall dmc_set_trajectory_splicing_error(WORD CardNo, WORD crd, double error);
DMC_API short __stdcall dmc_get_trajectory_splicing_error(WORD CardNo, WORD crd, double *error);

DMC_API short __stdcall dmc_sine_oscillate_set_cycle_num(WORD CardNo, WORD Axis, DWORD cycle_num);
DMC_API short __stdcall dmc_sine_oscillate_get_cycle_num(WORD CardNo, WORD Axis, DWORD *cycle_num);

DMC_API short __stdcall dmc_conti_wait_input_action(WORD CardNo,WORD Crd,WORD bitno,WORD on_off,double TimeOut,WORD action,long mark);

DMC_API short __stdcall dmc_line_change_pos_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList, double* TargetPos);

DMC_API short __stdcall dmc_arc_move_angle_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Cen_Pos,double Angle,double *Target_Pos,WORD posi_mode);
DMC_API short __stdcall dmc_arc_move_center_angle_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,double Angle,WORD Arc_Dir,long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_conti_line_change_pos_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList, double* TargetPos, int mark);

DMC_API short __stdcall nmc_sync_pmove_extern_unit(WORD CardNo,WORD AxisNum, WORD* AxisList, double* Dist, double* Max_Vel, WORD Posimode);
DMC_API short __stdcall dmc_pvt_get_run_index(WORD CardNo, WORD axis, DWORD* index);

DMC_API short __stdcall dmc_firmware_auto_update();
DMC_API short __stdcall dmc_conti_set_blend_distance(WORD CardNo, WORD Crd, WORD Enable, double BlendDistance);

DMC_API short __stdcall nmc_set_dc_mode(WORD CardNo, WORD PortNo,WORD mode);
DMC_API short __stdcall nmc_get_dc_mode(WORD CardNo, WORD PortNo,WORD* mode);

DMC_API short __stdcall dmc_set_gear_follow_ratio(WORD CardNo,WORD axis,double ratio);//?Z??
DMC_API short __stdcall dmc_get_gear_follow_ratio(WORD CardNo,WORD axis,double* ratio);
DMC_API short __stdcall dmc_ltc_set_outbit(WORD CardNo,WORD latch,WORD enable,WORD bitno,WORD logic,double delaytime_s,double outtime_s);
DMC_API short __stdcall dmc_ltc_get_outbit(WORD CardNo,WORD latch,WORD* enable,WORD* bitno,WORD* logic,double* delaytime_s,double* outtime_s);

DMC_API short __stdcall dmc_get_idle_crd_index(WORD CardNo);
DMC_API short __stdcall dmc_get_position_virtual(WORD CardNo,WORD axis,double* pos);


DMC_API short __stdcall dmc_check_encoder_done(WORD CardNo, WORD axis, WORD * state, double * EncoderPos);
DMC_API short __stdcall dmc_set_check_target_encoder(WORD CardNo, WORD axis, WORD TargetCheckEnable,  double TargetError, double TargetCheckTime_s);

DMC_API short __stdcall dmc_get_check_target_encoder(WORD CardNo, WORD axis, WORD * TargetCheckEnable,  double * TargetError, double * TargetCheckTime_s)

;
DMC_API short __stdcall dmc_set_check_inp_encoder(WORD CardNo, WORD axis, WORD  InpCheckEnable,double InpError, double InpCheckTime_s);
DMC_API short __stdcall dmc_get_check_inp_encoder(WORD CardNo, WORD axis, WORD * InpCheckEnable,double * InpError, double * InpCheckTime_s);
DMC_API short __stdcall dmc_set_connect_to_encoder(WORD CardNo, WORD axis, WORD enable,double error);
DMC_API short __stdcall dmc_get_connect_to_encoder(WORD CardNo, WORD axis, WORD *enable,double *error);

DMC_API short __stdcall dmc_set_robot_config(WORD CardNo, WORD Crd, short robot_type, short elbow, short joint_num, short* joint_list,double* rx, double* tx, double* rz, double* tz);
DMC_API short __stdcall dmc_get_robot_config(WORD CardNo, WORD Crd, short* robot_type, short* elbow, short* joint_num, short* joint_list,double* rx, double* tx, double* rz, double* tz);
DMC_API short __stdcall dmc_set_robot_enable(WORD CardNo, WORD Crd, short user_crd, short tool_crd, short enable);
DMC_API short __stdcall dmc_robot_ptp_move(WORD CardNo, WORD Crd, short joint_num, short* joint_list, double* joint_pos);
DMC_API short __stdcall dmc_get_robot_sts(WORD CardNo, WORD Crd, short* complete, short* user_crd, short* tool_crd, short* enable);
DMC_API short __stdcall dmc_get_robot_pos(WORD CardNo, WORD Crd, double* pos);
DMC_API short __stdcall dmc_set_robot_kinematics_calib(WORD CardNo, WORD Crd, double* delta_rx, double* delta_tx, double* delta_rz, double* delta_tz);
DMC_API short __stdcall dmc_get_robot_kinematics_calib(WORD CardNo, WORD Crd, double* delta_rx, double* delta_tx, double* delta_rz, double* delta_tz);
DMC_API short __stdcall dmc_robot_kinematics_calib(WORD CardNo, WORD Crd, double* ja, double* jb, double* jc, double* jd, double* je, double* jf, double* jg, double* jh, double* ji, double* delta_x, double* delta_y, double* delta_z);
DMC_API short __stdcall dmc_set_robot_user_coordinate(WORD CardNo, WORD Crd, short user_crd, short complete, double* mat);
DMC_API short __stdcall dmc_get_robot_user_coordinate(WORD CardNo, WORD Crd, short user_crd, short* complete, double* mat);
DMC_API short __stdcall dmc_robot_user_coordinate(WORD CardNo, WORD Crd, short user_crd, double* p0, double* px, double* py);
DMC_API short __stdcall dmc_set_robot_tool_coordinate(WORD CardNo, WORD Crd, short tool_crd, short complete, double* mat);
DMC_API short __stdcall dmc_get_robot_tool_coordinate(WORD CardNo, WORD Crd, short tool_crd, short* complete, double* mat);
DMC_API short __stdcall dmc_robot_tool_coordinate(WORD CardNo, WORD Crd, short tool_crd, double* p1, double* p2, double* p3, double* p4, double* p5, double* p6, double* p0, double* px, double* py);
DMC_API short __stdcall dmc_robot_workspace_detect(WORD CardNo, WORD Crd, double* pos);


DMC_API short __stdcall dmc_conti_set_wait_flag(WORD CardNo, WORD Crd, int mark, WORD wait_flag);
DMC_API short __stdcall dmc_conti_get_wait_flag(WORD CardNo, WORD Crd, int * mark, WORD * wait_flag);

DMC_API short __stdcall dmc_conti_set_arc_blend_enable(WORD CardNo, WORD Crd, WORD Enable);
DMC_API short __stdcall dmc_conti_get_arc_blend_enable(WORD CardNo, WORD Crd, WORD * Enable);

DMC_API short __stdcall dmc_t_pmove_extern_unit_ex(WORD CardNo, WORD axis, double MidPos,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,WORD posi_mode);

DMC_API short __stdcall dmc_check_success_pulse_ex(WORD CardNo,WORD axis,int delay_ms);
DMC_API short __stdcall dmc_check_success_encoder_ex(WORD CardNo,WORD axis,int delay_ms);

DMC_API short __stdcall dmc_cam_write_points_packet(WORD CardNo,WORD cam_table_id, WORD cam_point_num, double s_range_up, double s_range_dn, double * master_pos, double * slave_pos, double * slave_vel, double * slave_acc, double * slave_jerk, WORD* type);

DMC_API short __stdcall dmc_cam_read_points_packet(WORD CardNo,WORD cam_table_id, WORD* cam_point_num, double * s_range_up, double * s_range_dn, double * master_pos, double * slave_pos, double * slave_vel, double * slave_acc, double * slave_jerk, WORD* type);

DMC_API short __stdcall dmc_set_pwm_output_extern(WORD CardNo, WORD pwm, WORD enable, double width_us, double frequency, DWORD number)
;

DMC_API short __stdcall dmc_spline_pmove(WORD CardNo, WORD axis, double pos, double vs, double vm, double ve, double as, double ae, double rmd_as, double rmd_ae, int num_ts,int num_tm,int num_te, double cur_as, double cur_ae, WORD posi_mode);

DMC_API short __stdcall dmc_set_plan_mode(WORD CardNo, WORD axis, WORD mode)
;
DMC_API short __stdcall dmc_get_plan_mode(WORD CardNo, WORD axis, WORD* mode)
;

DMC_API short __stdcall dmc_set_emg_lock(WORD CardNo, WORD enable, WORD bit_no, WORD level,DWORD out_mark, DWORD out_level);
DMC_API short __stdcall dmc_get_emg_lock(WORD CardNo, WORD * enable, WORD * bit_no, WORD * level,DWORD* out_mark, DWORD* out_level);
DMC_API short __stdcall dmc_emg_unlock(WORD CardNo);
DMC_API short __stdcall dmc_emg_lock_status(WORD CardNo, WORD * lock_status, WORD * lock_type);

DMC_API short __stdcall dmc_set_vector_profile_extern(WORD CardNo,WORD Crd,double Min_Vel,double Max_Vel,double Acc,double Dec,double Ajerk,double Djerk,double stop_vel);
DMC_API short __stdcall dmc_get_vector_profile_extern(WORD CardNo,WORD Crd,double *Min_Vel,double *Max_Vel,double *Acc,double *Dec,double *Ajerk,double *Djerk,double *stop_vel);
DMC_API short __stdcall dmc_set_vector_plan_mode(WORD CardNo, WORD Crd, WORD mode)
;
DMC_API short __stdcall dmc_get_vector_plan_mode(WORD CardNo, WORD Crd, WORD* mode)
;

DMC_API short __stdcall nmc_set_data_offset_time(WORD CardNo, int offset_us);
DMC_API short __stdcall nmc_get_data_offset_time(WORD CardNo, int * offset_us);
DMC_API short __stdcall dmc_check_done_multicoor_extern(WORD CardNo, WORD Crd, WORD * crd_state, DWORD * crd_stop_reason, DWORD * axis_stop_reason);
DMC_API short __stdcall dmc_get_error_description(int errcocode, char* description);
DMC_API short __stdcall dmc_write_outport_mask(WORD CardNo, WORD port, DWORD mask, DWORD state, DWORD reverse_time_ms);

DMC_API short __stdcall dmc_set_pso_output_delay(WORD CardNo, WORD axis, DWORD delay_cycle);
DMC_API short __stdcall dmc_get_pso_output_delay(WORD CardNo, WORD axis, DWORD* delay_cycle);
DMC_API short __stdcall dmc_set_gap_cmp_space(WORD CardNo, WORD crd, double space);
DMC_API short __stdcall dmc_get_gap_cmp_space(WORD CardNo, WORD crd, double* space);

DMC_API short __stdcall nmc_ecat_read_slave_register(WORD CardNo, WORD wSlaveAddress, WORD wRegisterOffset, WORD wLen, char * pdwData);
DMC_API short __stdcall nmc_ecat_write_slave_register(WORD CardNo, WORD wSlaveAddress, WORD wRegisterOffset, WORD wLen, char * pdwData);

DMC_API short __stdcall dmc_set_inp_map_input(WORD CardNo, WORD axis, WORD enable, WORD index, WORD sub_index, WORD bit_index, WORD inp_validvalue, WORD connect2checkdone);
DMC_API short __stdcall dmc_get_inp_map_input(WORD CardNo, WORD axis, WORD* enable, WORD* index, WORD* sub_index, WORD* bit_index, WORD* inp_validvalue, WORD* connect2checkdone);
DMC_API short __stdcall dmc_get_pwm_state(WORD CardNo, WORD channel, WORD* state);

DMC_API short __stdcall dmc_rtcp_rotation_axis_transform_param(WORD CardNo, WORD axis,  double rod_len);
DMC_API short __stdcall dmc_rtcp_get_rotation_axis_transform_param(WORD CardNo, WORD axis, double* rod_len);
DMC_API short __stdcall dmc_rtcp_rotation_axis_transform_enable(WORD CardNo, WORD axis, WORD enable);
DMC_API short __stdcall dmc_rtcp_get_rotation_axis_transform_enable(WORD CardNo, WORD axis, WORD* enable);
DMC_API short __stdcall dmc_circle_move_3point_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double *Mid_Pos, long Circle,WORD posi_mode);
DMC_API short __stdcall dmc_conti_circle_move_3point_unit(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mid_Pos, long Circle,WORD posi_mode,long mark);

DMC_API short __stdcall dmc_get_cam_in(WORD CardNo, WORD slave_axis, WORD* master_axis, WORD*execute, WORD*conti_update,WORD* cam_table, WORD* periodic, WORD*master_abs, WORD*slave_abs, double* master_offset, double*slave_offset,double* master_scaling, double*slave_scaling,double*master_start_dist,double*master_sync_pos,double* active_pos, WORD* acvive_mode ,WORD* start_mode, double* velocity, double* acc, double* dec, double* jerk, WORD* master_source, WORD* buffer_mode);
DMC_API short __stdcall dmc_mc_phasing(WORD CardNo, WORD slave_axis,WORD master_axis, WORD execute, double phase_shift, double velocity,double acc, double dec,double jerk);
DMC_API short __stdcall dmc_get_mc_phasing_status(WORD CardNo, WORD slave_axis, WORD * done, WORD * busy, WORD * cmd_aborted, WORD * error,DWORD* error_id);
DMC_API short __stdcall dmc_get_mc_phasing(WORD CardNo, WORD slave_axis,WORD* master_axis, WORD* execute, double* phase_shift, double* velocity,double* acc, double* dec ,double* jerk);

DMC_API short __stdcall dmc_set_axis_pwm_follow_speed(WORD CardNo, WORD pwm_no, WORD axis, WORD mode, double max_vel, double min_vel,double max_value, double out_value, double min_value,WORD min_ctl_mode);
DMC_API short __stdcall dmc_get_axis_pwm_follow_speed(WORD CardNo, WORD pwm_no, WORD* axis, WORD* mode, double* max_vel, double* min_vel,double* max_value, double* out_value, double* min_value,WORD* min_ctl_mode);
DMC_API short __stdcall dmc_set_axis_pwm_follow_speed_enable(WORD CardNo, WORD pwm_no, WORD enable);
DMC_API short __stdcall dmc_get_axis_pwm_follow_speed_enable(WORD CardNo, WORD pwm_no, WORD* enable);

DMC_API short __stdcall dmc_umove_unit(WORD CardNo, WORD group, WORD mode, WORD sources, WORD io_index, WORD io_value,WORD up_axis,double up_pos, double up_safe_distance, WORD move_num, WORD* move_axis_list, double* move_pos,double* move_safe_distance,WORD down_axis, double down_pos, WORD posi_mode);
DMC_API short __stdcall dmc_get_umove_runsts(WORD CardNo, WORD group, WORD* runsts);
DMC_API short __stdcall dmc_umove_stop(WORD CardNo, WORD group);

DMC_API short __stdcall dmc_set_knife_positioned(WORD CardNo, WORD Crd, double SecondVel, double SecondPos);
DMC_API short __stdcall dmc_get_knife_positioned(WORD CardNo, WORD Crd, double* SecondVel, double* SecondPos);
DMC_API short __stdcall dmc_set_knife_positioned_enable(WORD CardNo, WORD Crd, WORD enable);
DMC_API short __stdcall dmc_get_knife_positioned_enable(WORD CardNo, WORD Crd, WORD* enable);
DMC_API short __stdcall dmc_return_to_zero(WORD CardNo, WORD axis);

DMC_API short __stdcall dmc_conti_set_lookahead_path_error(WORD CardNo, WORD Crd, double patherr);
DMC_API short __stdcall dmc_conti_get_lookahead_path_error(WORD CardNo, WORD Crd, double *patherr);
DMC_API short __stdcall dmc_conti_safe_pause_list(WORD CardNo, WORD Crd, WORD safe_axis_num, WORD *safe_axis_list, double *distance, double *vstart, double *vsteady, double *vend, double *acc_time, double *dec_time, WORD posi_mode);
DMC_API short __stdcall dmc_set_axis_da_follow_speed(WORD CardNo, WORD da_channel, WORD axis, WORD mode, double max_vel,double min_vel ,double max_value, double min_value, double offset);
DMC_API short __stdcall dmc_get_axis_da_follow_speed(WORD CardNo, WORD da_channel, WORD*axis, WORD*mode, double* max_vel,double * min_vel, double* max_value, double* min_value, double*offset);
DMC_API short __stdcall dmc_set_axis_da_follow_speed_enable(WORD CardNo, WORD da_channel, WORD enable);
DMC_API short __stdcall dmc_get_axis_da_follow_speed_enable(WORD CardNo, WORD da_channel, WORD*enable);
DMC_API short __stdcall dmc_conti_force_set_position(WORD CardNo, WORD Crd, WORD axis_num, WORD*axis_list, double*position);

DMC_API short __stdcall dmc_set_axis_da_follow_speed_extern(WORD CardNo, WORD da_channel, WORD axis, WORD mode, WORD segment, double*vel,double* value);
DMC_API short __stdcall dmc_get_axis_da_follow_speed_extern(WORD CardNo, WORD da_channel, WORD*axis, WORD*mode, WORD *segment, double* vel,double* value);

DMC_API short __stdcall dmc_conti_set_node_da_enable(WORD CardNo,WORD Crd, WORD node_id, WORD channel, WORD enable);
DMC_API short __stdcall dmc_conti_set_node_da_output(WORD CardNo,WORD Crd, WORD node_id, WORD channel, double Vout);
DMC_API short __stdcall dmc_conti_gantry_move(WORD CardNo,WORD Crd, WORD master_axis, WORD slave_num, WORD* slave_axis_list,WORD on_off);
DMC_API short __stdcall dmc_conti_set_gantry_error_protect_unit(WORD CardNo,WORD Crd, WORD master_axis, double dstp_err, double emg_err, WORD on_off);

DMC_API short __stdcall dmc_m_add_sigaxis_moveseg_data_ex(WORD CardNo, WORD group, WORD Axis, double Target_pos, DWORD mark);
DMC_API short __stdcall dmc_m_add_wait_event_data(WORD CardNo, WORD group, WORD event, WORD num,WORD CompareOperator, double target_value, DWORD mark);
DMC_API short __stdcall dmc_m_add_trigger_data(WORD CardNo, WORD group, WORD mode, WORD num, double Target_Value, DWORD mark);
DMC_API short __stdcall dmc_m_add_time_delay(WORD CardNo, WORD group, double Time_delay, DWORD mark);

DMC_API short __stdcall dmc_set_feedforward_profile(WORD CardNo,WORD Axis,double vel_offset_coef,double tor_offset_coef);
DMC_API short __stdcall dmc_get_feedforward_profile(WORD CardNo,WORD Axis,double* vel_offset_coef,double* tor_offset_coef);
DMC_API short __stdcall dmc_set_modulo_profile(WORD CardNo, WORD axis,WORD enable,double Modulo_Vel);
DMC_API short __stdcall dmc_get_modulo_profile(WORD CardNo, WORD axis,WORD *enable,double *Modulo_Vel);

DMC_API short __stdcall dmc_line_mutli_line(WORD CardNo, WORD Crd, WORD AxisNum, WORD *AxisList, WORD PointNum, double(*pos)[8], WORD wait_mark, WORD wait_enable, WORD posi_mode);

DMC_API short __stdcall dmc_conti_line_unit_G0 (WORD CardNo, WORD Crd, WORD AxisNum, WORD *AxisList, double*pPosList, WORD posi_mode, int mark);
DMC_API short __stdcall dmc_conti_arc_move_center_unit_G0 (WORD CardNo, WORD Crd, WORD AxisNum, WORD *AxisList, double* Target_Pos, double* Cen_Pos,WORD Arc_Dir, int Circle, WORD posi_mode, int mark);
DMC_API short __stdcall dmc_conti_arc_move_radius_unit_G0 (WORD CardNo, WORD Crd, WORD AxisNum, WORD *AxisList, double* Target_Pos, WORD Arc_Radius, WORD Arc_Dir, int Circle, WORD posi_mode, int mark);
DMC_API short __stdcall dmc_conti_arc_move_3points_unit_G0 (WORD CardNo, WORD Crd, WORD AxisNum, WORD *AxisList, double* Target_Pos, double* Mid_Pos, int Circle, WORD posi_mode, int mark);

DMC_API short __stdcall dmc_read_outport_array(WORD CardNo, uint32 portNum, uint32 *status);
DMC_API short __stdcall dmc_read_inport_array(WORD CardNo, uint32 portNum, uint32 *status);

DMC_API short __stdcall dmc_m_set_wait_flag(WORD CardNo, WORD Group, WORD FlagNo, WORD Wait_Flag);
DMC_API short __stdcall dmc_m_get_wait_flag(WORD CardNo, WORD Group, WORD FlagNo, WORD* Wait_Flag);

DMC_API short __stdcall dmc_set_backlash_unit_extern(WORD CardNo, WORD axis,double backlash, WORD mode, short dir,double vel,double acc,WORD time_ms);
DMC_API short __stdcall dmc_get_backlash_unit_extern(WORD CardNo, WORD axis,double *backlash, WORD*mode, short* dir,double* vel, double* acc,WORD* time_ms);

DMC_API short __stdcall nmc_write_rxpdo(WORD CardNo, WORD portnum, WORD slave_station_addr, WORD index, WORD subindex, WORD bitlength, BYTE* data);
DMC_API short __stdcall nmc_read_rxpdo(WORD CardNo, WORD portnum, WORD slave_station_addr, WORD index, WORD subindex, WORD bitlength, BYTE* data);
DMC_API short __stdcall nmc_read_txpdo(WORD CardNo, WORD portnum, WORD slave_station_addr, WORD index, WORD subindex, WORD bitlength, BYTE* data);

DMC_API short __stdcall dmc_syncmotion_set_enable(WORD CardNo, WORD slaveAxisNo, WORD enable);
DMC_API short __stdcall dmc_syncmotion_get_enable(WORD CardNo, WORD slaveAxisNo, WORD* enable);
DMC_API short __stdcall dmc_set_syncmotion_configparams (WORD CardNo, WORD masterAxisNo, WORD slaveAxisNo, WORD follow_src_sel, WORD master_type, double scale_coe, WORD dir_rev);
DMC_API short __stdcall dmc_get_syncmotion_configparams (WORD CardNo, WORD slaveAxisNo, WORD* masterAxisNo, WORD* follow_src_sel, WORD* master_type, double*scale_coe, WORD*dir_rev);
DMC_API short __stdcall dmc_syncmotion_cancle(WORD CardNo, WORD slaveAxisNo, double dec, double jerk);
DMC_API short __stdcall dmc_syncmotion_updatescale(WORD CardNo, WORD slaveAxisNo, double scale_coe);

DMC_API short __stdcall dmc_set_alm_control_function(WORD CardNo,WORD axis,WORD control_function);
DMC_API short __stdcall dmc_get_alm_control_function(WORD CardNo,WORD axis,WORD* control_function);

DMC_API short __stdcall dmc_calculate_axis_plan_time(WORD CardNo, double Dis,double Start_Vel, double Max_Vel,double End_Vel,double Tacc, double Tdec,double sTime, double* Tsum);



DMC_API short __stdcall dmc_conti_set_coordinate_params(WORD CardNo, WORD Crd, double T, double radius, double limit_vel);
DMC_API short __stdcall dmc_conti_get_coordinate_params(WORD CardNo, WORD Crd, double* T, double* radius,double* limit_vel);
DMC_API short __stdcall dmc_conti_set_corner_angle_param(WORD CardNo, WORD Crd, double dec_angle, double stop_angle, WORD enable);
DMC_API short __stdcall dmc_conti_get_corner_angle_param(WORD CardNo, WORD Crd, double* dec_angle, double* stop_angle, WORD* enable);
DMC_API short __stdcall dmc_conti_set_transvelocity_mode(WORD CardNo, WORD Crd, WORD transvel_mode);
DMC_API short __stdcall dmc_conti_get_transvelocity_mode(WORD CardNo, WORD Crd, WORD* transvel_mode);

DMC_API short __stdcall dmc_send_pack(WORD CardNo, WORD mode, WORD length,char* pBuf);

DMC_API short __stdcall dmc_conti_delay_set_mode(WORD CardNo, WORD Crd, WORD mode);



DMC_API short __stdcall dmc_conti_delay_get_mode(WORD CardNo, WORD Crd, WORD* mode);


DMC_API short __stdcall nmc_set_alias_address_enable(WORD CardNo, WORD enable);


DMC_API short __stdcall nmc_get_alias_address_enable(WORD CardNo, WORD* enable, WORD* states);


DMC_API short __stdcall dmc_get_system_version(WORD CardNo, char *SystemVersion);

DMC_API short __stdcall dmc_set_track_config_unit(WORD CardNo, WORD m_slave_axis_num, WORD* m_master_axis, WORD* m_slave_axis, WORD* m_start_distance, WORD* m_coordinate_axis, double* m_angle, double* m_master_vel, double* m_start_time, double* m_finish_time, double* m_sync_start_pos, double* m_sync_end_pos, double* m_finish_pos);
DMC_API short __stdcall dmc_get_track_config_unit(WORD CardNo, WORD slave_axis,WORD* m_track_state);
DMC_API short __stdcall dmc_add_move_config_unit(WORD CardNo, WORD add_axis, WORD added_axis, WORD enable);

DMC_API short __stdcall nmc_get_same_alias_address_slaves(WORD CardNo, WORD* SlaveNum, WORD* SlaveList);

DMC_API short __stdcall dmc_input_shaper_on(WORD CardNo,WORD axis, double* cnvA,DWORD* cnvT,WORD num);

DMC_API short __stdcall dmc_input_shaper_off (WORD CardNo,WORD axis);

DMC_API short __stdcall dmc_get_input_shaper_status(WORD CardNo, WORD axis ,WORD* status);

DMC_API short __stdcall dmc_set_input_shaper_compesation_enable(WORD CardNo,WORD axis, WORD enable);

DMC_API short __stdcall dmc_get_input_shaper_compesation_enable(WORD CardNo,WORD axis, WORD* enable);

DMC_API short __stdcall dmc_set_input_shaper_compesation(WORD CardNo,WORD axis, double T,double freq,double zeta,double alpha);

DMC_API short __stdcall dmc_get_input_shaper_compesation(WORD CardNo,WORD axis, double* T,double* freq,double* zeta,double* alpha);
DMC_API short __stdcall dmc_set_input_shaper_compesation_error(WORD CardNo, WORD axis, double delay_T, double pos_error);

DMC_API short __stdcall dmc_get_input_shaper_compesation_error(WORD CardNo, WORD axis, double* delay_T, double* pos_error);

DMC_API short __stdcall dmc_m_add_sigaxis_moveseg_data_multi(WORD CardNo, WORD group, WORD AxisNum, WORD* AxisList, double* Target_pos, uint32* mark);
DMC_API short __stdcall dmc_set_func_enable(WORD CardNo, WORD mode, WORD enable);
DMC_API short __stdcall dmc_get_func_enable(WORD CardNo, WORD mode, WORD* enable);
DMC_API short __stdcall nmc_set_home_profile_acc(WORD CardNo ,WORD axis,WORD home_mode,double Low_Vel, double High_Vel,double Tacc,double Tdec,double offsetpos );
DMC_API short __stdcall nmc_get_home_profile_acc(WORD CardNo,WORD axis,WORD* home_mode,double* Low_Vel,double* High_Vel,double* Tacc,double* Tdec,double* Offsetpos);
DMC_API short __stdcall dmc_get_ad_input_append(WORD CardNo,WORD Channel, double* Vout);

DMC_API short __stdcall dmc_conti_set_pwm_output_extern(WORD CardNo, WORD Crd, WORD pwm_no, WORD enable, double width_us, double frequency,uint32 number);
DMC_API short __stdcall dmc_set_pwm_mode(WORD CardNo, WORD pwm_no, WORD startmode, WORD stopmode);
DMC_API short __stdcall dmc_get_pwm_mode (WORD CardNo, WORD pwm_no, WORD* startmode, WORD* stopmode);
DMC_API short __stdcall dmc_conti_set_profile_unit(WORD CardNo,WORD Crd,double Min_Vel,double Max_vel,double Tacc,double Tdec,double Stop_Vel);
DMC_API short __stdcall dmc_conti_pmove_extern_unit(WORD CardNo, WORD Crd, WORD axis, double dist, double Min_Vel, double Max_Vel, double Tacc, double Tdec, double Stop_Vel, WORD posi_mode, WORD mode, int mark);
DMC_API short __stdcall dmc_conti_pack_on(WORD CardNo);
DMC_API short __stdcall dmc_conti_pack_off(WORD CardNo);
DMC_API short __stdcall dmc_conti_pack_flush(WORD CardNo);

DMC_API short __stdcall dmc_sine_oscillate_extern(WORD CardNo,WORD Axis,double Amplitude,double Frequency,double cycle,WORD param);
DMC_API short __stdcall dmc_sine_oscillate_extern_unit(WORD CardNo,WORD Axis,double Amplitude,double Frequency,double cycle,WORD param);
DMC_API short __stdcall dmc_write_outport_array(WORD CardNo, uint32 portNum, uint32 *status);
DMC_API short __stdcall dmc_conti_set_node_od(WORD CardNo,WORD Crd,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength, uint32 Value);
DMC_API short __stdcall dmc_m_add_trigger_set_od(WORD CardNo,WORD Group,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,uint32 Value, uint32 Mark);
DMC_API short __stdcall dmc_conti_set_rxpdo_extra(WORD CardNo,WORD Crd,WORD Address, WORD DataLen,WORD Mode,WORD ModeVal,uint32 Value);
DMC_API short __stdcall dmc_m_add_trigger_set_rxpdo_extra(WORD CardNo,WORD Group,WORD Address, WORD DataLen,WORD Mode,WORD ModeVal,uint32 Value,uint32 Mark);

DMC_API short __stdcall dmc_set_profile_limit_by_axis(WORD CardNo, WORD axis, WORD enable);
DMC_API short __stdcall dmc_get_profile_limit_by_axis(WORD CardNo, WORD axis, WORD* enable);

DMC_API short __stdcall dmc_set_leadscrew_comp_2D_config_unit(WORD CardNo, WORD axis, WORD* ref_axis,double* ref_axis_start_pos, double* ref_axis_length,WORD* ref_axis_segment, double* Compos);
DMC_API short __stdcall dmc_get_leadscrew_comp_2D_config_unit(WORD CardNo, WORD axis, WORD* ref_axis,double* ref_axis_start_pos, double* ref_axis_length, WORD* ref_axis_segment, double* Compos);
DMC_API short __stdcall dmc_set_leadscrew_comp_2D_angle_unit(WORD CardNo, WORD axis, WORD*ref_axis, double* ref_axis_start_pos, double* ref_axis_length, double angle);
DMC_API short __stdcall dmc_get_leadscrew_comp_2D_angle_unit(WORD CardNo, WORD axis, WORD*ref_axis, double* ref_axis_start_pos, double* ref_axis_length, double *angle);
DMC_API short __stdcall dmc_set_leadscrew_comp_2D_enable(WORD CardNo, WORD axis, WORD mode, WORD enable);
DMC_API short __stdcall dmc_get_leadscrew_comp_2D_enable(WORD CardNo, WORD axis, WORD *mode, WORD *enable);

DMC_API short __stdcall dmc_set_leadscrew_comp_2D_config_unit_ex(WORD CardNo, WORD table_index, WORD comp_axis, WORD* ref_axis, double* ref_axis_start_pos, double* ref_axis_length, WORD* ref_axis_segment, double* value);
DMC_API short __stdcall dmc_get_leadscrew_comp_2D_config_unit_ex(WORD CardNo, WORD table_index, WORD* comp_axis, WORD* ref_axis, double* ref_axis_start_pos, double* ref_axis_length, WORD* ref_axis_segment, double* value);
DMC_API short __stdcall dmc_set_leadscrew_comp_2D_angle_unit_ex(WORD CardNo, WORD table_index, WORD comp_axis, WORD* ref_axis, double* ref_axis_start_pos, double* ref_axis_length, double angle);
DMC_API short __stdcall dmc_get_leadscrew_comp_2D_angle_unit_ex(WORD CardNo, WORD table_index, WORD* comp_axis, WORD* ref_axis, double* ref_axis_start_pos, double* ref_axis_length, double *angle);
DMC_API short __stdcall dmc_set_leadscrew_comp_2D_enable_ex(WORD CardNo, WORD table_index, WORD mode, WORD enable);
DMC_API short __stdcall dmc_get_leadscrew_comp_2D_enable_ex(WORD CardNo, WORD table_index, WORD* mode, WORD* enable);

//ip???
DMC_API short __stdcall dmc_set_ipaddr( WORD CardNo,const char* IpAddr);
DMC_API short __stdcall dmc_get_ipaddr( WORD CardNo,char* IpAddr);

DMC_API short __stdcall dmc_clear_io_action(WORD CardNo,WORD Crd,WORD IoMask, WORD Mode);

DMC_API short __stdcall dmc_m_remain_space(WORD CardNo, WORD Crd, uint32* data);

DMC_API short __stdcall dmc_conti_set_trans_arc_speed_mode(WORD CardNo, WORD Crd, WORD mode);
DMC_API short __stdcall dmc_conti_get_trans_arc_speed_mode(WORD CardNo, WORD Crd, WORD *mode);

DMC_API short __stdcall dmc_conti_set_node_da_follow_speed(WORD CardNo, WORD Crd, WORD node_id, WORD da_no, double MaxVel, double MaxValue, double acc_offset, double dec_offset, double acc_dist, double dec_dist);
DMC_API short __stdcall dmc_conti_get_node_da_follow_speed(WORD CardNo, WORD Crd, WORD node_id, WORD da_no, double* MaxVel, double* MaxValue, double* acc_offset, double* dec_offset, double* acc_dist, double* dec_dist);
DMC_API short __stdcall dmc_m_set_factor_error(WORD CardNo, WORD axis, WORD enable, double Positive_error, double Negative_error, WORD retain);

DMC_API short __stdcall dmc_rAxis_comp_config(WORD CardNo,short axis, short enable, int cnt, double *angle, double *xPos, double *yPos);
DMC_API short __stdcall dmc_rAxis_comp_loop(WORD CardNo,short axis, short mode, double angle, double xPos, double yPos, double *new_xPos, double *new_yPos);

DMC_API short __stdcall dmc_pmove_pro_unit(WORD CardNo, WORD axis_no, double pos, WORD pos_mode, WORD plan_mode, unsigned char*  vel_param,unsigned char* plan_result);

DMC_API short __stdcall dmc_conti_delay_node_da_follow_to_start(WORD CardNo, WORD Crd, WORD node_id, WORD da_no, WORD delay_mode, double delay_value, double min_value, double max_value, double period);
DMC_API short __stdcall dmc_conti_ahead_node_da_follow_to_stop(WORD CardNo, WORD Crd, WORD node_id, WORD da_no, WORD ahead_mode, double ahead_value, double min_value, double max_value, double period);

DMC_API short __stdcall dmc_printall_time();
DMC_API short __stdcall dmc_clear_time();

DMC_API short __stdcall dmc_get_pmove_consumed_time (WORD CardNo,WORD* time_receive, WORD* time_process, WORD* time_send);

DMC_API short __stdcall dmc_t_pmove_extern_unit_acc(WORD CardNo, WORD axis, double MidPos,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,WORD posi_mode);
DMC_API short __stdcall dmc_t_pmove_extern_acc(WORD CardNo, WORD axis, double MidPos,double TargetPos, double Min_Vel,double Max_Vel, double stop_Vel, double acc,double dec,WORD posi_mode);
DMC_API short __stdcall dmc_t_pmove_extern_softstart_unit_acc(WORD CardNo, WORD axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, DWORD delay_ms, double Max_Vel2,double stop_vel2, double acc_time, double dec_time, WORD posi_mode);
DMC_API short __stdcall dmc_t_pmove_extern_softstart_acc(WORD CardNo, WORD axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, DWORD delay_ms, double Max_Vel2,double stop_vel2, double acc_time, double dec_time, WORD posi_mode);
DMC_API short __stdcall dmc_change_speed_unit_acc(WORD CardNo,WORD axis, double New_Vel,double Taccdec);
DMC_API short __stdcall dmc_change_speed_acc(WORD CardNo,WORD axis,double Curr_Vel,double Taccdec);

DMC_API short __stdcall dmc_set_node_da_follow_speed_extern(WORD CardNo, WORD node_id, WORD da_channel, WORD axis, WORD mode, WORD segment, double*vel,double*value);
DMC_API short __stdcall dmc_get_node_da_follow_speed_extern(WORD CardNo, WORD node_id, WORD da_channel, WORD*axis, WORD*mode, WORD *segment, double* vel,double* value);
DMC_API short __stdcall dmc_set_node_da_follow_speed_enable(WORD CardNo, WORD node_id, WORD da_channel, WORD enable);
DMC_API short __stdcall dmc_get_node_da_follow_speed_enable(WORD CardNo, WORD node_id, WORD da_channel, WORD *enable);


DMC_API short __stdcall  nmc_set_clear_fieldbus_state_on_soft_reset(WORD CardNo, WORD enable);
DMC_API short __stdcall  nmc_get_clear_fieldbus_state_on_soft_reset(WORD CardNo, WORD *enable);

DMC_API short __stdcall dmc_m_set_profile_unit_acc(WORD CardNo, WORD group, WORD axis, double start_vel, double max_vel, double tacc, double tdec, double stop_vel);
DMC_API short __stdcall nmc_get_master_state(WORD CardNo, uint32*States);

DMC_API short __stdcall dmc_m_set_profile_unit_acc(WORD CardNo, WORD group, WORD axis, double start_vel, double max_vel, double tacc, double tdec, double stop_vel);

#ifdef __cplusplus
}
#endif

#endif
