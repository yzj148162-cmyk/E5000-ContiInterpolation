using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;

namespace Leadshine

{
    public static class LTSMC
    {

        /*********************************************************************************************************
功能函数 
***********************************************************************************************************/
        //板卡配置	
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_connect_status(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_board_init(ushort ConnectNo, ushort ConnectType, string pconnectstring, uint baud);
        [DllImport("LTSMC.dll")]
        public static extern short smc_board_init_ex(ushort ConnectNo, ushort type, string pconnectstring, uint dwBaudRate, uint dwByteSize, uint dwParity, uint dwStopBits);
        [DllImport("LTSMC.dll")]
        public static extern short smc_board_close(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_CardInfList(ref ushort CardNum, uint[] CardTypeList, ushort[] ConnectTypeList);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_card_version(ushort ConnectNo, ref uint CardVersion);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_card_soft_version(ushort ConnectNo, ref uint FirmID, ref uint SubFirmID);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_release_version(ushort ConnectNo, byte[] ReleaseVersion);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_card_lib_version(ref uint LibVer);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_total_axes(ushort ConnectNo, ref uint TotalAxis); 	//读取指定卡轴数
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_total_ionum(ushort ConnectNo, ref ushort TotalIn, ref ushort TotalOut);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_total_adcnum(ushort ConnectNo, ref ushort TotalIn, ref ushort TotalOut);

        [DllImport("LTSMC.dll")]
        public static extern short smc_get_total_liners(ushort ConnectNo, ref uint TotalLiner);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_debug_mode(ushort mode, string FileName);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_debug_mode(ref ushort mode, byte[] FileName);
        [DllImport("LTSMC.dll")]
        public static extern short smc_format_flash(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_ipaddr(ushort ConnectNo, byte[] IpAddr);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ipaddr(ushort ConnectNo, byte[] IpAddr);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_com(ushort ConnectNo, ushort com, uint dwBaudRate, ushort wByteSize, ushort wParity, ushort wStopBits);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_com(ushort ConnectNo, ushort com, ref uint dwBaudRate, ref ushort wByteSize, ref ushort wParity, ref ushort dwStopBits);

        //序列号
        [DllImport("LTSMC.dll")]
        public static extern short smc_write_sn(ushort ConnectNo, UInt64 sn);
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_sn(ushort ConnectNo, ref UInt64 sn);

        //加密
        [DllImport("LTSMC.dll")]
        public static extern short smc_write_password(ushort ConnectNo, string str_pass);
        [DllImport("LTSMC.dll")]
        public static extern short smc_check_password(ushort ConnectNo, string str_pass);

        [DllImport("LTSMC.dll")]
        //public static extern short smc_enter_password(ushort ConnectNo, byte[] str_pass);
        public static extern short smc_enter_password(ushort ConnectNo, string str_pass);

        [DllImport("LTSMC.dll")]
        public static extern short smc_modify_password(ushort ConnectNo, string spassold, string spass);


        //脉冲模式		
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_pulse_outmode(ushort ConnectNo, ushort axis, ushort outmode);	//设定脉冲输出模式
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pulse_outmode(ushort ConnectNo, ushort axis, ref ushort outmode);	//读取脉冲输出模式
        //脉冲当量
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_equiv(ushort ConnectNo, ushort axis, double equiv);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_equiv(ushort ConnectNo, ushort axis, ref double equiv);
        //反向间隙
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_backlash_unit(ushort ConnectNo, ushort axis, double backlash);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_backlash_unit(ushort ConnectNo, ushort axis, ref double backlash);



        //编码器		
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_counter_inmode(ushort ConnectNo, ushort axis, ushort mode);	//设定编码器的计数方式
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_counter_inmode(ushort ConnectNo, ushort axis, ref ushort mode);	//读取编码器的计数方式
        [DllImport("LTSMC.dll")]
        public static extern int smc_get_encoder(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_encoder(ushort ConnectNo, ushort axis, int encoder_value);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_ez_mode(ushort ConnectNo, ushort axis, ushort ez_logic, ushort ez_mode, double filter);	//设置EZ信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ez_mode(ushort ConnectNo, ushort axis, ref ushort ez_logic, ref ushort ez_mode, ref double filter);	//读取设置EZ信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_counter_reverse(ushort ConnectNo, ushort axis, ushort reverse);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_counter_reverse(ushort ConnectNo, ushort axis, ref ushort reverse);

        //辅助编码器
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_extra_encoder(ushort ConnectNo, ushort axis, int pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_extra_encoder(ushort ConnectNo, ushort axis, ref int pos);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_extra_encoder_mode(ushort ConnectNo, ushort channel, ushort inmode, ushort multi);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_extra_encoder_mode(ushort ConnectNo, ushort channel, ref ushort inmode, ref ushort multi);



        //单轴速度参数
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_profile_unit(ushort ConnectNo, ushort axis, double Min_Vel, double Max_Vel, double Tacc, double Tdec, double Stop_Vel);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_profile_unit(ushort ConnectNo, ushort axis, ref double Min_Vel, ref double Max_Vel, ref double Tacc, ref double Tdec, ref double Stop_Vel);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_profile_unit_acc(ushort ConnectNo, ushort axis, double Min_Vel, double Max_Vel, double acc, double dec, double Stop_Vel);

        [DllImport("LTSMC.dll")]
        public static extern short smc_get_profile_unit_acc(ushort ConnectNo, ushort axis, ref double Min_Vel, ref double Max_Vel, ref double acc, ref double dec, ref double Stop_Vel);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_s_profile(ushort ConnectNo, ushort axis, ushort s_mode, double s_para);	//设置平滑速度曲线参数
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_s_profile(ushort ConnectNo, ushort axis, ushort s_mode, ref double s_para);	//读取平滑速度曲线参数 兼容smc5800 s_mode改用指针返回参数值
        //单轴运动
        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_unit(ushort ConnectNo, ushort axis, double Dist, ushort posi_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_vmove(ushort ConnectNo, ushort axis, ushort dir);	//指定轴做连续运动
        [DllImport("LTSMC.dll")]
        public static extern short smc_change_speed(ushort ConnectNo, ushort axis, double Curr_Vel, double Taccdec);	//在线改变指定轴的当前运动速度

        //回零运动	
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_home_pin_logic(ushort ConnectNo, ushort axis, ushort org_logic, double filter); 	//设置HOME信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_home_pin_logic(ushort ConnectNo, ushort axis, ref ushort org_logic, ref double filter); 	//读取设置HOME信号       
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_home_profile_unit(ushort ConnectNo, ushort axis, double Low_Vel, double High_Vel, double Tacc, double Tdec);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_home_profile_unit(ushort ConnectNo, ushort axis, ref double Low_Vel, ref double High_Vel, ref double Tacc, ref double Tdec);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_ez_count(ushort ConnectNo, ushort axis, ushort ez_count);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ez_count(ushort ConnectNo, ushort axis, ref ushort ez_count);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_homemode(ushort ConnectNo, ushort axis, ushort home_dir, double vel_mode, ushort mode, ushort pos_source);//设定指定轴的回原点模式
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_homemode(ushort ConnectNo, ushort axis, ref ushort home_dir, ref double vel_mode, ref ushort home_mode, ref ushort pos_source);//读取指定轴的回原点模式
        [DllImport("LTSMC.dll")]
        public static extern short smc_home_move(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_home_result(ushort ConnectNo, ushort axis, ref ushort state);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_home_position_unit(ushort ConnectNo, ushort axis, ushort enable, double position);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_home_position_unit(ushort ConnectNo, ushort axis, ref ushort enable, ref double position);
        //手轮运动
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_set_axislist(ushort ConnectNo, ushort AxisSelIndex, ushort AxisNum, ushort[] AxisList);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_get_axislist(ushort ConnectNo, ushort AxisSelIndex, ref ushort AxisNum, ushort[] AxisList);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_set_ratiolist(ushort ConnectNo, ushort AxisSelIndex, ushort StartRatioIndex, ushort RatioSelNum, double[] RatioList);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_get_ratiolist(ushort ConnectNo, ushort AxisSelIndex, ushort StartRatioIndex, ushort RatioSelNum, double[] RatioList);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_set_mode(ushort ConnectNo, ushort InMode, ushort IfHardEnable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_get_mode(ushort ConnectNo, ref ushort InMode, ref ushort IfHardEnable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_set_index(ushort ConnectNo, ushort AxisSelIndex, ushort RatioSelIndex);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_get_index(ushort ConnectNo, ref ushort AxisSelIndex, ref ushort RatioSelIndex);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_move(ushort ConnectNo, ushort ForceMove);
        [DllImport("LTSMC.dll")]
        public static extern short smc_handwheel_stop(ushort ConnectNo);
        //原点锁存
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_homelatch_mode(ushort ConnectNo, ushort axis, ushort enable, ushort logic, ushort source);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_homelatch_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort logic, ref ushort source);
        [DllImport("LTSMC.dll")]
        public static extern int smc_get_homelatch_flag(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_reset_homelatch_flag(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_homelatch_value_unit(ushort ConnectNo, ushort axis, ref double value);
        //EZ锁存
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_ezlatch_mode(ushort ConnectNo, ushort axis, ushort enable, ushort logic, ushort source);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ezlatch_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort logic, ref ushort source);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ezlatch_flag(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_reset_ezlatch_flag(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ezlatch_value_unit(ushort ConnectNo, ushort axis, ref double pos_by_mm);
        //[DllImport("LTSMC.dll")]
        //public static extern short smc_ltc_get_value_unit(ushort ConnectNo, ushort latch, ushort axis, ref double value);
        //PVT运动
        [DllImport("LTSMC.dll")]
        public static extern short smc_pvt_table_unit(ushort ConnectNo, ushort iaxis, uint count, double[] pTime, double[] pPos, double[] pVel);
        [DllImport("LTSMC.dll")]
        public static extern short smc_pts_table_unit(ushort ConnectNo, ushort iaxis, uint count, double[] pTime, double[] pPos, double[] pPercent);
        [DllImport("LTSMC.dll")]
        public static extern short smc_pvts_table_unit(ushort ConnectNo, ushort iaxis, uint count, double[] pTime, double[] pPos, double velBegin, double velEnd);
        [DllImport("LTSMC.dll")]
        public static extern short smc_ptt_table_unit(ushort ConnectNo, ushort iaxis, uint count, double[] pTime, double[] pPos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_pvt_move(ushort ConnectNo, ushort AxisNum, ushort[] AxisList);
        //安全机制
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_el_mode(ushort ConnectNo, ushort axis, ushort enable, ushort el_logic, ushort el_mode); 	//设置EL信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_el_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort el_logic, ref ushort el_mode); 	//读取设置EL信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_emg_mode(ushort ConnectNo, ushort axis, ushort enable, ushort emg_logic); 	//设置EMG信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_emg_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort emg_logic); 	//读取设置EMG信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_softlimit_unit(ushort ConnectNo, ushort axis, ushort enable, ushort source_sel, ushort SL_action, double N_limit, double P_limit);	//设置软限位参数
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_softlimit_unit(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort source_sel, ref ushort SL_action, ref double N_limit, ref double P_limit);	//读取软限位参数
        //轴IO映射
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_axis_io_map(ushort ConnectNo, ushort Axis, ushort IoType, ushort MapIoType, ushort MapIoIndex, double Filter);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_axis_io_map(ushort ConnectNo, ushort Axis, ushort IoType, ref ushort MapIoType, ref ushort MapIoIndex, ref double Filter);
        //状态监控
        [DllImport("LTSMC.dll")]
        public static extern short smc_check_done(ushort ConnectNo, ushort axis);	//读取指定轴的运动状态

        [DllImport("LTSMC.dll")]
        public static extern short smc_check_done_ex(ushort ConnectNo, ushort axis, ref ushort state);	//读取指定轴的运动状态
        [DllImport("LTSMC.dll")]
        public static extern short smc_stop(ushort ConnectNo, ushort axis, ushort stop_mode);	//单轴停止
        [DllImport("LTSMC.dll")]
        public static extern short smc_emg_stop(ushort ConnectNo);	//紧急停止所有轴       
        [DllImport("LTSMC.dll")]
        public static extern short smc_check_done_multicoor(ushort ConnectNo, ushort Crd);
        [DllImport("LTSMC.dll")]
        public static extern short smc_stop_multicoor(ushort ConnectNo, ushort Crd, ushort stop_mode);
        [DllImport("LTSMC.dll")]
        public static extern uint smc_axis_io_status(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_axis_io_status_ex(ushort ConnectNo, ushort axis, ref uint state);
        [DllImport("LTSMC.dll")]
        public static extern uint smc_axis_io_enable_status(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_axis_run_mode(ushort ConnectNo, ushort axis, ref ushort run_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_current_speed_unit(ushort ConnectNo, ushort axis, ref double current_speed);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_position_unit(ushort ConnectNo, ushort axis, double pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_position_unit(ushort ConnectNo, ushort axis, ref double pos);
        [DllImport("LTSMC.dll")]
        public static extern int smc_get_target_position_unit(ushort ConnectNo, ushort axis, ref double pos);	//读取指定轴的目标位置
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_workpos_unit(ushort ConnectNo, ushort axis, double pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_workpos_unit(ushort ConnectNo, ushort axis, ref double pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_stop_reason(ushort ConnectNo, ushort axis, ref int StopReason);
        [DllImport("LTSMC.dll")]
        public static extern short smc_clear_stop_reason(ushort ConnectNo, ushort axis);
        //通用IO		
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_inbit(ushort ConnectNo, ushort bitno); 	//读取输入口的状态
        [DllImport("LTSMC.dll")]
        public static extern short smc_write_outbit(ushort ConnectNo, ushort bitno, ushort on_off); 	//设置输出口的状态
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_outbit(ushort ConnectNo, ushort bitno);  	//读取输出口的状态
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_outbit_ex(ushort ConnectNo, ushort bitno, ref ushort state);  	//读取输出口的状态
        [DllImport("LTSMC.dll")]
        public static extern uint smc_read_inport(ushort ConnectNo, ushort portno); 	//读取输入端口的值
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_inport_ex(ushort ConnectNo, ushort portno, ref uint state); 	//读取输入端口的值
        [DllImport("LTSMC.dll")]
        public static extern uint smc_read_outport(ushort ConnectNo, ushort portno); 	//读取输出端口的值
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_outport_ex(ushort ConnectNo, ushort portno, ref uint state); 	//读取输出端口的值
        [DllImport("LTSMC.dll")]
        public static extern short smc_write_outport(ushort ConnectNo, ushort portno, uint outport_val);  	//设置输出端口的值
        [DllImport("LTSMC.dll")]
        public static extern short smc_reverse_outbit(ushort ConnectNo, ushort bitno, double reverse_time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_io_count_mode(ushort ConnectNo, ushort bitno, ushort mode, double filter);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_io_count_mode(ushort ConnectNo, ushort bitno, ref ushort mode, ref double filter);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_io_count_value(ushort ConnectNo, ushort bitno, uint CountValue);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_io_count_value(ushort ConnectNo, ushort bitno, ref uint CountValue);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_io_dstp_mode(ushort ConnectNo, ushort axis, ushort enable, ushort logic); 	//enable:0-禁用，1-按时间减速停止，2-按距离减速停止
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_io_dstp_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort logic);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_dec_stop_time(ushort ConnectNo, ushort axis, double time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_dec_stop_time(ushort ConnectNo, ushort axis, ref double time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_io_pwmoutput(ushort ConnectNo, ushort outbit, double fre, double duty, uint counts);//设置IO输出一定脉冲个数的PWM波形曲线
        [DllImport("LTSMC.dll")]
        public static extern short smc_clear_io_pwmoutput(ushort ConnectNo, ushort outbit);//清除IO输出PWM波形曲线
        //专用IO操作
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_alm_mode(ushort ConnectNo, ushort axis, ushort enable, ushort alm_logic, ushort alm_action);	//设置ALM信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_alm_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort alm_logic, ref ushort alm_action);	//读取设置ALM信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_inp_mode(ushort ConnectNo, ushort axis, ushort enable, ushort inp_logic);	//设置INP信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_inp_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort inp_logic);	//读取设置INP信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_rdy_mode(ushort ConnectNo, ushort axis, ushort enable, ushort rdy_logic);	//设置RDY信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_rdy_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort rdy_logic);	//读取设置RDY信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_erc_mode(ushort ConnectNo, ushort axis, ushort enable, ushort erc_logic, ushort erc_width, ushort erc_off_time);	//设置ERC信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_erc_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort erc_logic, ref ushort erc_width, ref ushort erc_off_time);	//读取设置ERC信号

        [DllImport("LTSMC.dll")]
        public static extern short smc_write_sevon_pin(ushort ConnectNo, ushort axis, ushort on_off); 	//输出SEVON信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_sevon_pin(ushort ConnectNo, ushort axis); 	//读取SEVON信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_rdy_pin(ushort ConnectNo, ushort axis); 	//读取RDY状态
        [DllImport("LTSMC.dll")]
        public static extern short smc_write_erc_pin(ushort ConnectNo, ushort axis, ushort on_off); 	//控制ERC信号输出
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_erc_pin(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_write_sevrst_pin(ushort ConnectNo, ushort axis, ushort on_off); 	//输出伺服复位信号
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_sevrst_pin(ushort ConnectNo, ushort axis); 	//读伺服复位信号

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_dstp_mode(ushort ConnectNo, ushort axis, ushort enable, ushort logic, uint time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_dstp_mode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort logic, ref uint time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_dstp_time(ushort ConnectNo, ushort axis, uint time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_dstp_time(ushort ConnectNo, ushort axis, ref uint time);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_io_dstp_bitno(ushort ConnectNo, ushort axis, ushort bitno, double filter); 	//设置通用输入口的一位位减速停止IO口
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_io_dstp_bitno(ushort ConnectNo, ushort axis, ref ushort bitno, ref double filter);
        //插补参数设置
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_vector_profile_unit(ushort ConnectNo, ushort Crd, double Min_Vel, double Max_Vel, double Tacc, double Tdec, double Stop_Vel);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_vector_profile_unit(ushort ConnectNo, ushort Crd, ref double Min_Vel, ref double Max_Vel, ref double Tacc, ref double Tdec, ref double Stop_Vel);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_vector_tacc(ushort ConnectNo, ushort Crd, double Tacc);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_vector_acc(ushort ConnectNo, ushort Crd, double acc);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_vector_speed_unit(ushort ConnectNo, ushort Crd, double Max_vel);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_vector_s_profile(ushort ConnectNo, ushort Crd, ushort s_mode, double s_para);	//设置平滑速度曲线参数
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_vector_s_profile(ushort ConnectNo, ushort Crd, ushort s_mode, ref double s_para);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_vector_dec_stop_time(ushort ConnectNo, ushort Crd, double time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_vector_dec_stop_time(ushort ConnectNo, ushort Crd, ref double time);

        //单段插补       
        [DllImport("LTSMC.dll")]
        public static extern short smc_line_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] Dist, ushort posi_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_arc_move_center_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] Target_Pos, double[] Cen_Pos, ushort Arc_Dir, int Circle, ushort posi_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_arc_move_radius_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] Target_Pos, double Arc_Radius, ushort Arc_Dir, int Circle, ushort posi_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_arc_move_3points_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] Target_Pos, double[] Mid_Pos, int Circle, ushort posi_mode);

        //单段矩形插补
        [DllImport("LTSMC.dll")]
        public static extern short smc_rectangle_move_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] TargetPos, double[] MaskPos, int Count, ushort rect_mode, ushort posi_mode);

        //连续矩形插补
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_rectangle_move_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] TargetPos, double[] MaskPos, int Count, ushort rect_mode, ushort posi_mode, int mark);


        [DllImport("LTSMC.dll")]
        public static extern short smc_axis_follow_line_enable(ushort ConnectNo, ushort Crd, ushort enable_flag);



        //连续插补
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_open_list(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList);	//打开连续缓存区
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_close_list(ushort ConnectNo, ushort Crd);	//关闭连续缓存区
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_stop_list(ushort ConnectNo, ushort Crd, ushort stop_mode);	//连续插补中停止
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_pause_list(ushort ConnectNo, ushort Crd);	//连续插补中暂停
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_start_list(ushort ConnectNo, ushort Crd);	//开始连续插补
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_check_done(ushort ConnectNo, ushort Crd);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_get_run_state(ushort ConnectNo, ushort Crd);//0-运行，1-暂停，2-正常停止，3-未启动，4-空闲
        [DllImport("LTSMC.dll")]
        public static extern int smc_conti_remain_space(ushort ConnectNo, ushort Crd);	//查连续插补剩余缓存数
        [DllImport("LTSMC.dll")]
        public static extern int smc_conti_read_current_mark(ushort ConnectNo, ushort Crd);	//读取当前连续插补段的标号
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_change_speed_ratio(ushort ConnectNo, ushort Crd, double percent);	//设置插补中动态变速
        //Blend模式
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_blend(ushort ConnectNo, ushort Crd, ushort enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_get_blend(ushort ConnectNo, ushort Crd, ref ushort enable);
        //小线段前瞻
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_lookahead_mode(ushort ConnectNo, ushort Crd, ushort enable, int LookaheadSegments, double PathError, double LookaheadAcc);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_get_lookahead_mode(ushort ConnectNo, ushort Crd, ref ushort enable, ref int LookaheadSegments, ref double PathError, ref double LookaheadAcc);
        //设置每段速度
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_taccdec(ushort ConnectNo, ushort Crd, double Taccdec);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_speed_unit(ushort ConnectNo, ushort Crd, double Max_vel);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_override(ushort ConnectNo, ushort Crd, double Percent);
        //连续插补轨迹
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_line_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] pPosList, ushort posi_mode, int mark);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_arc_move_center_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] Target_Pos, double[] Cen_Pos, ushort Arc_Dir, int Circle, ushort posi_mode, int mark);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_arc_move_radius_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] Target_Pos, double Arc_Radius, ushort Arc_Dir, int Circle, ushort posi_mode, int mark);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_arc_move_3points_unit(ushort ConnectNo, ushort Crd, ushort AxisNum, ushort[] AxisList, double[] Target_Pos, double[] Mid_Pos, int Circle, ushort posi_mode, int mark);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_pmove_unit(ushort ConnectNo, ushort Crd, ushort axis, double dist, ushort posi_mode, ushort mode, int imark);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_involute_mode(ushort ConnectNo, ushort Crd, ushort mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_get_involute_mode(ushort ConnectNo, ushort Crd, ref ushort mode);
        //连续插补PWM
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_pwm_output(ushort ConnectNo, ushort Crd, ushort PwmNo, double fDuty, double fFre);
        /**********PWM速度跟随**************
        mode:跟随模式0-不跟随 保持状态 1-不跟随 输出低电平2-不跟随 输出高电平3-跟随 占空比自动调整4-跟随 频率自动调整
        MaxVel:最大运行速度，单位unit
        MaxValue:最大输出占空比或者频率
        OutValue：设置输出频率或占空比
        *************************************/
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_pwm_follow_speed(ushort ConnectNo, ushort Crd, ushort pwm_no, ushort mode, double MaxVel, double MaxValue, double OutValue);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_get_pwm_follow_speed(ushort ConnectNo, ushort Crd, ushort pwm_no, ref ushort mode, ref double MaxVel, ref double MaxValue, ref double OutValue);
        //设置PWM开关对应的占空比
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_pwm_onoff_duty(ushort ConnectNo, ushort PwmNo, double fOnDuty, double fOffDuty);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pwm_onoff_duty(ushort ConnectNo, ushort PwmNo, ref double fOnDuty, ref double fOffDuty);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_delay_pwm_to_start(ushort ConnectNo, ushort Crd, ushort pwmno, ushort on_off, double delay_value, ushort delay_mode, double ReverseTime);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_delay_pwm_to_stop(ushort ConnectNo, ushort Crd, ushort pwmno, ushort on_off, double delay_time, double ReverseTime);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_ahead_pwm_to_stop(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double ahead_value, ushort ahead_mode, double ReverseTime);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_write_pwm(ushort ConnectNo, ushort Crd, ushort pwmno, ushort on_off, double ReverseTime);
        //PWM功能
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_pwm_enable(ushort ConnectNo, ushort PwmNo, ushort enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pwm_enable(ushort ConnectNo, ushort PwmNo, ref ushort enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_pwm_output(ushort ConnectNo, ushort PwmNo, double fDuty, double fFre);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pwm_output(ushort ConnectNo, ushort PwmNo, ref double fDuty, ref double fFre);
        //位置比较
        //单轴位置比较		
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_set_config(ushort ConnectNo, ushort axis, ushort enable, ushort cmp_source); 	//配置比较器
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_get_config(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort cmp_source);	//读取配置比较器
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_clear_points(ushort ConnectNo, ushort axis); 	//清除所有比较点
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_add_point_unit(ushort ConnectNo, ushort axis, double pos, ushort dir, ushort action, uint actpara); 	//添加比较点
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_get_current_point_unit(ushort ConnectNo, ushort axis, ref double pos); 	//读取当前比较点
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_get_points_runned(ushort ConnectNo, ushort axis, ref int pointNum); 	//查询已经比较过的点
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_get_points_remained(ushort ConnectNo, ushort axis, ref int pointNum); 	//查询可以加入的比较点数量

        //二维位置比较
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_set_config_extern(ushort ConnectNo, ushort enable, ushort cmp_source); 	//配置比较器
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_get_config_extern(ushort ConnectNo, ref ushort enable, ref ushort cmp_source);	//读取配置比较器
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_clear_points_extern(ushort ConnectNo); 	//清除所有比较点
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_add_point_extern_unit(ushort ConnectNo, ushort[] axis, double[] pos, ushort[] dir, ushort action, uint actpara); 	//添加两轴位置比较点
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_get_current_point_extern_unit(ushort ConnectNo, double[] pos); 	//读取当前比较点
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_get_points_runned_extern(ushort ConnectNo, ref int pointNum); 	//查询已经比较过的点
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_get_points_remained_extern(ushort ConnectNo, ref int pointNum); 	//查询可以加入的比较点数量

        //高速位置比较
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_set_mode(ushort ConnectNo, ushort hcmp, ushort cmp_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_get_mode(ushort ConnectNo, ushort hcmp, ref ushort cmp_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_set_config(ushort ConnectNo, ushort hcmp, ushort axis, ushort cmp_source, ushort cmp_logic, int time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_get_config(ushort ConnectNo, ushort hcmp, ref ushort axis, ref ushort cmp_source, ref ushort cmp_logic, ref int time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_add_point_unit(ushort ConnectNo, ushort hcmp, double cmp_pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_set_liner_unit(ushort ConnectNo, ushort hcmp, double Increment, int Count);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_get_liner_unit(ushort ConnectNo, ushort hcmp, ref double Increment, ref int Count);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_get_current_state_unit(ushort ConnectNo, ushort hcmp, ref int remained_points, ref double current_point, ref int runned_points);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_clear_points(ushort ConnectNo, ushort hcmp);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_2d_set_enable(ushort ConnectNo, ushort hcmp, ushort cmp_enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_2d_get_current_state_unit(ushort ConnectNo, ushort hcmp, ref int remained_points, ref double x_current_point, ref double y_current_point, ref int runned_points, ref ushort current_state, ref ushort current_outbit);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_2d_fifo_get_state(ushort ConnectNo, ushort hcmp, ref long remained_points);
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_cmp_pin(ushort ConnectNo, ushort hcmp);
        [DllImport("LTSMC.dll")]
        public static extern short smc_write_cmp_pin(ushort ConnectNo, ushort hcmp, ushort on_off);
        //点胶参数设置
        [DllImport("LTSMC.dll")]
        public static extern short smc_glue_set_profile(ushort ConnectNo, ushort glue, ushort io, ushort on_off, double[] Offset, double[] dist, double[] time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_glue_get_profile(ushort ConnectNo, ushort glue, ref ushort io, ref ushort on_off, double[] Offset, double[] dist, double[] time);
        //文件操作
        [DllImport("LTSMC.dll")]
        public static extern short smc_download_file(ushort ConnectNo, string pfilename, byte[] pfilenameinControl, ushort filetype);
        [DllImport("LTSMC.dll")]
        public static extern short smc_download_memfile(ushort ConnectNo, byte[] pbuffer, uint buffsize, byte[] pfilenameinControl, ushort filetype);
        [DllImport("LTSMC.dll")]
        public static extern short smc_upload_file(ushort ConnectNo, string pfilename, byte[] pfilenameinControl, ushort filetype);
        [DllImport("LTSMC.dll")]
        public static extern short smc_upload_memfile(ushort ConnectNo, byte[] pbuffer, uint buffsize, byte[] pfilenameinControl, ref uint puifilesize, ushort filetype);
        [DllImport("LTSMC.dll")]
        public static extern short smc_download_file_to_ram(ushort ConnectNo, string pfilename, ushort filetype);
        [DllImport("LTSMC.dll")]
        public static extern short smc_download_memfile_to_ram(ushort ConnectNo, byte[] pbuffer, uint buffsize, ushort filetype);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_progress(ushort ConnectNo, ref float progress);
        //寄存器操作
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_modbus_0x(ushort ConnectNo, ushort start, ushort inum, byte[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_modbus_0x(ushort ConnectNo, ushort start, ushort inum, byte[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_modbus_4x(ushort ConnectNo, ushort start, ushort inum, ushort[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_modbus_4x(ushort ConnectNo, ushort start, ushort inum, ushort[] pdata);
        //Basic变量
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_array(ushort ConnectNo, string name, uint index, long[] var, ref int num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_modify_array(ushort ConnectNo, string name, uint index, long[] var, int num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_var(ushort ConnectNo, string varstring, long[] var, ref int num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_modify_var(ushort ConnectNo, string varstring, long[] var, int varnum);
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_array_ex(ushort ConnectNo, string name, uint index, double[] var, ref int num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_modify_array_ex(ushort ConnectNo, string name, uint index, double[] var, int num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_var_ex(ushort ConnectNo, string varstring, double[] var, ref int num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_modify_var_ex(ushort ConnectNo, string varstring, double[] var, int varnum);
        [DllImport("LTSMC.dll")]
        public static extern short smc_write_array_ex(ushort ConnectNo, string name, uint startindex, double[] var, int num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_stringtype(ushort ConnectNo, string varstring, ref int m_Type, ref int num);
        //Basic控制
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_state(ushort ConnectNo, ref ushort State);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_run(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_pause(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_stop(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_continue_run(ushort ConnectNo);
        //Basic调试
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_current_line(ushort ConnectNo, ref int line);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_break_info(ushort ConnectNo, int[] line, int linenum);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_step_over(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_step_run(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_message(ushort ConnectNo, byte[] pbuff, uint uimax, ref uint puiread);
        [DllImport("LTSMC.dll")]
        public static extern short smc_basic_command(ushort ConnectNo, byte[] pszCommand, byte[] psResponse, uint uiResponseLength);
        //G代码

        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_start(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_stop(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_pause(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_state(ushort ConnectNo, ref ushort state);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_clear_file(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_delete_file(ushort ConnectNo, byte[] pfilenameinControl);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_current_line(ushort ConnectNo, ref uint line, byte[] pCurLine);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_set_current_file(ushort ConnectNo, byte[] pFileName);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_get_current_file(ushort ConnectNo, byte[] pFileName, ref short fileid);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_get_first_file(ushort ConnectNo, byte[] pfilenameinControl, ref uint pFileSize);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_get_next_file(ushort ConnectNo, byte[] pfilenameinControl, ref uint pFileSize);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_check_file(ushort ConnectNo, byte[] pfilenameinControl, ref byte pbIfExist, ref uint pFileSize);
        [DllImport("LTSMC.dll")]
        public static extern short smc_gcode_get_file_profile(ushort ConnectNo, ref uint maxfilenum, ref uint maxfilesize);

        //public static extern short smc_gcode_set_step_state();


        //当前位置
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_encoder_unit(ushort ConnectNo, ushort axis, double pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_encoder_unit(ushort ConnectNo, ushort axis, ref double pos);

        //变速变位
        [DllImport("LTSMC.dll")]
        public static extern short smc_reset_target_position_unit(ushort ConnectNo, ushort axis, double New_Pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_update_target_position_unit(ushort ConnectNo, ushort axis, double New_Pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_change_speed_unit(ushort ConnectNo, ushort axis, double New_Vel, double Taccdec);


        //掉电保存寄存器操作
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg(ushort ConnectNo, ushort start, ushort inum, byte[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg(ushort ConnectNo, ushort start, ushort inum, byte[] pdata);
        //参数文件
        [DllImport("LTSMC.dll")]
        //public static extern short smc_download_parafile(ushort ConnectNo, byte[] FileName);
        public static extern short smc_download_parafile(ushort ConnectNo, string FileName);
        [DllImport("LTSMC.dll")]
        public static extern short smc_upload_parafile(ushort ConnectNo, byte[] FileName);
        //圆弧限制
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_arc_limit(ushort ConnectNo, ushort Crd, ushort Enable, double MaxCenAcc, double MaxArcError);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_arc_limit(ushort ConnectNo, ushort Crd, ref ushort Enable, ref double MaxCenAcc, ref double MaxArcError);
        //
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_trace_data(ushort ConnectNo, ushort axis, int bufsize, double[] time, double[] pos, double[] vel, double[] acc, ref int recv_num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_trace_start(ushort ConnectNo, ushort AxisNum, ushort[] AxisList);
        [DllImport("LTSMC.dll")]
        public static extern short smc_trace_stop(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short smc_trace_set_source(ushort ConnectNo, ushort source);
        //
        //实时时钟
        [DllImport("LTSMC.dll")]
        public static extern short smc_rtc_get_time(ushort ConnectNo, ref int year, ref int month, ref int day, ref int hour, ref int min, ref int sec);
        [DllImport("LTSMC.dll")]
        public static extern short smc_rtc_set_time(ushort ConnectNo, int year, int month, int day, int hour, int min, int sec);

        //
        //
        [DllImport("LTSMC.dll")]
        public static extern short smc_soft_reset(ushort ConnectNo);

        [DllImport("LTSMC.dll")]
        public static extern short smc_board_reset(ushort ConnectNo);

        [DllImport("LTSMC.dll")]
        public static extern short smc_original_reset(ushort ConnectNo);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_reset_canopen(ushort ConnectNo);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_total_axes(ushort ConnectNo, ref uint TotalAxis); 	//读取指定卡轴数
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_total_ionum(ushort ConnectNo, ref ushort TotalIn, ref ushort TotalOut);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_total_adcnum(ushort ConnectNo, ref ushort TotalIn, ref ushort TotalOut);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_slave_nodes(ushort ConnectNo, ushort PortNum, ushort baudrate, ushort[] NodeID, ref ushort NodeNum);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_slave_state(ushort ConnectNo, ushort SlaveId, ref ushort SlaveState);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_clear_errcode(ushort ConnectNo, ushort PortNo);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_EmergeneyMessege_Nodes(ushort ConnectNo, ushort PortNum, uint[] NodeMsg, ref ushort MsgNum);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_errcode(ushort ConnectNo, ushort PortNum, ref ushort errcode);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_axis_errcode(ushort ConnectNo, ushort axis, ref uint Errcode);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_clear_axis_errcode(ushort ConnectNo, ushort iaxis);




        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_LostHeartbeat_Nodes(ushort ConnectNo, ushort PortNum, ushort[] NodeID, ref ushort NodeNum);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_manager_od(ushort ConnectNo, ushort PortNum, ushort index, ushort subindex, ushort valuelength, ref uint value);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_manager_para(ushort ConnectNo, ushort PortNum, ref uint baudrate, ref ushort ManagerID);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_node_od(ushort ConnectNo, ushort PortNum, ushort nodenum, ushort index, ushort subindex, ushort valuelength, ref uint value);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_reset_to_factory(ushort ConnectNo, ushort PortNum, ushort NodeNum);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_SendNmtCommand(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort NmtCommand);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_alarm_clear(ushort ConnectNo, ushort PortNum, ushort nodenum);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_axis_enable(ushort ConnectNo, ushort PortNum, ushort nodenum);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_manager_od(ushort ConnectNo, ushort PortNum, ushort index, ushort subindex, ushort valuelength, uint value);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_manager_para(ushort ConnectNo, ushort PortNum, int baudrate, ushort ManagerID);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_node_od(ushort ConnectNo, ushort PortNum, ushort nodenum, ushort index, ushort subindex, ushort valuelength, uint value);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_syn_move(ushort ConnectNo, ushort AxisNum, ushort[] AxisList, int[] Position, ushort[] PosiMode);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_write_outbit(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort IoBit, ushort IoValue);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_read_outbit(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort IoBit, ref ushort IoValue);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_read_inbit(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort IoBit, ref ushort IoValue);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_write_outport(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort PortNo, int IoValue);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_read_outport(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort PortNo, ref int IoValue);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_read_inport(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort PortNo, ref int IoValue);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_controller_workmode(ushort ConnectNo, ushort controller_mode);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_controller_workmode(ushort ConnectNo, ref ushort controller_mode);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_cycletime(ushort ConnectNo, ushort FieldbusType, int CycleTime);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_cycletime(ushort ConnectNo, ushort FieldbusType, ref int CycleTime);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_axis_state_machine(ushort ConnectNo, ushort axis, ref ushort Axis_StateMachine);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_axis_statusword(ushort ConnectNo, ushort axis, ref int statusword);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_axis_setting_contrlmode(ushort ConnectNo, ushort axis, ref int contrlmode);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_axis_contrlword(ushort ConnectNo, ushort axis, ref int contrlword);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_axis_type(ushort ConnectNo, ushort axis, ref ushort Axis_Type);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_consume_time_fieldbus(ushort ConnectNo, ushort Fieldbustype, ref uint Average_time, ref uint Max_time, ref UInt64 Cycles);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_clear_consume_time_fieldbus(ushort ConnectNo, ushort Fieldbustype);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_axis_enable(ushort ConnectNo, ushort axis);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_axis_disable(ushort ConnectNo, ushort axis);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_axis_node_address(ushort ConnectNo, ushort axis, ref ushort SlaveAddr, ref ushort Sub_SlaveAddr);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_total_slaves(ushort ConnectNo, ushort PortNum, ref ushort TotalSlaves);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_home_profile(ushort ConnectNo, ushort axis, ushort home_mode, double High_Vel, double Low_Vel, double Tacc, double Tdec, double offsetpos);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_home_profile(ushort ConnectNo, ushort axis, ref ushort home_mode, ref double High_Vel, ref double Low_Vel, ref double Tacc, ref double Tdec, ref double offsetpos);
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_home_move(ushort ConnectNo, ushort axis);

        //铁电存储
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg_byte(ushort ConnectNo, int start, int inum, char[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg_byte(ushort ConnectNo, int start, int inum, byte[] pdata);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg_float(ushort ConnectNo, int start, int inum, float[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg_float(ushort ConnectNo, int start, int inum, float[] pdata);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg_int(ushort ConnectNo, int start, int inum, int[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg_int(ushort ConnectNo, int start, int inum, int[] pdata);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg_short(ushort ConnectNo, int start, int inum, short[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg_short(ushort ConnectNo, int start, int inum, short[] pdata);

        //延时翻转
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_outbit_delay_reverse(ushort ConnectNo, ushort channel, ushort bitno, ushort level, double reverse_time, ushort outmode);

        //虚拟IO映射
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_io_map_virtual(ushort ConnectNo, ushort bitno, ushort MapIoType, ushort MapIoIndex, double filter_time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_io_map_virtual(ushort ConnectNo, ushort bitno, ref ushort MapIoType, ref ushort MapIoIndex, ref double filter_time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_inbit_virtual(ushort ConnectNo, ushort bitno);


        //手轮
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_handwheel_channel(UInt16 ConnectNo, UInt16 index);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_handwheel_channel(UInt16 ConnectNo, ref UInt16 index);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_handwheel_inmode(UInt16 ConnectNo, UInt16 axis, UInt16 inmode, Int32 multi, double vh);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_handwheel_inmode(UInt16 ConnectNo, UInt16 axis, ref UInt16 inmode, ref Int32 multi, ref double vh);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_handwheel_inmode_decimals(UInt16 ConnectNo, UInt16 axis, UInt16 inmode, double multi, double vh);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_handwheel_inmode_decimals(UInt16 ConnectNo, UInt16 axis, ref UInt16 inmode, ref double multi, ref double vh);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_handwheel_inmode_extern(UInt16 ConnectNo, UInt16 inmode, UInt16 AxisNum, UInt16[] AxisList, Int32[] multi);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_handwheel_inmode_extern(UInt16 ConnectNo, ref UInt16 inmode, ref UInt16 AxisNum, UInt16[] AxisList, Int32[] multi);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_handwheel_inmode_extern_decimals(UInt16 ConnectNo, UInt16 inmode, UInt16 AxisNum, UInt16[] AxisList, double[] multi);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_handwheel_inmode_extern_decimals(UInt16 ConnectNo, ref UInt16 inmode, ref UInt16 AxisNum, UInt16[] AxisList, double[] multi);


        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_gear_unit(UInt16 ConnectNo, ushort Crd, UInt16 axis, double dist, ushort follow_mode, UInt32 imark);


        //---------------------读写掉电保持区------------------
        //写入字符数据到断电保持区（smc5000系列卡受限使用）
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg_byte(ushort ConnectNo, uint start, uint inum, byte[] pdata);
        //从断电保持区读取写入的字符（smc5000系列卡受限使用）
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg_byte(ushort ConnectNo, uint start, uint inum, byte[] pdata);
        //写入浮点型数据到断电保持区（smc5000系列卡受限使用）
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg_float(ushort ConnectNo, uint start, uint inum, float[] pdata);
        //从断电保持区读取写入的浮点型数据（smc5000系列卡受限使用）
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg_float(ushort ConnectNo, uint start, uint inum, float[] pdata);
        //写入整型数据到断电保持区（smc5000系列卡受限使用）
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg_int(ushort ConnectNo, uint start, uint inum, int[] pdata);
        //从断电保持区读取写入的整型数据（smc5000系列卡受限使用）
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg_int(ushort ConnectNo, uint start, uint inum, int[] pdata);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_persistent_reg_short(ushort ConnectNo, uint start, uint inum, short[] pdata);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_persistent_reg_short(ushort ConnectNo, uint start, uint inum, short[] pdata);


        //        螺距补偿功能(新)
        [DllImport("LTSMC.dll")]
        public static extern short smc_enable_leadscrew_comp(ushort ConnectNo, ushort axis, ushort enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_leadscrew_comp_config_unit(ushort ConnectNo, ushort axis, ushort n, double startpos, double lenpos, double[] pCompPos, double[] pCompNeg);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_leadscrew_comp_config_unit(ushort ConnectNo, ushort axis, ref ushort n, ref double startpos, ref double lenpos, double[] pCompPos, double[] pCompNeg);

        //螺距补偿后的脉冲位置，编码器位置 当量
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_position_ex_unit(ushort ConnectNo, ushort axis, ref double pos);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_encoder_ex_unit(ushort ConnectNo, ushort axis, ref double pos);

        //插补轴脉冲补偿
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_interp_compensation(ushort ConnectNo, ushort axis, double comp_value, double comp_time);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_interp_compensation(ushort ConnectNo, ushort axis, ref double comp_value, ref double comp_time);

        //新看门狗功能
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_watchdog_action_event(ushort ConnectNo, uint event_mask);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_watchdog_action_event(ushort ConnectNo, ref uint event_mask);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_watchdog_enable(ushort ConnectNo, double timer_period, uint enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_watchdog_enable(ushort ConnectNo, ref double timer_period, ref uint enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_reset_watchdog_timer(ushort ConnectNo);


        //单个或者多个IO触发减速停止
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_io_exactstop(ushort ConnectNo, ushort axis, ushort ioNum, ushort[] ioList, ushort enable, ushort valid_logic, ushort action, ushort move_dir);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_dec_stop_dist_unit(ushort ConnectNo, ushort axis, double dist);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_dec_stop_dist_unit(ushort ConnectNo, ushort axis, ref double dist);



        // 设置坐标系切向跟随
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_tangent_follow(ushort ConnectNo, ushort Crd, ushort axis, ushort follow_curve, ushort rotate_dir, double degree_equivalent);
        // 获取指定坐标系切向跟随参数
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_tangent_follow_param(ushort ConnectNo, ushort Crd, ref ushort axis, ref ushort follow_curve, ref ushort rotate_dir, ref double degree_equivalent);
        // 取消坐标系跟随
        [DllImport("LTSMC.dll")]
        public static extern short smc_disable_follow_move(ushort ConnectNo, ushort Crd);


        // 椭圆插补
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_ellipse_move_unit(UInt16 ConnectNo, ushort Crd, UInt16 axisNum, UInt16[] Axis_List, double[] Target_Pos, double[] Cen_Pos, double A_Axis_Len, double B_Axis_Len, UInt16 Dir, UInt16 Pos_Mode, short mark);
        [DllImport("LTSMC.dll")]
        public static extern short smc_ellipse_move(UInt16 ConnectNo, ushort Crd, UInt16 axisNum, UInt16[] Axis_List, double[] Target_Pos, double[] Cen_Pos, double A_Axis_Len, double B_Axis_Len, UInt16 Dir, UInt16 Pos_Mode);




        ///*********************

        //龙门功能
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_gear_follow_profile(UInt16 ConnectNo, UInt16 axis, UInt16 enable, UInt16 master_axis, double ratio);//双Z轴
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_gear_follow_profile(UInt16 ConnectNo, UInt16 axis, ref UInt16 enable, ref UInt16 master_axis, ref double ratio);
        //龙门模式的误差保护
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_grant_error_protect_unit(UInt16 ConnectNo, UInt16 axis, UInt16 enable, double dstp_error, double emg_error);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_grant_error_protect_unit(UInt16 ConnectNo, UInt16 axis, ref UInt16 enable, ref double dstp_error, ref double emg_error);


        //轴碰撞检测功能接口 
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_axis_conflict_config_unit(UInt16 ConnectNo, UInt16[] axis_list, UInt16[] axis_depart_dir, double home_dist, double conflict_dist, UInt16 stop_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_axis_conflict_config_unit(UInt16 ConnectNo, UInt16[] axis_list, UInt16[] axis_depart_dir, ref double home_dist, ref double conflict_dist, ref UInt16 stop_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_axis_conflict_config_en(UInt16 ConnectNo, UInt16 enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_axis_conflict_config_en(UInt16 ConnectNo, ref UInt16 enable);


        //1、	启用缓存方式添加比较位置：
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_fifo_set_mode(UInt16 ConnectNo, UInt16 hcmp, UInt16 fifo_mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_fifo_get_mode(UInt16 ConnectNo, UInt16 hcmp, ref UInt16 fifo_mode);
        //2、	读取剩余缓存状态，上位机通过此函数判断是否继续添加比较位置
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_fifo_get_state(UInt16 ConnectNo, UInt16 hcmp, ref short remained_points);
        //3、	按数组的方式批量添加比较位置
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_fifo_add_point_unit(UInt16 ConnectNo, UInt16 hcmp, UInt16 num, double[] cmp_pos);
        //4、	清除比较位置,也会把FPGA的位置同步清除掉
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_fifo_clear_points(UInt16 ConnectNo, UInt16 hcmp);
        //添加大数据，会堵塞一段时间，指导数据添加完成
        [DllImport("LTSMC.dll")]
        public static extern short smc_hcmp_fifo_add_table(UInt16 ConnectNo, UInt16 hcmp, UInt16 num, double[] cmp_pos);




        //功能：高速IO触发在线变速变位置参数配置
        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_change_pos_speed_config(UInt16 ConnectNo, UInt16 axis, double tar_vel, double tar_rel_pos, UInt16 trig_mode, UInt16 source);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pmove_change_pos_speed_config(UInt16 ConnectNo, UInt16 axis, ref double tar_vel, ref double tar_rel_pos, ref UInt16 trig_mode, ref UInt16 source);
        //功能：高速IO触发在线变速变位置使能 enable 0-禁用，1-多次触发，2-只触发一次
        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_change_pos_speed_enable(UInt16 ConnectNo, UInt16 axis, UInt16 enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pmove_change_pos_speed_enable(UInt16 ConnectNo, UInt16 axis, ref UInt16 enable);
        //trig_num 触发次数，trig_pos 触发位置
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pmove_change_pos_speed_state(UInt16 ConnectNo, UInt16 axis, ref UInt16 trig_num, ref double trig_pos);
        //指定输入口
        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_change_pos_speed_inbit(UInt16 ConnectNo, UInt16 axis, UInt16 inbit, UInt16 enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pmove_change_pos_speed_inbit(UInt16 ConnectNo, UInt16 axis, ref UInt16 inbit, ref UInt16 enable);


        //软着陆   
        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_unit_extern(UInt16 ConnectNo, UInt16 axis, double MidPos, double TargetPos, double Min_Vel, double Max_Vel, double stop_Vel, double acc_time, double dec_time, double smooth_time, UInt16 posi_mode);

        [DllImport("LTSMC.dll")]
        public static extern short smc_t_pmove_extern_unit(UInt16 ConnectNo, UInt16 axis, double MidPos, double TargetPos, double Min_Vel, double Max_Vel, double stop_Vel, double acc_time, double dec_time, UInt16 posi_mode);

        [DllImport("LTSMC.dll")]
        public static extern short smc_update_target_position_extern_unit(UInt16 ConnectNo, UInt16 axis, double mid_pos, double aim_pos, double vel, UInt16 posi_mode);   //软着陆强制变位




        //位置速度合并PMOVE
        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_extern_unit(ushort ConnectNo, ushort axis, double TargetPos, double Min_Vel, double Max_Vel, double stop_Vel, double acc_time, double dec_time, double smooth_time, ushort posi_mode);

        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_extern_unit_acc(ushort ConnectNo, ushort axis, double TargetPos, double Min_Vel, double Max_Vel, double stop_Vel, double acc, double dec, double smooth_time, ushort posi_mode);







        //软启动
        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_unit_extern_softstart(UInt16 ConnectNo, UInt16 axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, double delay_time, double Max_Vel2, double stop_vel2, double acc_time, double dec_time, double smooth_time, UInt16 posi_mode);



        //软启动
        [DllImport("LTSMC.dll")]
        public static extern short smc_pmove_extern_softstart_unit(UInt16 ConnectNo, UInt16 axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, double delay_time, double Max_Vel2, double stop_vel2, double acc_time, double dec_time, double smooth_time, UInt16 posi_mode);

        [DllImport("LTSMC.dll")]
        public static extern short smc_t_pmove_extern_softstart_unit(UInt16 ConnectNo, UInt16 axis, double MidPos, double TargetPos, double start_Vel, double Max_Vel, double stop_Vel, double delay_time, double Max_Vel2, double stop_vel2, double acc_time, double dec_time, UInt16 posi_mode);



        //正弦振荡曲线
        [DllImport("LTSMC.dll")]
        public static extern short smc_sine_oscillate_set_mode(ushort ConnectNo, ushort axis, ushort mode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_sine_oscillate_set_amplitude_tacc(ushort ConnectNo, ushort axis, double tacc_s);
        [DllImport("LTSMC.dll")]
        public static extern short smc_sine_oscillate_unit(UInt16 ConnectNo, UInt16 Axis, double Amplitude, double Frequency);
        [DllImport("LTSMC.dll")]
        public static extern short smc_sine_oscillate_stop(UInt16 ConnectNo, UInt16 Axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_sine_oscillate_set_cycle_num(ushort ConnectNo, ushort axis, uint cycle_num);
        [DllImport("LTSMC.dll")]
        public static extern short smc_sine_oscillate_get_cycle_num(ushort ConnectNo, ushort axis, ref uint cycle_num);

        //编码器检测
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_factor_error_unit(UInt16 ConnectNo, UInt16 axis, double factor, double error);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_factor_error_unit(UInt16 ConnectNo, UInt16 axis, ref double factor, ref double error);
        [DllImport("LTSMC.dll")]
        public static extern short smc_check_success_pulse(UInt16 ConnectNo, UInt16 axis);
        [DllImport("LTSMC.dll")]
        public static extern short smc_check_success_encoder(UInt16 ConnectNo, UInt16 axis);
        ///*
        //功能：设置脉冲计数值和编码器反馈值之间差值的报警阀值
        //输入参数：ConnectNo 卡号
        //axis 轴号
        //error 差值报警报警阀值
        //输出参数：无
        //*/
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_pulse_encoder_count_error_unit(UInt16 ConnectNo, UInt16 axis, double error);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_pulse_encoder_count_error_unit(UInt16 ConnectNo, UInt16 axis, ref double error);
        ///*

        //功能：检查脉冲计数值和编码器反馈值之间差值是否超过报警阀值
        //    输入参数：ConnectNo 卡号
        //    axis 轴号
        //    输出参数;无
        //    返回参数：0：差值小于报警阀值
        //    1：差值大于等于报警阀值
        //*/
        [DllImport("LTSMC.dll")]
        public static extern short smc_check_pulse_encoder_count_error_unit(UInt16 ConnectNo, UInt16 axis, ref double pulse_position, ref double enc_position);

        //使能和设置跟踪编码器误差不在范围内时轴的停止模式
        //检测指令位置与编码器偏差超过报警阀值时停止运动
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_encoder_count_error_action_config(UInt16 ConnectNo, UInt16 enable, UInt16 stopmode);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_encoder_count_error_action_config(UInt16 ConnectNo, ref UInt16 enable, ref UInt16 stopmode);

        //圆形限位
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_arc_zone_limit_config_unit(UInt16 ConnectNo, UInt16[] AxisList, UInt16 AxisNum, double[] Center, double Radius, UInt16 Source, UInt16 StopMode);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_arc_zone_limit_config(UInt16 ConnectNo, UInt16[] AxisList, UInt16 AxisNum, double[] Center, double Radius, UInt16 Source, UInt16 StopMode);

        [DllImport("LTSMC.dll")]
        public static extern short smc_get_arc_zone_limit_config_unit(UInt16 ConnectNo, UInt16[] AxisList, ref UInt16 AxisNum, double[] Center, ref double Radius, ref UInt16 Source, ref UInt16 StopMode);

        [DllImport("LTSMC.dll")]
        public static extern short smc_get_arc_zone_limit_axis_status(UInt16 CardNo, UInt16 AxisNo);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_arc_zone_limit_enable(UInt16 CardNo, UInt16 enable);

        [DllImport("LTSMC.dll")]
        public static extern short smc_get_arc_zone_limit_enable(UInt16 CardNo, ref UInt16 enable);



        //        /*********************************************************************************************************
        //高速锁存  锁存急停相关函数
        //*********************************************************************************************************/
        //        //配置锁存器：锁存模式0-单次锁存，1-连续锁存；3-锁存急停 锁存边沿0-下降沿，1-上升沿，2-双边沿；滤波时间，单位us

        //        //触发急停轴列表设置
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_latch_stop_time(ushort ConnectNo, ushort axis, int time); //触发急停时间,周期数为单位
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_latch_stop_time(ushort ConnectNo, ushort axis, ref int time);

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_latch_stop_axis(ushort ConnectNo, ushort latch, ushort num, UInt16[] axislist);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_latch_stop_axis(ushort ConnectNo, ushort latch, ref ushort num, UInt16[] axislist);


        //高速锁存(新)
        [DllImport("LTSMC.dll")]
        public static extern short smc_ltc_set_mode(ushort ConnectNo, ushort latch, ushort ltc_mode, ushort ltc_logic, double filter);
        [DllImport("LTSMC.dll")]
        public static extern short smc_ltc_get_mode(ushort ConnectNo, ushort latch, ref ushort ltc_mode, ref ushort ltc_logic, ref double filter);
        [DllImport("LTSMC.dll")]
        public static extern short smc_ltc_set_source(ushort ConnectNo, ushort latch, ushort axis, ushort ltc_source);
        [DllImport("LTSMC.dll")]
        public static extern short smc_ltc_get_source(ushort ConnectNo, ushort latch, ushort axis, ref ushort ltc_source);
        [DllImport("LTSMC.dll")]
        public static extern short smc_ltc_reset(ushort ConnectNo, ushort latch);

        [DllImport("LTSMC.dll")]
        public static extern short smc_ltc_get_number(ushort ConnectNo, ushort latch, ushort axis, ref int number);     //获取锁存个数
        [DllImport("LTSMC.dll")]
        public static extern short smc_ltc_get_value_unit(ushort ConnectNo, ushort latch, ushort axis, ref double value);       //获取锁存值





        //设置连续插补DA
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_da_output(ushort ConnectNo, ushort Crd, ushort da_no, double Vout);
        //设置连续DA使能
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_da_enable(ushort ConnectNo, ushort Crd, ushort enable, ushort da_no, int mark);
        /**********DA速度跟随**************
        da_no:通道号
        MaxVel:最大运行速度，单位unit
        MaxValue:最大电压
        *************************************/
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_da_follow_speed(ushort ConnectNo, ushort Crd, ushort da_no, double MaxVel, double MaxValue, double acc_offset, double dec_offset, double acc_dist, double dec_dist);

        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_get_da_follow_speed(ushort ConnectNo, ushort Crd, ushort da_no, ref double MaxVel, ref double MaxValue, ref double acc_offset, ref double dec_offset, ref double acc_dist, ref double dec_dist);



        //DA

        [DllImport("LTSMC.dll")]
        public static extern short smc_set_da_enable(ushort ConnectNo, ushort da_no, ushort enable);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_da_enable(ushort ConnectNo, ushort da_no, ref ushort enable);


        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ad_input(ushort ConnectNo, ushort da_no, ref double Vout);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ad_input_all(ushort ConnectNo, ref double Vout);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_da_output(ushort ConnectNo, ushort da_no, double Vout);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_da_output(ushort ConnectNo, ushort da_no, ref double Vout);


        //模拟量操作
        //AD
        [DllImport("LTSMC.dll")]
        public static extern double smc_get_ain(ushort ConnectNo, ushort channel);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_ain_action(ushort ConnectNo, ushort channel, ushort mode, double fvoltage, ushort action, double actpara);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ain_action(ushort ConnectNo, ushort channel, ref ushort mode, ref double fvoltage, ref ushort action, ref double actpara);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_ain_state(ushort ConnectNo, ushort channel);
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_ain_state(ushort ConnectNo, ushort channel);





        ////高速锁存(旧)
        //[DllImport("LTSMC.dll")]
        //public static extern short smc_set_ltc_mode(ushort ConnectNo, ushort axis, ushort ltc_logic, ushort ltc_mode, double filter); 	//设置LTC信号
        //[DllImport("LTSMC.dll")]
        //public static extern short smc_get_ltc_mode(ushort ConnectNo, ushort axis, ref ushort ltc_logic, ref ushort ltc_mode, ref double filter);	//读取设置LTC信号
        //[DllImport("LTSMC.dll")]
        //public static extern short smc_set_latch_mode(ushort ConnectNo, ushort axis, ushort latchmode, ushort latch_source, ushort latch_channel); 	//设置锁存方式
        //[DllImport("LTSMC.dll")]
        //public static extern short smc_get_latch_mode(ushort ConnectNo, ushort axis, ref ushort latchmode, ref ushort latch_source, ref ushort latch_channel);
        //[DllImport("LTSMC.dll")]
        //public static extern short smc_SetLtcOutMode(ushort ConnectNo, ushort axis, ushort enable, ushort bitno);//反相输出
        //[DllImport("LTSMC.dll")]
        //public static extern short smc_GetLtcOutMode(ushort ConnectNo, ushort axis, ref ushort enable, ref ushort bitno);
        //[DllImport("LTSMC.dll")]
        //public static extern short smc_get_latch_flag(ushort ConnectNo, ushort axis); 	//读取锁存器标志
        [DllImport("LTSMC.dll")]
        public static extern short smc_reset_latch_flag(ushort ConnectNo, ushort axis);     //复位锁存器标志
                                                                                            //[DllImport("LTSMC.dll")]
                                                                                            //public static extern short smc_get_latch_value_unit(ushort ConnectNo, ushort axis, ref double value); 	//读取编码器锁存器的值
                                                                                            //[DllImport("LTSMC.dll")]
                                                                                            //public static extern short smc_get_latch_flag_extern(ushort ConnectNo, ushort axis); 	//读取锁存器标志
                                                                                            //[DllImport("LTSMC.dll")]
                                                                                            //public static extern int smc_get_latch_value_extern(ushort ConnectNo, ushort axis, ushort index); //按索引取值
                                                                                            //public static extern short smc_ltc_get_number(ushort ConnectNo, ushort latch, ushort axis, ref int number);

        //一维比较
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_add_point_cycle(ushort ConnectNo, ushort axis, double pos, ushort dir, uint bitno, uint cycle, ushort level);



        //二维比较
        [DllImport("LTSMC.dll")]
        public static extern short smc_compare_add_point_cycle_2d(ushort ConnectNo, ushort[] axis, double[] pos, ushort[] dir, uint bitno, uint cycle, int level);



        //拓展IO模块读写
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_write_outport_extern(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort PortNo, uint IoValue);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_read_outport_extern(ushort ConnectNo, ushort PortNum, ushort NodeID, ushort PortNo, ref int IoValue);


        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_slave_output_retain(ushort ConnectNo, ushort Enable);


        [DllImport("LTSMC.dll")]
        public static extern short nmcs_read_inbit_extern(ushort ConnectNo, ushort Channel, ushort NoteID, ushort IoBit, ref ushort IoValue);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_read_inport_extern(ushort ConnectNo, ushort Channel, ushort NoteID, ushort PortNo, ref int IoValue);


        [DllImport("LTSMC.dll")]
        public static extern short smc_set_extra_counter_reverse(ushort CardNo, ushort channel, ushort reverse);




        [DllImport("LTSMC.dll")]
        public static extern short smc_get_extra_counter_reverse(ushort CardNo, ushort channel, ref ushort reverse);


        [DllImport("LTSMC.dll")]
        public static extern short smc_cam_table_unit(ushort ConnectNo, ushort MasterAxisNo, ushort SlaveAxisNo, uint Count, double[] pMasterPos, double[] pSlavePos, ushort SrcMode);
        //功 能：添加电子凸轮表
        //参  数：MasterAxisNo 主轴轴号，0-实际轴数-1
        //SlaveAxisNo 从轴轴号，0-实际轴数-1
        //SrcMode 主轴位置模式：0-指令位置，1-反馈位置
        //Count              数据个数

        [DllImport("LTSMC.dll")]
        public static extern short smc_cam_move(ushort ConnectNo, ushort AxisNo);
        //功 能：启动从轴电子凸轮运动
        //参  数：SlaveAxisNo 从轴轴号，0-实际轴数-1

        [DllImport("LTSMC.dll")]
        public static extern short smc_cam_move_cycle(ushort ConnectNo, ushort iaxis);


        //总长度和剩余未运行的长度
        [DllImport("LTSMC.dll")]
        public static extern short smc_read_vector_length_unit(ushort ConnectNo, ushort Crd, ref double total_length, ref double left_length);




        //
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_pmove_unit_pausemode(UInt16 CardNo, UInt16 axis, double TargetPos, double Min_Vel, double Max_Vel, double stop_Vel, double acc, double dec, double smooth_time, UInt16 posi_mode);

        //
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_return_pausemode(UInt16 CardNo, UInt16 Crd, UInt16 axis);


        //连续插补vmove
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_vmove_unit(UInt16 ConnectNo, UInt16 Crd, UInt16 axis, double vel, double acc, UInt16 dir, int imark);


        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_stop_axis(UInt16 ConnectNo, UInt16 Crd, UInt16 axis, double dec, int imark);




        //连续插补IO
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_wait_input(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double TimeOut, int mark);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_delay_outbit_to_start(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double delay_value, ushort delay_mode, double ReverseTime);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_delay_outbit_to_stop(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double delay_time, double ReverseTime);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_ahead_outbit_to_stop(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double ahead_value, ushort ahead_mode, double ReverseTime);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_accurate_outbit_unit(ushort ConnectNo, ushort Crd, ushort cmp_no, ushort on_off, ushort map_axis, double rel_dist, ushort pos_source, double ReverseTime);

        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_write_outbit(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double ReverseTime);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_clear_io_action(ushort ConnectNo, ushort Crd, uint Io_Mask);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_set_pause_output(ushort ConnectNo, ushort Crd, ushort action, int mask, int state);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_get_pause_output(ushort ConnectNo, ushort Crd, ref ushort action, ref int mask, ref int state);
        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_delay(ushort ConnectNo, ushort Crd, double delay_time, int mark);//延时指令



        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_delay_outbit_to_start_path(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double delay_value, ushort delay_mode, double ReverseTime);



        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_delay_outbit_to_stop_path(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double delay_time, double ReverseTime);



        [DllImport("LTSMC.dll")]
        public static extern short smc_conti_ahead_outbit_to_stop_path(ushort ConnectNo, ushort Crd, ushort bitno, ushort on_off, double ahead_value, ushort ahead_mode, double ReverseTime);


        [DllImport("LTSMC.dll")]
        public static extern short nmcs_write_rxpdo_extra(UInt16 CardNo, UInt16 PortNum, UInt16 address, UInt16 DataLen, Int32 Value);

        //转矩控制功能函数
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_torque_move(UInt16 CardNo, UInt16 axis, int Torque, UInt16 PosLimitValid, double PosLimitValue, UInt16 PosMode);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_change_torque(UInt16 CardNo, UInt16 axis, int Torque);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_torque(UInt16 CardNo, UInt16 axis, ref int Torque);

        //设置偏移量的位置值
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_offset_pos(UInt16 CardNo, UInt16 axis, double offset_pos);

        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_offset_pos(UInt16 CardNo, UInt16 axis, ref double offset_pos);
        //设置控制卡的总线同步偏移，单位us
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_data_offset_time(ushort ConnectNo, int offset_us);
        //SMC_API short __stdcall nmcs_set_data_offset_time(WORD ConnectNo, int offset_us);
        //获取控制卡的总线同步偏移，单位us
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_get_data_offset_time(ushort ConnectNo, ref int offset_us);
        //SMC_API short __stdcall nmcs_get_data_offset_time(WORD ConnectNo, int* offset_us);

        //设置单轴的点位运动模式，模式：0 CSP模式，1 PP模式
        [DllImport("LTSMC.dll")]
        public static extern short nmcs_set_axis_run_mode(ushort ConnectNo, ushort axis, ushort run_mode);

        //速度前馈功能，过程数据RxPDO必须包含0x60B1
        [DllImport("LTSMC.dll")]
        public static extern short smc_set_feedforward_profile(ushort ConnectNo, ushort axis, double vel_offset_coef, double tor_offset_coef);
        [DllImport("LTSMC.dll")]
        public static extern short smc_get_feedforward_profile(ushort ConnectNo, ushort axis, ref double vel_offset_coef, ref double tor_offset_coef);



    }
}
