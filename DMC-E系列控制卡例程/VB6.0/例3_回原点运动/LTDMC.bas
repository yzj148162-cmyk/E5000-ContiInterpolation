Attribute VB_Name = "Module1"

Option Explicit
''''''''''''''''''''''''''''''''''  Leadshine technology  ''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'''                                 LTDMC V1.1 函数列表                           ''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        
        '函数库打印输出
        Declare Function dmc_set_debug_mode Lib "LTDMC.dll" (ByVal mode As Integer, ByVal FileName As String) As Integer
        Declare Function dmc_get_debug_mode Lib "LTDMC.dll" (ByRef mode As Integer, ByRef FileName As String) As Integer
        '板卡配置
        Declare Function dmc_board_init Lib "LTDMC.dll" () As Integer           '初始化控制卡
        Declare Function dmc_board_init_onecard Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        Declare Function dmc_board_close_onecard Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        Declare Function dmc_get_CardInfList Lib "LTDMC.dll" (ByRef CardNum As Integer, ByRef CardTypeList As Long, ByRef CardIdList As Integer) As Integer '获取所有卡号
        Declare Function dmc_board_close Lib "LTDMC.dll" () As Integer          '关闭控制卡
        Declare Function dmc_board_reset Lib "LTDMC.dll" () As Integer
        Declare Function dmc_board_reset_onecard Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        Declare Function dmc_soft_reset Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        Declare Function dmc_cool_reset Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        Declare Function dmc_original_reset Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        Declare Function dmc_get_card_ID Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef CardID As Long) As Integer
        Declare Function dmc_get_release_version Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal ReleaseVersion As String) As Integer
        Declare Function dmc_get_card_version Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef CardVersion As Long) As Integer     '读取控制卡硬件版本
        Declare Function dmc_get_card_soft_version Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef FirmID As Long, ByRef SubFirmID As Long) As Integer           '读取控制卡硬件的固件版本
        Declare Function dmc_get_card_lib_version Lib "LTDMC.dll" (ByRef LibVer As Long) As Integer       '读取控制卡动态库版本
        Declare Function dmc_get_total_axes Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef TotalAxis As Long) As Integer         '读取指定卡轴数
        Declare Function dmc_get_total_liners Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef TotalAxis As Long) As Integer
        Declare Function dmc_get_total_ionum Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef TotalIn As Integer, ByRef TotalOut As Integer) As Integer
        Declare Function dmc_get_total_adcnum Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef TotalIn As Integer, ByRef TotalOut As Integer) As Integer
        
                '密码管理
        Declare Function dmc_write_sn Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal str_sn As String) As Integer
        Declare Function dmc_check_sn Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal str_sn As String) As Integer
        
         '运动模块脉冲模式
        Declare Function dmc_set_pulse_outmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal outmode As Integer) As Integer    '设定脉冲输出模式
        Declare Function dmc_get_pulse_outmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef outmode As Integer) As Integer      '读取脉冲输出模式
        Declare Function dmc_get_equiv Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef equiv As Double) As Integer       '脉冲当量
        Declare Function dmc_set_equiv Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal equiv As Double) As Integer
        Declare Function dmc_set_backlash_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal backlash As Double) As Integer  '反向间隙
        Declare Function dmc_get_backlash_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef backlash As Double) As Integer
        Declare Function dmc_set_backlash Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal backlash As Long) As Integer  '反向间隙
        Declare Function dmc_get_backlash Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef backlash As Long) As Integer
        
         '通用文件下载
       ''函数参数必须严格保持一致性
        Declare Function dmc_download_file Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal pFileName As String, ByRef pfilenameinControl As Byte, ByVal filetype As Integer) As Integer
        Declare Function dmc_download_memfile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef pbuffer As Byte, ByVal buffsize As Integer, ByVal pFileName As String, ByRef pfilenameinControl As Byte, ByVal filetype As Integer) As Integer
        Declare Function dmc_upload_file Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal pFileName As String, ByRef pfilenameinControl As Byte, ByVal filetype As Integer) As Integer
        Declare Function dmc_upload_memfile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef pbuffer As Byte, ByVal buffsize As Integer, ByVal pFileName As String, ByRef pfilenameinControl As Byte, ByRef puifilesize As Integer, ByVal filetype As Integer) As Integer
        '下载参数文件
        Declare Function dmc_download_configfile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal FileName As String) As Integer
        '下载固件文件
        Declare Function dmc_download_firmware Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal FileName As String) As Integer
        Declare Function dmc_get_progress Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef process As Single) As Integer
        
         '限位/异常设置
        Declare Function dmc_set_softlimit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal source_sel As Integer, ByVal SL_action As Integer, ByVal N_limit As Long, ByVal P_limit As Long) As Integer  '设置软限位参数
        Declare Function dmc_get_softlimit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef source_sel As Integer, ByRef SL_action As Integer, ByRef N_limit As Long, ByRef P_limit As Long) As Integer    '读取软限位参数
        Declare Function dmc_set_el_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal el_enable As Integer, ByVal el_logic As Integer, ByVal el_mode As Integer) As Integer      '设置EL信号
        Declare Function dmc_get_el_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef el_enable As Integer, ByRef el_logic As Integer, ByRef el_mode As Integer) As Integer   '读取设置EL信号
        Declare Function dmc_set_emg_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal emg_logic As Integer) As Integer    '设置EMG信号
        Declare Function dmc_get_emg_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enbale As Integer, ByRef emg_logic As Integer) As Integer        '读取设置EMG信号
        
        '单轴运动
        '速度设置
        Declare Function dmc_set_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Min_Vel As Double, ByVal Max_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal Stop_vel As Double) As Integer '设定速度曲线参数
        Declare Function dmc_get_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Min_Vel As Double, ByRef Max_Vel As Double, ByRef Tacc As Double, ByRef Tdec As Double, ByRef Stop_vel As Double) As Integer '读取速度曲线参数
        Declare Function dmc_set_profile_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Min_Vel As Double, ByVal Max_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal Stop_vel As Double) As Integer   '单轴速度参数
        Declare Function dmc_get_profile_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Min_Vel As Double, ByRef Max_Vel As Double, ByRef Tacc As Double, ByRef Tdec As Double, ByRef Stop_vel As Double) As Integer
        Declare Function dmc_set_acc_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Min_Vel As Double, ByVal Max_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal Stop_vel As Double) As Integer '设定速度曲线参数
        Declare Function dmc_get_acc_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Min_Vel As Double, ByRef Max_Vel As Double, ByRef Tacc As Double, ByRef Tdec As Double, ByRef Stop_vel As Double) As Integer '读取速度曲线参数
        Declare Function dmc_set_profile_unit_acc Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Min_Vel As Double, ByVal Max_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal Stop_vel As Double) As Integer   '单轴速度参数
        Declare Function dmc_get_profile_unit_acc Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Min_Vel As Double, ByRef Max_Vel As Double, ByRef Tacc As Double, ByRef Tdec As Double, ByRef Stop_vel As Double) As Integer
        Declare Function dmc_set_s_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal s_mode As Integer, ByVal s_para As Double) As Integer        '设置平滑速度曲线参数
        Declare Function dmc_get_s_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal s_mode As Integer, ByRef s_para As Double) As Integer    '读取平滑速度曲线参数
    
        '点位运动
        Declare Function dmc_pmove Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Long, ByVal posi_mode As Integer) As Integer  '指定轴做定长位移运动
        Declare Function dmc_pmove_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Double, ByVal posi_mode As Integer) As Integer  '定长
         'JOG运动
        Declare Function dmc_vmove Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dir As Integer) As Integer    '指定轴做连续运动
        Declare Function dmc_pmove_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Double, ByVal Min_Vel As Double, ByVal Max_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal Stop_vel As Double, ByVal s_para As Double, ByVal posi_mode As Integer) As Integer '设定速度曲线参数
        
        Declare Function dmc_reset_target_position Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Long, ByVal posi_mode As Integer) As Integer  '运动中改变目标位置
        Declare Function dmc_change_speed Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Curr_Vel As Double, ByVal Taccdec As Double) As Integer        '在线改变指定轴的当前运动速度
        Declare Function dmc_update_target_position Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Long, ByVal posi_mode As Integer) As Integer  '强行改变目标位置
        Declare Function dmc_reset_target_position_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal New_pos As Double) As Integer   '在线变位
        Declare Function dmc_update_target_position_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal New_pos As Double) As Integer   '强行变位
        Declare Function dmc_change_speed_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal New_vel As Double, ByVal Taccdec As Double) As Integer           '在线变速

        Declare Function dmc_set_vector_profile_multicoor Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal Min_Vel As Double, ByVal Max_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal Stop_vel As Double) As Integer
        Declare Function dmc_get_vector_profile_multicoor Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef Min_Vel As Double, ByRef Max_Vel As Double, ByRef Tacc As Double, ByRef Tdec As Double, ByRef Stop_vel As Double) As Integer
        Declare Function dmc_set_vector_s_profile_multicoor Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal s_mode As Integer, ByVal s_para As Double) As Integer
        Declare Function dmc_get_vector_s_profile_multicoor Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal s_mode As Integer, ByRef s_para As Double) As Integer
        
        Declare Function dmc_set_vector_profile_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal Min_Vel As Double, ByVal Max_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal Stop_vel As Double) As Integer    '单段插补速度参数
        Declare Function dmc_get_vector_profile_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef Min_Vel As Double, ByRef Max_Vel As Double, ByRef Tacc As Double, ByRef Tdec As Double, ByRef Stop_vel As Double) As Integer
        Declare Function dmc_set_vector_s_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal s_mode As Integer, ByVal s_para As Double) As Integer
        Declare Function dmc_get_vector_s_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal s_mode As Integer, ByRef s_para As Double) As Integer
        '直线插补
        Declare Function dmc_line_multicoor Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef DistList As Long, ByVal posi_mode As Integer) As Integer     '指定轴直线插补运动
        '平面圆弧
        Declare Function dmc_arc_move_multicoor Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef AxisList As Integer, ByRef target_Pos As Long, ByRef cen_Pos As Long, ByVal arc_dir As Integer, ByVal posi_mode As Integer) As Integer
   
        Declare Function dmc_line_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByVal posi_mode As Integer) As Integer   '单段直线
        Declare Function dmc_arc_move_center_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByRef cen_Pos As Double, ByVal arc_dir As Integer, ByVal CircleNum As Long, ByVal posi_mode As Integer) As Integer    '圆心终点式圆弧/螺旋线/渐开线
        Declare Function dmc_arc_move_radius_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByVal arc_radius As Double, ByVal arc_dir As Integer, ByVal CircleNum As Long, ByVal posi_mode As Integer) As Integer   '半径终点式圆弧/螺旋线
        Declare Function dmc_arc_move_3points_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByRef mid_pos As Double, ByVal CircleNum As Long, ByVal posi_mode As Integer) As Integer    '三点式圆弧/螺旋线
        Declare Function dmc_rectangle_move_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByRef Mark_Pos As Double, ByVal Count As Long, ByVal rect_mode As Integer, ByVal posi_mode As Integer) As Integer
        
         'PVT运动
        Declare Function dmc_PvtTable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Long, ByRef pVel As Double) As Integer
        Declare Function dmc_PtsTable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Long, ByRef pPercent As Double) As Integer
        Declare Function dmc_PvtsTable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Long, ByVal velBegin As Double, ByVal velEnd As Double) As Integer
        Declare Function dmc_PttTable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Long) As Integer
        Declare Function dmc_PvtMove Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer) As Integer
        
        Declare Function dmc_PttTable_add Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Long) As Integer
        Declare Function dmc_PtsTable_add Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Long, ByRef pPercent As Double) As Integer
        Declare Function dmc_pvt_get_remain_space Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        
        Declare Function dmc_pvt_table_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Double, ByRef pVel As Double) As Integer
        Declare Function dmc_pts_table_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Double, ByRef pPercent As Double) As Integer
        Declare Function dmc_pvts_table_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Double, ByVal velBegin As Double, ByVal velEnd As Double) As Integer
        Declare Function dmc_ptt_table_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Count As Long, ByRef pTime As Double, ByRef pPos As Double) As Integer
        Declare Function dmc_pvt_move Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer) As Integer
        
        Declare Function dmc_SetGearProfile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal MasterType As Integer, ByVal MasterIndex As Integer, ByVal MasterEven As Long, ByVal SlaveEven As Long, ByVal MasterSlope As Long) As Integer
        Declare Function dmc_GetGearProfile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef MasterType As Integer, ByRef MasterIndex As Integer, ByRef MasterEven As Long, ByRef SlaveEven As Long, ByRef MasterSlope As Long) As Integer
        Declare Function dmc_GearMove Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer) As Integer
        
         '回零运动
        Declare Function dmc_set_home_pin_logic Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal org_logic As Integer, ByVal filter As Double) As Integer         '设置HOME信号
        Declare Function dmc_get_home_pin_logic Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef org_logic As Integer, ByRef filter As Double) As Integer     '读取设置HOME信号
        Declare Function dmc_set_homemode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal home_dir As Integer, ByVal vel_mode As Double, ByVal mode As Integer, ByVal EZ_count As Integer) As Integer '设定指定轴的回原点模式
        Declare Function dmc_get_homemode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef home_dir As Integer, ByRef vel_mode As Double, ByRef home_mode As Integer, ByRef EZ_count As Integer) As Integer '读取指定轴的回原点模式
        Declare Function dmc_home_move Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        Declare Function dmc_set_home_profile_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Low_Vel As Double, ByVal High_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double) As Integer '设定回零速度曲线参数
        Declare Function dmc_get_home_profile_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Low_Vel As Double, ByRef High_Vel As Double, ByRef Tacc As Double, ByRef Tdec As Double) As Integer '读取回零速度曲线参数
        Declare Function dmc_get_home_result Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef state As Integer) As Integer
        Declare Function dmc_set_home_position_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal position As Double) As Integer
        Declare Function dmc_get_home_position_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef position As Double) As Integer
        Declare Function dmc_set_el_home Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal mode As Integer) As Integer
        
        '原点锁存
        '3800,3600,5800,5600专用
        Declare Function dmc_set_homelatch_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal logic As Integer, ByVal Source As Integer) As Integer
        Declare Function dmc_get_homelatch_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef logic As Integer, ByRef Source As Integer) As Integer
        Declare Function dmc_get_homelatch_flag Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long
        Declare Function dmc_reset_homelatch_flag Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        Declare Function dmc_get_homelatch_value Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long
        'ez锁存
        Declare Function dmc_set_ezlatch_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal logic As Integer, ByVal Source As Integer) As Integer
        Declare Function dmc_get_ezlatch_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef logic As Integer, ByRef Source As Integer) As Integer
        Declare Function dmc_get_ezlatch_flag Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long
        Declare Function dmc_reset_ezlatch_flag Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        Declare Function dmc_get_ezlatch_value Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long
        
         '手轮运动
         '一个手轮信号控制单个轴运动
        Declare Function dmc_set_handwheel_inmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal inmode As Integer, ByVal multi As Long, ByVal vh As Double) As Integer      '设置输入手轮脉冲信号的工作方式
        Declare Function dmc_get_handwheel_inmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef inmode As Integer, ByRef multi As Long, ByRef vh As Double) As Integer    '读取输入手轮脉冲信号的工作方式
        Declare Function dmc_set_handwheel_inmode_decimals Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal inmode As Integer, ByVal multi As Double, ByVal vh As Double) As Integer      '设置输入手轮脉冲信号的工作方式
        Declare Function dmc_get_handwheel_inmode_decimals Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef inmode As Integer, ByRef multi As Double, ByRef vh As Double) As Integer    '读取输入手轮脉冲信号的工作方式
         '启动手轮运动
        Declare Function dmc_handwheel_move Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer         '启动指定轴的手轮脉冲运动
        '3800,3600,5800,5600专用 手轮通道选择
        Declare Function dmc_set_handwheel_channel Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal index As Integer) As Integer
        Declare Function dmc_get_handwheel_channel Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef index As Integer) As Integer
       '3800,3600,5800,5600专用 一个手轮信号控制多轴运动
        Declare Function dmc_set_handwheel_inmode_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal inmode As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef multi As Long) As Integer     '设置输入手轮脉冲信号的工作方式
        Declare Function dmc_get_handwheel_inmode_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef inmode As Integer, ByRef AxisNum As Integer, ByRef AxisList As Integer, ByRef multi As Long) As Integer   '读取输入手轮脉冲信号的工作方式
        Declare Function dmc_set_handwheel_inmode_extern_decimals Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal inmode As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef multi As Double) As Integer     '设置输入手轮脉冲信号的工作方式
        Declare Function dmc_get_handwheel_inmode_extern_decimals Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef inmode As Integer, ByRef AxisNum As Integer, ByRef AxisList As Integer, ByRef multi As Double) As Integer   '读取输入手轮脉冲信号的工作方式
        
        Declare Function dmc_handwheel_set_axislist Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisSelIndex As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer) As Integer
        Declare Function dmc_handwheel_get_axislist Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisSelIndex As Integer, ByRef AxisNum As Integer, ByRef AxisList As Integer) As Integer
        Declare Function dmc_handwheel_set_ratiolist Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisSelIndex As Integer, ByVal StartRatioIndex As Integer, ByVal RatioSelNum As Integer, ByRef RatioList As Double) As Integer
        Declare Function dmc_handwheel_get_ratiolist Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisSelIndex As Integer, ByVal StartRatioIndex As Integer, ByVal RatioSelNum As Integer, ByRef RatioList As Double) As Integer
        Declare Function dmc_handwheel_set_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal inmode As Integer, ByVal IfHardEnable As Integer) As Integer
        Declare Function dmc_handwheel_get_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef inmode As Integer, ByRef IfHardEnable As Integer) As Integer
        Declare Function dmc_handwheel_set_index Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisSelIndex As Integer, ByVal RatioSelIndex As Integer) As Integer
        Declare Function dmc_handwheel_get_index Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef AxisSelIndex As Integer, ByRef RatioSelIndex As Integer) As Integer
        'Declare Function dmc_handwheel_move Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal ForceMove As Integer) As Integer
        Declare Function dmc_handwheel_stop Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        
          '锁存
        Declare Function dmc_set_ltc_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal ltc_logic As Integer, ByVal ltc_mode As Integer, ByVal filter As Double) As Integer    '设置LTC信号
        Declare Function dmc_get_ltc_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef ltc_logic As Integer, ByRef ltc_mode As Integer, ByRef filter As Double) As Integer  '读取设置LTC信号
        Declare Function dmc_set_latch_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal all_enable As Integer, ByVal latch_source As Integer, ByVal latch_channel As Integer) As Integer     '设置锁存方式
        Declare Function dmc_get_latch_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef all_enable As Integer, ByRef latch_source As Integer, ByRef latch_channel As Integer) As Integer
        '3800,3600,5800,5600专用 LTC反相输出
        Declare Function dmc_SetLtcOutMode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal bitno As Integer) As Integer
        Declare Function dmc_GetLtcOutMode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef bitno As Integer) As Integer
        Declare Function dmc_get_latch_flag Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer             '读取控制卡内有效锁存个数
        Declare Function dmc_reset_latch_flag Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer       '复位标志
        Declare Function dmc_get_latch_value Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long           '读取控制卡内锁存值，再连续锁存模式下读取一次控制卡内有效锁存个数将会自动减1,并将锁存值保存在PC缓冲区内
        Declare Function dmc_get_latch_value_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef pos_by_mm As Double) As Long           '读取控制卡内锁存值，再连续锁存模式下读取一次控制卡内有效锁存个数将会自动减1,并将锁存值保存在PC缓冲区内
        Declare Function dmc_get_latch_flag_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer            '读取PC缓冲区中已保存的已锁存值个数
        Declare Function dmc_get_latch_value_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal index As Integer) As Long  '按索引号读取PC缓冲区中已保存的锁存值，读的时候会将控制卡内的有效锁存值全部保存在PC缓冲区中
        'LTC端口触发延时急停时间 单位us
        Declare Function dmc_set_latch_stop_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal time As Long) As Integer
        Declare Function dmc_get_latch_stop_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef time As Long) As Integer
        
        Declare Function dmc_ltc_set_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal ltc_mode As Integer, ByVal ltc_logic As Integer, ByVal filter As Double) As Integer    '设置LTC信号
        Declare Function dmc_ltc_get_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByRef ltc_mode As Integer, ByRef ltc_logic As Integer, ByRef filter As Double) As Integer    '设置LTC信号
        Declare Function dmc_ltc_set_source Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal axis As Integer, ByVal ltc_source As Integer) As Integer
        Declare Function dmc_ltc_get_source Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal axis As Integer, ByRef ltc_source As Integer) As Integer
        Declare Function dmc_ltc_reset Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer) As Integer
        Declare Function dmc_ltc_get_number Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal axis As Integer, ByRef number As Integer) As Integer
        Declare Function dmc_ltc_get_value_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal axis As Integer, ByRef Value As Double) As Integer
        
         '单轴低速位置比较
        Declare Function dmc_compare_set_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal cmp_source As Integer) As Integer       '配置比较器
        Declare Function dmc_compare_get_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef cmp_source As Integer) As Integer   '读取配置比较器
        Declare Function dmc_compare_clear_points Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer          '清除所有比较点
        Declare Function dmc_compare_add_point Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal pos As Long, ByVal Dir As Integer, ByVal action As Integer, ByVal actpara As Long) As Integer    '添加比较点
        Declare Function dmc_compare_get_current_point Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef pos As Long) As Integer    '读取当前比较点
        Declare Function dmc_compare_get_points_runned Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef pointNum As Long) As Integer       '查询已经比较过的点
        Declare Function dmc_compare_get_points_remained Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef pointNum As Long) As Integer     '查询可以加入的比较点数量
        '二维低速位置比较
        Declare Function dmc_compare_set_config_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal enable As Integer, ByVal cmp_source As Integer) As Integer           '配置比较器
        Declare Function dmc_compare_get_config_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef enable As Integer, ByRef cmp_source As Integer) As Integer           '读取配置比较器
        Declare Function dmc_compare_clear_points_extern Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer          '清除所有比较点
        Declare Function dmc_compare_add_point_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef axis As Integer, ByRef pos As Long, ByRef Dir As Integer, ByVal action As Integer, ByVal actpara As Long) As Integer          '添加两轴位置比较点
        Declare Function dmc_compare_add_point_extern_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef axis As Integer, ByRef pos As Double, ByRef Dir As Integer, ByVal action As Integer, ByVal actpara As Long) As Integer          '添加两轴位置比较点
        Declare Function dmc_compare_get_current_point_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef pos As Long) As Integer    '读取当前比较点
        Declare Function dmc_compare_get_current_point_extern_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef pos As Double) As Integer    '读取当前比较点
        Declare Function dmc_compare_get_points_runned_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef pointNum As Long) As Integer       '查询已经比较过的点
        Declare Function dmc_compare_get_points_remained_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef pointNum As Long) As Integer      '查询可以加入的二维比较点数量
         
        Declare Function dmc_compare_set_config_multi Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal queue As Integer, ByVal enable As Integer, ByVal axis As Integer, ByVal cmp_source As Integer) As Integer       '配置比较器
        Declare Function dmc_compare_get_config_multi Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal queue As Integer, ByRef enable As Integer, ByRef axis As Integer, ByRef cmp_source As Integer) As Integer
        Declare Function dmc_compare_add_point_multi Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal cmp As Integer, ByVal pos As Long, ByVal Dir As Integer, ByVal action As Integer, ByVal actpara As Long, ByVal times As Double) As Integer    '添加比较点
        
         '单轴高速位置比较函数
        Declare Function dmc_hcmp_set_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal cmp_mode As Integer) As Integer
        Declare Function dmc_hcmp_get_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByRef cmp_mode As Integer) As Integer
        Declare Function dmc_hcmp_set_config_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal axis As Integer, ByVal cmp_source As Integer, ByVal cmp_logic As Integer, ByVal cmp_mode As Integer, ByVal Dist As Long, ByVal time As Long) As Integer
        Declare Function dmc_hcmp_get_config_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByRef axis As Integer, ByRef cmp_source As Integer, ByRef cmp_logic As Integer, ByRef cmp_mode As Integer, ByRef Dist As Long, ByRef time As Long) As Integer
        Declare Function dmc_hcmp_set_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal axis As Integer, ByVal cmp_source As Integer, ByVal cmp_logic As Integer, ByVal time As Long) As Integer
        Declare Function dmc_hcmp_get_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByRef axis As Integer, ByRef cmp_source As Integer, ByRef cmp_logic As Integer, ByRef time As Long) As Integer
        Declare Function dmc_hcmp_add_point Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal cmp_pos As Long) As Integer
        Declare Function dmc_hcmp_set_liner Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal Increment As Long, ByVal Count As Long) As Integer
        Declare Function dmc_hcmp_get_liner Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByRef Increment As Long, ByRef Count As Long) As Integer
        Declare Function dmc_hcmp_get_current_state Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByRef remained_points As Long, ByRef current_point As Long, ByRef runned_points As Long) As Integer
        Declare Function dmc_hcmp_clear_points Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer) As Integer
        Declare Function dmc_read_cmp_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer) As Integer
        Declare Function dmc_write_cmp_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal on_off As Integer) As Integer
         '二维高速位置比较功能
        Declare Function dmc_hcmp_2d_set_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal cmp_enable As Integer) As Integer
        Declare Function dmc_hcmp_2d_get_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByRef cmp_enable As Integer) As Integer
        Declare Function dmc_hcmp_2d_set_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal cmp_mode As Integer, ByVal x_axis As Integer, ByVal x_cmp_source As Integer, ByVal y_axis As Integer, ByVal y_cmp_source As Integer, ByVal error As Long, ByVal cmp_logic As Integer, ByVal time As Long, ByVal pwm_enable As Integer, ByVal duty As Double, ByVal freq As Long, ByVal port_sel As Integer, ByVal pwm_number As Integer) As Integer
        Declare Function dmc_hcmp_2d_get_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByRef cmp_mode As Integer, ByRef x_axis As Integer, ByRef x_cmp_source As Integer, ByRef y_axis As Integer, ByRef y_cmp_source As Integer, ByRef error As Long, ByRef cmp_logic As Integer, ByRef time As Long, ByRef pwm_enable As Integer, ByRef duty As Double, ByRef freq As Long, ByRef port_sel As Integer, ByRef pwm_number As Integer) As Integer
        Declare Function dmc_hcmp_2d_add_point Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal x_cmp_pos As Long, ByVal y_cmp_pos As Long) As Integer
        Declare Function dmc_hcmp_2d_get_current_state Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByRef remained_points As Long, ByRef x_current_point As Long, ByRef y_current_point As Long, ByRef runned_points As Long, ByRef current_state As Integer) As Integer
        Declare Function dmc_hcmp_2d_clear_points Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer) As Integer
        Declare Function dmc_hcmp_2d_force_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal hcmp As Integer, ByVal enable As Integer) As Integer
        
        '通用IO
        Declare Function dmc_read_inbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer) As Integer            '读取输入口的状态
        Declare Function dmc_write_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal on_off As Integer) As Integer         '设置输出口的状态
        Declare Function dmc_read_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer) As Integer           '读取输出口的状态
        Declare Function dmc_read_inport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal portno As Integer) As Long     '读取输入端口的值
        Declare Function dmc_read_outport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal portno As Integer) As Long            '读取输出端口的值
        Declare Function dmc_write_outport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal portno As Integer, ByVal outport_val As Long) As Integer       '设置输出端口的值
        Declare Function dmc_write_outport_16X Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal portno As Integer, ByVal outport_val As Long) As Integer
        'DMC5800专用 虚拟IO映射  用于读取滤波后的IO口电平状态
        Declare Function dmc_set_io_map_virtual Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal MapIoType As Integer, ByVal MapIoIndex As Integer, ByVal filter As Double) As Integer
        Declare Function dmc_get_io_map_virtual Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByRef MapIoType As Integer, ByRef MapIoIndex As Integer, ByRef filter As Double) As Integer
        Declare Function dmc_read_inbit_virtual Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer) As Integer
        
        '以上函数以毫秒为单位可继续使用，新函数将时间统一到秒为单位
        Declare Function dmc_reverse_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal reverse_time As Double) As Integer
        Declare Function dmc_set_io_count_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal mode As Integer, ByVal filter As Double) As Integer
        Declare Function dmc_get_io_count_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByRef mode As Integer, ByRef filter As Double) As Integer
        Declare Function dmc_set_io_count_value Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal CountValue As Long) As Integer
        Declare Function dmc_get_io_count_value Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByRef CountValue As Long) As Integer
        
        '3800,3600,5800,5600专用 轴IO映射配置
        'Declare Function dmc_set_AxisIoMap Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal MsgType As Integer, ByVal MapPortType As Integer, ByVal MapPortIndex As Integer, ByVal Filter As Long) As Integer
        'Declare Function dmc_get_AxisIoMap Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal MsgType As Integer, ByRef MapPortType As Integer, ByRef MapPortIndex As Integer, ByRef Filter As Long) As Integer
        '以上函数以毫秒为单位可继续使用，新函数将时间统一到秒为单位
        Declare Function dmc_set_axis_io_map Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal IoType As Integer, ByVal MapIoType As Integer, ByVal MapIoIndex As Integer, ByVal filter As Double) As Integer
        Declare Function dmc_get_axis_io_map Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal IoType As Integer, ByRef MapIoType As Integer, ByRef MapIoIndex As Integer, ByRef filter As Double) As Integer
        Declare Function dmc_set_special_input_filter Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal filter_time As Double) As Integer      '设置所有专用IO滤波时间
       
        '3410专用 回原点中的减速信号
        Declare Function dmc_set_sd_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal sd_logic As Integer, ByVal sd_mode As Integer) As Integer      '设置SD信号
        Declare Function dmc_get_sd_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef sd_logic As Integer, ByRef sd_mode As Integer) As Integer    '读取设置SD信号
  
        '伺服专用IO
        Declare Function dmc_set_inp_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal inp_logic As Integer) As Integer      '设置INP信号
        Declare Function dmc_get_inp_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef inp_logic As Integer) As Integer  '读取设置INP信号
        Declare Function dmc_set_rdy_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal rdy_logic As Integer) As Integer
        Declare Function dmc_get_rdy_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef rdy_logic As Integer) As Integer
        Declare Function dmc_set_erc_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal erc_logic As Integer, ByVal erc_width As Integer, ByVal erc_off_time As Integer) As Integer   '设置ERC信号
        Declare Function dmc_get_erc_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef erc_logic As Integer, ByRef erc_width As Integer, ByRef erc_off_time As Integer) As Integer   '读取设置ERC信号
        Declare Function dmc_set_alm_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal alm_logic As Integer, ByVal alm_action As Integer) As Integer '设置ALM信号
        Declare Function dmc_get_alm_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef alm_logic As Integer, ByRef alm_action As Integer) As Integer     '读取设置ALM信号
        Declare Function dmc_set_ez_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal ez_logic As Integer, ByVal ez_mode As Integer, ByVal filter As Double) As Integer       '设置EZ信号
        Declare Function dmc_get_ez_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef ez_logic As Integer, ByRef ez_mode As Integer, ByRef filter As Double) As Integer     '读取设置EZ信号

        Declare Function dmc_write_sevon_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal on_off As Integer) As Integer       '输出SEVON信号
        Declare Function dmc_read_sevon_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer         '读取SEVON信号
        Declare Function dmc_read_rdy_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer           '读取RDY状态
        Declare Function dmc_write_erc_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal on_off As Integer) As Integer    '控制ERC信号输出
        Declare Function dmc_read_erc_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer    '读取ERC信号输出端口
        Declare Function dmc_write_sevrst_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal on_off As Integer) As Integer
        Declare Function dmc_read_sevrst_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        
        '3800,3600,5800,5600专用 外部减速停止信号及减速停止时间配置
        'Declare Function dmc_set_dstp_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal logic As Integer, ByVal time As Long) As Integer
        'Declare Function dmc_get_dstp_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef logic As Integer, ByRef time As Long) As Integer
        'Declare Function dmc_set_dstp_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal time As Long) As Integer
        'Declare Function dmc_get_dstp_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef time As Long) As Integer
        '以上函数以毫秒为单位可继续使用，新函数将时间统一到秒为单位
        Declare Function dmc_set_io_dstp_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal logic As Integer) As Integer
        Declare Function dmc_get_io_dstp_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef logic As Integer) As Integer
        Declare Function dmc_set_dec_stop_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal stop_time As Double) As Integer
        Declare Function dmc_get_dec_stop_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef stop_time As Double) As Integer
        Declare Function dmc_set_vector_dec_stop_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal stop_time As Double) As Integer
        Declare Function dmc_get_vector_dec_stop_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef stop_time As Double) As Integer
        Declare Function dmc_set_dec_stop_dist Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Long) As Integer
        Declare Function dmc_get_dec_stop_dist Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Dist As Long) As Integer
        Declare Function dmc_set_io_dstp_bitno Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal bitno As Integer, ByVal filter As Double) As Integer
        Declare Function dmc_get_io_dstp_bitno Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef bitno As Integer, ByRef filter As Double) As Integer
       
        '编码器
        Declare Function dmc_set_counter_inmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal mode As Integer) As Integer      '设定编码器的计数方式
        Declare Function dmc_get_counter_inmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef mode As Integer) As Integer        '读取编码器的计数方式
        Declare Function dmc_get_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long
        Declare Function dmc_set_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal encoder_value As Long) As Integer
        Declare Function dmc_set_encoder_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal pos As Double) As Integer   '当前反馈位置
        Declare Function dmc_get_encoder_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef pos As Double) As Integer
        Declare Function dmc_set_handwheel_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByVal pos As Long) As Integer
        Declare Function dmc_get_handwheel_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef pos As Long) As Integer
        Declare Function dmc_set_extra_encoder_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByVal inmode As Integer, ByVal multi As Integer) As Integer
        Declare Function dmc_get_extra_encoder_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef inmode As Integer, ByRef multi As Integer) As Integer
        Declare Function dmc_set_extra_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByVal pos As Integer) As Integer
        Declare Function dmc_get_extra_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef pos As Integer) As Integer
        
        Declare Function dmc_get_position Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long      '读取指定轴的当前位置
        Declare Function dmc_set_position Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal current_position As Long) As Integer   '设定指定轴的当前位置
        Declare Function dmc_set_position_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal pos As Double) As Integer    '当前指令位置
        Declare Function dmc_get_position_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef pos As Double) As Integer
      
       
        Declare Function dmc_set_auxiliary_encoder_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByVal inmode As Integer, ByVal multi As Integer) As Integer
        Declare Function dmc_get_auxiliary_encoder_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef inmode As Integer, ByRef multi As Integer) As Integer
        Declare Function dmc_set_auxiliary_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByVal pos As Integer) As Integer
        Declare Function dmc_get_auxiliary_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef inmode As Integer) As Integer
  
        
        '3800,3600,5800,5600专用 IO辅助功能函数
        Declare Function dmc_IO_TurnOutDelay Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal DelayTime As Long) As Integer
        Declare Function dmc_SetIoCountMode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal mode As Integer, ByVal filter As Long) As Integer
        Declare Function dmc_GetIoCountMode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByRef mode As Integer, ByRef filter As Long) As Integer
        Declare Function dmc_SetIoCountValue Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal CountValue As Long) As Integer
        Declare Function dmc_GetIoCountValue Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByRef CountValue As Long) As Integer
    
        
       
        '运动状态
        Declare Function dmc_read_current_speed Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Double      '读取指定轴的当前速度
        Declare Function dmc_read_current_speed_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef current_speed As Double) As Integer  '轴当前运行速度
             
        Declare Function dmc_get_target_position Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long      '读取目标位置
        Declare Function dmc_get_target_position_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef pos As Double) As Long      '读取目标位置
        Declare Function dmc_check_done Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer     '读取指定轴的运动状态
        Declare Function dmc_axis_io_status Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Long    '读取指定轴有关运动信号的状态
        Declare Function dmc_stop Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal stop_mode As Integer) As Integer       '单轴停止
        Declare Function dmc_check_done_multicoor Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Integer
        Declare Function dmc_stop_multicoor Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal stop_mode As Integer) As Integer
        Declare Function dmc_emg_stop Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer      '紧急停止所有轴
        '3800,3600,5800,5600专用 主卡与接线盒通讯状态
        Declare Function dmc_LinkState Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef state As Integer) As Integer      '连接状态
        Declare Function dmc_get_axis_run_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef run_mode As Integer) As Integer       '轴运动模式
        Declare Function dmc_get_stop_reason Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef StopReason As Long) As Integer   '读取轴停止原因
        Declare Function dmc_clear_stop_reason Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer   '清除轴停止原因
        
        '检测轴到位状态
        Declare Function dmc_set_factor_error Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal factor As Double, ByVal error As Long) As Integer
        Declare Function dmc_get_factor_error Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef factor As Double, ByRef error As Long) As Integer
        Declare Function dmc_check_success_pulse Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        Declare Function dmc_check_success_encoder Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
       
        'DMC5000系列专用，基于脉冲当量的高级运动功能，连续插补运动功能
        Declare Function dmc_set_trace Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer) As Integer      'trace功能
        Declare Function dmc_get_trace Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer) As Integer
        Declare Function dmc_read_trace_data Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal data_option As Integer, ByRef ReceiveSize As Long, ByRef time As Double, ByRef data As Double, ByRef remain_num As Long) As Integer
        Declare Function dmc_trace_start Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer) As Integer
        Declare Function dmc_trace_stop Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer

        Declare Function dmc_calculate_arclength_center Lib "LTDMC.dll" (ByRef start_pos As Double, ByRef target_Pos As Double, ByRef cen_Pos As Double, ByVal arc_dir As Integer, ByVal ArcCircle As Double, ByRef ArcLength As Double) As Integer
        Declare Function dmc_calculate_arclength_3point Lib "LTDMC.dll" (ByRef start_pos As Double, ByRef mid_pos As Double, ByRef target_Pos As Double, ByVal ArcCircle As Double, ByRef ArcLength As Double) As Integer
        Declare Function dmc_calculate_arclength_radius Lib "LTDMC.dll" (ByRef start_pos As Double, ByRef target_Pos As Double, ByVal arc_radius As Double, ByVal arc_dir As Integer, ByVal ArcCircle As Double, ByRef ArcLength As Double) As Integer

        '连续插补
        Declare Function dmc_conti_open_list Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer) As Integer     '打开连续缓存区
        Declare Function dmc_conti_close_list Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Integer     '关闭连续缓存区
        Declare Function dmc_conti_reset_list Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Integer
        Declare Function dmc_conti_stop_list Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal stop_mode As Integer) As Integer    '连续插补中停止
        Declare Function dmc_conti_pause_list Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Integer     '连续插补中暂停
        Declare Function dmc_conti_start_list Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Integer    '开始连续插补
        Declare Function dmc_conti_get_run_state Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Integer    '读取连续插补状态：0-运行，1-暂停，2-正常停止，3-异常停止
        Declare Function dmc_conti_remain_space Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Long     '查连续插补剩余缓存数
        Declare Function dmc_conti_read_current_mark Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Long  '读取当前连续插补段的标号
        Declare Function dmc_conti_set_blend Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal enable As Integer) As Integer    'blend拐角过度模式
        Declare Function dmc_conti_get_blend Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef enable As Integer) As Integer
        'Declare Function dmc_conti_set_lookahead_end_vel_zero Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal enable As Integer) As Integer
        Declare Function dmc_conti_set_override Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal Percent As Double) As Integer   '设置每段速度比例  缓冲区指令
        Declare Function dmc_conti_change_speed_ratio Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal Percent As Double) As Integer   '连续插补动态变速
        
        Declare Function dmc_conti_set_lookahead_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal enable As Integer, ByVal LookaheadSegments As Long, ByVal PathError As Double, ByVal LookaheadAcc As Double) As Integer
        Declare Function dmc_conti_get_lookahead_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef enable As Integer, ByRef LookaheadSegments As Long, ByRef PathError As Double, ByRef LookaheadAcc As Double) As Integer

        'Declare Function dmc_conti_set_profile_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal Min_Vel As Double, ByVal Max_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal Stop_vel As Double) As Integer   '设置连续插补速度
        'Declare Function dmc_conti_set_s_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal s_mode As Integer, ByVal s_para As Double) As Integer      '设置连续插补平滑时间
       ' Declare Function dmc_conti_get_s_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal s_mode As Integer, ByRef s_para As Double) As Integer
        Declare Function dmc_conti_wait_input Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal bitno As Integer, ByVal on_off As Integer, ByVal TimeOut As Double, ByVal mark As Long) As Integer   '设置连续插补等待输入
        Declare Function dmc_conti_delay_outbit_to_start Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal bitno As Integer, ByVal on_off As Integer, ByVal delay_value As Double, ByVal delay_mode As Integer, ByVal ReverseTime As Double) As Integer    '相对于轨迹段起点IO滞后输出
        Declare Function dmc_conti_delay_outbit_to_stop Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal bitno As Integer, ByVal on_off As Integer, ByVal delay_time As Double, ByVal ReverseTime As Double) As Integer   '相对于轨迹段终点IO滞后输出
        Declare Function dmc_conti_ahead_outbit_to_stop Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal bitno As Integer, ByVal on_off As Integer, ByVal ahead_value As Double, ByVal ahead_mode As Integer, ByVal ReverseTime As Double) As Integer  '相对轨迹段终点IO提前输出
        Declare Function dmc_conti_accurate_outbit_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal cmp_no As Integer, ByVal on_off As Integer, ByVal map_axis As Integer, ByVal abs_pos As Double, ByVal pos_source As Integer, ByVal ReverseTime As Double) As Integer     '确定位置精确输出
        Declare Function dmc_conti_write_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal bitno As Integer, ByVal on_off As Integer, ByVal ReverseTime As Double) As Integer     '缓冲区立即IO输出
        Declare Function dmc_conti_clear_io_action Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal IoMask As Long) As Integer   '清除段内未执行完的IO动作
        Declare Function dmc_conti_set_pause_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal action As Integer, ByVal mask As Long, ByVal state As Long) As Integer   '暂停时IO输出 action 0, 不工作；1， 暂停时输出io_state; 2 暂停时输出io_state, 继续运行时首先恢复原来的io; 3,在2的基础上，停止时也生效
        Declare Function dmc_conti_get_pause_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef action As Integer, ByRef mask As Long, ByRef state As Long) As Integer
        'Declare Function dmc_conti_check_done Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer) As Integer      '检测连续插补运动状态：0-运行，1-停止
        Declare Function dmc_conti_delay Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal delay_time As Double, ByVal mark As Long) As Integer    '添加延时指令
        Declare Function dmc_conti_reverse_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal bitno As Integer, ByVal reverse_time As Double) As Integer
        Declare Function dmc_conti_delay_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal bitno As Integer, ByVal on_off As Integer, ByVal delay_time As Double) As Integer
       
        Declare Function dmc_conti_line_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByVal posi_mode As Integer, ByVal mark As Long) As Integer    '连续插补直线
        Declare Function dmc_conti_arc_move_center_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByRef cen_Pos As Double, ByVal arc_dir As Integer, ByVal CircleNum As Long, ByVal posi_mode As Integer, ByVal mark As Long) As Integer    '圆心终点式圆弧/螺旋线/渐开线
        Declare Function dmc_conti_arc_move_radius_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByVal arc_radius As Double, ByVal arc_dir As Integer, ByVal CircleNum As Long, ByVal posi_mode As Integer, ByVal mark As Long) As Integer   '半径终点式圆弧/螺旋线
        Declare Function dmc_conti_arc_move_3points_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByRef mid_pos As Double, ByVal CircleNum As Long, ByVal posi_mode As Integer, ByVal mark As Long) As Integer    '三点式圆弧/螺旋线
        Declare Function dmc_conti_rectangle_move_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef target_Pos As Double, ByRef Mark_Pos As Double, ByVal Count As Long, ByVal rect_mode As Integer, ByVal posi_mode As Integer, ByVal mark As Long) As Integer
        Declare Function dmc_conti_pmove_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal axis As Integer, ByVal Dist As Double, ByVal posi_mode As Integer, ByVal mode As Integer, ByVal mark As Long) As Integer
  
        Declare Function dmc_conti_set_involute_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal mode As Integer) As Integer
        Declare Function dmc_conti_get_involute_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef mode As Integer) As Integer
        
        Declare Function dmc_set_gear_follow_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal master_axis As Integer, ByVal Ratio As Double) As Integer
        Declare Function dmc_get_gear_follow_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef master_axis As Integer, ByRef Ratio As Double) As Integer
             
        Declare Function dmc_set_pwm_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal portno As Integer, ByVal on_off As Integer, ByVal dfreqency As Double, ByVal dduty As Double) As Integer
        Declare Function dmc_get_pwm_pin Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal portno As Integer, ByRef on_off As Integer, ByRef dfreqency As Double, ByRef dduty As Double) As Integer
        Declare Function dmc_set_pwm_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal enable As Integer) As Integer
        Declare Function dmc_get_pwm_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef enable As Integer) As Integer
        Declare Function dmc_set_pwm_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal pwm_no As Integer, ByVal fDuty As Double, ByVal fFre As Double) As Integer
        Declare Function dmc_get_pwm_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal pwm_no As Integer, ByRef fDuty As Double, ByRef fFre As Double) As Integer
        Declare Function dmc_conti_set_pwm_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal pwm_no As Integer, ByVal fDuty As Double, ByVal fFre As Double) As Integer
        Declare Function dmc_set_pwm_enable_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByVal enable As Integer) As Integer
        Declare Function dmc_get_pwm_enable_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef enable As Integer) As Integer

        Declare Function dmc_conti_set_pwm_follow_speed Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal pwm_no As Integer, ByVal mode As Integer, ByVal MaxVel As Double, ByVal MaxValue As Double, ByVal OutValue As Double) As Integer
        Declare Function dmc_conti_get_pwm_follow_speed Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal pwm_no As Integer, ByRef mode As Integer, ByRef MaxVel As Double, ByRef MaxValue As Double, ByRef OutValue As Double) As Integer
       
        Declare Function dmc_set_pwm_onoff_duty Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal pwm_no As Integer, ByVal fOnDuty As Double, ByVal fOffDuty As Double) As Integer
        Declare Function dmc_get_pwm_onoff_duty Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal pwm_no As Integer, ByRef fOnDuty As Double, ByRef fOffDuty As Double) As Integer
        Declare Function dmc_conti_delay_pwm_to_start Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal pwm_no As Integer, ByVal on_off As Integer, ByVal delay_value As Double, ByVal delay_mode As Integer, ByVal ReverseTime As Double) As Integer
        Declare Function dmc_conti_delay_pwm_to_stop Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal pwm_no As Integer, ByVal on_off As Integer, ByVal delay_time As Integer, ByVal ReverseTime As Double) As Integer
        Declare Function dmc_conti_ahead_pwm_to_stop Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal pwm_no As Integer, ByVal on_off As Integer, ByVal ahead_value As Double, ByVal ahead_mode As Integer, ByVal ReverseTime As Double) As Integer
        Declare Function dmc_conti_write_pwm Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal pwm_no As Integer, ByVal on_off As Integer, ByVal ReverseTime As Double) As Integer

        Declare Function dmc_set_da_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal enable As Integer) As Integer
        Declare Function dmc_get_da_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef enable As Integer) As Integer
        Declare Function dmc_set_da_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByVal Vout As Double) As Integer
        Declare Function dmc_get_da_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef Vout As Double) As Integer
        Declare Function dmc_get_ad_input Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef Vout As Double) As Integer
        
        Declare Function dmc_conti_set_da_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal channel As Integer, ByVal Vout As Double) As Integer
        Declare Function dmc_conti_set_da_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal enable As Integer, ByVal channel As Integer, ByVal mark As Long) As Integer
        
        Declare Function dmc_conti_set_da_follow_speed Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal da_no As Integer, ByVal MaxVel As Double, ByVal MaxValue As Double, ByVal acc_offset As Double, ByVal dec_offset As Double, ByVal acc_dist As Double, ByVal dec_dist As Double) As Integer
        Declare Function dmc_conti_get_da_follow_speed Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal da_no As Integer, ByRef MaxVel As Double, ByRef MaxValue As Double, ByRef acc_offset As Double, ByRef dec_offset As Double, ByRef acc_dist As Double, ByRef dec_dist As Double) As Integer
        
        Declare Function dmc_set_arc_limit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal enable As Integer, ByVal MaxCenAcc As Double, ByVal MaxArcError As Double) As Integer
        Declare Function dmc_get_arc_limit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef enable As Integer, ByRef MaxCenAcc As Double, ByRef MaxArcError As Double) As Integer

        '3800,3600,5800,5600专用 CAN-IO扩展
        Declare Function dmc_set_can_state Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeNum As Integer, ByVal state As Integer, ByVal baud As Integer) As Integer
        Declare Function dmc_get_can_state Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef NodeNum As Integer, ByRef state As Integer) As Integer
        Declare Function dmc_write_can_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Node As Integer, ByVal bitno As Integer, ByVal on_off As Integer) As Integer
        Declare Function dmc_read_can_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Node As Integer, ByVal bitno As Integer) As Integer
        Declare Function dmc_read_can_inbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Node As Integer, ByVal bitno As Integer) As Integer
        Declare Function dmc_write_can_outport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Node As Integer, ByVal portno As Integer, ByVal outport_val As Long) As Integer
        Declare Function dmc_read_can_outport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Node As Integer, ByVal portno As Integer) As Long
        Declare Function dmc_read_can_inport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Node As Integer, ByVal portno As Integer) As Long
        Declare Function dmc_get_can_errcode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef errcode As Integer) As Integer
        
        Declare Function dmc_get_can_errcode_extern Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef errcode As Integer, ByRef msg_losed As Integer, ByRef emg_msg_num As Integer, ByRef lostHeartB As Integer, ByRef EmgMsg As Integer) As Integer

        Declare Function dmc_t_pmove_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Double, ByVal posi_mode As Integer) As Integer  '对称T型定长
        Declare Function dmc_ex_t_pmove_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Double, ByVal posi_mode As Integer) As Integer '非对称T型定长
        Declare Function dmc_s_pmove_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Double, ByVal posi_mode As Integer) As Integer    '对称S型定长
        Declare Function dmc_ex_s_pmove_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Dist As Double, ByVal posi_mode As Integer) As Integer  '非对称S型定长
        
        '软锁存
        Declare Function dmc_softltc_set_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal ltc_enable As Integer, ByVal ltc_mode As Integer, ByVal ltc_inbit As Integer, ByVal ltc_logic As Integer, ByVal filter As Double) As Integer
        Declare Function dmc_softltc_get_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByRef ltc_enable As Integer, ByRef ltc_mode As Integer, ByRef ltc_inbit As Integer, ByRef ltc_logic As Integer, ByRef filter As Double) As Integer
        
        Declare Function dmc_softltc_set_source Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal axis As Integer, ByVal ltc_source As Integer) As Integer
        Declare Function dmc_softltc_get_source Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal axis As Integer, ByRef ltc_source As Integer) As Integer
        
        Declare Function dmc_softltc_reset Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer) As Integer
        Declare Function dmc_softltc_get_number Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal axis As Integer, ByRef number As Integer) As Integer
        Declare Function dmc_softltc_get_value_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal latch As Integer, ByVal axis As Integer, ByRef Value As Double) As Integer
        
        Declare Function dmc_set_IoFilter Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByVal filter As Double) As Integer
        Declare Function dmc_get_IoFilter Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal bitno As Integer, ByRef filter As Double) As Integer
        
        
        Declare Function dmc_set_lsc_index_value Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal IndexID As Integer, ByVal IndexValue As Long) As Integer
        Declare Function dmc_get_lsc_index_value Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal IndexID As Integer, ByRef IndexValue As Long) As Integer
        
        Declare Function dmc_set_lsc_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Origin As Integer, ByVal Interal As Long, ByVal NegIndex As Long, ByVal PosIndex As Long, ByVal Ratio As Double) As Integer
        Declare Function dmc_get_lsc_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Origin As Integer, ByRef Interal As Long, ByRef NegIndex As Long, ByRef PosIndex As Long, ByRef Ratio As Double) As Integer
        
        Declare Function dmc_set_watchdog Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal enable As Integer, ByVal time As Long) As Integer
        Declare Function dmc_call_watchdog Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        
        Declare Function dmc_set_zone_limit_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef axis As Integer, ByRef Source As Integer, ByVal x_pos_p As Long, ByVal x_pos_n As Long, ByVal y_pos_p As Long, ByVal y_pos_n As Long, ByVal action_para As Integer) As Integer
        Declare Function dmc_get_zone_limit_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef axis As Integer, ByRef Source As Integer, ByRef x_pos_p As Long, ByRef x_pos_n As Long, ByRef y_pos_p As Long, ByRef y_pos_n As Long, ByRef action_para As Integer) As Integer
        Declare Function dmc_set_zone_limit_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal enable As Integer) As Integer
        
        Declare Function dmc_set_interlock_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef axis As Integer, ByRef Source As Integer, ByVal delta_pos As Long, ByVal action_para As Integer) As Integer
        Declare Function dmc_get_interlock_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef axis As Integer, ByRef Source As Integer, ByRef delta_pos As Long, ByRef action_para As Integer) As Integer
        Declare Function dmc_set_interlock_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal enable As Integer) As Integer
       
        Declare Function dmc_set_grant_error_protect Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal dstp_error As Long, ByVal emg_error As Long) As Integer
        Declare Function dmc_get_grant_error_protect Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef dstp_error As Long, ByRef emg_error As Long) As Integer
            
        Declare Function dmc_set_camerablow_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal camerablow_en As Integer, ByVal cameraPos As Long, ByVal piece_num As Integer, ByVal piece_distance As Long, ByVal axis_sel As Integer, ByVal latch_distance_min As Long) As Integer
        Declare Function dmc_get_camerablow_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef camerablow_en As Integer, ByRef cameraPos As Long, ByRef piece_num As Integer, ByRef piece_distance As Long, ByRef axis_sel As Integer, ByRef latch_distance_min As Long) As Integer
        Declare Function dmc_clear_camerablow_errorcode Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        Declare Function dmc_get_camerablow_errorcode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef errorcode As Integer) As Integer
        
        Declare Function dmc_set_encoder_da_follow_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer) As Integer
        Declare Function dmc_get_encoder_da_follow_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer) As Integer
        
        Declare Function dmc_set_io_limit_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal portno As Integer, ByVal enable As Integer, ByVal axis_sel As Integer, ByVal el_mode As Integer, ByVal el_logic As Integer) As Integer
        Declare Function dmc_get_io_limit_config Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal portno As Integer, ByRef enable As Integer, ByRef axis_sel As Integer, ByRef el_mode As Integer, ByRef el_logic As Integer) As Integer
        
        Declare Function dmc_set_handwheel_filter Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal filter_factor As Double) As Integer
        Declare Function dmc_get_handwheel_filter Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef filter_factor As Double) As Integer
        
        Declare Function dmc_conti_get_interp_map Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef AxisNum As Integer, ByRef AxisList As Integer, ByRef pPosList As Double) As Integer
        
        Declare Function dmc_conti_get_crd_errcode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef errcode As Integer) As Integer
        
        Declare Function dmc_line_unit_follow Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef Dist As Double, ByVal posi_mode As Integer) As Integer
        Declare Function dmc_conti_line_unit_follow Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef pPosList As Double, ByVal posi_mode As Integer, ByVal mark As Long) As Integer
               
        Declare Function dmc_conti_set_da_action Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal mode As Integer, ByVal portno As Integer, ByVal dvalue As Double) As Integer
        
        Declare Function dmc_read_encoder_speed Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef current_speed As Double) As Integer
        
        Declare Function dmc_axis_follow_line_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal enable_flag As Integer) As Integer
        
        Declare Function dmc_set_interp_compensation Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal dvalue As Double, ByVal time As Double) As Integer
        Declare Function dmc_get_interp_compensation Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef dvalue As Double, ByRef time As Double) As Integer
        
        Declare Function dmc_set_io_exactstop Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal ioNum As Integer, ByRef ioList As Integer, ByVal enable As Integer, ByVal valid_logic As Integer, ByVal action As Integer) As Integer
        
        Declare Function dmc_get_distance_to_start Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByRef distance_x As Double, ByRef distance_y As Double, ByVal imark As Long) As Integer
        
        Declare Function dmc_set_start_distance_flag Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal flag As Double) As Integer
        
        Declare Function dmc_set_home_soft_limit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal N_limit As Long, ByVal P_limit As Long) As Integer
        Declare Function dmc_get_home_soft_limit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef N_limit As Long, ByRef P_limit As Long) As Integer
        
        Declare Function dmc_conti_gear_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Crd As Integer, ByVal axis As Integer, ByVal Dist As Double, ByVal follow_mode As Integer,ByVal imark As Long) As Integer
        
        '总线参数
        Declare Function nmc_set_home_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal home_mode As Integer, ByVal High_Vel As Double, ByVal Low_Vel As Double, ByVal Tacc As Double, ByVal Tdec As Double, ByVal offsetpos As Double) As Integer
        Declare Function nmc_get_home_profile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef home_mode As Integer, ByRef High_Vel As Double, ByRef Low_Vel As Double, ByRef Tacc As Double, ByRef Tdec As Double, ByRef offsetpos As Double) As Integer
        Declare Function nmc_home_move Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        
        Declare Function nmc_set_manager_para Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal Baudrate As Long, ByVal ManagerID As Integer) As Integer
        Declare Function nmc_get_manager_para Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByRef Baudrate As Long, ByRef ManagerID As Integer) As Integer
        Declare Function nmc_set_manager_od Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal index As Integer, ByVal subindex As Integer, ByVal valuelength As Integer, ByVal Value As Long) As Integer
        Declare Function nmc_get_manager_od Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal index As Integer, ByVal subindex As Integer, ByVal valuelength As Integer, ByRef Value As Long) As Integer
        Declare Function nmc_set_node_od Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal NodeNum As Integer, ByVal index As Integer, ByVal subindex As Integer, ByVal valuelength As Integer, ByVal Value As Long) As Integer
        Declare Function nmc_get_node_od Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal NodeNum As Integer, ByVal index As Integer, ByVal subindex As Integer, ByVal valuelength As Integer, ByRef Value As Long) As Integer
        Declare Function nmc_upload_configfile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal FileName As String) As Integer
        Declare Function nmc_reset_to_factory Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal NodeNum As Integer) As Integer
        Declare Function nmc_write_to_pci Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal NodeNum As Integer) As Integer
        Declare Function nmc_download_configfile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal FileName As String) As Integer
        Declare Function nmc_download_mapfile Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal FileName As String) As Integer
        
        Declare Function nmc_set_axis_enable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        Declare Function nmc_set_axis_disable Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        Declare Function nmc_set_alarm_clear Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal NodeNum As Integer) As Integer
        Declare Function nmc_get_slave_nodes Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal Baudrate As Integer, ByRef NodeId As Integer, ByRef NodeNum As Integer) As Integer
        
        Declare Function nmc_get_total_axes Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef TotalAxis As Integer) As Integer
        Declare Function nmc_get_total_adcnum Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef TotalIn As Integer, ByRef TotalOut As Integer) As Integer
        Declare Function nmc_get_total_ionum Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef TotalIn As Integer, ByRef TotalOut As Integer) As Integer
        
        Declare Function nmc_clear_alarm_fieldbus Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer) As Integer
        'EtherCAT
        Declare Function nmc_set_controller_workmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal controller_mode As Integer) As Integer
        Declare Function nmc_get_controller_workmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef controller_mode As Integer) As Integer
        Declare Function nmc_set_cycletime Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Fieldbustype As Integer, ByVal CycleTime As Long) As Integer
        Declare Function nmc_get_cycletime Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Fieldbustype As Integer, ByRef CycleTime As Long) As Integer
        Declare Function nmc_set_axis_run_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal run_mode As Integer) As Integer
        Declare Function dmc_get_perline_time Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal TypeIndex As Integer, ByRef Averagetime As Long, ByRef Maxtime As Long, ByRef Cycles As Long) As Integer
        
        Declare Function nmc_write_rxpdo_extra Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal address As Integer, ByVal DataLen As Integer, ByVal Value As Long) As Integer
        Declare Function nmc_read_rxpdo_extra Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal address As Integer, ByVal DataLen As Integer, ByRef Value As Long) As Integer
        Declare Function nmc_read_txpdo_extra Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal address As Integer, ByVal DataLen As Integer, ByRef Value As Long) As Integer
        
        Declare Function nmc_get_axis_type Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Axis_Type As Integer) As Integer
        Declare Function nmc_get_consume_time_fieldbus Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Fieldbustype As Integer, ByRef Average_time As Integer, ByRef Max_time As Integer, ByRef Cycles As Long) As Integer
        Declare Function nmc_clear_consume_time_fieldbus Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal Fieldbustype As Integer) As Integer
        Declare Function nmc_stop_etc Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef ETCState As Integer) As Integer
          
        Declare Function nmc_get_axis_statusword Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef statusword As Long) As Integer
        Declare Function nmc_set_axis_contrlword Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal contrlword As Long) As Integer
        Declare Function nmc_get_axis_contrlword Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef contrlword As Long) As Integer
        Declare Function nmc_set_axis_contrlmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal Contrlmode As Long) As Integer
        Declare Function nmc_get_axis_contrlmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Contrlmode As Long) As Integer
        
        Declare Function nmc_get_errcode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer, ByRef errcode As Integer) As Integer
        Declare Function nmc_get_card_errcode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef errcode As Integer) As Integer
        Declare Function nmc_get_axis_errcode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef errcode As Integer) As Integer
        Declare Function nmc_clear_errcode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal channel As Integer) As Integer
        Declare Function nmc_clear_card_errcode Lib "LTDMC.dll" (ByVal CardNo As Integer) As Integer
        Declare Function nmc_clear_axis_errcode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        
        Declare Function nmc_get_LostHeartbeat_Nodes Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByRef NodeId As Integer, ByRef NodeNum As Integer) As Integer
        Declare Function nmc_get_EmergeneyMessege_Nodes Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByRef NodeMsg As Long, ByRef MsgNum As Integer) As Integer
        Declare Function nmc_SendNmtCommand Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal NodeId As Integer, ByVal NmtCommand As Integer) As Integer
        Declare Function nmc_syn_move Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef position As Long, ByRef PosiMode As Integer) As Integer
        Declare Function nmc_syn_move_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef position As Double, ByRef PosiMode As Integer) As Integer
    
        Declare Function nmc_sync_pmove_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef Dist As Double, ByRef PosiMode As Integer) As Integer
        Declare Function nmc_sync_vmove_unit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AxisNum As Integer, ByRef AxisList As Integer, ByRef Dir As Integer) As Integer
        
        Declare Function nmc_set_master_para Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal Baudrate As Integer, ByVal NodeCnt As Long, ByVal MasterId As Integer) As Integer
        Declare Function nmc_get_master_para Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByRef Baudrate As Integer, ByRef NodeCnt As Long, ByRef MasterId As Integer) As Integer
        
        Declare Function nmc_write_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal IoBit As Integer, ByVal IoValue As Integer) As Integer
        Declare Function nmc_read_outbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal IoBit As Integer, ByRef IoValue As Integer) As Integer
        Declare Function nmc_read_inbit Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal IoBit As Integer, ByRef IoValue As Integer) As Integer
        
        Declare Function nmc_set_da_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal channel As Integer, ByVal Value As Double) As Integer
        Declare Function nmc_get_da_output Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal channel As Integer, ByRef Value As Double) As Integer
        Declare Function nmc_get_ad_input Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal channel As Integer, ByRef Value As Double) As Integer
        
        Declare Function nmc_set_ad_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal channel As Integer, ByVal mode As Integer, ByVal buffer_nums As Long) As Integer
        Declare Function nmc_get_ad_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal channel As Integer, ByRef mode As Integer, ByVal buffer_nums As Long) As Integer
        Declare Function nmc_set_da_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal channel As Integer, ByVal mode As Integer, ByVal buffer_nums As Long) As Integer
        Declare Function nmc_get_da_mode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal channel As Integer, ByRef mode As Integer, ByVal buffer_nums As Long) As Integer
        
        Declare Function nmc_write_to_flash Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByVal NodeNum As Integer) As Integer
        
        Declare Function nmc_set_connect_state Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeNum As Integer, ByVal state As Integer, ByVal baud As Integer) As Integer
        Declare Function nmc_get_connect_state Lib "LTDMC.dll" (ByVal CardNo As Integer, ByRef NodeNum As Integer, ByRef state As Integer) As Integer
        
        Declare Function nmc_write_outport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal portno As Integer, ByVal IoValue As Integer) As Integer
        Declare Function nmc_read_outport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal portno As Integer, ByRef IoValue As Integer) As Integer
        Declare Function nmc_read_inport Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal NodeId As Integer, ByVal portno As Integer, ByRef IoValue As Integer) As Integer
        
        Declare Function nmc_get_axis_state_machine Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Axis_StateMachine As Integer) As Integer
        
        Declare Function nmc_get_axis_setting_contrlmode Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef Contrlmode As Integer) As Integer
              
        Declare Function nmc_get_total_slaves Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal PortNum As Integer, ByRef TotalSlaves As Integer) As Integer
        Declare Function nmc_get_axis_node_address Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef SlaveAddr As Integer, ByRef Sub_SlaveAddr As Integer) As Integer
        Declare Function nmc_set_axis_io_out Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal iostate As Long) As Integer
        Declare Function nmc_get_axis_io_out Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        Declare Function nmc_get_axis_io_in Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer) As Integer
        
        'Declare Function dmc_set_safety_param Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByVal enable As Integer, ByVal safety_pos As Integer) As Integer
        'Declare Function dmc_get_safety_param Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal axis As Integer, ByRef enable As Integer, ByRef safety_pos As Integer) As Integer
        'Declare Function nmc_start_scan_ethercat Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AddressID As Integer) As Integer
        'Declare Function nmc_stop_scan_ethercat Lib "LTDMC.dll" (ByVal CardNo As Integer, ByVal AddressID As Integer) As Integer

        
        
       
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'''                                 LTDMC V1.1 end of module                       '''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

