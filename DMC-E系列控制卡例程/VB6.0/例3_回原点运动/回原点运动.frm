VERSION 5.00
Begin VB.Form FormMain 
   Caption         =   "回原点运动"
   ClientHeight    =   4620
   ClientLeft      =   60
   ClientTop       =   450
   ClientWidth     =   12675
   LinkTopic       =   "Form1"
   ScaleHeight     =   4620
   ScaleWidth      =   12675
   StartUpPosition =   2  '屏幕中心
   Begin VB.CommandButton Command1 
      Caption         =   "脉冲轴回零"
      Height          =   495
      Left            =   2400
      TabIndex        =   55
      Top             =   3960
      Width           =   1215
   End
   Begin VB.Frame Frame4 
      Caption         =   "回零速度"
      Height          =   735
      Left            =   3720
      TabIndex        =   52
      Top             =   2880
      Width           =   2895
      Begin VB.OptionButton Option3 
         Caption         =   "高速"
         Height          =   255
         Index           =   1
         Left            =   1440
         TabIndex        =   54
         Top             =   360
         Width           =   735
      End
      Begin VB.OptionButton Option3 
         Caption         =   "低速"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   53
         Top             =   360
         Value           =   -1  'True
         Width           =   735
      End
   End
   Begin VB.Frame HomeDie 
      Caption         =   "回零方向"
      Height          =   855
      Left            =   3720
      TabIndex        =   49
      Top             =   1800
      Width           =   2895
      Begin VB.OptionButton Option2 
         Caption         =   "负向"
         Height          =   180
         Index           =   1
         Left            =   1440
         TabIndex        =   51
         Top             =   480
         Width           =   975
      End
      Begin VB.OptionButton Option2 
         Caption         =   "正向"
         Height          =   180
         Index           =   0
         Left            =   240
         TabIndex        =   50
         Top             =   480
         Value           =   -1  'True
         Width           =   975
      End
   End
   Begin VB.Frame HomeMode 
      Caption         =   "回零模式"
      Height          =   1455
      Left            =   3720
      TabIndex        =   45
      Top             =   120
      Width           =   2895
      Begin VB.OptionButton Option1 
         Caption         =   "二次回零"
         Height          =   255
         Index           =   2
         Left            =   240
         TabIndex        =   48
         Top             =   1080
         Width           =   1335
      End
      Begin VB.OptionButton Option1 
         Caption         =   "一次回零+反找"
         Height          =   255
         Index           =   1
         Left            =   240
         TabIndex        =   47
         Top             =   720
         Width           =   1815
      End
      Begin VB.OptionButton Option1 
         Caption         =   "一次回零"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   46
         Top             =   360
         Value           =   -1  'True
         Width           =   1335
      End
   End
   Begin VB.Frame Frame3 
      Caption         =   "复位操作及总线状态"
      Height          =   1335
      Left            =   6960
      TabIndex        =   28
      Top             =   2520
      Width           =   5535
      Begin VB.CommandButton Btn_SoftReset 
         Caption         =   "软件复位"
         Height          =   495
         Left            =   1560
         TabIndex        =   43
         Top             =   720
         Width           =   1095
      End
      Begin VB.CommandButton Btn_HandReset 
         Caption         =   "硬件复位"
         Height          =   495
         Left            =   240
         TabIndex        =   42
         Top             =   720
         Width           =   1095
      End
      Begin VB.TextBox Text2 
         Height          =   375
         Left            =   1080
         TabIndex        =   29
         Text            =   " "
         Top             =   240
         Width           =   1695
      End
      Begin VB.Label display 
         BackColor       =   &H00FFFFFF&
         BorderStyle     =   1  'Fixed Single
         Enabled         =   0   'False
         Height          =   735
         Left            =   2880
         TabIndex        =   31
         Top             =   360
         Width           =   2415
      End
      Begin VB.Label Label4 
         Alignment       =   2  'Center
         Caption         =   "总线状态"
         Height          =   375
         Index           =   4
         Left            =   120
         TabIndex        =   30
         Top             =   360
         Width           =   855
      End
   End
   Begin VB.Frame Frame2 
      Caption         =   "信息显示"
      Height          =   2295
      Left            =   6960
      TabIndex        =   14
      Top             =   120
      Width           =   5535
      Begin VB.TextBox Text1 
         DataMember      =   "state"
         Height          =   375
         Index           =   0
         Left            =   1080
         TabIndex        =   20
         Text            =   " "
         Top             =   1680
         Width           =   2055
      End
      Begin VB.TextBox vel 
         Height          =   375
         Index           =   0
         Left            =   1080
         TabIndex        =   19
         Text            =   " "
         Top             =   720
         Width           =   1095
      End
      Begin VB.TextBox My_NowPos 
         Height          =   375
         Index           =   0
         Left            =   1080
         TabIndex        =   18
         Top             =   240
         Width           =   1095
      End
      Begin VB.TextBox Text3 
         Height          =   495
         Index           =   0
         Left            =   3120
         TabIndex        =   17
         Text            =   "原点"
         Top             =   600
         Width           =   615
      End
      Begin VB.TextBox Text3 
         Height          =   495
         Index           =   1
         Left            =   3840
         TabIndex        =   16
         Text            =   "正限位"
         Top             =   600
         Width           =   615
      End
      Begin VB.TextBox Text3 
         Height          =   495
         Index           =   2
         Left            =   4560
         TabIndex        =   15
         Text            =   "负限位"
         Top             =   600
         Width           =   615
      End
      Begin VB.Label Label13 
         Caption         =   "unit/s"
         Height          =   255
         Left            =   2280
         TabIndex        =   41
         Top             =   840
         Width           =   735
      End
      Begin VB.Label Label12 
         Caption         =   "unit"
         Height          =   255
         Left            =   2280
         TabIndex        =   40
         Top             =   360
         Width           =   735
      End
      Begin VB.Label Label8 
         Caption         =   "运动状态"
         Height          =   255
         Left            =   240
         TabIndex        =   25
         Top             =   1320
         Width           =   735
      End
      Begin VB.Label Lable_STATUS 
         BackColor       =   &H00FFFFFF&
         BorderStyle     =   1  'Fixed Single
         Height          =   375
         Left            =   1080
         TabIndex        =   24
         Top             =   1200
         Width           =   1095
      End
      Begin VB.Label Label4 
         Alignment       =   2  'Center
         Caption         =   "状态机"
         Height          =   375
         Index           =   2
         Left            =   240
         TabIndex        =   23
         Top             =   1800
         Width           =   855
      End
      Begin VB.Label Label4 
         Alignment       =   2  'Center
         Caption         =   "速度"
         Height          =   375
         Index           =   1
         Left            =   360
         TabIndex        =   22
         Top             =   840
         Width           =   855
      End
      Begin VB.Label Label4 
         Alignment       =   2  'Center
         Caption         =   "位置"
         Height          =   375
         Index           =   3
         Left            =   360
         TabIndex        =   21
         Top             =   360
         Width           =   855
      End
   End
   Begin VB.CommandButton Btn_ResetPos 
      Caption         =   "位置清零"
      Height          =   495
      Left            =   7440
      TabIndex        =   10
      Top             =   3960
      Width           =   1215
   End
   Begin VB.Timer Timer1 
      Interval        =   100
      Left            =   10680
      Top             =   3960
   End
   Begin VB.CommandButton Btn_Close 
      Caption         =   "退出"
      Height          =   495
      Left            =   9120
      TabIndex        =   9
      Top             =   3960
      Width           =   1215
   End
   Begin VB.CommandButton Btn_EmgStop 
      Caption         =   "急停停止"
      Height          =   495
      Left            =   5760
      TabIndex        =   8
      Top             =   3960
      Width           =   1215
   End
   Begin VB.CommandButton Btn_StopMove 
      Caption         =   "减速停止"
      Height          =   495
      Left            =   4080
      TabIndex        =   7
      Top             =   3960
      Width           =   1215
   End
   Begin VB.CommandButton Btn_Move 
      Caption         =   "总线轴回零"
      Height          =   495
      Left            =   480
      TabIndex        =   6
      Top             =   3960
      Width           =   1215
   End
   Begin VB.Frame Frame1 
      Caption         =   "运动参数"
      Height          =   3735
      Left            =   240
      TabIndex        =   0
      Top             =   120
      Width           =   3255
      Begin VB.TextBox Tacc 
         Height          =   420
         Left            =   1200
         TabIndex        =   44
         Text            =   "0.01"
         Top             =   2160
         Width           =   1095
      End
      Begin VB.TextBox Mode 
         Height          =   375
         Left            =   1200
         TabIndex        =   39
         Text            =   "1"
         Top             =   3120
         Width           =   1095
      End
      Begin VB.TextBox Tdcc 
         Height          =   375
         Left            =   1200
         TabIndex        =   35
         Text            =   "0.01"
         Top             =   2640
         Width           =   1095
      End
      Begin VB.TextBox Axis 
         Height          =   375
         Left            =   1200
         TabIndex        =   27
         Text            =   "0"
         Top             =   240
         Width           =   1095
      End
      Begin VB.TextBox OffsetPos 
         Height          =   420
         Left            =   1200
         TabIndex        =   11
         Text            =   "100"
         Top             =   1680
         Width           =   1095
      End
      Begin VB.TextBox HightVel 
         Height          =   375
         Left            =   1200
         TabIndex        =   4
         Text            =   "1000"
         Top             =   1200
         Width           =   1095
      End
      Begin VB.TextBox LowVel 
         Height          =   375
         Left            =   1200
         TabIndex        =   2
         Text            =   "500"
         Top             =   720
         Width           =   1095
      End
      Begin VB.Label Label11 
         Caption         =   "s"
         Height          =   255
         Left            =   2520
         TabIndex        =   38
         Top             =   2760
         Width           =   495
      End
      Begin VB.Label Label10 
         Caption         =   "s"
         Height          =   255
         Left            =   2520
         TabIndex        =   37
         Top             =   2280
         Width           =   495
      End
      Begin VB.Label Label5 
         Caption         =   "减速时间"
         Height          =   255
         Index           =   2
         Left            =   240
         TabIndex        =   36
         Top             =   2760
         Width           =   975
      End
      Begin VB.Label Label9 
         Caption         =   "unit"
         Height          =   255
         Left            =   2400
         TabIndex        =   34
         Top             =   1800
         Width           =   735
      End
      Begin VB.Label Label7 
         Caption         =   "unit/s"
         Height          =   255
         Left            =   2400
         TabIndex        =   33
         Top             =   1320
         Width           =   735
      End
      Begin VB.Label Label2 
         Caption         =   "unit/s"
         Height          =   255
         Left            =   2400
         TabIndex        =   32
         Top             =   840
         Width           =   735
      End
      Begin VB.Label Label1 
         Alignment       =   2  'Center
         Caption         =   "电机轴号"
         Height          =   375
         Left            =   120
         TabIndex        =   26
         Top             =   360
         Width           =   975
      End
      Begin VB.Label Label5 
         Caption         =   "回零模式"
         Height          =   255
         Index           =   1
         Left            =   240
         TabIndex        =   13
         Top             =   3240
         Width           =   975
      End
      Begin VB.Label Label6 
         Caption         =   "回零偏移"
         Height          =   255
         Left            =   240
         TabIndex        =   12
         Top             =   1800
         Width           =   735
      End
      Begin VB.Label Label5 
         Caption         =   "加速时间"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   5
         Top             =   2280
         Width           =   975
      End
      Begin VB.Label Label4 
         Caption         =   "回零高速"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   3
         Top             =   1320
         Width           =   855
      End
      Begin VB.Label Label3 
         Caption         =   "回零低速"
         Height          =   255
         Left            =   240
         TabIndex        =   1
         Top             =   840
         Width           =   855
      End
   End
End
Attribute VB_Name = "FormMain"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Option Explicit
Public g_nCardId As Integer         '当前使用的卡号
Public g_nAxis As Integer         '当前使用的轴号
Private Declare Function timeGetTime Lib "winmm.dll" () As Long '该声明得到系统开机到现在的时间(单位：毫秒)

Public Function WaitForMS(T As Long)
    Dim Savetime As Long
    Savetime = timeGetTime '记下开始时的时间
    While timeGetTime < Savetime + T '循环等待
        DoEvents '转让控制权
    Wend
End Function

Private Sub Btn_Close_Click()
 dmc_board_close
     g_nAxis = Val(Axis)
    nmc_set_axis_disable g_nCardId, g_nAxis
    Unload Me
End Sub

Private Sub Btn_EmgStop_Click()
dmc_emg_stop g_nCardId
End Sub

Private Sub Btn_HandReset_Click()
display.Caption = "请勿操作，总线卡硬件复位进行中……"
    dmc_board_reset
    dmc_board_close
    Dim lSaveTime As Long
    WaitForMS (15000)
    dmc_board_init
    display.Caption = "总线卡硬件复位完成,请确认总线状态"

End Sub

Private Sub Btn_Move_Click()
 Dim dStartVel As Double
    Dim dMaxVel As Double
    Dim doffset As Double
    Dim dTAcc As Double
    Dim dTdcc As Double
    Dim HomeMode As Integer
    
    dStartVel = Val(LowVel)  '提取输入信息
    dMaxVel = Val(HightVel)
    doffset = Val(OffsetPos)
    dTAcc = Val(Tacc)
    dTdcc = Val(Tdcc)
    g_nAxis = Val(Axis)
    
    HomeMode = Val(Mode)
    nmc_set_home_profile g_nCardId, g_nAxis, HomeMode, dMaxVel, dStartVel, dTAcc, dTdcc, doffset
    nmc_home_move g_nCardId, g_nAxis                '启动回原点运动
End Sub

Private Sub Btn_ResetPos_Click()
g_nAxis = Val(Axis)
  dmc_set_position_unit g_nCardId, g_nAxis, 0
End Sub

Private Sub Btn_SoftReset_Click()
display.Caption = "请勿操作，总线卡软件复位进行中……"
    dmc_soft_reset g_nCardId
    WaitForMS (15000)
    display.Caption = "总线卡软件复位完成,请确认总线状态"
End Sub

Private Sub Btn_StopMove_Click()
 g_nAxis = Val(Axis)
    dmc_stop g_nCardId, g_nAxis, 0
End Sub


Private Sub Command1_Click()
    Dim dStartVel As Double
    Dim dMaxVel As Double
    Dim doffset As Double
    Dim dTAcc As Double
    Dim dTdcc As Double
    Dim HomeMode As Integer
    Dim usDir As Integer
    Dim usVelMode As Integer
    
    dStartVel = Val(LowVel)  '提取输入信息
    dMaxVel = Val(HightVel)
    doffset = Val(OffsetPos)
    dTAcc = Val(Tacc)
    dTdcc = Val(Tdcc)
    g_nAxis = Val(Axis)
    
    If (Option1(0) = True) Then            '读取回零模式
        HomeMode = 0
    ElseIf (Option1(1) = True) Then
       HomeMode = 1
    Else
       HomeMode = 2
    End If
    
    
    If (Option2(0) = True) Then            '读取回零方向
        usDir = 1
    Else
        usDir = 0
    End If
    
    If (Option3(0) = True) Then            '读取回零速度模式
        usVelMode = 0
    Else
        usVelMode = 1
    End If
    
    
    
    '//将正限位信号、负限位信号和原点信号分别映射到通用输入口0、1、2
    dmc_set_axis_io_map g_nCardId, g_nAxis, 0, 6, 0, 0
    dmc_set_axis_io_map g_nCardId, g_nAxis, 1, 6, 1, 0
    dmc_set_axis_io_map g_nCardId, g_nAxis, 2, 6, 2, 0
    
    '设置正、负限位信号低电平有效且遇限位立即停止
    dmc_set_el_mode g_nCardId, g_nAxis, 1, 0, 0
    
    '设置原点信号低电平有效
    dmc_set_home_pin_logic g_nCardId, g_nAxis, 0, 0
    
    '设置回零模式
    dmc_set_homemode g_nCardId, g_nAxis, usDir, usVelMode, HomeMode, 0
    '设置回零速度
    dmc_set_home_profile_unit g_nCardId, g_nAxis, dStartVel, dMaxVel, dTAcc, dTdcc
    '启动回零
    dmc_home_move g_nCardId, g_nAxis
    
    HomeMode = Val(Mode)
    nmc_set_home_profile g_nCardId, g_nAxis, HomeMode, dMaxVel, dStartVel, dTAcc, dTdcc, doffset
    nmc_home_move g_nCardId, g_nAxis                '启动回原点运动
End Sub

Private Sub Form_Load()
    Dim nCardNum As Integer     '定义卡数
    Dim arr_nCardList(7) As Integer '定义卡号数组
    Dim arr_nCardTypeList(7) As Long  '定义各卡类型
    nCardNum = dmc_board_init()                  '控制卡的初始化操作，调用后必须使用dmc_board_close关闭卡。
                                            '中间不可再次调用该初始化函数。
    
    If (nCardNum <= 0) Or (nCardNum > 8) Then            '正常的卡数在1- 8之间
        MsgBox "初始化LTDMC卡失败！", vbOKOnly, "出错"
   Else
    dmc_get_CardInfList nCardNum, arr_nCardTypeList(0), arr_nCardList(0)  '获取正在使用的卡号列表
    g_nCardId = arr_nCardList(0)
    MsgBox "初始化卡成功，当前选择的卡号为：" + Str(g_nCardId), , "卡号选择提示"
    End If
    g_nAxis = Val(Axis)
    nmc_set_axis_enable g_nCardId, g_nAxis
    g_nAxis = 0                           '默认选择X轴
    
End Sub

Private Sub Form_Unload(Cancel As Integer)  '释放系统分配给板卡的资源
    dmc_board_close
End Sub


'选择轴号
Private Sub OptionMoveAxis_Click(index As Integer)
    g_nAxis = index
End Sub



'刷新位置
Private Sub Timer1_Timer()
   Dim dPos As Double
   Dim dSpeed As Double
  
   Dim g_nAxis As Integer
   
   g_nAxis = Val(Axis)
   dmc_read_current_speed_unit g_nCardId, g_nAxis, dSpeed
   dmc_get_position_unit g_nCardId, 0, dPos         '显示位置信息
   My_NowPos(0).Text = dPos
   vel(0).Text = dSpeed
   
   Dim flag As Integer
   flag = dmc_check_done(g_nCardId, g_nAxis)
   If flag = 0 Then
       Lable_STATUS.Caption = "运行中"
       Lable_STATUS.BackColor = 65280
   Else
       Lable_STATUS.Caption = "停止中"
       Lable_STATUS.BackColor = 255
   End If
   
   Dim statemachine1 As Integer
   nmc_get_axis_state_machine g_nCardId, 0, statemachine1
   Select Case (statemachine1)
      Case 0
            Text1(0).Text = "轴处于未启动状态"
            Text1(0).BackColor = 255
       Case 1
            Text1(0).Text = "轴处于启动禁止状态"
            Text1(0).BackColor = 255
       Case 2
            Text1(0).Text = "轴处于准备启动状态"
            Text1(0).BackColor = 255
       Case 3
           Text1(0).Text = "轴处于启动状态"
            Text1(0).BackColor = 65280
       Case 4
            Text1(0).Text = "轴处于操作使能状态"
             Text1(0).BackColor = 65280
       Case 5
            Text1(0).Text = "轴处于停止状态"
            Text1(0).BackColor = 255
       Case 6
           Text1(0).Text = "轴处于错误触发状态"
           Text1(0).BackColor = 255
       Case 7
            Text1(0).Text = "轴处于错误状态"
            Text1(0).BackColor = 255
   End Select
   
   Dim errcode As Integer
    nmc_get_errcode g_nCardId, 2, errcode
    If errcode = 0 Then
      Text2.Text = "EtherCAT总线正常"
      Text2.BackColor = 65280
    Else
      Text2.Text = "EtherCAT总线出错"
      Text2.BackColor = 255
    End If
  Dim iostate As Long
  
  iostate = dmc_axis_io_status(g_nCardId, g_nAxis)
  
   If ((iostate And 2) = 2) Then  '检测正限位信号
        Text3(1).BackColor = 65280
   Else
        Text3(1).BackColor = 255
  End If
  
  If ((iostate And 4) = 4) Then  '检测正限位信号
        Text3(2).BackColor = 65280
   Else
      Text3(2).BackColor = 255
  End If
  
  If ((iostate And 16) = 16) Then  '检测正限位信号
       Text3(0).BackColor = 65280
   Else
      Text3(0).BackColor = 255
  End If
  
  
End Sub

