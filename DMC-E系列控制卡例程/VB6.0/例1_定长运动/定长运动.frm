VERSION 5.00
Begin VB.Form FORM11 
   BackColor       =   &H8000000B&
   Caption         =   "定长运动"
   ClientHeight    =   6300
   ClientLeft      =   60
   ClientTop       =   450
   ClientWidth     =   12705
   LinkTopic       =   "Form1"
   ScaleHeight     =   6300
   ScaleWidth      =   12705
   StartUpPosition =   2  '屏幕中心
   Begin VB.Frame Frame5 
      Caption         =   "复位操作及总线状态"
      Height          =   2175
      Left            =   3960
      TabIndex        =   52
      Top             =   3120
      Width           =   4215
      Begin VB.TextBox display 
         Height          =   735
         Left            =   240
         TabIndex        =   57
         Top             =   1320
         Width           =   3735
      End
      Begin VB.CommandButton Btn_SoftReset 
         Caption         =   "软件复位"
         Height          =   375
         Left            =   2280
         TabIndex        =   56
         Top             =   840
         Width           =   975
      End
      Begin VB.CommandButton Btn_HandReset 
         Caption         =   "硬件复位"
         Height          =   375
         Left            =   720
         TabIndex        =   55
         Top             =   840
         Width           =   975
      End
      Begin VB.TextBox state 
         Height          =   375
         Left            =   1440
         TabIndex        =   54
         Top             =   360
         Width           =   2175
      End
      Begin VB.Label Label11 
         Caption         =   "总线状态:"
         Height          =   255
         Left            =   600
         TabIndex        =   53
         Top             =   480
         Width           =   975
      End
   End
   Begin VB.CommandButton Btn_StartMove 
      Caption         =   "执行运动"
      Height          =   735
      Left            =   120
      TabIndex        =   37
      Top             =   5400
      Width           =   1455
   End
   Begin VB.Frame Frame4 
      Caption         =   "在线变位"
      Height          =   2175
      Left            =   8280
      TabIndex        =   33
      Top             =   3120
      Width           =   3855
      Begin VB.CommandButton Btn_ChangePos 
         Caption         =   "在线变位"
         Height          =   495
         Left            =   1320
         TabIndex        =   36
         Top             =   1080
         Width           =   1455
      End
      Begin VB.TextBox AimPos 
         Height          =   400
         Left            =   1320
         TabIndex        =   34
         Text            =   "10000"
         Top             =   480
         Width           =   1500
      End
      Begin VB.Label Label10 
         Caption         =   "unit"
         Height          =   255
         Index           =   13
         Left            =   2880
         TabIndex        =   51
         Top             =   600
         Width           =   495
      End
      Begin VB.Label Label9 
         Caption         =   "目标位置"
         Height          =   375
         Left            =   360
         TabIndex        =   35
         Top             =   600
         Width           =   975
      End
   End
   Begin VB.Frame Frame3 
      Caption         =   "在线变速"
      Height          =   2655
      Left            =   8280
      TabIndex        =   29
      Top             =   240
      Width           =   3855
      Begin VB.TextBox ChangeVel 
         Height          =   375
         Left            =   1200
         TabIndex        =   65
         Text            =   "3500"
         Top             =   600
         Width           =   1455
      End
      Begin VB.TextBox ChangeTime 
         Height          =   375
         Left            =   1200
         TabIndex        =   64
         Text            =   "0.1"
         Top             =   1200
         Width           =   1455
      End
      Begin VB.CommandButton Btn_ChangeVel 
         Caption         =   "在线变速"
         Height          =   495
         Left            =   1200
         TabIndex        =   32
         Top             =   1800
         Width           =   1455
      End
      Begin VB.Label Label10 
         Caption         =   "s"
         Height          =   255
         Index           =   12
         Left            =   2760
         TabIndex        =   50
         Top             =   1320
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   11
         Left            =   2760
         TabIndex        =   49
         Top             =   720
         Width           =   975
      End
      Begin VB.Label Label8 
         Caption         =   "变速时间"
         Height          =   255
         Index           =   1
         Left            =   360
         TabIndex        =   31
         Top             =   1320
         Width           =   735
      End
      Begin VB.Label Label8 
         Caption         =   "运行速度"
         Height          =   255
         Index           =   0
         Left            =   360
         TabIndex        =   30
         Top             =   720
         Width           =   735
      End
   End
   Begin VB.Frame Frame2 
      Caption         =   "信息显示"
      Height          =   2655
      Left            =   3960
      TabIndex        =   23
      Top             =   240
      Width           =   4215
      Begin VB.TextBox MoveState 
         Height          =   375
         Left            =   1320
         TabIndex        =   63
         Top             =   1680
         Width           =   1455
      End
      Begin VB.TextBox Encoder 
         Height          =   375
         Left            =   1320
         TabIndex        =   62
         Top             =   1200
         Width           =   1455
      End
      Begin VB.TextBox CurrentVel 
         Height          =   375
         Left            =   1320
         TabIndex        =   61
         Top             =   240
         Width           =   1455
      End
      Begin VB.TextBox CurrentPos 
         Height          =   375
         Left            =   1320
         TabIndex        =   60
         Top             =   720
         Width           =   1455
      End
      Begin VB.TextBox Machine 
         Height          =   375
         Left            =   1320
         TabIndex        =   59
         Top             =   2160
         Width           =   2175
      End
      Begin VB.Label label15 
         Caption         =   "轴状态机"
         Height          =   255
         Index           =   6
         Left            =   360
         TabIndex        =   58
         Top             =   2280
         Width           =   735
      End
      Begin VB.Label Label10 
         Caption         =   "unit"
         Height          =   255
         Index           =   10
         Left            =   2880
         TabIndex        =   48
         Top             =   1320
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "unit"
         Height          =   255
         Index           =   9
         Left            =   2880
         TabIndex        =   47
         Top             =   840
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   8
         Left            =   2880
         TabIndex        =   46
         Top             =   360
         Width           =   975
      End
      Begin VB.Label label15 
         Caption         =   "运行状态"
         Height          =   255
         Index           =   5
         Left            =   360
         TabIndex        =   28
         Top             =   1800
         Width           =   735
      End
      Begin VB.Label label15 
         Caption         =   "反馈位置"
         Height          =   255
         Index           =   4
         Left            =   360
         TabIndex        =   27
         Top             =   1320
         Width           =   735
      End
      Begin VB.Label label15 
         Caption         =   "当前位置"
         Height          =   255
         Index           =   3
         Left            =   360
         TabIndex        =   26
         Top             =   840
         Width           =   735
      End
      Begin VB.Label label15 
         Caption         =   "当前速度"
         Height          =   255
         Index           =   2
         Left            =   360
         TabIndex        =   25
         Top             =   360
         Width           =   735
      End
   End
   Begin VB.CommandButton Btn_SetEquiv 
      Caption         =   "设定脉冲当量"
      Height          =   735
      Left            =   8040
      TabIndex        =   16
      Top             =   5400
      Width           =   1575
   End
   Begin VB.CommandButton Btn_ResetPos 
      Caption         =   "指令清零"
      Height          =   735
      Left            =   5400
      TabIndex        =   7
      Top             =   5400
      Width           =   1575
   End
   Begin VB.Timer Timer1 
      Interval        =   100
      Left            =   12240
      Top             =   360
   End
   Begin VB.CommandButton Btn_Close 
      Caption         =   "退出程序"
      Height          =   735
      Left            =   10560
      TabIndex        =   6
      Top             =   5400
      Width           =   1575
   End
   Begin VB.CommandButton Btn_StopMove 
      Caption         =   "停止运动"
      Height          =   735
      Index           =   0
      Left            =   2640
      TabIndex        =   5
      Top             =   5400
      Width           =   1575
   End
   Begin VB.Frame Frame1 
      Caption         =   "输入参数"
      Height          =   5055
      Left            =   120
      TabIndex        =   0
      Top             =   240
      Width           =   3735
      Begin VB.TextBox Equiv 
         Height          =   400
         Left            =   1080
         TabIndex        =   21
         Text            =   "1"
         Top             =   960
         Width           =   1500
      End
      Begin VB.TextBox Axis 
         Height          =   400
         Left            =   1080
         TabIndex        =   20
         Text            =   "0"
         Top             =   480
         Width           =   1500
      End
      Begin VB.TextBox STime 
         Height          =   400
         Left            =   1080
         TabIndex        =   17
         Text            =   "0"
         Top             =   3360
         Width           =   1500
      End
      Begin VB.TextBox Tdcc 
         Height          =   400
         Left            =   1080
         TabIndex        =   14
         Text            =   "0.1"
         Top             =   2880
         Width           =   1500
      End
      Begin VB.TextBox StartVel 
         Height          =   400
         Left            =   1080
         TabIndex        =   12
         Text            =   "0"
         Top             =   1440
         Width           =   1500
      End
      Begin VB.TextBox StopVel 
         Height          =   400
         Left            =   1080
         TabIndex        =   10
         Text            =   "0"
         Top             =   3810
         Width           =   1500
      End
      Begin VB.TextBox Dist 
         Height          =   400
         Left            =   1080
         TabIndex        =   8
         Text            =   "50000"
         Top             =   4440
         Width           =   1500
      End
      Begin VB.TextBox Tacc 
         Height          =   400
         Left            =   1080
         TabIndex        =   4
         Text            =   "0.1"
         Top             =   2400
         Width           =   1500
      End
      Begin VB.TextBox MaxVel 
         Height          =   400
         Left            =   1080
         TabIndex        =   2
         Text            =   "10000"
         Top             =   1920
         Width           =   1500
      End
      Begin VB.Label Label10 
         Caption         =   "unit"
         Height          =   255
         Index           =   7
         Left            =   2640
         TabIndex        =   45
         Top             =   4560
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   6
         Left            =   2640
         TabIndex        =   44
         Top             =   3960
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "s"
         Height          =   255
         Index           =   5
         Left            =   2640
         TabIndex        =   43
         Top             =   3480
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "s"
         Height          =   255
         Index           =   4
         Left            =   2640
         TabIndex        =   42
         Top             =   3000
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "s"
         Height          =   255
         Index           =   3
         Left            =   2640
         TabIndex        =   41
         Top             =   2520
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   2
         Left            =   2640
         TabIndex        =   40
         Top             =   2040
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   1
         Left            =   2640
         TabIndex        =   39
         Top             =   1560
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "pulse/unit"
         Height          =   255
         Index           =   0
         Left            =   2640
         TabIndex        =   38
         Top             =   1080
         Width           =   975
      End
      Begin VB.Label Label7 
         Caption         =   "脉冲当量"
         Height          =   255
         Left            =   240
         TabIndex        =   22
         Top             =   1080
         Width           =   750
      End
      Begin VB.Label label15 
         Caption         =   "电机轴号"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   19
         Top             =   600
         Width           =   735
      End
      Begin VB.Label Lable7 
         Caption         =   "S段时间"
         Height          =   195
         Left            =   240
         TabIndex        =   18
         Top             =   3480
         Width           =   735
      End
      Begin VB.Label Label6 
         Caption         =   "减速时间"
         Height          =   255
         Left            =   240
         TabIndex        =   15
         Top             =   3000
         Width           =   735
      End
      Begin VB.Label Label3 
         Caption         =   "起始速度"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   13
         Top             =   1560
         Width           =   735
      End
      Begin VB.Label Label2 
         Caption         =   "停止速度"
         Height          =   255
         Left            =   240
         TabIndex        =   11
         Top             =   3960
         Width           =   735
      End
      Begin VB.Label Label1 
         AutoSize        =   -1  'True
         Caption         =   "运动距离"
         Height          =   195
         Left            =   240
         TabIndex        =   9
         Top             =   4560
         Width           =   720
      End
      Begin VB.Label Label5 
         Caption         =   "加速时间"
         Height          =   255
         Left            =   240
         TabIndex        =   3
         Top             =   2520
         Width           =   735
      End
      Begin VB.Label Label4 
         Caption         =   "运行速度"
         Height          =   255
         Left            =   240
         TabIndex        =   1
         Top             =   2040
         Width           =   855
      End
   End
   Begin VB.Label label15 
      Caption         =   "电机轴号"
      Height          =   255
      Index           =   1
      Left            =   5040
      TabIndex        =   24
      Top             =   720
      Width           =   735
   End
End
Attribute VB_Name = "FORM11"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Option Explicit
Public g_nCardId As Integer         '当前使用的卡号
Public g_nAxis As Integer         '当前使用的轴号


Private Declare Function timeGetTime Lib "winmm.dll" () As Long '该声明得到系统开机到现在的时间(单位：毫秒)

Public Function WaitForMS(T As Long)
    Dim lSaveTime As Long
    lSaveTime = timeGetTime '记下开始时的时间
    While timeGetTime < lSaveTime + T '循环等待
        DoEvents '转让控制权
    Wend
End Function


Private Sub Btn_ChangePos_Click()
    Dim lDist As Long
    lDist = Val(AimPos)
    g_nAxis = Val(Axis)
    dmc_reset_target_position_unit g_nCardId, g_nAxis, lDist  '在线变位到一个绝对位置
End Sub

Private Sub Btn_ChangeVel_Click()
Dim dChang_V As Double
    Dim dTAcc As Double
    dTAcc = Val(Tacc)
    
    dChang_V = Val(ChangeVel)
    g_nAxis = Val(Axis)
    dmc_change_speed_unit g_nCardId, g_nAxis, dChang_V, dTAcc    '在线变速
End Sub

Private Sub Btn_Close_Click()
dmc_board_close
g_nAxis = Val(Axis)                          '默认选择X轴
nmc_set_axis_disable g_nCardId, g_nAxis
    End
End Sub

Private Sub Btn_HandReset_Click()
display.Text = "请勿操作，总线卡硬件复位进行中……"
    dmc_board_reset
    dmc_board_close
    Dim lSaveTime As Long
    WaitForMS (15000)
    dmc_board_init
    display.Text = "总线卡硬件复位完成,请确认总线状态"
End Sub

Private Sub Btn_ResetPos_Click()
  dmc_set_position_unit g_nCardId, Val(Axis), 0    '位置清零
End Sub

Private Sub Btn_SetEquiv_Click()
 Dim lUnits As Long
    lUnits = Val(Equiv)
    g_nAxis = Val(Axis)
    dmc_set_equiv g_nCardId, g_nAxis, lUnits
End Sub

Private Sub Btn_SoftReset_Click()
display.Text = "请勿操作，总线卡软件复位进行中……"
    dmc_soft_reset g_nCardId
    WaitForMS (15000)
    display.Text = "总线卡软件复位完成,请确认总线状态"
End Sub

Private Sub Btn_StartMove_Click()
 Dim dStart_V As Double
    Dim dMax_V As Double
    Dim dStop_V As Double
    Dim dTAcc As Double
    Dim dTDec As Double
    Dim lDist As Long
    Dim dSTime As Double
    Dim dUnits As Double
    
    
    dUnits = Val(Equiv)
    dStart_V = Val(StartVel)
    dMax_V = Val(MaxVel)
    dStop_V = Val(StopVel)
    dTAcc = Val(Tacc)
    dTDec = Val(Tdcc)
    lDist = Val(Dist)
    dSTime = Val(STime)
    g_nAxis = Val(Axis)
    
    dmc_set_equiv g_nCardId, g_nAxis, dUnits
    dmc_set_s_profile g_nCardId, g_nAxis, 0, dSTime    '设置S段时间（0-0.05s)
    dmc_set_profile_unit g_nCardId, g_nAxis, dStart_V, dMax_V, dTAcc, dTDec, dStop_V
          '设置起始速度、运行速度、停止速度、加速时间、减速时间
    dmc_pmove_unit g_nCardId, g_nAxis, lDist, 0     '定长运动
End Sub

Private Sub Btn_StopMove_Click(index As Integer)
    g_nAxis = Val(Axis)
    dmc_stop g_nCardId, g_nAxis, 1    '1:立即停止,0减速停止
End Sub

Private Sub BUTTON_CHANGESPEED_Click()
    Dim dChang_V As Double
    Dim dTAcc As Double
    dTAcc = Val(Tacc)
    
    dChang_V = Val(ChangeVel)
    g_nAxis = Val(Axis)
    dmc_change_speed g_nCardId, g_nAxis, dChang_V, dTAcc    '在线变速
End Sub

'位置清零
Private Sub BUTTON_CLEAN_Click()
   g_nAxis = Val(Axis)
  dmc_set_position g_nCardId, g_nAxis, 0     '位置清零
End Sub

'关闭控制界面
Private Sub BUTTON_CLOSE_Click()
    dmc_board_close
    End
End Sub




Private Sub BUTTON_DELSTOP_Click(index As Integer)
    g_nAxis = Val(Axis)
    dmc_stop g_nCardId, g_nAxis, 1    '1:立即停止,0减速停止
End Sub

'启动运行
Private Sub BUTTON_MOVE_Click()
    
    Dim dStart_V As Double
    Dim dMax_V As Double
    Dim dStop_V As Double
    Dim dTAcc As Double
    Dim dTDec As Double
    Dim lDist As Long
    Dim dSTime As Double
    Dim dUnits As Double
    
    
    dUnits = Val(Equiv)
    dStart_V = Val(StartVel)
    dMax_V = Val(MaxVel)
    dStop_V = Val(StopVel)
    dTAcc = Val(Tacc)
    dTDec = Val(Tdcc)
    lDist = Val(Dist.Text)
    dSTime = Val(STime.Text)
    g_nAxis = Val(Axis)
    
    dmc_set_equiv g_nCardId, g_nAxis, dUnits
    dmc_set_s_profile g_nCardId, g_nAxis, 0, dSTime    '设置S段时间（0-0.05s)
    dmc_set_profile g_nCardId, g_nAxis, dStart_V, dMax_V, dTAcc, dTDec, dStop_V
          '设置起始速度、运行速度、停止速度、加速时间、减速时间
    dmc_pmove g_nCardId, g_nAxis, lDist, 0     '定长运动
End Sub



Private Sub BUTTON_RESETPOS_Click()
    Dim lDist As Long
    lDist = Val(AimPos)
    g_nAxis = Val(Axis)
    dmc_reset_target_position_unit g_nCardId, g_nAxis, lDist  '在线变位到一个绝对位置
End Sub

Private Sub Command1_Click()

End Sub

Private Sub Form_Load()
    Dim nCardNum As Integer     '定义卡数
    Dim arr_nCardList(7) As Integer '定义卡号数组
    Dim arr_nCardTypeList(7) As Long  '定义各卡类型
    
    nCardNum = dmc_board_init()             '控制卡的初始化操作，调用后必须使用dmc_board_close关闭卡。
                                            '中间不可再次调用该初始化函数。
         
    If (nCardNum <= 0) Or (nCardNum > 8) Then              '正常的卡数在1- 8之间
        MsgBox "初始化控制卡失败！", vbOKOnly, "出错"
    Else
    dmc_get_CardInfList nCardNum, arr_nCardTypeList(0), arr_nCardList(0)  '获取正在使用的卡号列表
    g_nCardId = arr_nCardList(0)
    MsgBox "初始化卡成功，当前选择的卡号为：" + Str(g_nCardId), , "卡号选择提示"
     End If
    g_nAxis = Val(Axis)                          '默认选择X轴
    
    nmc_set_axis_enable g_nCardId, g_nAxis
    dmc_set_pulse_outmode g_nCardId, g_nAxis, 0    '设定脉冲输出模式
 
    
End Sub

Private Sub Form_Unload(Cancel As Integer)  '释放系统分配给板卡的资源
    dmc_board_close
    End
End Sub

'选择轴号
Private Sub OptionMoveAxis_Click(index As Integer)
    g_nAxis = index
End Sub

Private Sub set_units_Click()
    Dim lUnits As Long
    lUnits = Val(Equiv)
    g_nAxis = Val(Axis)
    dmc_set_equiv g_nCardId, g_nAxis, lUnits
End Sub

'刷新位置
Private Sub Timer1_Timer()
   Dim dPos As Double
   Dim dSpeed As Double
   Dim lFlag As Long
   Dim dEncoder As Double
   Dim nStateMachine As Integer
   Dim nerrcode As Integer
   
   g_nAxis = Val(Axis)
   
   lFlag = dmc_check_done(g_nCardId, g_nAxis)
   If lFlag = 0 Then
        MoveState.Text = "运行中"
         MoveState.BackColor = 65280
         
   Else
         MoveState.Text = "停止中"
          MoveState.BackColor = 255
   End If
      
       nmc_get_axis_state_machine g_nCardId, g_nAxis, nStateMachine
   Select Case (nStateMachine)
       Case 0
            Machine.Text = "轴处于未启动状态"
            Machine.BackColor = 255
       Case 1
            Machine.Text = "轴处于启动禁止状态"
            Machine.BackColor = 255
       Case 2
           Machine.Text = "轴处于准备启动状态"
           Machine.BackColor = 255
       Case 3
           Machine.Text = "轴处于启动状态"
          Machine.BackColor = 65280
       Case 4
         Machine.Text = "轴处于操作使能状态"
         Machine.BackColor = 65280
       Case 5
         Machine.Text = "轴处于停止状态"
          Machine.BackColor = 255
       Case 6
          Machine.Text = "轴处于错误触发状态"
           Machine.BackColor = 255
       Case 7
           Machine.Text = "轴处于错误状态"
            Machine.BackColor = 255
           
   End Select
   
   
   nmc_get_errcode g_nCardId, 2, nerrcode
        If (nerrcode = 0) Then
            state.Text = "EtherCAT总线正常"
            state.BackColor = 65280
        Else
            state.Text = "EtherCAT总线出错"
            state.BackColor = 255
        End If
      
      
   dmc_read_current_speed_unit g_nCardId, g_nAxis, dSpeed
   dmc_get_position_unit g_nCardId, g_nAxis, dPos
    CurrentPos.Text = dPos
   CurrentVel.Text = dSpeed
   
   dmc_get_encoder_unit g_nCardId, g_nAxis, dEncoder
   Encoder.Text = dEncoder
   
   
End Sub
