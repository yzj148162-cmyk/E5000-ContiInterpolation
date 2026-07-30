VERSION 5.00
Begin VB.Form FormMain 
   Caption         =   "连续运动"
   ClientHeight    =   5025
   ClientLeft      =   60
   ClientTop       =   450
   ClientWidth     =   9435
   LinkTopic       =   "Form1"
   ScaleHeight     =   5025
   ScaleWidth      =   9435
   StartUpPosition =   2  '屏幕中心
   Begin VB.Frame Frame5 
      Caption         =   "复位操作及总线状态"
      Height          =   1455
      Left            =   3480
      TabIndex        =   57
      Top             =   2760
      Width           =   5775
      Begin VB.TextBox display 
         Height          =   975
         Left            =   2880
         TabIndex        =   62
         Top             =   360
         Width           =   2655
      End
      Begin VB.CommandButton Btn_SoftReset 
         Caption         =   "软件复位"
         Height          =   495
         Left            =   1440
         TabIndex        =   61
         Top             =   840
         Width           =   1215
      End
      Begin VB.CommandButton Btn_HandReset 
         Caption         =   "硬件复位"
         Height          =   495
         Left            =   120
         TabIndex        =   60
         Top             =   840
         Width           =   1215
      End
      Begin VB.TextBox EcatState 
         Height          =   390
         Left            =   1080
         TabIndex        =   59
         Top             =   360
         Width           =   1695
      End
      Begin VB.Label Label14 
         Caption         =   "总线状态:"
         Height          =   255
         Left            =   240
         TabIndex        =   58
         Top             =   480
         Width           =   975
      End
   End
   Begin VB.Frame Frame2 
      Caption         =   "信息显示"
      Height          =   2535
      Left            =   6360
      TabIndex        =   30
      Top             =   120
      Width           =   2895
      Begin VB.TextBox Machine 
         Height          =   270
         Left            =   1080
         TabIndex        =   56
         Top             =   1800
         Width           =   1575
      End
      Begin VB.TextBox CurrentEncorrde 
         Height          =   270
         Left            =   1080
         TabIndex        =   53
         Top             =   1080
         Width           =   975
      End
      Begin VB.TextBox Text_MoveState 
         Height          =   270
         Left            =   1080
         TabIndex        =   37
         Top             =   1440
         Width           =   975
      End
      Begin VB.TextBox Text_CurrentPos 
         Height          =   270
         Left            =   1080
         TabIndex        =   34
         Top             =   720
         Width           =   975
      End
      Begin VB.TextBox Text1_CurrentVel 
         Height          =   270
         Left            =   1080
         TabIndex        =   32
         Top             =   360
         Width           =   975
      End
      Begin VB.Label Label13 
         Caption         =   "轴状态机"
         Height          =   255
         Left            =   240
         TabIndex        =   55
         Top             =   1800
         Width           =   735
      End
      Begin VB.Label Label12 
         Caption         =   "uint"
         Height          =   255
         Index           =   11
         Left            =   2160
         TabIndex        =   51
         Top             =   1080
         Width           =   495
      End
      Begin VB.Label Label12 
         Caption         =   "unit"
         Height          =   255
         Index           =   10
         Left            =   2160
         TabIndex        =   50
         Top             =   720
         Width           =   615
      End
      Begin VB.Label Label12 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   9
         Left            =   2160
         TabIndex        =   49
         Top             =   360
         Width           =   615
      End
      Begin VB.Label Label11 
         Caption         =   "运动状态"
         Height          =   255
         Left            =   240
         TabIndex        =   36
         Top             =   1440
         Width           =   735
      End
      Begin VB.Label Label10 
         Caption         =   "反馈位置"
         Height          =   255
         Left            =   240
         TabIndex        =   35
         Top             =   1080
         Width           =   735
      End
      Begin VB.Label Label9 
         Caption         =   "当前位置"
         Height          =   255
         Left            =   240
         TabIndex        =   33
         Top             =   720
         Width           =   735
      End
      Begin VB.Label Label7 
         Caption         =   "当前速度"
         Height          =   255
         Left            =   240
         TabIndex        =   31
         Top             =   360
         Width           =   855
      End
   End
   Begin VB.Frame Frame4 
      Caption         =   "在线变速"
      Height          =   1695
      Left            =   3480
      TabIndex        =   22
      Top             =   120
      Width           =   2775
      Begin VB.TextBox Text_ChangeTime 
         Height          =   270
         Left            =   960
         TabIndex        =   38
         Text            =   "0.1"
         Top             =   840
         Width           =   855
      End
      Begin VB.TextBox Text_ChangeVel 
         Height          =   270
         Left            =   960
         TabIndex        =   24
         Text            =   "5000"
         Top             =   360
         Width           =   855
      End
      Begin VB.CommandButton CmBtn_ChangeVel 
         Caption         =   "在线变速"
         Height          =   375
         Left            =   840
         TabIndex        =   23
         Top             =   1200
         Width           =   975
      End
      Begin VB.Label Label12 
         Caption         =   "s"
         Height          =   255
         Index           =   8
         Left            =   1920
         TabIndex        =   48
         Top             =   840
         Width           =   735
      End
      Begin VB.Label Label12 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   7
         Left            =   1920
         TabIndex        =   47
         Top             =   360
         Width           =   735
      End
      Begin VB.Label Label8 
         Caption         =   "变速时间"
         Height          =   255
         Left            =   120
         TabIndex        =   39
         Top             =   840
         Width           =   735
      End
      Begin VB.Label Label1 
         Caption         =   "运行速度"
         Height          =   255
         Left            =   120
         TabIndex        =   29
         Top             =   360
         Width           =   735
      End
   End
   Begin VB.TextBox Text_STime 
      Height          =   270
      Index           =   1
      Left            =   1200
      TabIndex        =   19
      Text            =   "0"
      Top             =   2640
      Width           =   1095
   End
   Begin VB.Frame Frame3 
      Caption         =   "运动方向"
      Height          =   735
      Left            =   3480
      TabIndex        =   13
      Top             =   1920
      Width           =   2775
      Begin VB.OptionButton OptionMoveDir 
         Caption         =   "反向"
         Height          =   375
         Index           =   0
         Left            =   1680
         TabIndex        =   15
         Top             =   240
         Width           =   735
      End
      Begin VB.OptionButton OptionMoveDir 
         Caption         =   "正向"
         Height          =   375
         Index           =   1
         Left            =   600
         TabIndex        =   14
         Top             =   240
         Value           =   -1  'True
         Width           =   735
      End
   End
   Begin VB.CommandButton Btn_StopMove 
      Caption         =   "停止运动"
      Height          =   495
      Left            =   2040
      TabIndex        =   11
      Top             =   4440
      Width           =   1215
   End
   Begin VB.CommandButton Btn_ResetPos 
      Caption         =   "指令清零"
      Height          =   495
      Left            =   5640
      TabIndex        =   8
      Top             =   4440
      Width           =   1215
   End
   Begin VB.Timer Timer1 
      Interval        =   100
      Left            =   8880
      Top             =   4440
   End
   Begin VB.CommandButton Btn_Close 
      Caption         =   "退出程序"
      Height          =   495
      Left            =   7440
      TabIndex        =   7
      Top             =   4440
      Width           =   1215
   End
   Begin VB.CommandButton Btn_SetEquiv 
      Caption         =   "设置脉冲当量"
      Height          =   495
      Left            =   3840
      TabIndex        =   6
      Top             =   4440
      Width           =   1215
   End
   Begin VB.CommandButton Btn_StartMove 
      Caption         =   "执行运动"
      Height          =   495
      Left            =   240
      TabIndex        =   5
      Top             =   4440
      Width           =   1215
   End
   Begin VB.Frame Frame1 
      Caption         =   "运动参数"
      Height          =   4095
      Left            =   120
      TabIndex        =   0
      Top             =   120
      Width           =   3255
      Begin VB.TextBox Equiv 
         Height          =   270
         Left            =   1080
         TabIndex        =   54
         Text            =   "1"
         Top             =   720
         Width           =   1095
      End
      Begin VB.TextBox Text_SetAxis 
         Height          =   270
         Left            =   1080
         TabIndex        =   28
         Text            =   "0"
         Top             =   360
         Width           =   1095
      End
      Begin VB.TextBox Text_Tdec 
         Height          =   270
         Left            =   1080
         TabIndex        =   27
         Text            =   "0.1"
         Top             =   2160
         Width           =   1095
      End
      Begin VB.TextBox Text_StopVel 
         Height          =   270
         Left            =   1080
         TabIndex        =   26
         Text            =   "0"
         Top             =   2880
         Width           =   1095
      End
      Begin VB.TextBox Text_StartVel 
         Height          =   270
         Left            =   1080
         TabIndex        =   25
         Text            =   "0"
         Top             =   1080
         Width           =   1095
      End
      Begin VB.TextBox Text_SetPos 
         Height          =   285
         Index           =   1
         Left            =   1080
         TabIndex        =   21
         Text            =   "50000"
         Top             =   3240
         Width           =   1095
      End
      Begin VB.TextBox Text_Tacc 
         Height          =   270
         Left            =   1080
         TabIndex        =   4
         Text            =   "0.1"
         Top             =   1800
         Width           =   1095
      End
      Begin VB.TextBox Text_MaxVel 
         Height          =   270
         Left            =   1080
         TabIndex        =   2
         Text            =   "10000"
         Top             =   1440
         Width           =   1095
      End
      Begin VB.Label Label12 
         Caption         =   "pulse/unit"
         Height          =   255
         Index           =   12
         Left            =   2280
         TabIndex        =   52
         Top             =   720
         Width           =   855
      End
      Begin VB.Label Label12 
         Caption         =   "unit"
         Height          =   255
         Index           =   6
         Left            =   2280
         TabIndex        =   46
         Top             =   3240
         Width           =   495
      End
      Begin VB.Label Label12 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   5
         Left            =   2280
         TabIndex        =   45
         Top             =   2880
         Width           =   855
      End
      Begin VB.Label Label12 
         Caption         =   "s"
         Height          =   255
         Index           =   4
         Left            =   2280
         TabIndex        =   44
         Top             =   2520
         Width           =   495
      End
      Begin VB.Label Label12 
         Caption         =   "s"
         Height          =   255
         Index           =   3
         Left            =   2280
         TabIndex        =   43
         Top             =   2160
         Width           =   495
      End
      Begin VB.Label Label12 
         Caption         =   "s"
         Height          =   255
         Index           =   2
         Left            =   2280
         TabIndex        =   42
         Top             =   1800
         Width           =   495
      End
      Begin VB.Label Label12 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   1
         Left            =   2280
         TabIndex        =   41
         Top             =   1440
         Width           =   855
      End
      Begin VB.Label Label12 
         Caption         =   "unit/s"
         Height          =   255
         Index           =   0
         Left            =   2280
         TabIndex        =   40
         Top             =   1080
         Width           =   855
      End
      Begin VB.Label Label2 
         Caption         =   "运行距离"
         Height          =   255
         Index           =   1
         Left            =   240
         TabIndex        =   20
         Top             =   3240
         Width           =   735
      End
      Begin VB.Label Label6 
         Caption         =   "S段时间"
         Height          =   255
         Index           =   1
         Left            =   240
         TabIndex        =   18
         Top             =   2520
         Width           =   735
      End
      Begin VB.Label Label3 
         Caption         =   "脉冲当量"
         Height          =   255
         Index           =   2
         Left            =   240
         TabIndex        =   17
         Top             =   720
         Width           =   735
      End
      Begin VB.Label Label3 
         Caption         =   "电机轴号"
         Height          =   255
         Index           =   1
         Left            =   240
         TabIndex        =   16
         Top             =   360
         Width           =   735
      End
      Begin VB.Label Label6 
         Caption         =   "减速时间"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   12
         Top             =   2160
         Width           =   735
      End
      Begin VB.Label Label3 
         Caption         =   "起始速度"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   10
         Top             =   1080
         Width           =   735
      End
      Begin VB.Label Label2 
         Caption         =   "停止速度"
         Height          =   255
         Index           =   0
         Left            =   240
         TabIndex        =   9
         Top             =   2880
         Width           =   735
      End
      Begin VB.Label Label5 
         Caption         =   "加速时间"
         Height          =   255
         Left            =   240
         TabIndex        =   3
         Top             =   1800
         Width           =   735
      End
      Begin VB.Label Label4 
         Caption         =   "运行速度"
         Height          =   255
         Left            =   240
         TabIndex        =   1
         Top             =   1440
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
Public lPos As Double
Public g_nMoveDir As Integer         '运动方向
Private Declare Function timeGetTime Lib "winmm.dll" () As Long '该声明得到系统开机到现在的时间(单位：毫秒)

Public Function WaitForMS(T As Long)
    Dim lSaveTime As Long
    lSaveTime = timeGetTime '记下开始时的时间
    While timeGetTime < lSaveTime + T '循环等待
        DoEvents '转让控制权
    Wend
End Function

Private Sub Btn_Close_Click()
    nmc_set_axis_disable g_nCardId, g_nAxis
    dmc_board_close
    Unload Me
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
 dmc_set_position_unit g_nCardId, g_nAxis, 0
End Sub

Private Sub Btn_SetEquiv_Click()
 Dim dEquiv As Double
    dEquiv = Val(Equiv.Text)
    dmc_set_equiv g_nCardId, g_nAxis, dEquiv
End Sub

Private Sub Btn_SoftReset_Click()
display.Text = "请勿操作，总线卡软件复位进行中……"
    dmc_soft_reset g_nCardId
    WaitForMS (15000)
    display.Text = "总线卡软件复位完成,请确认总线状态"
End Sub

Private Sub Btn_StartMove_Click()
 Dim dStartVel As Double
    Dim dMaxVel As Double
    Dim dStopVel As Double
    Dim dTacc As Double
    Dim dTdcc As Double
    Dim lDist As Long
    
    g_nAxis = Val(Text_SetAxis.Text)
    dStartVel = Val(Text_StartVel.Text)
    dMaxVel = Val(Text_MaxVel.Text)
    dStopVel = Val(Text_StopVel.Text)
    dTacc = Val(Text_Tacc.Text)
    dTdcc = Val(Text_Tdec.Text)
    
    If (OptionMoveDir(0) = True) Then            '读取运动方向
        g_nMoveDir = 0
    Else
        g_nMoveDir = 1
    End If
    
    dmc_set_s_profile g_nCardId, g_nAxis, 0, 0.01  '设置平滑时间（0-0.05s)
     dmc_set_profile_unit g_nCardId, g_nAxis, dStartVel, dMaxVel, dTacc, dTdcc, dStopVel
    '设置起始速度、运行速度、停止速度、加速时间、减速时间
    dmc_vmove g_nCardId, g_nAxis, g_nMoveDir    '连续运动
End Sub

Private Sub Btn_StopMove_Click()
dmc_stop g_nCardId, g_nAxis, 1
End Sub

'在线变速

Private Sub CmBtn_ChangeVel_Click()
Dim dChangVel As Double
Dim dChangTime As Double
dChangVel = Val(Text_ChangeVel.Text)
dChangTime = Val(Text_ChangeTime.Text)
dmc_change_speed_unit g_nCardId, g_nAxis, dChangVel, dChangTime
End Sub

'初始化卡

Private Sub Form_Load()
    Dim nNum As Integer
    Dim arr_nCardList(7) As Integer
    Dim arr_nCardTypeList(7) As Long
    
    nNum = dmc_board_init()             '控制卡的初始化操作，调用后必须使用dmc_board_close关闭卡。
                                            '中间不可再次调用该初始化函数。
         
    If (nNum <= 0) Or (nNum > 8) Then              '正常的卡数在1- 8之间
        MsgBox "初始化LTDMC卡失败！", vbOKOnly, "出错"
    Else
    dmc_get_CardInfList nNum, arr_nCardTypeList(0), arr_nCardList(0)  '获取正在使用的卡号列表
    g_nCardId = arr_nCardList(0)
        MsgBox "初始化卡成功，当前选择的卡号为：" + Str(g_nCardId), , "卡号选择提示"
    End If
    g_nAxis = 0
    nmc_set_axis_enable g_nCardId, g_nAxis
    OptionMoveDir(1).Value = True
    dmc_set_pulse_outmode g_nCardId, g_nAxis, 0    '设定脉冲输出模式
 
    
End Sub

'释放系统分配给板卡的资源

Private Sub Form_Unload(Cancel As Integer)
    dmc_board_close
End Sub

'选择轴号

Private Sub OptionMoveAxis_Click(index As Integer)
    g_nAxis = index
End Sub


Private Sub OptionMoveDir_Click(index As Integer)
    g_nMoveDir = index
End Sub

'刷新位置

Private Sub Timer1_Timer()
   
   Dim dEncoder As Double
   
   Dim dCurrentVel As Double
   Dim str_DisplayPos As String
   Dim bCardStatus As Boolean
   Dim nStateMachine As Integer
   Dim nerrcode As Integer
   
        
   dmc_read_current_speed_unit g_nCardId, g_nAxis, dCurrentVel        '显示当前速度
   Text1_CurrentVel = Str(dCurrentVel)
      
   dmc_get_encoder_unit g_nCardId, g_nAxis, dEncoder    '显示当前编码器位置
   
   CurrentEncorrde.Text = dEncoder
   
   dmc_get_position_unit g_nCardId, g_nAxis, lPos    '显示当前指令位置
   Text_CurrentPos.Text = lPos
   
   nmc_get_errcode g_nCardId, 2, nerrcode
        If (nerrcode = 0) Then
            EcatState.Text = "EtherCAT总线正常"
            EcatState.BackColor = 65280
        Else
            EcatState.Text = "EtherCAT总线出错"
            EcatState.BackColor = 255
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
   
   bCardStatus = dmc_check_done(g_nCardId, 0)
   If bCardStatus Then                   '显示运行情况
     Text_MoveState.Text = "停止"
   Else
     Text_MoveState.Text = "运行"
   End If

End Sub

