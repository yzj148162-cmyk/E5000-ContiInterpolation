VERSION 5.00
Begin VB.Form 主站从站操作 
   Caption         =   "总线操作"
   ClientHeight    =   8070
   ClientLeft      =   120
   ClientTop       =   450
   ClientWidth     =   11625
   LinkTopic       =   "Form1"
   ScaleHeight     =   8070
   ScaleWidth      =   11625
   StartUpPosition =   3  '窗口缺省
   Begin VB.Timer Timer1 
      Interval        =   100
      Left            =   3600
      Top             =   4320
   End
   Begin VB.Frame Frame4 
      Caption         =   "轴信息"
      Height          =   6495
      Left            =   7800
      TabIndex        =   36
      Top             =   360
      Width           =   3615
      Begin VB.CommandButton Command8 
         Caption         =   "轴失能"
         Height          =   375
         Index           =   0
         Left            =   1800
         TabIndex        =   54
         Top             =   5880
         Width           =   1215
      End
      Begin VB.TextBox axis_flag 
         Height          =   375
         Index           =   9
         Left            =   1920
         TabIndex        =   52
         Text            =   " "
         Top             =   5160
         Width           =   1575
      End
      Begin VB.TextBox axis_flag 
         Height          =   375
         Index           =   8
         Left            =   1920
         TabIndex        =   51
         Text            =   " "
         Top             =   4440
         Width           =   1335
      End
      Begin VB.TextBox axis_flag 
         Height          =   375
         Index           =   7
         Left            =   1920
         TabIndex        =   50
         Text            =   " "
         Top             =   3720
         Width           =   1335
      End
      Begin VB.TextBox axis_flag 
         Height          =   375
         Index           =   6
         Left            =   1920
         TabIndex        =   49
         Text            =   " "
         Top             =   3000
         Width           =   1335
      End
      Begin VB.TextBox axis_flag 
         Height          =   375
         Index           =   5
         Left            =   1920
         TabIndex        =   48
         Text            =   " "
         Top             =   2280
         Width           =   1335
      End
      Begin VB.TextBox axis_flag 
         Height          =   375
         Index           =   4
         Left            =   1920
         TabIndex        =   47
         Text            =   " "
         Top             =   1560
         Width           =   1335
      End
      Begin VB.TextBox axis_flag 
         Height          =   375
         Index           =   1
         Left            =   1920
         TabIndex        =   46
         Text            =   " "
         Top             =   840
         Width           =   1335
      End
      Begin VB.TextBox axis_flag 
         Height          =   375
         Index           =   0
         Left            =   1920
         TabIndex        =   45
         Text            =   "0"
         Top             =   360
         Width           =   1095
      End
      Begin VB.CommandButton Command7 
         Caption         =   "轴使能"
         Height          =   375
         Index           =   7
         Left            =   240
         TabIndex        =   44
         Top             =   5880
         Width           =   1215
      End
      Begin VB.CommandButton Command6 
         Caption         =   "轴状态机"
         Height          =   375
         Index           =   6
         Left            =   240
         TabIndex        =   43
         Top             =   5160
         Width           =   1215
      End
      Begin VB.CommandButton Command5 
         Caption         =   "轴控制字"
         Height          =   375
         Index           =   5
         Left            =   240
         TabIndex        =   42
         Top             =   4440
         Width           =   1215
      End
      Begin VB.CommandButton Command4 
         Caption         =   "轴状态字"
         Height          =   375
         Index           =   4
         Left            =   240
         TabIndex        =   41
         Top             =   3720
         Width           =   1215
      End
      Begin VB.CommandButton Command3 
         Caption         =   "轴错误码"
         Height          =   375
         Index           =   3
         Left            =   240
         TabIndex        =   40
         Top             =   3000
         Width           =   1215
      End
      Begin VB.CommandButton Command2 
         Caption         =   "轴类型"
         Height          =   375
         Index           =   2
         Left            =   240
         TabIndex        =   39
         Top             =   2280
         Width           =   1215
      End
      Begin VB.CommandButton Command1 
         Caption         =   "轴地址ID"
         Height          =   375
         Index           =   1
         Left            =   240
         TabIndex        =   38
         Top             =   1560
         Width           =   1215
      End
      Begin VB.CommandButton Command1 
         Caption         =   "轴地址"
         Height          =   375
         Index           =   0
         Left            =   240
         TabIndex        =   37
         Top             =   840
         Width           =   1215
      End
      Begin VB.Label Label10 
         Alignment       =   2  'Center
         Caption         =   "轴号"
         Height          =   375
         Left            =   360
         TabIndex        =   53
         Top             =   360
         Width           =   1095
      End
   End
   Begin VB.Frame Frame3 
      Caption         =   "总线状态"
      Height          =   3975
      Left            =   3960
      TabIndex        =   14
      Top             =   360
      Width           =   3615
      Begin VB.TextBox display 
         Height          =   1335
         Left            =   360
         TabIndex        =   19
         Text            =   " "
         Top             =   2040
         Width           =   2775
      End
      Begin VB.CommandButton Btn_ResetSoft 
         Caption         =   "软件复位"
         Height          =   495
         Left            =   1800
         TabIndex        =   18
         Top             =   1200
         Width           =   1215
      End
      Begin VB.CommandButton Btn_HandReset 
         Caption         =   "硬件复位"
         Height          =   495
         Left            =   360
         TabIndex        =   17
         Top             =   1200
         Width           =   1215
      End
      Begin VB.TextBox bus_state 
         Height          =   375
         Left            =   1320
         TabIndex        =   16
         Text            =   " "
         Top             =   360
         Width           =   1695
      End
      Begin VB.Label Label5 
         Alignment       =   2  'Center
         Caption         =   "总线状态"
         Height          =   375
         Left            =   120
         TabIndex        =   15
         Top             =   480
         Width           =   1215
      End
   End
   Begin VB.Frame Frame2 
      Caption         =   "从站信息"
      Height          =   3135
      Left            =   480
      TabIndex        =   13
      Top             =   4680
      Width           =   7095
      Begin VB.TextBox ECAT_AOUTNUM 
         Height          =   375
         Left            =   5640
         TabIndex        =   35
         Text            =   " "
         Top             =   1920
         Width           =   1095
      End
      Begin VB.TextBox ECAT_AINNUM 
         Height          =   375
         Left            =   5640
         TabIndex        =   34
         Text            =   " "
         Top             =   1440
         Width           =   1095
      End
      Begin VB.TextBox ECAT_OUTNUM 
         Height          =   375
         Left            =   5640
         TabIndex        =   33
         Text            =   " "
         Top             =   960
         Width           =   1095
      End
      Begin VB.TextBox ECAT_INNUM 
         Height          =   375
         Left            =   5640
         TabIndex        =   32
         Text            =   " "
         Top             =   480
         Width           =   1095
      End
      Begin VB.TextBox out_num 
         Height          =   375
         Left            =   2280
         TabIndex        =   27
         Text            =   " "
         Top             =   1920
         Width           =   1095
      End
      Begin VB.TextBox in_num 
         Height          =   375
         Left            =   2280
         TabIndex        =   26
         Text            =   " "
         Top             =   1440
         Width           =   1095
      End
      Begin VB.TextBox AxisNum 
         Height          =   375
         Left            =   2280
         TabIndex        =   25
         Text            =   " "
         Top             =   960
         Width           =   1095
      End
      Begin VB.TextBox slave_num 
         Height          =   375
         Left            =   2280
         TabIndex        =   24
         Top             =   480
         Width           =   1095
      End
      Begin VB.Label Label9 
         Caption         =   "EtherCAT AD输出数："
         Height          =   375
         Index           =   2
         Left            =   3720
         TabIndex        =   31
         Top             =   1920
         Width           =   1935
      End
      Begin VB.Label Label8 
         Caption         =   "EtherCAT AD输入数："
         Height          =   375
         Index           =   2
         Left            =   3720
         TabIndex        =   30
         Top             =   1440
         Width           =   1815
      End
      Begin VB.Label Label9 
         Caption         =   "EtherCAT IO输出数："
         Height          =   375
         Index           =   1
         Left            =   3720
         TabIndex        =   29
         Top             =   960
         Width           =   1935
      End
      Begin VB.Label Label8 
         Caption         =   "EtherCAT IO输入数："
         Height          =   375
         Index           =   1
         Left            =   3720
         TabIndex        =   28
         Top             =   480
         Width           =   1815
      End
      Begin VB.Label Label9 
         Caption         =   "总线卡本地IO输出数："
         Height          =   375
         Index           =   0
         Left            =   120
         TabIndex        =   23
         Top             =   1920
         Width           =   1935
      End
      Begin VB.Label Label8 
         Caption         =   "总线卡本地IO输入数："
         Height          =   375
         Index           =   0
         Left            =   120
         TabIndex        =   22
         Top             =   1440
         Width           =   1815
      End
      Begin VB.Label Label7 
         Caption         =   "EtherCAT轴数："
         Height          =   375
         Left            =   120
         TabIndex        =   21
         Top             =   960
         Width           =   1455
      End
      Begin VB.Label Label6 
         Caption         =   "从站总数："
         Height          =   375
         Left            =   120
         TabIndex        =   20
         Top             =   480
         Width           =   1335
      End
   End
   Begin VB.Frame Frame1 
      Caption         =   "对象字典操作"
      Height          =   3975
      Left            =   480
      TabIndex        =   0
      Top             =   360
      Width           =   3255
      Begin VB.CommandButton Btn_WriteData 
         Caption         =   "写入参数"
         Height          =   495
         Left            =   1560
         TabIndex        =   12
         Top             =   3000
         Width           =   1095
      End
      Begin VB.CommandButton Btn_GetDate 
         Caption         =   "读取参数"
         Height          =   495
         Left            =   240
         TabIndex        =   11
         Top             =   3000
         Width           =   1095
      End
      Begin VB.TextBox data 
         Height          =   375
         Left            =   1560
         TabIndex        =   10
         Text            =   " "
         Top             =   2280
         Width           =   1095
      End
      Begin VB.ComboBox length 
         Height          =   300
         ItemData        =   "总线操作.frx":0000
         Left            =   1560
         List            =   "总线操作.frx":0012
         TabIndex        =   9
         Text            =   "1"
         Top             =   1800
         Width           =   1095
      End
      Begin VB.TextBox s_index 
         Height          =   375
         Left            =   1560
         TabIndex        =   8
         Text            =   "0"
         Top             =   1200
         Width           =   1095
      End
      Begin VB.TextBox m_index 
         Height          =   375
         Left            =   1560
         TabIndex        =   7
         Text            =   "6098"
         Top             =   720
         Width           =   1095
      End
      Begin VB.TextBox master_id 
         Height          =   375
         Left            =   1560
         TabIndex        =   6
         Text            =   "1001"
         Top             =   300
         Width           =   1095
      End
      Begin VB.Label Label4 
         Alignment       =   2  'Center
         Caption         =   "值："
         Height          =   375
         Left            =   240
         TabIndex        =   5
         Top             =   2400
         Width           =   735
      End
      Begin VB.Label Label3 
         Alignment       =   2  'Center
         Caption         =   "长度："
         Height          =   375
         Left            =   240
         TabIndex        =   4
         Top             =   1800
         Width           =   855
      End
      Begin VB.Label Label2 
         Alignment       =   2  'Center
         Caption         =   "子索引：16#"
         Height          =   375
         Index           =   1
         Left            =   120
         TabIndex        =   3
         Top             =   1320
         Width           =   1215
      End
      Begin VB.Label Label2 
         Alignment       =   2  'Center
         Caption         =   "主索引：16#"
         Height          =   375
         Index           =   0
         Left            =   120
         TabIndex        =   2
         Top             =   840
         Width           =   1215
      End
      Begin VB.Label Label1 
         Alignment       =   2  'Center
         Caption         =   "从站ID:"
         Height          =   255
         Left            =   360
         TabIndex        =   1
         Top             =   360
         Width           =   855
      End
   End
End
Attribute VB_Name = "主站从站操作"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Option Explicit
Public g_nCardId As Integer         '当前使用的卡号
Public g_nAxisNum As Integer         '当前使用的轴号

Private Declare Function timeGetTime Lib "winmm.dll" () As Long '该声明得到系统开机到现在的时间(单位：毫秒)

Public Function WaitForMS(T As Long)
    Dim lSaveTime As Long
    lSaveTime = timeGetTime '记下开始时的时间
    While timeGetTime < lSaveTime + T '循环等待
        DoEvents '转让控制权
    Wend
End Function

Private Sub Btn_GetDate_Click()
Dim nRtn As Integer
    Dim lValue As Long
    Dim strStr1 As String
    Dim strStr2 As String
    strStr1 = m_index.Text
    strStr2 = CLng("&H" & strStr1)
    nRtn = nmc_get_node_od(g_nCardId, 2, Val(master_id.Text), Val(strStr2), Val(s_index.Text), Val(length.Text), lValue)
    If nRtn = 0 Then
        data.Text = "0x" + Hex(lValue)
        MsgBox "对象字典读取成功！", vbOKOnly, "提示"
    Else
        MsgBox "对象字典读取失败！错误码" + Str(nRtn), vbOKOnly, "提示"
        data.Text = 0
    End If
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

Private Sub Btn_ResetSoft_Click()
 display.Text = "请勿操作，总线卡软件复位进行中……"
    dmc_soft_reset g_nCardId
    WaitForMS (15000)
    display.Text = "总线卡软件复位完成,请确认总线状态"
End Sub

Private Sub Btn_WriteData_Click()
 Dim nRtn As Integer
    Dim lValue As Integer
    Dim strStr1 As String
    Dim strStr2 As String
    strStr1 = m_index.Text
    strStr2 = CLng("&H" & strStr1)
    
    lValue = data.Text
    nRtn = nmc_set_node_od(g_nCardId, 2, Val(master_id.Text), Val(strStr2), Val(s_index.Text), Val(length.Text), lValue)
    If nRtn = 0 Then
       
        MsgBox "对象字典写入成功！", vbOKOnly, "提示"
    Else
        MsgBox "对象字典写入失败！错误码" + Str(nRtn), vbOKOnly, "提示"
        
    End If
End Sub

Private Sub Command1_Click(index As Integer)
    Dim nAxisNum As Integer
    Dim nSlave As Integer
     Dim nSubslave As Integer
     nAxisNum = Val(axis_flag(0).Text)
    nmc_get_axis_node_address g_nCardId, nAxisNum, nSlave, nSubslave
    axis_flag(1).Text = nSlave
    axis_flag(4).Text = nSubslave
End Sub

Private Sub Command2_Click(index As Integer)
    Dim nAxisType As Integer
    Dim nAxisNum As Integer
    nAxisNum = Val(axis_flag(0).Text)
     nAxisType = nmc_get_axis_type(g_nCardId, nAxisNum, nAxisType)
    Select Case (nAxisType)
     
        Case 0: axis_flag(5).Text = Str(nAxisNum) + " 轴是虚拟轴"
                
        Case 1:
                axis_flag(5).Text = Str(nAxisNum) + " 轴是EtherCAT轴"
           
        Case 2:
                axis_flag(5).Text = Str(nAxisNum) + " 轴是CANopen轴"
            
        Case 3:
              axis_flag(5).Text = Str(nAxisNum) + " 轴是脉冲轴"
            
        Case 4:
              axis_flag(5).Text = Str(nAxisNum) + " 轴是未知类型轴"
            
     End Select
End Sub

Private Sub Command3_Click(index As Integer)
    Dim nErrcode As Integer
    Dim nAxisNum As Integer
    nAxisNum = Val(axis_flag(0).Text)
    
    nmc_get_axis_errcode g_nCardId, nAxisNum, nErrcode
    axis_flag(6).Text = nErrcode
End Sub

Private Sub Command4_Click(index As Integer)
    Dim statusword As Long
    Dim nAxisNum As Integer
    nAxisNum = Val(axis_flag(0).Text)
    
    nmc_get_axis_statusword g_nCardId, nAxisNum, statusword
    axis_flag(7).Text = statusword
    
End Sub

Private Sub Command5_Click(index As Integer)
    Dim ControlWord As Long
    Dim nAxisNum As Integer
    nAxisNum = Val(axis_flag(0).Text)
    
    nmc_get_axis_contrlword g_nCardId, nAxisNum, ControlWord
    axis_flag(8).Text = ControlWord
End Sub

Private Sub Command7_Click(index As Integer)
    Dim nAxisNum As Integer
    nAxisNum = Val(axis_flag(0).Text)
    nmc_set_axis_enable g_nCardId, nAxisNum
End Sub

Private Sub Command8_Click(index As Integer)
    Dim nAxisNum As Integer
    nAxisNum = Val(axis_flag(0).Text)
    nmc_set_axis_disable g_nCardId, nAxisNum
End Sub

Private Sub Form_Load()
    Dim nCardNum As Integer     '定义卡数
    Dim arr_CardList(7) As Integer '定义卡号数组
    Dim arr_CardTypeList(7) As Long  '定义各卡类型
    
    nCardNum = dmc_board_init()
    If (nCardNum <= 0) Or (nCardNum > 8) Then              '正常的卡数在1- 8之间
        MsgBox "初始化控制卡失败！", vbOKOnly, "出错"
    End If
    dmc_get_CardInfList nCardNum, arr_CardTypeList(0), arr_CardList(0)  '获取正在使用的卡号列表
    g_nCardId = arr_CardList(0)
   
End Sub



Private Sub Timer1_Timer()
   Dim nStateMachine As Integer
    Dim nErrcode As Integer
   g_nAxisNum = Val(axis_flag(0).Text)
    
   
   nmc_get_errcode g_nCardId, 2, nErrcode
   If nErrcode = 0 Then
        bus_state.Text = "EtherCAT总线正常"
        bus_state.BackColor = 65280
   Else
        bus_state.Text = "EtherCAT总线出错"
         bus_state.BackColor = 255
   End If
    
   
   nmc_get_axis_state_machine g_nCardId, g_nAxisNum, nStateMachine
   Select Case (nStateMachine)
       Case 0
            axis_flag(9).Text = "轴处于未启动状态"
            axis_flag(9).BackColor = 255
       Case 1
            axis_flag(9).Text = "轴处于启动禁止状态"
            axis_flag(9).BackColor = 255
       Case 2
            axis_flag(9).Text = "轴处于准备启动状态"
            axis_flag(9).BackColor = 255
       Case 3
            axis_flag(9).Text = "轴处于启动状态"
           axis_flag(9).BackColor = 65280
       Case 4
           axis_flag(9).Text = "轴处于操作使能状态"
           axis_flag(9).BackColor = 65280
       Case 5
           axis_flag(9).Text = "轴处于停止状态"
           axis_flag(9).BackColor = 255
       Case 6
           axis_flag(9).Text = "轴处于错误触发状态"
           axis_flag(9).BackColor = 255
       Case 7
            axis_flag(9).Text = "轴处于错误状态"
            axis_flag(9).BackColor = 255
           
   End Select
   Dim nTotalslave As Integer
   nmc_get_total_slaves g_nCardId, 2, nTotalslave
   slave_num.Text = nTotalslave
   
   Dim nTotalAxis As Integer
   nmc_get_total_axes g_nCardId, nTotalAxis
   AxisNum.Text = nTotalAxis
   
   Dim nTotalIn As Integer
   Dim nTotalOut As Integer
   nmc_get_total_ionum g_nCardId, nTotalIn, nTotalOut
   ECAT_INNUM.Text = nTotalIn
   ECAT_OUTNUM.Text = nTotalOut
   
   Dim nTotalain As Integer
   Dim nTotalaout As Integer
   nmc_get_total_adcnum g_nCardId, nTotalain, nTotalaout
   ECAT_AINNUM.Text = nTotalain
   ECAT_AOUTNUM.Text = nTotalaout
   
   Dim nTolIn As Integer
   Dim nTolOut As Integer
   dmc_get_total_ionum g_nCardId, nTolIn, nTolOut
   in_num.Text = nTolIn
   out_num.Text = nTolOut
   
End Sub

