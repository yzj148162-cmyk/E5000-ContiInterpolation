VERSION 5.00
Begin VB.Form reset_HAND 
   Caption         =   "通用专用输入输出"
   ClientHeight    =   6840
   ClientLeft      =   60
   ClientTop       =   450
   ClientWidth     =   10980
   LinkTopic       =   "Form1"
   ScaleHeight     =   6840
   ScaleWidth      =   10980
   StartUpPosition =   3  '窗口缺省
   Begin VB.Frame Frame6 
      Caption         =   "复位操作及总线状态"
      Height          =   3015
      Left            =   6480
      TabIndex        =   59
      Top             =   1920
      Width           =   4335
      Begin VB.TextBox BUS_STATE 
         Height          =   495
         Left            =   1560
         TabIndex        =   63
         Top             =   360
         Width           =   2055
      End
      Begin VB.CommandButton Btn_SoftReset 
         Caption         =   "软件复位"
         Height          =   495
         Left            =   2640
         TabIndex        =   62
         Top             =   1080
         Width           =   1215
      End
      Begin VB.TextBox display 
         Alignment       =   2  'Center
         Height          =   1095
         Left            =   480
         TabIndex        =   61
         Top             =   1800
         Width           =   3495
      End
      Begin VB.CommandButton Btn_HangReset 
         Caption         =   "硬件复位"
         Height          =   495
         Left            =   600
         TabIndex        =   60
         Top             =   1080
         Width           =   1215
      End
      Begin VB.Label Label2 
         Alignment       =   2  'Center
         Caption         =   "总线状态:"
         Height          =   255
         Left            =   600
         TabIndex        =   64
         Top             =   480
         Width           =   1095
      End
   End
   Begin VB.Frame Frame5 
      Caption         =   "EtherCAT IO"
      Height          =   1575
      Left            =   6480
      TabIndex        =   54
      Top             =   120
      Width           =   4335
      Begin VB.TextBox IN_NUM 
         Height          =   375
         Left            =   2400
         TabIndex        =   56
         Text            =   " "
         Top             =   360
         Width           =   975
      End
      Begin VB.TextBox OUT_NUM 
         Height          =   375
         Left            =   2400
         TabIndex        =   55
         Top             =   960
         Width           =   975
      End
      Begin VB.Label Label3 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "EtherCAT IO输入口:"
         Height          =   375
         Left            =   240
         TabIndex        =   58
         Top             =   480
         Width           =   2055
      End
      Begin VB.Label Label4 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "EtherCAT IO输出口:"
         Height          =   375
         Left            =   240
         TabIndex        =   57
         Top             =   1080
         Width           =   2175
      End
   End
   Begin VB.CommandButton Btn_Close 
      Caption         =   "退出程序"
      Height          =   615
      Left            =   7680
      TabIndex        =   49
      Top             =   5760
      Width           =   1575
   End
   Begin VB.Frame Frame4 
      Caption         =   "EtherCAT-输入口(1-15)"
      Height          =   1575
      Left            =   360
      TabIndex        =   15
      Top             =   3120
      Width           =   6015
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "15"
         Height          =   255
         Index           =   15
         Left            =   5400
         TabIndex        =   31
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "14"
         Height          =   255
         Index           =   14
         Left            =   4680
         TabIndex        =   30
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "13"
         Height          =   255
         Index           =   13
         Left            =   3960
         TabIndex        =   29
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "12"
         Height          =   255
         Index           =   12
         Left            =   3240
         TabIndex        =   28
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "11"
         Height          =   255
         Index           =   11
         Left            =   2520
         TabIndex        =   27
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "10"
         Height          =   255
         Index           =   10
         Left            =   1800
         TabIndex        =   26
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "9"
         Height          =   255
         Index           =   9
         Left            =   1080
         TabIndex        =   25
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "8"
         Height          =   255
         Index           =   8
         Left            =   360
         TabIndex        =   24
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "7"
         Height          =   255
         Index           =   7
         Left            =   5400
         TabIndex        =   23
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "6"
         Height          =   255
         Index           =   6
         Left            =   4680
         TabIndex        =   22
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "5"
         Height          =   255
         Index           =   5
         Left            =   3960
         TabIndex        =   21
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "4"
         Height          =   255
         Index           =   4
         Left            =   3240
         TabIndex        =   20
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "3"
         Height          =   255
         Index           =   3
         Left            =   2520
         TabIndex        =   19
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "2"
         Height          =   255
         Index           =   2
         Left            =   1800
         TabIndex        =   18
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "1"
         Height          =   255
         Index           =   1
         Left            =   1080
         TabIndex        =   17
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECAT_IN 
         Alignment       =   2  'Center
         BackColor       =   &H80000014&
         Caption         =   "0"
         Height          =   255
         Index           =   0
         Left            =   360
         TabIndex        =   16
         Top             =   360
         Width           =   495
      End
   End
   Begin VB.Frame Frame3 
      Caption         =   "EtherCAT-输出口(1-15)"
      Height          =   1575
      Left            =   360
      TabIndex        =   14
      Top             =   4920
      Width           =   6015
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "15"
         Height          =   255
         Index           =   15
         Left            =   5280
         TabIndex        =   47
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "14"
         Height          =   255
         Index           =   14
         Left            =   4560
         TabIndex        =   46
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "13"
         Height          =   255
         Index           =   13
         Left            =   3840
         TabIndex        =   45
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "12"
         Height          =   255
         Index           =   12
         Left            =   3120
         TabIndex        =   44
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "11"
         Height          =   255
         Index           =   11
         Left            =   2400
         TabIndex        =   43
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "10"
         Height          =   255
         Index           =   10
         Left            =   1800
         TabIndex        =   42
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "9"
         Height          =   255
         Index           =   9
         Left            =   1080
         TabIndex        =   41
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "8"
         Height          =   255
         Index           =   8
         Left            =   360
         TabIndex        =   40
         Top             =   960
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "7"
         Height          =   255
         Index           =   7
         Left            =   5280
         TabIndex        =   39
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "6"
         Height          =   255
         Index           =   6
         Left            =   4560
         TabIndex        =   38
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "5"
         Height          =   255
         Index           =   5
         Left            =   3840
         TabIndex        =   37
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "4"
         Height          =   255
         Index           =   4
         Left            =   3120
         TabIndex        =   36
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "3"
         Height          =   255
         Index           =   3
         Left            =   2400
         TabIndex        =   35
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "2"
         Height          =   255
         Index           =   2
         Left            =   1800
         TabIndex        =   34
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "1"
         Height          =   255
         Index           =   1
         Left            =   1080
         TabIndex        =   33
         Top             =   360
         Width           =   495
      End
      Begin VB.Label ECT_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "0"
         Height          =   255
         Index           =   0
         Left            =   360
         TabIndex        =   32
         Top             =   360
         Width           =   495
      End
   End
   Begin VB.Timer Timer1 
      Interval        =   100
      Left            =   6360
      Top             =   5040
   End
   Begin VB.Frame Frame2 
      Caption         =   "通用输出口(1-15)"
      Height          =   1335
      Left            =   360
      TabIndex        =   1
      Top             =   1560
      Width           =   6015
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "15"
         Height          =   255
         Index           =   15
         Left            =   5280
         TabIndex        =   72
         Top             =   840
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "14"
         Height          =   255
         Index           =   14
         Left            =   4560
         TabIndex        =   71
         Top             =   840
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "13"
         Height          =   255
         Index           =   13
         Left            =   3960
         TabIndex        =   70
         Top             =   840
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "12"
         Height          =   255
         Index           =   12
         Left            =   3240
         TabIndex        =   69
         Top             =   840
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "11"
         Height          =   255
         Index           =   11
         Left            =   2520
         TabIndex        =   68
         Top             =   840
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "10"
         Height          =   255
         Index           =   10
         Left            =   1800
         TabIndex        =   67
         Top             =   840
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "9"
         Height          =   255
         Index           =   9
         Left            =   1080
         TabIndex        =   66
         Top             =   840
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "8"
         Height          =   255
         Index           =   8
         Left            =   360
         TabIndex        =   65
         Top             =   840
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "7"
         Height          =   255
         Index           =   7
         Left            =   5280
         TabIndex        =   53
         Top             =   480
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "6"
         Height          =   255
         Index           =   6
         Left            =   4560
         TabIndex        =   52
         Top             =   480
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "5"
         Height          =   255
         Index           =   5
         Left            =   3960
         TabIndex        =   51
         Top             =   480
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "4"
         Height          =   255
         Index           =   4
         Left            =   3240
         TabIndex        =   50
         Top             =   480
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "3"
         Height          =   255
         Index           =   3
         Left            =   2520
         TabIndex        =   13
         Top             =   480
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "2"
         Height          =   255
         Index           =   2
         Left            =   1800
         TabIndex        =   12
         Top             =   480
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "1"
         Height          =   255
         Index           =   1
         Left            =   1080
         TabIndex        =   11
         Top             =   480
         Width           =   495
      End
      Begin VB.Label Label_OUT 
         Alignment       =   2  'Center
         BorderStyle     =   1  'Fixed Single
         Caption         =   "0"
         Height          =   255
         Index           =   0
         Left            =   360
         TabIndex        =   10
         Top             =   480
         Width           =   495
      End
   End
   Begin VB.Frame Frame1 
      Caption         =   "通用输入口(1-15)"
      Height          =   1215
      Left            =   360
      TabIndex        =   0
      Top             =   120
      Width           =   6015
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H00FFFFFF&
         Caption         =   "15"
         Height          =   255
         Index           =   15
         Left            =   5400
         TabIndex        =   80
         Top             =   720
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "14"
         Height          =   255
         Index           =   14
         Left            =   4680
         TabIndex        =   79
         Top             =   720
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "13"
         Height          =   255
         Index           =   13
         Left            =   3960
         TabIndex        =   78
         Top             =   720
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "12"
         Height          =   255
         Index           =   12
         Left            =   3240
         TabIndex        =   77
         Top             =   720
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "11"
         Height          =   255
         Index           =   11
         Left            =   2520
         TabIndex        =   76
         Top             =   720
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "10"
         Height          =   255
         Index           =   10
         Left            =   1800
         TabIndex        =   75
         Top             =   720
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "9"
         Height          =   255
         Index           =   9
         Left            =   1080
         TabIndex        =   74
         Top             =   720
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "8"
         Height          =   255
         Index           =   8
         Left            =   360
         TabIndex        =   73
         Top             =   720
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "7"
         Height          =   255
         Index           =   7
         Left            =   5400
         TabIndex        =   9
         Top             =   360
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "6"
         Height          =   255
         Index           =   6
         Left            =   4680
         TabIndex        =   8
         Top             =   360
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "5"
         Height          =   255
         Index           =   5
         Left            =   3960
         TabIndex        =   7
         Top             =   360
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "4"
         Height          =   255
         Index           =   4
         Left            =   3240
         TabIndex        =   6
         Top             =   360
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "3"
         Height          =   255
         Index           =   3
         Left            =   2520
         TabIndex        =   5
         Top             =   360
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "2"
         Height          =   255
         Index           =   2
         Left            =   1800
         TabIndex        =   4
         Top             =   360
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H8000000E&
         Caption         =   "1"
         Height          =   255
         Index           =   1
         Left            =   1080
         TabIndex        =   3
         Top             =   360
         Width           =   495
      End
      Begin VB.Label Label_IN 
         Alignment       =   2  'Center
         BackColor       =   &H00FFFFFF&
         Caption         =   "0"
         Height          =   255
         Index           =   0
         Left            =   360
         TabIndex        =   2
         Top             =   360
         Width           =   495
      End
   End
   Begin VB.Label Label1 
      Alignment       =   2  'Center
      Caption         =   $"通用专用输入输出.frx":0000
      Height          =   615
      Left            =   6840
      TabIndex        =   48
      Top             =   5040
      Width           =   3495
   End
End
Attribute VB_Name = "reset_HAND"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Dim g_nAxis As Integer
Dim g_nCardId As Integer        '卡号定义

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
    End
End Sub

Private Sub Btn_HangReset_Click()

    display.Text = "请勿操作，总线卡硬件复位进行中……"
    dmc_board_reset
    dmc_board_close
    Dim Savetime As Long
    WaitForMS (15000)
    dmc_board_init
    display.Text = "总线卡硬件复位完成,请确认总线状态"
End Sub

Private Sub Btn_SoftReset_Click()
 display.Text = "请勿操作，总线卡软件复位进行中……"
    dmc_soft_reset g_nCardId
    WaitForMS (15000)
    display.Text = "总线卡软件复位完成,请确认总线状态"
End Sub


Private Sub ECT_OUT_Click(index As Integer)
    Dim nOutBit As Integer
    nOutBit = index + 16
    
    If dmc_read_outbit(g_nCardId, nOutBit) = 1 Then
       dmc_write_outbit g_nCardId, nOutBit, 0
    Else
       dmc_write_outbit g_nCardId, nOutBit, 1
    End If
End Sub

Private Sub Form_Load()
    Dim My_CardNum As Integer     '定义卡数
    Dim My_CardList(7) As Integer '定义卡号数组
    Dim My_CardTypeList(7) As Long  '定义各卡类型
    
    If dmc_board_init() <= 0 Then '控制卡的初始化操作
        MsgBox "初始化控制卡失败！", vbOKOnly, "出错"
   Else
    dmc_get_CardInfList My_CardNum, My_CardTypeList(0), My_CardList(0)  '获取正在使用的卡号列表
    g_nCardId = My_CardList(0)       '初始化选择卡
    g_nAxis = 0
     MsgBox "当前选择的卡号为：" + Str(g_nCardId), , "卡号选择提示"
      End If
    
    For i = 0 To 7
        Label_IN(i).BackColor = vbYellow
       
    Next
    
    For i = 0 To 7
       
        Label_OUT(i).BackColor = vbYellow
    Next
    
     For i = 0 To 15
        ECAT_IN(i).BackColor = vbYellow
       
    Next
    
    For i = 0 To 15
       
        ECT_OUT(i).BackColor = vbYellow
    Next
   
    
    
 End Sub

Private Sub Form_Unload(Cancel As Integer)
    dmc_board_close
    Unload Me
End Sub

Private Sub Label_OUT_Click(index As Integer)
    Dim nOutBit As Integer
    nOutBit = index
    
    If dmc_read_outbit(g_nCardId, nOutBit) = 1 Then
       dmc_write_outbit g_nCardId, nOutBit, 0
    Else
       dmc_write_outbit g_nCardId, nOutBit, 1
    End If
End Sub



Private Sub Timer1_Timer()
    Dim i As Integer
    Dim AxisStatus As Integer
    Dim tempState As Long
    
    For i = 0 To 15
        If dmc_read_inbit(g_nCardId, i) = 0 Then
            Label_IN(i).BackColor = vbGreen
        ElseIf dmc_read_inbit(g_nCardId, i) = 1 Then
            Label_IN(i).BackColor = vbRed
        End If
    Next i
    
    For i = 0 To 15
        If dmc_read_outbit(g_nCardId, i) = 0 Then
            Label_OUT(i).BackColor = vbGreen
        ElseIf dmc_read_outbit(g_nCardId, i) = 1 Then
            Label_OUT(i).BackColor = vbRed
        End If
    Next i
    
    For i = 16 To 31
        If dmc_read_inbit(g_nCardId, i) = 0 Then
            ECAT_IN(i - 16).BackColor = vbGreen
        ElseIf dmc_read_inbit(g_nCardId, i) = 1 Then
            ECAT_IN(i - 16).BackColor = vbRed
        End If
    Next i
    For i = 16 To 31
        If dmc_read_outbit(g_nCardId, i) = 0 Then
            ECT_OUT(i - 16).BackColor = vbGreen
        ElseIf dmc_read_outbit(g_nCardId, i) = 1 Then
            ECT_OUT(i - 16).BackColor = vbRed
        End If
    Next i
    
   Dim errcode As Integer
   nmc_get_errcode g_nCardId, 2, errcode
   If errcode = 0 Then
        BUS_STATE.Text = "EtherCAT总线正常"
        BUS_STATE.BackColor = 65280
   Else
        BUS_STATE.Text = "EtherCAT总线出错"
        BUS_STATE.BackColor = 255
   End If
   Dim TotalIn As Integer
   Dim TotalOut As Integer
   nmc_get_total_ionum g_nCardId, TotalIn, TotalOut
   IN_NUM.Text = Val(TotalIn)
   OUT_NUM.Text = Val(TotalOut)
End Sub

