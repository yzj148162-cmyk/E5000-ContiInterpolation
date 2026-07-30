VERSION 5.00
Object = "{F9043C88-F6F2-101A-A3C9-08002B2F49FB}#1.2#0"; "comdlg32.ocx"
Begin VB.Form 下载 
   Caption         =   "下载配置"
   ClientHeight    =   8070
   ClientLeft      =   120
   ClientTop       =   450
   ClientWidth     =   11625
   LinkTopic       =   "Form1"
   ScaleHeight     =   8070
   ScaleWidth      =   11625
   StartUpPosition =   3  '窗口缺省
   Begin MSComDlg.CommonDialog CommonDialog1 
      Left            =   600
      Top             =   720
      _ExtentX        =   847
      _ExtentY        =   847
      _Version        =   393216
   End
   Begin VB.CommandButton lode 
      Caption         =   "2.下载ENI文件"
      Height          =   495
      Left            =   3480
      TabIndex        =   10
      Top             =   1920
      Width           =   1695
   End
   Begin VB.CommandButton done 
      Caption         =   "1.下载配置文件"
      Height          =   495
      Left            =   3480
      TabIndex        =   9
      Top             =   960
      Width           =   1695
   End
   Begin VB.Frame Frame3 
      Caption         =   "总线状态"
      Height          =   3975
      Left            =   5400
      TabIndex        =   2
      Top             =   360
      Width           =   4935
      Begin VB.CommandButton initi 
         Caption         =   "初始值复位"
         Height          =   495
         Left            =   3240
         TabIndex        =   8
         Top             =   1200
         Width           =   1335
      End
      Begin VB.TextBox bus_state 
         Height          =   375
         Left            =   1320
         TabIndex        =   6
         Text            =   " "
         Top             =   360
         Width           =   2895
      End
      Begin VB.CommandButton reset_HAND 
         Caption         =   "硬件复位"
         Height          =   495
         Left            =   360
         TabIndex        =   5
         Top             =   1200
         Width           =   1215
      End
      Begin VB.CommandButton reset_soft 
         Caption         =   "软件复位"
         Height          =   495
         Left            =   1800
         TabIndex        =   4
         Top             =   1200
         Width           =   1215
      End
      Begin VB.TextBox display 
         Height          =   1335
         Left            =   360
         TabIndex        =   3
         Text            =   " "
         Top             =   2040
         Width           =   4335
      End
      Begin VB.Label Label5 
         Alignment       =   2  'Center
         Caption         =   "总线状态"
         Height          =   375
         Left            =   120
         TabIndex        =   7
         Top             =   480
         Width           =   1215
      End
   End
   Begin VB.CommandButton disable 
      Caption         =   "所有轴失能"
      Height          =   495
      Left            =   3360
      TabIndex        =   1
      Top             =   3720
      Width           =   1455
   End
   Begin VB.CommandButton enable 
      Caption         =   "所有轴使能"
      Height          =   495
      Left            =   1440
      TabIndex        =   0
      Top             =   3720
      Width           =   1455
   End
   Begin VB.Timer Timer1 
      Interval        =   100
      Left            =   4800
      Top             =   7440
   End
End
Attribute VB_Name = "下载"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Option Explicit
Public m_UseCard As Integer         '当前使用的卡号
Public m_UseAxis As Integer         '当前使用的轴号

Private Declare Function timeGetTime Lib "winmm.dll" () As Long '该声明得到系统开机到现在的时间(单位：毫秒)

Private Declare Function MultiByteToWideChar Lib "kernel32 " (ByVal CodePage As Long, ByVal dwFlags As Long, ByVal lpMultiByteStr As Long, ByVal cchMultiByte As Long, ByVal lpWideCharStr As Long, ByVal cchWideChar As Long) As Long
Private Declare Function WideCharToMultiByte Lib "kernel32 " (ByVal CodePage As Long, ByVal dwFlags As Long, ByVal lpWideCharStr As Long, ByVal cchWideChar As Long, ByVal lpMultiByteStr As Long, ByVal cchMultiByte As Long, ByVal lpDefaultChar As Long, ByVal lpUsedDefaultChar As Long) As Long
Private Const CP_ACP = 0 ' default to ANSI code page
Private Const CP_UTF8 = 65001 ' default to UTF-8 code page
'字符转 UTF8
Public Function EncodeToBytes(ByVal sData As String) As Byte() ' Note: Len(sData) > 0
Dim aRetn() As Byte
Dim nSize As Long
nSize = WideCharToMultiByte(CP_UTF8, 0, StrPtr(sData), -1, 0, 0, 0, 0) - 1
If nSize = 0 Then Exit Function
ReDim aRetn(0 To nSize - 1) As Byte
WideCharToMultiByte CP_UTF8, 0, StrPtr(sData), -1, VarPtr(aRetn(0)), nSize, 0, 0
EncodeToBytes = aRetn
Erase aRetn
End Function



Public Function WaitForMS(T As Long)
    Dim Savetime As Long
    Savetime = timeGetTime '记下开始时的时间
    While timeGetTime < Savetime + T '循环等待
        DoEvents '转让控制权
    Wend
End Function


Private Sub disable_Click()
 nmc_set_axis_disable m_UseCard, 255
End Sub

Private Sub done_Click()
     CommonDialog1.Flags = cdlOFNHideReadOnly ' 设置过滤器
     CommonDialog1.filter = "All Files (*.*)|*.*|Text Files" & "(*.ini)|*.ini|Batch Files (*.txt)|*.txt"
    ' 指定缺省的过滤器

    CommonDialog1.FilterIndex = 2

    ' 显示“打开”对话框

    CommonDialog1.ShowOpen

    ' 显示选定文件的名字
    Dim ret As Integer
    Dim filetype As Integer
    Dim buffer() As Byte
    Dim fileincontrol() As Byte
  
    Dim path As String
    Dim str As String
    Dim LinStr As String
    Dim Encoding As String
    Dim lenT As Long
    Dim pfilenameinControl As Byte
    str = ""
    path = CommonDialog1.FileName
    
    Open path For Input As #1       '以读的方式打开文件
    Do While Not EOF(1)    ' 循环至文件尾
    
     Line Input #1, LinStr  '读入一行用户名
    str = str & LinStr & vbLf
     Loop
    Close #1   ' 关闭文件
   
     buffer = StrConv(str, vbFromUnicode)
    
     filetype = 201
     ret = -1
     
               lenT = UBound(buffer)
               ret = nmc_set_cycletime(m_UseCard, 2, 500)      '//设置总线周期
               ret = dmc_download_memfile(m_UseCard, buffer(0), lenT, 0, filetype) '//下载配置文件
          
                If ret = 0 Then
                  MsgBox ("下载成功！")
                Else
                  MsgBox ("下载失败，检查文件！")
                End If
                   

End Sub

Private Sub enable_Click()
    nmc_set_axis_enable m_UseCard, 255
End Sub

Private Sub Form_Load()
    Dim My_CardNum As Integer     '定义卡数
    Dim My_CardList(7) As Integer '定义卡号数组
    Dim My_CardTypeList(7) As Long  '定义各卡类型
    
    My_CardNum = dmc_board_init()
    If (My_CardNum <= 0) Or (My_CardNum > 8) Then              '正常的卡数在1- 8之间
        MsgBox "初始化控制卡失败！", vbOKOnly, "出错"
    End If
    dmc_get_CardInfList My_CardNum, My_CardTypeList(0), My_CardList(0)  '获取正在使用的卡号列表
    m_UseCard = My_CardList(0)
 
   
End Sub


Private Sub initi_Click()
  display.Text = "请勿操作，总线卡初始值复位进行中……"
    dmc_original_reset (m_UseCard)
    dmc_board_close
    Dim Savetime As Long
    WaitForMS (15000)
    dmc_board_init
    display.Text = "总线卡初始值复位完成,请确认总线状态"
End Sub

Private Sub lode_Click()
     CommonDialog1.Flags = cdlOFNHideReadOnly ' 设置过滤器
     CommonDialog1.filter = "All Files (*.*)|*.*|Text Files" & "(*.eni)|*.eni|Batch Files (*.txt)|*.txt"

    ' 指定缺省的过滤器

    CommonDialog1.FilterIndex = 2

    ' 显示“打开”对话框

    CommonDialog1.ShowOpen

    ' 显示选定文件的名字
    Dim ret As Integer
    Dim filetype As Integer
    Dim buffer() As Byte
    Dim fileincontrol() As Byte
  
    Dim path As String
    Dim str As String
    Dim LinStr As String
    Dim Encoding As String
     Dim lenT As Long
    
    str = ""
    path = CommonDialog1.FileName
    
    Open path For Input As #1       '以读的方式打开文件
    Do While Not EOF(1)    ' 循环至文件尾
    
     Line Input #1, LinStr  '读入一行用户名
    str = str & LinStr & vbLf
     Loop
    Close #1   ' 关闭文件
   
     buffer = StrConv(str, vbFromUnicode)
    
     filetype = 200
     ret = -1
     
               lenT = UBound(buffer)
               ret = nmc_set_cycletime(m_UseCard, 2, 500)      '//设置总线周期
               ret = dmc_download_memfile(m_UseCard, buffer(0), lenT, 0, filetype) '//下载配置文件
          
                If ret = 0 Then
                  MsgBox ("下载成功！")
                  Else
                  MsgBox ("下载失败，检查文件！")
                End If
                
End Sub

Private Sub reset_HAND_Click()
    display.Text = "请勿操作，总线卡硬件复位进行中……"
    dmc_board_reset
    dmc_board_close
    Dim Savetime As Long
    WaitForMS (15000)
    dmc_board_init
    display.Text = "总线卡硬件复位完成,请确认总线状态"
End Sub

Private Sub reset_soft_Click()
    display.Text = "请勿操作，总线卡软件复位进行中……"
    dmc_soft_reset m_UseCard
    dmc_board_close
    WaitForMS (15000)
    dmc_board_init
    display.Text = "总线卡软件复位完成,请确认总线状态"
End Sub

Private Sub Timer1_Timer()
   Dim statemachine As Integer
    Dim errcode As Integer
    
   
   nmc_get_errcode m_UseCard, 2, errcode
   If errcode = 0 Then
        bus_state.Text = "EtherCAT总线正常"
        bus_state.BackColor = 65280
   Else
        bus_state.Text = "EtherCAT总线出错"
         bus_state.BackColor = 255
   End If
    
   
End Sub

