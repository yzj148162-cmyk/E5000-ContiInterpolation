// DMCd1Dlg.cpp : implementation file
//

#include "stdafx.h"
#include "DMCd1.h"
#include "DMCd1Dlg.h"
#include "LTDMC.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDMCd1Dlg dialog

CDMCd1Dlg::CDMCd1Dlg(CWnd* pParent /*=NULL*/)
	: CDialog(CDMCd1Dlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CDMCd1Dlg)
	m_wAxis = 0;
	m_dEquiv = 0.0;
	m_dStartSpeed = 0.0;
	m_dSpeed = 0.0;
	m_dAccTime = 0.0;
	m_dChangeTime = 0.0;
	m_dDecTime = 0.0;
	m_dChangeSpeed = 0.0;
	m_dStopSpeed = 0.0;
	m_dSParaTime = 0.0;
	m_nDir = 0;
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CDMCd1Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDMCd1Dlg)
	DDX_Control(pDX, IDC_COMBO_AxisNum, m_nCombox_AxisNum);
	DDX_Text(pDX, IDC_EDIT_Equiv, m_dEquiv);
	DDX_Text(pDX, IDC_EDIT_StartSpeed, m_dStartSpeed);
	DDX_Text(pDX, IDC_EDIT_Speed, m_dSpeed);
	DDX_Text(pDX, IDC_EDIT_AccTime, m_dAccTime);
	DDX_Text(pDX, IDC_EDIT_ChangeTime, m_dChangeTime);
	DDX_Text(pDX, IDC_EDIT_DecTime, m_dDecTime);
	DDX_Text(pDX, IDC_EDIT_ChangeSpeed, m_dChangeSpeed);
	DDX_Text(pDX, IDC_EDIT_StopSpeed, m_dStopSpeed);
	DDX_Text(pDX, IDC_EDIT_SPara, m_dSParaTime);
	DDX_Radio(pDX, IDC_Radio_DirP, m_nDir);
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDMCd1Dlg, CDialog)
	//{{AFX_MSG_MAP(CDMCd1Dlg)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_Button_StartMove, OnButtonStartMove)
	ON_BN_CLICKED(IDC_Button_Stop, OnButtonStop)
	ON_BN_CLICKED(IDC_Button_SetEquiv, OnButtonSetEquiv)
	ON_BN_CLICKED(IDC_Button_Clear, OnButtonClear)
	ON_BN_CLICKED(IDC_Button_Exit, OnButtonExit)
	ON_BN_CLICKED(IDC_Button_ChangeSpeed, OnButtonChangeSpeed)

	//}}AFX_MSG_MAP

	ON_BN_CLICKED(IDC_Button_HardwareReset, &CDMCd1Dlg::OnBnClickedButtonHardwarereset)
	ON_BN_CLICKED(IDC_Button_SoftwareReset, &CDMCd1Dlg::OnBnClickedButtonSoftwarereset)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDMCd1Dlg message handlers

BOOL CDMCd1Dlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	WORD wCardNum ;      //定义卡数
	WORD arrwCardList[8];   //定义卡号数组
	DWORD arrdwCardTypeList[8];   //定义各卡类型
	if( dmc_board_init() <= 0 )      //控制卡的初始化操作
		MessageBox("初始化控制卡失败！","出错");
	dmc_get_CardInfList(&wCardNum, arrdwCardTypeList,arrwCardList );    //获取正在使用的卡号列表
	m_wCard = arrwCardList[0]; 

	SetTimer( IDC_TIMER, 100, NULL );

	m_dSpeed=3000;
	m_dStartSpeed=0;
	m_dAccTime=0.01;
	m_dDecTime=0.01;
	m_dSParaTime= 0;      //初始化单轴速度曲线参数
	m_dEquiv = 1;
	m_dStopSpeed = 0;
	m_nCombox_AxisNum.SetCurSel(0); 
	m_dChangeTime = 0.01;
	m_dChangeSpeed = 5000;

	short sRtn = 0;           
	sRtn =nmc_set_axis_enable(m_wCard, m_wAxis);// 使能对应轴
	UpdateData(false);
	return TRUE;  // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CDMCd1Dlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CDMCd1Dlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}


void CDMCd1Dlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	//获取当前轴位置
	CString stStr;
	double dPos = 0;
	int nRtn = 0;
	nRtn = dmc_get_position_unit( m_wCard,m_wAxis,&dPos);         
	stStr.Format("%.2f", dPos );
	GetDlgItem( IDC_EDIT_CurPos )->SetWindowText( stStr );

	// 读取轴编码器脉冲位置值
	double dEncValue = 0;
	nRtn = dmc_get_encoder_unit(m_wCard,m_wAxis, &dEncValue);
	stStr.Format("%.2f", dEncValue );
	GetDlgItem( IDC_EDIT_CurEncPos )->SetWindowText( stStr );   

	//获取当前轴速度
	double dSpeed = 0;
	nRtn = dmc_read_current_speed_unit( m_wCard,m_wAxis,&dSpeed );    
	stStr.Format("%.2f", dSpeed );
	GetDlgItem( IDC_EDIT_CurSpeed)->SetWindowText( stStr );

	//判断当前轴状态
	DWORD dwStatus = dmc_check_done(m_wCard, m_wAxis );           
	if (dwStatus == 1)
	{
		GetDlgItem( IDC_EDIT_Status )->SetWindowText( "静止" );
	}
	else
	{
		GetDlgItem( IDC_EDIT_Status )->SetWindowText( "运动" );
	}

	// 读取指定轴状态机
	unsigned short usAxisStateMachine = 0;
	nmc_get_axis_state_machine(m_wCard, m_wAxis, &usAxisStateMachine);	
	switch (usAxisStateMachine)
	{
	case 0: 
		GetDlgItem( IDC_EDIT_StateMachine )->SetWindowText( "轴处于未启动状态" );
		break;
	case 1:
		GetDlgItem( IDC_EDIT_StateMachine )->SetWindowText( "轴处于启动禁止状态" );       
		break;
	case 2:
		GetDlgItem( IDC_EDIT_StateMachine )->SetWindowText( "轴处于准备启动状态" );       
		break; 
	case 3:
		GetDlgItem( IDC_EDIT_StateMachine )->SetWindowText( "轴处于启动状态" );        
		break; 
	case 4:
		GetDlgItem( IDC_EDIT_StateMachine )->SetWindowText( "轴处于操作使能状态" );        
		break; 
	case 5:
		GetDlgItem( IDC_EDIT_StateMachine )->SetWindowText( "轴处于停止状态" );         
		break; 
	case 6:
		GetDlgItem( IDC_EDIT_StateMachine )->SetWindowText( "轴处于错误触发状态" );       
		break; 
	case 7:
		GetDlgItem( IDC_EDIT_StateMachine )->SetWindowText( "轴处于错误状态" );       
		break;
	default:
		break;
	};
	//读取总线状态
	unsigned short usErrCode = 1;
	nmc_get_errcode(m_wCard,2,&usErrCode);  
	if (usErrCode == 0)
	{		
		GetDlgItem( IDC_EDIT_EtherCatState )->SetWindowText( "EtherCAT总线正常" );     
	}
	else
	{		 
		GetDlgItem( IDC_EDIT_EtherCatState )->SetWindowText( "EtherCAT总线出错" );

	}
	CDialog::OnTimer(nIDEvent);
}

void CDMCd1Dlg::OnDestroy() 
{	
	short sRtn = 0;           
	sRtn =nmc_set_axis_disable(m_wCard, m_wAxis);// 失能对应轴
	dmc_board_close();	//非常之重要，释放其占用的系统资源
	KillTimer( IDC_TIMER );
	CDialog::OnDestroy();	
}

void CDMCd1Dlg::OnButtonStartMove() 
{	
	UpdateData(true);//刷新参数
	int nDir = 0;//负方向
	m_wAxis = m_nCombox_AxisNum.GetCurSel();	
	if (dmc_check_done( m_wCard,m_wAxis ) == 0) //已经在运动中
		return; 	
	dmc_set_profile_unit(m_wCard,m_wAxis,m_dStartSpeed,m_dSpeed,m_dAccTime,m_dDecTime,m_dStopSpeed);
	//设定S段时间
	dmc_set_s_profile(m_wCard,m_wAxis,0,m_dSParaTime);
	//连续运动
	if(m_nDir==0)
	{
		nDir = 1;//正方向
	}
	dmc_vmove(m_wCard, m_wAxis, nDir);
}

void CDMCd1Dlg::OnButtonStop() 
{	
	//减速停止
	m_wAxis = m_nCombox_AxisNum.GetCurSel();
	dmc_stop(m_wCard,m_wAxis,0);          
}


void CDMCd1Dlg::OnButtonSetEquiv() 
{	
	UpdateData(true);
	m_wAxis = m_nCombox_AxisNum.GetCurSel();
	dmc_set_equiv(m_wCard,m_wAxis,m_dEquiv);     
}

void CDMCd1Dlg::OnButtonClear() 
{	
	//设置零点
	for (int i=0;i<8;i++) 
	{
		dmc_set_position_unit(m_wCard,i,0);       
	}
}

void CDMCd1Dlg::OnButtonExit() 
{	
	OnCancel();
}

void CDMCd1Dlg::OnButtonChangeSpeed() 
{	
	UpdateData(true);//刷新参数
	m_wAxis = m_nCombox_AxisNum.GetCurSel();
	dmc_change_speed_unit(m_wCard, m_wAxis, m_dChangeSpeed, m_dChangeTime);  
}



void CDMCd1Dlg::OnBnClickedButtonHardwarereset()
{
	// TODO: 在此添加控件通知处理程序代码
	GetDlgItem( IDC_STATIC_Message )->SetWindowText( "请勿操作，总线卡硬件复位进行中……" );
	GetDlgItem(IDC_Button_HardwareReset)->EnableWindow(false);
	GetDlgItem(IDC_Button_SoftwareReset)->EnableWindow(false);
	dmc_board_reset();
	dmc_board_close();
	for (  int i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
	{
		Sleep(1000);
	}
	dmc_board_init();
	GetDlgItem( IDC_STATIC_Message )->SetWindowText( "总线卡硬件复位完成,请确认总线状态" );
	GetDlgItem(IDC_Button_HardwareReset)->EnableWindow(true);
	GetDlgItem(IDC_Button_SoftwareReset)->EnableWindow(true);
}


void CDMCd1Dlg::OnBnClickedButtonSoftwarereset()
{
	// TODO: 在此添加控件通知处理程序代码
	GetDlgItem( IDC_STATIC_Message )->SetWindowText( "请勿操作，总线卡软件复位进行中……" );
	GetDlgItem(IDC_Button_HardwareReset)->EnableWindow(false);
	GetDlgItem(IDC_Button_SoftwareReset)->EnableWindow(false);
	dmc_soft_reset(m_wCard);
	dmc_board_close();
	for (  int i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
	{
		Sleep(1000);
	}
	dmc_board_init();
	GetDlgItem( IDC_STATIC_Message )->SetWindowText( "总线卡软件复位完成,请确认总线状态" );
	GetDlgItem(IDC_Button_HardwareReset)->EnableWindow(true);
	GetDlgItem(IDC_Button_SoftwareReset)->EnableWindow(true);
}
