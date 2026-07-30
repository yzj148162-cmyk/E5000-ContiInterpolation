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
	m_nAxis = 0;
	m_odid = 0;
	m_value = 0;
	m_mindex = _T("");
	m_oindex = _T("");
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CDMCd1Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDMCd1Dlg)
	DDX_Control(pDX, IDC_COMBO_LENGTH, m_length);
	DDX_Control(pDX, IDC_COMBO_AXISNO, m_combox_axisno);
	DDX_Text(pDX, IDC_EDIT_ODID, m_odid);
	DDX_Text(pDX, IDC_EDIT_VALUE, m_value);
	DDX_Text(pDX, IDC_EDIT_MINDEX, m_mindex);
	DDX_Text(pDX, IDC_EDIT_OINDEX, m_oindex);
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDMCd1Dlg, CDialog)
	//{{AFX_MSG_MAP(CDMCd1Dlg)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BUTTON_ENABLE, OnButtonEnable)
	ON_BN_CLICKED(IDC_BUTTON_DISABLE, OnButtonDisable)
	ON_BN_CLICKED(IDC_BUTTON_READ, OnButtonRead)
	ON_BN_CLICKED(IDC_BUTTON_WRITE, OnButtonWrite)
	ON_BN_CLICKED(IDC_BUTTON_DO, OnButtonDo)
	ON_BN_CLICKED(IDC_BUTTON_CLEAR, OnButtonClear)
	ON_BN_CLICKED(IDC_BUTTON_EMGSTOP, OnButtonEmgstop)
	ON_BN_CLICKED(IDC_BUTTON_CHANGESPEED, OnButtonChangespeed)
	ON_BN_CLICKED(IDC_BUTTON_DECSTOP, OnButtonDecstop)
	ON_BN_CLICKED(IDC_BUTTON_CHANGEPOS, OnButtonChangepos)
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
	WORD My_CardNum ;      //定义卡数
	WORD My_CardList[8];   //定义卡号数组
	DWORD My_CardTypeList[8];   //定义各卡类型
	if( dmc_board_init() <= 0 )      //控制卡的初始化操作
		MessageBox("初始化控制卡失败！","出错");
	dmc_get_CardInfList(&My_CardNum, My_CardTypeList, My_CardList);    //获取正在使用的卡号列表
	m_nCard = My_CardList[0]; 
		UpdateControl();
	    

	m_odid = 1001;
	m_mindex = "6098";
	m_oindex = "0";
	m_value = 0;
	m_combox_axisno.SetCurSel(0); 
    m_length.SetCurSel(0); 
    SetTimer( IDC_TIMER, 100, NULL );
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

void CDMCd1Dlg::UpdateControl()
{
//	GetDlgItem( IDC_CHECK_LOGIC )->SetWindowText( m_bLogic?"逻辑方向：正":"逻辑方向：反");
}

void CDMCd1Dlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	CString string;
	int iret = 0;
	m_nAxis = m_combox_axisno.GetCurSel();

    unsigned short TotalSlaves=0;
    nmc_get_total_slaves(m_nCard, 2, &TotalSlaves);
	string.Format("%d", TotalSlaves );
	GetDlgItem( IDC_STATIC_OD )->SetWindowText( string );

	unsigned long TotalAxises=0;
    nmc_get_total_axes(m_nCard, &TotalAxises);
	string.Format("%d", TotalAxises );
	GetDlgItem( IDC_STATIC_OD2 )->SetWindowText( string );

	unsigned short TotalIn=0,TotalOut=0;
    nmc_get_total_ionum(m_nCard, &TotalIn, &TotalOut);
	string.Format("%d", TotalIn );
	GetDlgItem( IDC_STATIC_OD5 )->SetWindowText( string );
	string.Format("%d", TotalOut );
	GetDlgItem( IDC_STATIC_OD6 )->SetWindowText( string );

   	unsigned short TotalADIn = 0, TotalDAOut = 0;
    nmc_get_total_adcnum(m_nCard, &TotalADIn, &TotalDAOut);
	string.Format("%d", TotalADIn );
	GetDlgItem( IDC_STATIC_OD7 )->SetWindowText( string );
	string.Format("%d", TotalDAOut );
	GetDlgItem( IDC_STATIC_OD8 )->SetWindowText( string );


    unsigned short Axis_State_machine = 0;
    nmc_get_axis_state_machine(m_nCard, m_nAxis, &Axis_State_machine);
  
	switch (Axis_State_machine)// 读取指定轴状态机
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

	 unsigned short errcode = 1;
     nmc_get_errcode(m_nCard,2,&errcode);
  
     if (errcode == 0)
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
	// TODO: Add your message handler code here
	dmc_board_close();	//非常之重要，释放其占用的系统资源
	KillTimer( IDC_TIMER );
	CDialog::OnDestroy();
	
}
 



void CDMCd1Dlg::OnButtonEnable() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);//刷新参数
	
	m_nAxis = m_combox_axisno.GetCurSel();
	nmc_set_axis_enable(m_nCard, m_nAxis);
}

void CDMCd1Dlg::OnButtonDisable() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);//刷新参数
	
	m_nAxis = m_combox_axisno.GetCurSel();
	nmc_set_axis_disable(m_nCard, m_nAxis);
}



void CDMCd1Dlg::OnButtonRead() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);//刷新参数	
	CString string;
	int lenth;
	WORD m_len;
	m_nAxis = m_combox_axisno.GetCurSel();
    lenth = m_length.GetCurSel();
	long Read_Value=0;   
 
	switch (lenth)// 读取指定轴状态机
    {
	case 0: 
         m_len = 1;
         break;
	case 1:
         m_len = 8;
         break;
	case 2:
         m_len =16;
         break; 
	case 3:
         m_len =32;
         break; 

	default:
		break;
    };
	
	WORD mindex = strtol(m_mindex,NULL,16);
    WORD oindex = strtol(m_oindex,NULL,16);

    short ret1 = nmc_get_node_od(m_nCard, 2, m_odid, mindex,oindex, m_len, &Read_Value);
    if(ret1==0)
	{
		//String strA = Read_Value.ToString("x8");//10进制转化为16进制

		MessageBox("对象字典读取成功!","提示");
    	string.Format("%x", Read_Value );
	    GetDlgItem( IDC_EDIT_VALUE )->SetWindowText( string ); 
    }
    else
    {
		MessageBox("对象字典读取失败!","提示");
		return;
    } 



}

void CDMCd1Dlg::OnButtonWrite() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);//刷新参数	
	CString string;
	int lenth;
	WORD m_len;
	m_nAxis = m_combox_axisno.GetCurSel();
    lenth = m_length.GetCurSel();
	long Write_Value=0;
    Write_Value = m_value;
	switch (lenth)// 读取指定轴状态机
    {
	case 0: 
         m_len = 1;
         break;
	case 1:
         m_len = 8;
         break;
	case 2:
         m_len =16;
         break; 
	case 3:
         m_len =32;
         break; 

	default:
		break;
    };
	
	WORD mindex = strtol(m_mindex,NULL,16);
    WORD oindex = strtol(m_oindex,NULL,16);
    short ret1 = nmc_set_node_od(m_nCard, 2,m_odid, mindex,oindex, m_len, Write_Value);
    if (ret1 == 0)
    {                
    	MessageBox("对象字典设置成功!","提示");
    }
    else
    {
		MessageBox("对象字典设置失败!","提示");
		return;
    } 
}

void CDMCd1Dlg::OnButtonDo() 
{
	// TODO: Add your control notification handler code here
	CString string;
	m_nAxis = m_combox_axisno.GetCurSel();
	unsigned short SlaveAddress=0,Sub_SlaveAddress=0;
    nmc_get_axis_node_address(m_nCard,m_nAxis, &SlaveAddress, &Sub_SlaveAddress);
	string.Format("%d", SlaveAddress );
    GetDlgItem( IDC_STATIC_Axis )->SetWindowText( string );
   	string.Format("%d", Sub_SlaveAddress );
    GetDlgItem( IDC_STATIC_Axis1 )->SetWindowText( string );
}

void CDMCd1Dlg::OnButtonClear() 
{
	// TODO: Add your control notification handler code here
	CString string;
	m_nAxis = m_combox_axisno.GetCurSel();
	unsigned short SlaveAddress=0,Sub_SlaveAddress=0;
    nmc_get_axis_node_address(m_nCard,m_nAxis, &SlaveAddress, &Sub_SlaveAddress);
	string.Format("%d", SlaveAddress );
    GetDlgItem( IDC_STATIC_Axis )->SetWindowText( string );
   	string.Format("%d", Sub_SlaveAddress );
    GetDlgItem( IDC_STATIC_Axis1 )->SetWindowText( string );	
}

void CDMCd1Dlg::OnButtonEmgstop() 
{
	// TODO: Add your control notification handler code here
	unsigned short Axis_Type = 0;
	CString string;
	m_nAxis = m_combox_axisno.GetCurSel();
    nmc_get_axis_type(m_nCard, m_nAxis, &Axis_Type);
    switch(Axis_Type)
	{
	case 0: 
		string.Format("%d号轴是虚拟轴", m_nAxis );
		GetDlgItem( IDC_STATIC_Axis2 )->SetWindowText( string );                    
		break;
	case 1:
		string.Format("%d号轴是EtherCAT轴", m_nAxis );
		GetDlgItem( IDC_STATIC_Axis2 )->SetWindowText( string );  
		break;
	case 2:
		string.Format("%d号轴是CANopen轴", m_nAxis );
		GetDlgItem( IDC_STATIC_Axis2 )->SetWindowText( string );  
		break;
	case 3:
		string.Format("%d号轴是脉冲轴", m_nAxis );
		GetDlgItem( IDC_STATIC_Axis2 )->SetWindowText( string ); 
		break;
	case 4:
		string.Format("%d号轴是未知类型轴", m_nAxis );
		GetDlgItem( IDC_STATIC_Axis2 )->SetWindowText( string );        
		break;   
	}   	
}

void CDMCd1Dlg::OnButtonChangespeed() 
{
	// TODO: Add your control notification handler code here
	CString string;
	unsigned short ErrorCode=0;
	m_nAxis = m_combox_axisno.GetCurSel();
	nmc_get_axis_errcode(m_nCard, m_nAxis,&ErrorCode);
   	string.Format("%d", ErrorCode );
	GetDlgItem( IDC_STATIC_Axis3 )->SetWindowText( string );    
}

void CDMCd1Dlg::OnButtonDecstop() 
{
	// TODO: Add your control notification handler code here
	CString string;
	long StatusWord=0;
	m_nAxis = m_combox_axisno.GetCurSel();
	nmc_get_axis_statusword(m_nCard, m_nAxis,&StatusWord);
   	string.Format("%d", StatusWord );
	GetDlgItem( IDC_STATIC_Axis4 )->SetWindowText( string );   	
}

void CDMCd1Dlg::OnButtonChangepos() 
{
	// TODO: Add your control notification handler code here
	CString string;
	long ControlWord=0;
	m_nAxis = m_combox_axisno.GetCurSel();
	nmc_get_axis_contrlword(m_nCard, m_nAxis,&ControlWord);
   	string.Format("%d", ControlWord );
	GetDlgItem( IDC_STATIC_Axis5 )->SetWindowText( string );  
}


void CDMCd1Dlg::OnBnClickedButtonHardwarereset()
{
	// TODO: 在此添加控件通知处理程序代码
	GetDlgItem( IDC_STATIC_Message )->SetWindowText( "请勿操作，总线卡硬件复位进行中……" );
	GetDlgItem(IDC_Button_HardwareReset)->EnableWindow(false);
	GetDlgItem(IDC_Button_SoftwareReset)->EnableWindow(false);
	dmc_board_reset();
	dmc_board_close();
	for (int i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
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
	dmc_soft_reset(m_nCard);
	dmc_board_close();
	for (int i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
	{
		Sleep(1000);
	}
	dmc_board_init();
	GetDlgItem( IDC_STATIC_Message )->SetWindowText( "总线卡软件复位完成,请确认总线状态" );
	GetDlgItem(IDC_Button_HardwareReset)->EnableWindow(true);
	GetDlgItem(IDC_Button_SoftwareReset)->EnableWindow(true);
}
