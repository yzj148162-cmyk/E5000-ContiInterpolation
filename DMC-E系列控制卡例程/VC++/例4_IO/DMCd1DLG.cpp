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
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CDMCd1Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDMCd1Dlg)
	//  DDX_Control(pDX, IDC_COMBO_NODENUM, m_combox_nodenum);
	//}}AFX_DATA_MAP

}

BEGIN_MESSAGE_MAP(CDMCd1Dlg, CDialog)
	//{{AFX_MSG_MAP(CDMCd1Dlg)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BUTTON_OUT0, OnButtonOut0)
	ON_BN_CLICKED(IDC_BUTTON_OUT1, OnButtonOut1)
	ON_BN_CLICKED(IDC_BUTTON_OUT2, OnButtonOut2)
	ON_BN_CLICKED(IDC_BUTTON_OUT3, OnButtonOut3)
	ON_BN_CLICKED(IDC_BUTTON_OUT4, OnButtonOut4)
	ON_BN_CLICKED(IDC_BUTTON_OUT5, OnButtonOut5)
	ON_BN_CLICKED(IDC_BUTTON_OUT6, OnButtonOut6)
	ON_BN_CLICKED(IDC_BUTTON_OUT7, OnButtonOut7)	
	ON_BN_CLICKED(IDC_BUTTON_CANOUT0, OnButtonCanout0)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT1, OnButtonCanout1)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT2, OnButtonCanout2)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT3, OnButtonCanout3)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT4, OnButtonCanout4)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT5, OnButtonCanout5)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT6, OnButtonCanout6)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT7, OnButtonCanout7)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT8, OnButtonCanout8)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT9, OnButtonCanout9)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT10, OnButtonCanout10)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT11, OnButtonCanout11)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT12, OnButtonCanout12)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT13, OnButtonCanout13)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT14, OnButtonCanout14)
	ON_BN_CLICKED(IDC_BUTTON_CANOUT15, OnButtonCanout15)
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON_Exit, &CDMCd1Dlg::OnBnClickedButtonExit)
	ON_BN_CLICKED(IDC_Button_HardwareReset, &CDMCd1Dlg::OnBnClickedButtonHardwarereset)
	ON_BN_CLICKED(IDC_Button_SoftwareReset, &CDMCd1Dlg::OnBnClickedButtonSoftwarereset)
	ON_BN_CLICKED(IDC_BUTTON_OUT8, &CDMCd1Dlg::OnBnClickedButtonOut8)
	ON_BN_CLICKED(IDC_BUTTON_OUT9, &CDMCd1Dlg::OnBnClickedButtonOut9)
	ON_BN_CLICKED(IDC_BUTTON_OUT10, &CDMCd1Dlg::OnBnClickedButtonOut10)
	ON_BN_CLICKED(IDC_BUTTON_OUT11, &CDMCd1Dlg::OnBnClickedButtonOut11)
	ON_BN_CLICKED(IDC_BUTTON_OUT12, &CDMCd1Dlg::OnBnClickedButtonOut12)
	ON_BN_CLICKED(IDC_BUTTON_OUT13, &CDMCd1Dlg::OnBnClickedButtonOut13)
	ON_BN_CLICKED(IDC_BUTTON_OUT14, &CDMCd1Dlg::OnBnClickedButtonOut14)
	ON_BN_CLICKED(IDC_BUTTON_OUT15, &CDMCd1Dlg::OnBnClickedButtonOut15)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDMCd1Dlg message handlers

BOOL CDMCd1Dlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application'sTemp main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	WORD wCardNum ;      //定义卡数
	WORD arrwCardList[8];   //定义卡号数组
	DWORD arrdwCardTypeList[8];   //定义各卡类型
	if( dmc_board_init() <= 0 )      //控制卡的初始化操作
		MessageBox("初始化控制卡失败！","出错");
	dmc_get_CardInfList(&wCardNum, arrdwCardTypeList,arrwCardList );    //获取正在使用的卡号列表
	m_wCard = arrwCardList[1];  

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



void CDMCd1Dlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	CString stStr;
	DWORD dwValue = 0;
	int i = 0;
	double dPos = 0;
	int nRtn = 0;

	CStatic *Static_IN[16]= {(CStatic *)GetDlgItem(IDC_STATIC_IN0),(CStatic *)GetDlgItem(IDC_STATIC_IN1),(CStatic *)GetDlgItem(IDC_STATIC_IN2),(CStatic *)GetDlgItem(IDC_STATIC_IN3),(CStatic *)GetDlgItem(IDC_STATIC_IN4),
		(CStatic *)GetDlgItem(IDC_STATIC_IN5),(CStatic *)GetDlgItem(IDC_STATIC_IN6),(CStatic *)GetDlgItem(IDC_STATIC_IN7),(CStatic *)GetDlgItem(IDC_STATIC_IN8),(CStatic *)GetDlgItem(IDC_STATIC_IN9),(CStatic *)GetDlgItem(IDC_STATIC_IN10),
		(CStatic *)GetDlgItem(IDC_STATIC_IN11),(CStatic *)GetDlgItem(IDC_STATIC_IN12),(CStatic *)GetDlgItem(IDC_STATIC_IN13),(CStatic *)GetDlgItem(IDC_STATIC_IN14),(CStatic *)GetDlgItem(IDC_STATIC_IN15)};

	CStatic *Static_CANIN[16]= {(CStatic *)GetDlgItem(IDC_STATIC_CANIN0),(CStatic *)GetDlgItem(IDC_STATIC_CANIN1),(CStatic *)GetDlgItem(IDC_STATIC_CANIN2),(CStatic *)GetDlgItem(IDC_STATIC_CANIN3),(CStatic *)GetDlgItem(IDC_STATIC_CANIN4),
		(CStatic *)GetDlgItem(IDC_STATIC_CANIN5),(CStatic *)GetDlgItem(IDC_STATIC_CANIN6),(CStatic *)GetDlgItem(IDC_STATIC_CANIN7),(CStatic *)GetDlgItem(IDC_STATIC_CANIN8),(CStatic *)GetDlgItem(IDC_STATIC_CANIN9),(CStatic *)GetDlgItem(IDC_STATIC_CANIN10),
		(CStatic *)GetDlgItem(IDC_STATIC_CANIN11),(CStatic *)GetDlgItem(IDC_STATIC_CANIN12),(CStatic *)GetDlgItem(IDC_STATIC_CANIN13),(CStatic *)GetDlgItem(IDC_STATIC_CANIN14),(CStatic *)GetDlgItem(IDC_STATIC_CANIN15)};

	CButton *Button_out[16]= {(CButton *)GetDlgItem(IDC_BUTTON_OUT0),(CButton *)GetDlgItem(IDC_BUTTON_OUT1),(CButton *)GetDlgItem(IDC_BUTTON_OUT2),(CButton *)GetDlgItem(IDC_BUTTON_OUT3),(CButton *)GetDlgItem(IDC_BUTTON_OUT4),
		(CButton *)GetDlgItem(IDC_BUTTON_OUT5),(CButton *)GetDlgItem(IDC_BUTTON_OUT6),(CButton *)GetDlgItem(IDC_BUTTON_OUT7),(CButton *)GetDlgItem(IDC_BUTTON_OUT8),(CButton *)GetDlgItem(IDC_BUTTON_OUT9),(CButton *)GetDlgItem(IDC_BUTTON_OUT10),
		(CButton *)GetDlgItem(IDC_BUTTON_OUT11),(CButton *)GetDlgItem(IDC_BUTTON_OUT12),(CButton *)GetDlgItem(IDC_BUTTON_OUT13),(CButton *)GetDlgItem(IDC_BUTTON_OUT14),(CButton *)GetDlgItem(IDC_BUTTON_OUT15)};

	CButton *Button_Canout[16]= {(CButton *)GetDlgItem(IDC_BUTTON_CANOUT0),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT1),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT2),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT3),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT4),
		(CButton *)GetDlgItem(IDC_BUTTON_CANOUT5),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT6),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT7),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT8),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT9),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT10),
		(CButton *)GetDlgItem(IDC_BUTTON_CANOUT11),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT12),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT13),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT14),(CButton *)GetDlgItem(IDC_BUTTON_CANOUT15)};


	//读取总线输入输出点数
	WORD wTotalIn =  0;
	WORD wTotalOut = 0;
	nmc_get_total_ionum(m_wCard, &wTotalIn, &wTotalOut);
	stStr.Format("%d", wTotalIn );
	GetDlgItem( IDC_EDIT_CanNodeNum)->SetWindowText( stStr );
	stStr.Format("%d", wTotalIn );
	GetDlgItem( IDC_EDIT_CanStatus)->SetWindowText( stStr );

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
	//读取输入
	dwValue = dmc_read_inport(m_wCard, 0);
	for ( i = 0; i < 16; i++)
	{
		if (!(dwValue & 1) == 0)
		{
			Static_IN[i]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_RED));       
		}
		else
		{
			Static_IN[i]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_GREEN));
		}
		dwValue = dwValue >> 1;
	}

	dwValue = dmc_read_inport(m_wCard, 0);
	dwValue = dwValue >> 16;
	for ( i = 0; i < 16; i++)
	{
		if (!(dwValue & 1) == 0)
		{
			Static_CANIN[i]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_RED));       
		}
		else
		{
			Static_CANIN[i]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_GREEN));
		}
		dwValue = dwValue >> 1;
	}

	//dwValue = dmc_read_inport(m_wCard, 0);
	//dwValue = dwValue >> 16;
	//for ( i = 0; i < 8; i++)
	//{
	//	if (!(dwValue & 1) == 0)
	//	{
	//		Static_CANIN[i+8]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_RED));       
	//	}
	//	else
	//	{
	//		Static_CANIN[i+8]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_GREEN));
	//	}
	//	dwValue = dwValue >> 1;
	//}

	//读取输出
	dwValue = dmc_read_outport(m_wCard, 0);//输出
	for ( i = 0; i < 16; i++)
	{
		if (!(dwValue & 1) == 0)
		{
			Button_out[i]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_RED));       
		}
		else
		{
			Button_out[i]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_GREEN));
		}
		dwValue = dwValue >> 1;
	}	

	dwValue = dmc_read_outport(m_wCard, 0);
	dwValue = dwValue >> 16;
	for ( i = 0; i < 16; i++)
	{
		if (!(dwValue & 1) == 0)
		{
			Button_Canout[i]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_RED));       
		}
		else
		{
			Button_Canout[i]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_GREEN));
		}
		dwValue = dwValue >> 1;
	}

	//dwValue = dmc_read_outport(m_wCard, 0);
	//dwValue = dwValue >> 16;
	//for ( i = 0; i < 8; i++)
	//{
	//	if (!(dwValue & 1) == 0)
	//	{
	//		Button_Canout[i+8]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_RED));       
	//	}
	//	else
	//	{
	//		Button_Canout[i+8]->SetIcon(AfxGetApp()->LoadIcon(IDI_ICON_GREEN));
	//	}
	//	dwValue = dwValue >> 1;
	//}


	CDialog::OnTimer(nIDEvent);
}







void CDMCd1Dlg::OnDestroy() 
{
	// TODO: Add your message handler code here
	dmc_board_close();	//非常之重要，释放其占用的系统资源
	KillTimer( IDC_TIMER );
	CDialog::OnDestroy();

}


void CDMCd1Dlg::OnButtonOut0() 
{
	// TODO: Add your control notification handler code here
	int i = 0;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonOut1() 
{
	// TODO: Add your control notification handler code here
	int i = 1;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonOut2() 
{
	// TODO: Add your control notification handler code here
	int i = 2;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonOut3() 
{
	// TODO: Add your control notification handler code here
	int i = 3;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonOut4() 
{
	// TODO: Add your control notification handler code here
	int i = 4;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}	
}

void CDMCd1Dlg::OnButtonOut5() 
{
	// TODO: Add your control notification handler code here
	int i = 5;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonOut6() 
{
	// TODO: Add your control notification handler code here
	int i = 6;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonOut7() 
{
	// TODO: Add your control notification handler code here
	int i = 7;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}



void CDMCd1Dlg::OnButtonCanout0() 
{
	// TODO: Add your control notification handler code here
	int i = 16;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout1() 
{
	// TODO: Add your control notification handler code here
	int i = 17;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout2() 
{
	// TODO: Add your control notification handler code here
	int i = 18;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout3() 
{
	// TODO: Add your control notification handler code here
	int i = 19;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout4() 
{
	// TODO: Add your control notification handler code here
	int i =20;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout5() 
{
	// TODO: Add your control notification handler code here
	int i = 21;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout6() 
{
	// TODO: Add your control notification handler code here
	int i = 22;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout7() 
{
	// TODO: Add your control notification handler code here
	int i = 23;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout8() 
{
	// TODO: Add your control notification handler code here
	int i = 24;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout9() 
{
	// TODO: Add your control notification handler code here
	int i =25;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout10() 
{
	// TODO: Add your control notification handler code here
	int i = 26;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout11() 
{
	// TODO: Add your control notification handler code here
	int i = 27;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout12() 
{
	// TODO: Add your control notification handler code here
	int i = 28;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout13() 
{
	// TODO: Add your control notification handler code here
	int i = 29;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout14() 
{
	// TODO: Add your control notification handler code here
	int i = 30;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}

void CDMCd1Dlg::OnButtonCanout15() 
{
	// TODO: Add your control notification handler code here
	int i = 31;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}


void CDMCd1Dlg::OnBnClickedButtonExit()
{
	// TODO: 在此添加控件通知处理程序代码
	OnCancel();
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
	dmc_soft_reset(m_wCard);
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


void CDMCd1Dlg::OnBnClickedButtonOut8()
{
	// TODO: 在此添加控件通知处理程序代码
	int i = 8;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}


void CDMCd1Dlg::OnBnClickedButtonOut9()
{
	// TODO: 在此添加控件通知处理程序代码
		int i = 9;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}


void CDMCd1Dlg::OnBnClickedButtonOut10()
{
	// TODO: 在此添加控件通知处理程序代码
		int i =10;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}


void CDMCd1Dlg::OnBnClickedButtonOut11()
{
	// TODO: 在此添加控件通知处理程序代码
		int i = 11;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}


void CDMCd1Dlg::OnBnClickedButtonOut12()
{
	// TODO: 在此添加控件通知处理程序代码
		int i = 12;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}


void CDMCd1Dlg::OnBnClickedButtonOut13()
{
	// TODO: 在此添加控件通知处理程序代码
		int i = 13;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}


void CDMCd1Dlg::OnBnClickedButtonOut14()
{
	// TODO: 在此添加控件通知处理程序代码
		int i = 14;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}


void CDMCd1Dlg::OnBnClickedButtonOut15()
{
	// TODO: 在此添加控件通知处理程序代码
		int i = 15;
	short sTemp = dmc_read_outbit(m_wCard, i);
	if (sTemp == 1)
	{
		dmc_write_outbit(m_wCard, i, 0); 
	}
	else
	{
		dmc_write_outbit(m_wCard, i, 1);
	}
}
