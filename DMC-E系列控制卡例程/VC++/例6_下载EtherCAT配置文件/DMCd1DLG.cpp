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
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CDMCd1Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDMCd1Dlg)
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
	ON_BN_CLICKED(IDC_BUTTON_RESET, OnButtonReset)
	ON_BN_CLICKED(IDC_BUTTON_SOFT_RESET, OnButtonSoftReset)
	ON_BN_CLICKED(IDC_BUTTON_DOWNLOAD, OnButtonDownload)
	ON_BN_CLICKED(IDC_BUTTON_DOWNLOAD_ENI, OnButtonDownloadEni)
	ON_BN_CLICKED(IDC_BUTTON_ORG_RESET, OnButtonOrgReset)
	//}}AFX_MSG_MAP
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

	nmc_set_axis_enable(m_nCard, 255);
}

void CDMCd1Dlg::OnButtonDisable() 
{
	// TODO: Add your control notification handler code here
	UpdateData(true);//刷新参数

	nmc_set_axis_disable(m_nCard, 255);
}

void CDMCd1Dlg::OnButtonReset() 
{
	// TODO: Add your control notification handler code here
	int i = 0;
	GetDlgItem( IDC_STATIC_MSG )->SetWindowText( "请勿操作，总线卡冷复位进行中……" );
	GetDlgItem(IDC_BUTTON_RESET)->EnableWindow(false);
	GetDlgItem(IDC_BUTTON_SOFT_RESET)->EnableWindow(false);
	GetDlgItem(IDC_BUTTON_ORG_RESET)->EnableWindow(false);
	dmc_cool_reset(m_nCard);
	dmc_board_close();
	for ( i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
	{
		Sleep(1000);
	}
	dmc_board_init();
	GetDlgItem( IDC_STATIC_MSG )->SetWindowText( "总线卡冷复位完成,请确认总线状态" );
	GetDlgItem(IDC_BUTTON_RESET)->EnableWindow(true);
	GetDlgItem(IDC_BUTTON_SOFT_RESET)->EnableWindow(true);
	GetDlgItem(IDC_BUTTON_ORG_RESET)->EnableWindow(true);
}

void CDMCd1Dlg::OnButtonSoftReset() 
{
	// TODO: Add your control notification handler code here
	int i = 0;
	GetDlgItem( IDC_STATIC_MSG )->SetWindowText( "请勿操作，总线卡热复位进行中……" );
	GetDlgItem(IDC_BUTTON_RESET)->EnableWindow(false);
	GetDlgItem(IDC_BUTTON_SOFT_RESET)->EnableWindow(false);
	GetDlgItem(IDC_BUTTON_ORG_RESET)->EnableWindow(false);

	dmc_soft_reset(m_nCard);
	dmc_board_close();
	for ( i = 0; i < 8; i++)//总线卡硬件复位耗时15s左右
	{
		Sleep(1000);
	}
	dmc_board_init();
	GetDlgItem( IDC_STATIC_MSG )->SetWindowText( "总线卡热复位完成,请确认总线状态" );
	GetDlgItem(IDC_BUTTON_RESET)->EnableWindow(true);
	GetDlgItem(IDC_BUTTON_SOFT_RESET)->EnableWindow(true);
	GetDlgItem(IDC_BUTTON_ORG_RESET)->EnableWindow(true);
}
















//DEL void CDMCd1Dlg::OnButtonChangepos() 
//DEL {
//DEL 	// TODO: Add your control notification handler code here
//DEL 	CString string;
//DEL 	long ControlWord=0;
//DEL 	m_nAxis = m_combox_axisno.GetCurSel();
//DEL 	nmc_get_axis_contrlword(m_nCard, m_nAxis,&ControlWord);
//DEL    	string.Format("%d", ControlWord );
//DEL 	GetDlgItem( IDC_STATIC_Axis5 )->SetWindowText( string );  
//DEL }




void CDMCd1Dlg::OnButtonDownload() 
{
	// TODO: Add your control notification handler code here
	int32 iresult;

	TCHAR szFilter[] = _T("配置文件(*.ini)|*.ini|所有文件(*.*)|*.*||");   
	// 构造打开文件对话框   
	CFileDialog fileDlg(TRUE, _T("ini"), NULL, 0, szFilter, this);   
	CString strFilePath;
	//文件
	char * pbuffer;
	FILE *pfile;
	uint32 uifilesize;
	BYTE head[3];

	// 显示打开文件对话框   
	if (IDOK == fileDlg.DoModal())   
	{   
		// 如果点击了文件对话框上的“打开”按钮，则将选择的文件路径显示到编辑框里   
		strFilePath = fileDlg.GetPathName();   

		pfile = fopen(strFilePath, "rb");
		if(NULL == pfile)
		{
			return ;
		}
		fseek(pfile, 0, SEEK_END);
		uifilesize = ftell(pfile);

		//判断是否utf8
		fread(head, 1, 3, pfile);
		if(!(head[0]==0xEF && head[1]==0xBB && head[2]==0xBF))
		{
			fseek(pfile, 3, SEEK_SET);
			uifilesize = uifilesize-3;
		}
		else
		{
			fseek(pfile, 0, SEEK_SET);
		}
	
		//读取
		pbuffer = (char *)malloc(uifilesize);
		if(NULL == pbuffer)
		{
			fclose(pfile);
			return ;
		}

		if(fread(pbuffer, 1, uifilesize, pfile) != uifilesize)
		{
			fclose(pfile);
			free(pbuffer);
			return ;
		}
		fclose(pfile);

		WORD filetype =  201;
		iresult = dmc_download_memfile(m_nCard, pbuffer, uifilesize, NULL,filetype);
		if(iresult == 0)
		{
			MessageBox("下载成功！","成功");
		}
		free(pbuffer);	
	} 	
}

void CDMCd1Dlg::OnButtonDownloadEni() 
{
	// TODO: Add your control notification handler code here
	int32 iresult;

	TCHAR szFilter[] = _T("配置文件(*.eni)|*.eni|所有文件(*.*)|*.*||");   
	// 构造打开文件对话框   
	CFileDialog fileDlg(TRUE, _T("eni"), NULL, 0, szFilter, this);   
	CString strFilePath;
	//文件
	char * pbuffer;
	FILE *pfile;
	uint32 uifilesize;
	BYTE head[3];

	// 显示打开文件对话框   
	if (IDOK == fileDlg.DoModal())   
	{   
		// 如果点击了文件对话框上的“打开”按钮，则将选择的文件路径显示到编辑框里   
		strFilePath = fileDlg.GetPathName();   

		pfile = fopen(strFilePath, "rb");
		if(NULL == pfile)
		{
			return ;
		}
		fseek(pfile, 0, SEEK_END);
		uifilesize = ftell(pfile);

		//判断是否utf8
		fread(head, 1, 3, pfile);
		if(!(head[0]==0xEF && head[1]==0xBB && head[2]==0xBF))
		{
			fseek(pfile, 3, SEEK_SET);
			uifilesize = uifilesize-3;
		}
		else
		{
			fseek(pfile, 0, SEEK_SET);
		}

		//读取
		pbuffer = (char *)malloc(uifilesize);
		if(NULL == pbuffer)
		{
			fclose(pfile);
			return ;
		}

		if(fread(pbuffer, 1, uifilesize, pfile) != uifilesize)
		{
			fclose(pfile);
			free(pbuffer);
			return ;
		}
		fclose(pfile);
		WORD filetype =  200;
		iresult = dmc_download_memfile(m_nCard, pbuffer, uifilesize, NULL,filetype);
		if(iresult == 0)
		{
			MessageBox("下载成功！","成功");
		}
		free(pbuffer);	
	} 

}

void CDMCd1Dlg::OnButtonOrgReset() 
{
	// TODO: Add your control notification handler code here
	int i = 0;
	GetDlgItem( IDC_STATIC_MSG )->SetWindowText( "请勿操作，总线卡初始复位进行中……" );
	GetDlgItem(IDC_BUTTON_RESET)->EnableWindow(false);
	GetDlgItem(IDC_BUTTON_SOFT_RESET)->EnableWindow(false);
	GetDlgItem(IDC_BUTTON_ORG_RESET)->EnableWindow(false);

	dmc_original_reset(m_nCard);
	dmc_board_close();
	for ( i = 0; i < 8; i++)//总线卡硬件复位耗时15s左右
	{
		Sleep(1000);
	}
	dmc_board_init();
	GetDlgItem( IDC_STATIC_MSG )->SetWindowText( "总线卡初始复位完成,请确认总线状态" );
	GetDlgItem(IDC_BUTTON_RESET)->EnableWindow(true);
	GetDlgItem(IDC_BUTTON_SOFT_RESET)->EnableWindow(true);
	GetDlgItem(IDC_BUTTON_ORG_RESET)->EnableWindow(true);
}
