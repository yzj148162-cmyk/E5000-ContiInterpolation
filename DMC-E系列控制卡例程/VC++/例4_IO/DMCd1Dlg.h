// DMCd1Dlg.h : header file
//

#include "afxwin.h"
#if !defined(AFX_DMCd1DLG_H__916DC54F_7A09_4953_8006_E6FF4EA47261__INCLUDED_)
#define AFX_DMCd1DLG_H__916DC54F_7A09_4953_8006_E6FF4EA47261__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CDMCd1Dlg dialog

class CDMCd1Dlg : public CDialog
{
// Construction
public:
	CDMCd1Dlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	//{{AFX_DATA(CDMCd1Dlg)
	enum { IDD = IDD_DMCd1_DIALOG };
	WORD	m_wCard;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDMCd1Dlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CDMCd1Dlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnDestroy();


	afx_msg void OnButtonOut0();
	afx_msg void OnButtonOut1();
	afx_msg void OnButtonOut2();
	afx_msg void OnButtonOut3();
	afx_msg void OnButtonOut4();
	afx_msg void OnButtonOut5();
	afx_msg void OnButtonOut6();
	afx_msg void OnButtonOut7();
	afx_msg void OnBnClickedButtonOut8();
	afx_msg void OnBnClickedButtonOut9();
	afx_msg void OnBnClickedButtonOut10();
	afx_msg void OnBnClickedButtonOut11();
	afx_msg void OnBnClickedButtonOut12();
	afx_msg void OnBnClickedButtonOut13();
	afx_msg void OnBnClickedButtonOut14();
	afx_msg void OnBnClickedButtonOut15();



	afx_msg void OnButtonCanout0();
	afx_msg void OnButtonCanout1();
	afx_msg void OnButtonCanout2();
	afx_msg void OnButtonCanout3();
	afx_msg void OnButtonCanout4();
	afx_msg void OnButtonCanout5();
	afx_msg void OnButtonCanout6();
	afx_msg void OnButtonCanout7();
	afx_msg void OnButtonCanout8();
	afx_msg void OnButtonCanout9();
	afx_msg void OnButtonCanout10();
	afx_msg void OnButtonCanout11();
	afx_msg void OnButtonCanout12();
	afx_msg void OnButtonCanout13();
	afx_msg void OnButtonCanout14();
	afx_msg void OnButtonCanout15();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnBnClickedButtonExit();
	afx_msg void OnBnClickedButtonHardwarereset();
	afx_msg void OnBnClickedButtonSoftwarereset();

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DMCd1DLG_H__916DC54F_7A09_4953_8006_E6FF4EA47261__INCLUDED_)
