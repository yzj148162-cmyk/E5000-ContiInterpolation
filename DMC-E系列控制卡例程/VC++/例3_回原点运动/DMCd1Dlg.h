// DMCd1Dlg.h : header file
//

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
	WORD	m_wAxis;
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
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonStart();
	afx_msg void OnBnClickedButtonStopdec();
	afx_msg void OnBnClickedButtonStopemg();
	afx_msg void OnBnClickedButtonClearpos();
	afx_msg void OnBnClickedButtonExit();
	afx_msg void OnBnClickedButtonHardwarereset();
	afx_msg void OnBnClickedButtonSoftwarereset();

	double m_dSpeedLow;
	double m_dSpeedHigh;
	double m_dAccTime;
	double m_dDecTime;
	double m_dHomeOffset;
	int m_nHomeMode;
	CComboBox m_nCombox_AxisNum;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DMCd1DLG_H__916DC54F_7A09_4953_8006_E6FF4EA47261__INCLUDED_)
