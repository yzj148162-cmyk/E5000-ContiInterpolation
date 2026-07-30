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

	WORD	m_wCard;
	WORD 	m_wAxis;

	double	m_dEquiv;
	double	m_dStartSpeed;
	double	m_dSpeed;
	double	m_dAccTime;
	double	m_dChangeTime;
	double	m_dDecTime;
	double	m_dChangeSpeed;
	double	m_dStopSpeed;
	double	m_dSParaTime;
	int		m_nDir;
	CComboBox	m_nCombox_AxisNum;
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
	afx_msg void OnButtonStartMove();
	afx_msg void OnButtonStop();
	afx_msg void OnButtonSetEquiv();
	afx_msg void OnButtonClear();
	afx_msg void OnButtonExit();
	afx_msg void OnButtonChangeSpeed();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnBnClickedButtonHardwarereset();
	afx_msg void OnBnClickedButtonSoftwarereset();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DMCd1DLG_H__916DC54F_7A09_4953_8006_E6FF4EA47261__INCLUDED_)
