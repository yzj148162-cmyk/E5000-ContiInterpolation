Imports System.Threading

Public Class Form3
    Public g_nCardID As Short = 0

    Private Sub Form1_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        Dim sNum As Short
        Dim arr_nCardtypes(8) As UInteger
        Dim arr_nCardids(8) As UShort
        sNum = dmc_board_init() '获取卡数量

        If (sNum <= 0) Or (sNum > 8) Then             '正常的卡数在1- 8之间
            MsgBox("初始化LTDMC卡失败！", vbOKOnly, "出错")
        End If
        dmc_get_CardInfList(sNum, arr_nCardtypes, arr_nCardids) '
        g_nCardID = arr_nCardids(1) '
        Timer1.Start() '
    End Sub

    Private Sub Timer1_Tick(sender As Object, e As EventArgs) Handles Timer1.Tick
        Dim nErrcode As Int32
        nmc_get_errcode(g_nCardID, 2, nErrcode)
        If (nErrcode = 0) Then
            TextBox3.Text = "EtherCAT总线正常"
            TextBox3.BackColor = Color.Green
        Else
            TextBox3.Text = "EtherCAT总线出错"
            TextBox3.BackColor = Color.Red
        End If
        Dim nTotalIn As Integer
        Dim nTotalout As Integer
        Dim nRtn As Integer

        nRtn = nmc_get_total_ionum(g_nCardID, nTotalIn, nTotalout)
        textBox1.Text = nTotalIn.ToString()
        textBox2.Text = nTotalout.ToString()

        '显示输入的状态()
        If (dmc_read_inbit(g_nCardID, 0) = 0) Then
            IN0.BackColor = Color.Green
        Else
            IN0.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 1) = 0) Then
            IN1.BackColor = Color.Green
        Else
            IN1.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 2) = 0) Then
            IN2.BackColor = Color.Green
        Else
            IN2.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 3) = 0) Then
            IN3.BackColor = Color.Green
        Else
            IN3.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 4) = 0) Then
            IN4.BackColor = Color.Green
        Else
            IN4.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 5) = 0) Then
            IN5.BackColor = Color.Green
        Else
            IN5.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 6) = 0) Then
            IN6.BackColor = Color.Green
        Else
            IN6.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 7) = 0) Then
            IN7.BackColor = Color.Green
        Else
            IN7.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 8) = 0) Then
            IN8.BackColor = Color.Green
        Else
            IN8.BackColor = Color.Red
        End If


        If (dmc_read_inbit(g_nCardID, 9) = 0) Then
            IN9.BackColor = Color.Green
        Else
            IN9.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 10) = 0) Then
            IN10.BackColor = Color.Green
        Else
            IN10.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 11) = 0) Then
            IN11.BackColor = Color.Green
        Else
            IN11.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 12) = 0) Then
            IN12.BackColor = Color.Green
        Else
            IN12.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 13) = 0) Then
            IN13.BackColor = Color.Green
        Else
            IN13.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 14) = 0) Then
            IN14.BackColor = Color.Green
        Else
            IN14.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 15) = 0) Then
            IN15.BackColor = Color.Green
        Else
            IN15.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 16) = 0) Then
            IN16.BackColor = Color.Green
        Else
            IN16.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 17) = 0) Then
            IN17.BackColor = Color.Green
        Else
            IN17.BackColor = Color.Red
        End If

        If (dmc_read_inbit(g_nCardID, 18) = 0) Then
            IN18.BackColor = Color.Green
        Else
            IN18.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 19) = 0) Then
            IN19.BackColor = Color.Green
        Else
            IN19.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 20) = 0) Then
            IN20.BackColor = Color.Green
        Else
            IN20.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 21) = 0) Then
            IN21.BackColor = Color.Green
        Else
            IN21.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 22) = 0) Then
            IN22.BackColor = Color.Green
        Else
            IN22.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 23) = 0) Then
            IN23.BackColor = Color.Green
        Else
            IN23.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 24) = 0) Then
            IN24.BackColor = Color.Green
        Else
            IN24.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 25) = 0) Then
            IN25.BackColor = Color.Green
        Else
            IN25.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 26) = 0) Then
            IN26.BackColor = Color.Green
        Else
            IN26.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 27) = 0) Then
            IN27.BackColor = Color.Green
        Else
            IN27.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 28) = 0) Then
            IN28.BackColor = Color.Green
        Else
            IN28.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 29) = 0) Then
            IN29.BackColor = Color.Green
        Else
            IN29.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 30) = 0) Then
            IN30.BackColor = Color.Green
        Else
            IN30.BackColor = Color.Red
        End If
        If (dmc_read_inbit(g_nCardID, 31) = 0) Then
            IN31.BackColor = Color.Green
        Else
            IN31.BackColor = Color.Red
        End If
        ' 显示输出的状态

        If (dmc_read_outbit(g_nCardID, 0) = 0) Then
            OUT0.BackColor = Color.Green
        Else
            OUT0.BackColor = Color.Red
        End If


        If (dmc_read_outbit(g_nCardID, 1) = 0) Then
            OUT1.BackColor = Color.Green
        Else
            OUT1.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 2) = False) Then
            OUT2.BackColor = Color.Green
        Else
            OUT2.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 3) = 0) Then
            OUT3.BackColor = Color.Green
        Else
            OUT3.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 4) = 0) Then
            OUT4.BackColor = Color.Green
        Else
            OUT4.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 5) = 0) Then
            OUT5.BackColor = Color.Green
        Else
            OUT5.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 6) = False) Then
            OUT6.BackColor = Color.Green
        Else
            OUT6.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 7) = 0) Then
            OUT7.BackColor = Color.Green
        Else
            OUT7.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 8) = 0) Then
            OUT8.BackColor = Color.Green
        Else
            OUT8.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 9) = 0) Then
            OUT9.BackColor = Color.Green
        Else
            OUT9.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 10) = 0) Then
            OUT10.BackColor = Color.Green
        Else
            OUT10.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 11) = 0) Then
            OUT11.BackColor = Color.Green
        Else
            OUT11.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 12) = 0) Then
            OUT12.BackColor = Color.Green
        Else
            OUT12.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 13) = 0) Then
            OUT13.BackColor = Color.Green
        Else
            OUT13.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 14) = 0) Then
            OUT14.BackColor = Color.Green
        Else
            OUT14.BackColor = Color.Red
        End If

        If (dmc_read_outbit(g_nCardID, 15) = 0) Then
            OUT15.BackColor = Color.Green
        Else
            OUT15.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 16) = 0) Then
            OUT16.BackColor = Color.Green
        Else
            OUT16.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 17) = 0) Then
            OUT17.BackColor = Color.Green
        Else
            OUT17.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 18) = 0) Then
            OUT18.BackColor = Color.Green
        Else
            OUT18.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 19) = 0) Then
            OUT19.BackColor = Color.Green
        Else
            OUT19.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 20) = 0) Then
            OUT20.BackColor = Color.Green
        Else
            OUT20.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 21) = 0) Then
            OUT21.BackColor = Color.Green
        Else
            OUT21.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 22) = 0) Then
            OUT22.BackColor = Color.Green
        Else
            OUT22.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 23) = 0) Then
            OUT23.BackColor = Color.Green
        Else
            OUT23.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 24) = 0) Then
            OUT24.BackColor = Color.Green
        Else
            OUT24.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 25) = 0) Then
            OUT25.BackColor = Color.Green
        Else
            OUT25.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 26) = 0) Then
            OUT26.BackColor = Color.Green
        Else
            OUT26.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 27) = 0) Then
            OUT27.BackColor = Color.Green
        Else
            OUT27.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 28) = 0) Then
            OUT28.BackColor = Color.Green
        Else
            OUT28.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 29) = 0) Then
            OUT29.BackColor = Color.Green
        Else
            OUT29.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 30) = 0) Then
            OUT30.BackColor = Color.Green
        Else
            OUT30.BackColor = Color.Red
        End If
        If (dmc_read_outbit(g_nCardID, 31) = 0) Then
            OUT31.BackColor = Color.Green
        Else
            OUT31.BackColor = Color.Red
        End If
    End Sub

    Private Sub OUT0_Click(sender As Object, e As EventArgs) Handles OUT0.Click
        Dim nNum As Int32 = 0
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT1_Click(sender As Object, e As EventArgs) Handles OUT1.Click
        Dim nNum As Int32 = 1
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT2_Click(sender As Object, e As EventArgs) Handles OUT2.Click
        Dim nNum As Int32 = 2
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT3_Click(sender As Object, e As EventArgs) Handles OUT3.Click
        Dim nNum As Int32 = 3
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT4_Click_1(sender As System.Object, e As System.EventArgs) Handles OUT4.Click
        Dim nNum As Int32 = 4
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT5_Click_1(sender As System.Object, e As System.EventArgs) Handles OUT5.Click
        Dim nNum As Int32 = 5
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT6_Click_1(sender As System.Object, e As System.EventArgs) Handles OUT6.Click
        Dim nNum As Int32 = 6
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT7_Click_1(sender As System.Object, e As System.EventArgs) Handles OUT7.Click
        Dim nNum As Int32 = 7
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub


    Private Sub OUT8_Click_1(sender As System.Object, e As System.EventArgs) Handles OUT8.Click
        Dim nNum As Int32 = 8
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT9_Click(sender As System.Object, e As System.EventArgs) Handles OUT9.Click
        Dim nNum As Int32 = 9
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT11_Click(sender As System.Object, e As System.EventArgs) Handles OUT11.Click
        Dim nNum As Int32 = 11
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT10_Click(sender As System.Object, e As System.EventArgs) Handles OUT10.Click
        Dim nNum As Int32 = 10
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT12_Click(sender As System.Object, e As System.EventArgs) Handles OUT12.Click
        Dim nNum As Int32 = 12
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT13_Click(sender As System.Object, e As System.EventArgs) Handles OUT13.Click
        Dim nNum As Int32 = 13
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT14_Click(sender As System.Object, e As System.EventArgs) Handles OUT14.Click
        Dim nNum As Int32 = 14
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT15_Click(sender As System.Object, e As System.EventArgs) Handles OUT15.Click
        Dim nNum As Int32 = 15
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT16_Click(sender As System.Object, e As System.EventArgs) Handles OUT16.Click
        Dim nNum As Int32 = 16
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT17_Click(sender As System.Object, e As System.EventArgs) Handles OUT17.Click
        Dim nNum As Int32 = 17
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT18_Click(sender As System.Object, e As System.EventArgs) Handles OUT18.Click
        Dim nNum As Int32 = 18
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT19_Click(sender As System.Object, e As System.EventArgs) Handles OUT19.Click
        Dim nNum As Int32 = 19
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT20_Click(sender As System.Object, e As System.EventArgs) Handles OUT20.Click
        Dim nNum As Int32 = 20
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If

    End Sub

    Private Sub OUT21_Click(sender As System.Object, e As System.EventArgs) Handles OUT21.Click
        Dim nNum As Int32 = 21
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT22_Click(sender As System.Object, e As System.EventArgs) Handles OUT22.Click
        Dim nNum As Int32 = 22
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT23_Click(sender As System.Object, e As System.EventArgs) Handles OUT23.Click
        Dim nNum As Int32 = 23
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub Btn_HandReset_Click(sender As System.Object, e As System.EventArgs) Handles Btn_HandReset.Click
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡硬件复位进行中……")
        Btn_HandReset.Enabled = False
        Btn_SoftReset.Enabled = False

        dmc_board_reset()
        dmc_board_close()

        For i As Integer = 0 To 15 Step 1 '总线卡硬件复位耗时15s左右
            Application.DoEvents()
            Thread.Sleep(1000)
        Next

        dmc_board_init()
        RichTextBox1.AppendText("总线卡硬件复位完成,请确认总线状态")
        Btn_HandReset.Enabled = True
        Btn_SoftReset.Enabled = True
        Btn_HandReset.Focus()
    End Sub

    Private Sub Btn_SoftReset_Click(sender As System.Object, e As System.EventArgs) Handles Btn_SoftReset.Click
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡软件复位进行中……")
        Btn_HandReset.Enabled = False
        Btn_SoftReset.Enabled = False

        dmc_soft_reset(g_nCardID)
        dmc_board_close()

        For i As Integer = 0 To 15 Step 1 '总线卡硬件复位耗时15s左右
            Application.DoEvents()
            Thread.Sleep(1000)
        Next

        dmc_board_init()
        RichTextBox1.AppendText("总线卡软件复位完成,请确认总线状态")
        Btn_HandReset.Enabled = True
        Btn_SoftReset.Enabled = True
        Btn_SoftReset.Focus()
    End Sub

    Private Sub Btn_Close_Click(sender As System.Object, e As System.EventArgs) Handles Btn_Close.Click
        Me.Close()

    End Sub

    Private Sub OUT24_Click(sender As System.Object, e As System.EventArgs) Handles OUT24.Click
        Dim nNum As Int32 = 24
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT25_Click(sender As System.Object, e As System.EventArgs) Handles OUT25.Click
        Dim nNum As Int32 = 25
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT26_Click(sender As System.Object, e As System.EventArgs) Handles OUT26.Click
        Dim nNum As Int32 = 26
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT27_Click(sender As System.Object, e As System.EventArgs) Handles OUT27.Click
        Dim nNum As Int32 = 27
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT28_Click(sender As System.Object, e As System.EventArgs) Handles OUT28.Click
        Dim nNum As Int32 = 28
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT29_Click(sender As System.Object, e As System.EventArgs) Handles OUT29.Click
        Dim nNum As Int32 = 29
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT30_Click(sender As System.Object, e As System.EventArgs) Handles OUT30.Click
        Dim nNum As Int32 = 30
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub

    Private Sub OUT31_Click(sender As System.Object, e As System.EventArgs) Handles OUT31.Click
        Dim nNum As Int32 = 31
        If (dmc_read_outbit(g_nCardID, nNum) = 0) Then
            dmc_write_outbit(g_nCardID, nNum, 1)
        Else
            dmc_write_outbit(g_nCardID, nNum, 0)
        End If
    End Sub
End Class
