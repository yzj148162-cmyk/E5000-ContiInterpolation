Imports System.Threading

Public Class Form1
    Public g_usCardID As Short

    Private Sub Form1_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        Dim sNum As Short
        Dim arr_nCardtypes(8) As UInteger
        Dim arr_usCardids(8) As UShort
        sNum = dmc_board_init() '获取卡数量

        If (sNum <= 0) Or (sNum > 8) Then             '正常的卡数在1- 8之间
            MsgBox("初始化LTDMC卡失败！", vbOKOnly, "出错")
        Else
            MsgBox("初始化LTDMC卡成功！", vbOKOnly, "OK")
        End If
        dmc_get_CardInfList(sNum, arr_nCardtypes, arr_usCardids) '
        g_usCardID = arr_usCardids(1) '
        Dim axis As UShort
        axis = GetAxis() '
        nmc_set_axis_enable(g_usCardID, axis) '
        Timer1.Start() '
    End Sub

    Private Function GetAxis() As UShort

        Return Decimal.ToUInt16(numericUpDown_Axis.Value) '

    End Function


    Private Sub Timer1_Tick(sender As Object, e As EventArgs) Handles Timer1.Tick
        Dim usAxis As UShort
        Dim dcurrent_V As Double
        Dim dpos As Double
        Dim np As Double
        Dim errcode As Int32
        Dim Axis_State_machine As Int32

        usAxis = GetAxis() '获取轴号
        dmc_read_current_speed_unit(g_usCardID, usAxis, dcurrent_V) '获取速度
        textBox1.Text = Str(dcurrent_V) '
        dmc_get_position_unit(g_usCardID, usAxis, dpos) '获取位置
        textBox2.Text = Str(dpos) '
        dmc_get_encoder_unit(g_usCardID, usAxis, np) ' 获取位置
        textBox3.Text = Str(np) '

        If (dmc_check_done(g_usCardID, usAxis) = 0) Then
            textBox4.BackColor = Color.Green
            textBox4.Text = "运行中" '
        Else
            textBox4.BackColor = Color.Red
            textBox4.Text = "停止中" '
        End If

        nmc_get_errcode(g_usCardID, 2, errcode)
        If (errcode = 0) Then
            TextBox6.Text = "EtherCAT总线正常"
            TextBox6.BackColor = Color.Green
        Else
            TextBox6.Text = "EtherCAT总线出错"
            TextBox6.BackColor = Color.Red
        End If

        nmc_get_axis_state_machine(g_usCardID, GetAxis(), Axis_State_machine)
        Select Case (Axis_State_machine) ' 读取指定轴状态机

            Case 0 : TextBox5.Text = "轴处于未启动状态"
                TextBox5.BackColor = Color.Red

            Case 1
                TextBox5.Text = "轴处于启动禁止状态"
                TextBox5.BackColor = Color.Red

            Case 2
                TextBox5.Text = "轴处于准备启动状态"
                TextBox5.BackColor = Color.Red

            Case 3
                TextBox5.Text = "轴处于启动状态"
                TextBox5.BackColor = Color.Red

            Case 4
                TextBox5.Text = "轴处于操作使能状态"
                TextBox5.BackColor = Color.Green

            Case 5
                TextBox5.Text = "轴处于停止状态"
                TextBox5.BackColor = Color.Red

            Case 6
                TextBox5.Text = "轴处于错误触发状态"
                TextBox5.BackColor = Color.Red

            Case 7
                TextBox5.Text = "轴处于错误状态"
                TextBox5.BackColor = Color.Red

        End Select


    End Sub


    Private Sub Form1_FormClosing(sender As Object, e As FormClosingEventArgs) Handles MyBase.FormClosing
        Timer1.Stop()
        dmc_board_close()
    End Sub

    Private Sub Btn_changevel_Click(sender As System.Object, e As System.EventArgs) Handles Btn_changevel.Click
        dmc_change_speed_unit(g_usCardID, GetAxis(), Decimal.ToDouble(numericUpDown_ChangeVel.Value), Decimal.ToDouble(numericUpDown_ChangeTime.Value)) '在线变速
    End Sub



    Private Sub Btn_StartMove_Click(sender As System.Object, e As System.EventArgs) Handles Btn_StartMove.Click
        Dim usAxis As UShort
        Dim usDir As UShort
        usAxis = GetAxis() ' 获取轴号
        dmc_set_profile_unit(g_usCardID,
                usAxis,
                Decimal.ToDouble(numericUpDown_StartSpeed.Value),
                Decimal.ToDouble(numericUpDown_MaxVel.Value),
                Decimal.ToDouble(numericUpDown_Tacc.Value),
                Decimal.ToDouble(numericUpDown_Tdcc.Value),
                Decimal.ToDouble(numericUpDown_StopTime.Value)) '
        dmc_set_s_profile(g_usCardID, usAxis, 0, Decimal.ToDouble(numericUpDown_STime.Value)) ' 设置运动参数
        If radioButton8.Checked Then
            usDir = 1 '  获取方向
        Else
            usDir = 0
        End If
        dmc_vmove(g_usCardID, usAxis, usDir) '连续运动
    End Sub

    Private Sub Btn_PosRest_Click(sender As System.Object, e As System.EventArgs) Handles Btn_PosRest.Click
        dmc_set_position_unit(g_usCardID, GetAxis(), 0) '位置设置成0
    End Sub

    Private Sub Btn_SetEquiv_Click(sender As System.Object, e As System.EventArgs) Handles Btn_SetEquiv.Click
        dmc_set_equiv(g_usCardID, GetAxis(), Decimal.ToDouble(numericUpDown_Equiv.Value)) '设置脉冲当量
    End Sub

    Private Sub Btn_Close_Click(sender As System.Object, e As System.EventArgs) Handles Btn_Close.Click
        nmc_set_axis_disable(g_usCardID, 0)
        nmc_set_axis_disable(g_usCardID, 1)
        nmc_set_axis_disable(g_usCardID, 2)
        Me.Close() ' 关闭程序
    End Sub

    Private Sub Btn_StopVel_Click(sender As System.Object, e As System.EventArgs) Handles Btn_StopVel.Click
        Dim usaxis As UShort
        usaxis = GetAxis() '
        dmc_stop(g_usCardID, usaxis, 1)
    End Sub

    Private Sub Form1_FormClosed(sender As System.Object, e As System.Windows.Forms.FormClosedEventArgs) Handles MyBase.FormClosed
        dmc_board_close()
        Dim axis As UShort
        axis = GetAxis() '
        nmc_set_axis_disable(g_usCardID, axis) '
    End Sub

    Private Sub Btn_HardReset_Click(sender As System.Object, e As System.EventArgs) Handles Btn_HardReset.Click
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡硬件复位进行中……")
        Btn_HardReset.Enabled = False
        Btn_SoftReset.Enabled = False

        dmc_board_reset()
        dmc_board_close()

        For i As Integer = 0 To 15 Step 1 '总线卡硬件复位耗时15s左右
            Application.DoEvents()
            Thread.Sleep(1000)
        Next

        dmc_board_init()
        RichTextBox1.AppendText("总线卡硬件复位完成,请确认总线状态")
        Btn_HardReset.Enabled = True
        Btn_SoftReset.Enabled = True
        Btn_HardReset.Focus()
    End Sub

    Private Sub Btn_SoftReset_Click(sender As System.Object, e As System.EventArgs) Handles Btn_SoftReset.Click
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡软件复位进行中……")
        Btn_HardReset.Enabled = False
        Btn_SoftReset.Enabled = False

        dmc_soft_reset(g_usCardID)
        dmc_board_close()

        For i As Integer = 0 To 15 Step 1 '总线卡硬件复位耗时15s左右
            Application.DoEvents()
            Thread.Sleep(1000)
        Next

        dmc_board_init()
        RichTextBox1.AppendText("总线卡软件复位完成,请确认总线状态")
        Btn_HardReset.Enabled = True
        Btn_SoftReset.Enabled = True
        Btn_SoftReset.Focus()
    End Sub
End Class
