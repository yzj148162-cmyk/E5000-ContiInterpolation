Imports System.Threading
Public Class Form1
    Public g_sCardId As Short
    Private Sub Form1_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        Dim sNum As Short
        Dim arr_nCardtypes(8) As UInteger
        Dim arr_nCardids(8) As UShort
        sNum = dmc_board_init() '获取卡数量

        If (sNum <= 0) Or (sNum > 8) Then             '正常的卡数在1- 8之间
            MsgBox("初始化LTDMC卡失败！", vbOKOnly, "出错")
        End If
        MsgBox("初始化LTDMC卡成功！", vbOKOnly, "成功")
        dmc_get_CardInfList(sNum, arr_nCardtypes, arr_nCardids) '
        g_sCardId = arr_nCardids(0) '
        Dim axis As UShort
        axis = GetAxis() '
        nmc_set_axis_enable(g_sCardId, axis) '
        Timer1.Start() '
        Timer1.Start() '

    End Sub
    Private Function GetHomeDir() As UShort
        Dim usDir As UShort = 0

        If (radioButton8.Checked) Then
            usDir = 0
        ElseIf (radioButton7.Checked) Then
            usDir = 1
        End If

        Return usDir
    End Function

    Private Function GetHomeMode() As UShort
        Dim usMode As UShort = 0
        If (radioButton10.Checked) Then
            usMode = 0
        ElseIf (radioButton9.Checked) Then
            usMode = 1
        ElseIf (radioButton11.Checked) Then
            usMode = 2
        End If

        Return usMode
    End Function
    Private Function GetHomeSpeed() As UShort
        Dim usHomeVel As UShort = 0

        If (radioButton6.Checked) Then
            usHomeVel = 0
        ElseIf (radioButton5.Checked) Then
            usHomeVel = 1
        End If

        Return usHomeVel
    End Function

    Private Function GetAxis() As UShort
        Dim usAxis As UShort = 0

        usAxis = Decimal.ToUInt16(NumericUpDown_Axis.Value)

        Return usAxis

    End Function

    Private Sub Timer1_Tick(sender As Object, e As EventArgs) Handles Timer1.Tick
        Dim dCurrentPos As Double
        Dim dCurrentVel As Double
        Dim errcode As Int32
        Dim Axis_State_machine As Int32

        dmc_read_current_speed_unit(g_sCardId, GetAxis(), dCurrentVel)
        textBox3.Text = dCurrentVel.ToString()
        dmc_get_position_unit(g_sCardId, GetAxis(), dCurrentPos)
        TextBox2.Text = dCurrentPos.ToString()
        If dmc_check_done(g_sCardId, GetAxis()) Then
            textBox1.Text = "停止中"
        Else
            textBox1.Text = "运行中"
        End If

        nmc_get_errcode(g_sCardId, 2, errcode)
        If (errcode = 0) Then
            TextBox5.Text = "EtherCAT总线正常"
            TextBox5.BackColor = Color.Green
        Else
            TextBox5.Text = "EtherCAT总线出错"
            TextBox5.BackColor = Color.Red
        End If

        nmc_get_axis_state_machine(g_sCardId, GetAxis(), Axis_State_machine)
        Select Case (Axis_State_machine) ' 读取指定轴状态机

            Case 0 : TextBox6.Text = "轴处于未启动状态"
                TextBox6.BackColor = Color.Red

            Case 1
                TextBox6.Text = "轴处于启动禁止状态"
                TextBox6.BackColor = Color.Red

            Case 2
                TextBox6.Text = "轴处于准备启动状态"
                TextBox6.BackColor = Color.Red

            Case 3
                TextBox6.Text = "轴处于启动状态"
                TextBox6.BackColor = Color.Red

            Case 4
                TextBox6.Text = "轴处于操作使能状态"
                TextBox6.BackColor = Color.Green

            Case 5
                TextBox6.Text = "轴处于停止状态"
                TextBox6.BackColor = Color.Red

            Case 6
                TextBox6.Text = "轴处于错误触发状态"
                TextBox6.BackColor = Color.Red

            Case 7
                TextBox6.Text = "轴处于错误状态"
                TextBox6.BackColor = Color.Red

        End Select


        Dim IoState As UInt32
        IoState = dmc_axis_io_status(g_sCardId, GetAxis())
        If ((IoState And 2) = 2) Then '检测正限位信号
            Button6.BackColor = Color.Green
        Else
            Button6.BackColor = Color.Red
        End If

        If ((IoState And 4) = 4) Then '检测负限位信号
            Button8.BackColor = Color.Green
        Else
            Button8.BackColor = Color.Red
        End If

        If ((IoState And 16) = 16) Then '检测原点信号
            Button4.BackColor = Color.Green
        Else
            Button4.BackColor = Color.Red
        End If



    End Sub

    Private Sub Btn_StartMove_Click(sender As System.Object, e As System.EventArgs) Handles Btn_StartMove.Click
        Dim Rtn As Int32
        Dim low As Double = 500
        Dim axis As UShort
        axis = GetAxis()

        Rtn = nmc_set_home_profile(g_sCardId, GetAxis(), Decimal.ToInt16(NumericUpDown_HomeMode.Value), Decimal.ToDouble(numericUpDown_LowVel.Value), Decimal.ToDouble(numericUpDown_HighVel.Value),
        Decimal.ToDouble(numericUpDown_Tacc.Value), Decimal.ToDouble(numericUpDown_Tdcc.Value), Decimal.ToDouble(NumericUpDown_Homeoffset.Value))

        If (Rtn = 0) Then
            MessageBox.Show("回零参数配置成功", "注意！", MessageBoxButtons.OK)
        Else
            MessageBox.Show("回零参数配置失败", "注意！", MessageBoxButtons.OK)
        End If
        
        nmc_home_move(g_sCardId, axis)
    End Sub

    Private Sub Btn_TdccStopMove_Click(sender As System.Object, e As System.EventArgs) Handles Btn_TdccStopMove.Click
        Dim sAxis As Short
        Dim Rtn As Integer
        sAxis = GetAxis()
        Rtn = dmc_stop(g_sCardId, sAxis, 0)
    End Sub

    Private Sub Btn_StopMove_Click(sender As System.Object, e As System.EventArgs) Handles Btn_StopMove.Click
        Dim sAxis As UShort
        sAxis = GetAxis()
        dmc_stop(g_sCardId, sAxis, 1)
    End Sub

    Private Sub Btn_ResetPos_Click(sender As System.Object, e As System.EventArgs) Handles Btn_ResetPos.Click
        Dim usAxis As UShort
        usAxis = GetAxis()
        dmc_set_position_unit(g_sCardId, usAxis, 0)
    End Sub

    Private Sub Btn_Close_Click(sender As System.Object, e As System.EventArgs) Handles Btn_Close.Click
        Dim axis As UShort
        axis = GetAxis() '
        nmc_set_axis_disable(g_sCardId, axis) '
        Me.Close()
    End Sub

    Private Sub Form1_FormClosed(sender As System.Object, e As System.Windows.Forms.FormClosedEventArgs) Handles MyBase.FormClosed
        dmc_board_close()
        Dim axis As UShort
        axis = GetAxis() '
        nmc_set_axis_disable(g_sCardId, axis) '
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

        dmc_soft_reset(g_sCardId)
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

    Private Sub Button1_Click(sender As Object, e As EventArgs) Handles Button1.Click
        Dim usAxis As UShort
        Dim dStopVel As Double
        Dim dTacc As Double
        Dim dTdcc As Double
        Dim usAxisIoInMsg_PEL As UShort
        Dim usAxisIoInMsg_NEL As UShort
        Dim usAxisIoInMsg_ORG As UShort
        Dim HomeDir As UShort
        Dim HomeSpeed As UShort
        Dim HomeMode As UShort
        Dim ret As UShort
        HomeDir = GetHomeDir()
        HomeSpeed = GetHomeSpeed()
        HomeMode = GetHomeMode()


        dStopVel = Decimal.ToDouble(numericUpDown_StopVel.Value)
        dTacc = Decimal.ToDouble(numericUpDown_Tacc.Value)
        dTdcc = Decimal.ToDouble(numericUpDown_Tdcc.Value)
        usAxisIoInMsg_PEL = 0   '正限位信号
        usAxisIoInMsg_NEL = 1   '负限位信号
        usAxisIoInMsg_ORG = 2   '原点信号

        '将正限位信号、负限位信号和原点信号分别映射到通用输入口0、1、2
        dmc_set_axis_io_map(g_sCardId, usAxis, usAxisIoInMsg_PEL, 6, 0, 0)
        dmc_set_axis_io_map(g_sCardId, usAxis, usAxisIoInMsg_NEL, 6, 1, 0)
        dmc_set_axis_io_map(g_sCardId, usAxis, usAxisIoInMsg_ORG, 6, 2, 0)
        dmc_set_el_mode(g_sCardId, usAxis, 1, 0, 0) '/设置正、负限位信号低电平有效且遇限位立即停止
        ret = dmc_set_home_pin_logic(g_sCardId, usAxis, 0, 0) '//设置原点低电平有效
        ret = dmc_set_homemode(g_sCardId, usAxis, HomeDir, GetHomeSpeed(), HomeMode, 0) '//设置回零模式
        ret = dmc_set_home_profile_unit(g_sCardId, usAxis, Decimal.ToDouble(numericUpDown_LowVel.Value), Decimal.ToDouble(numericUpDown_HighVel.Value), dTacc, dTdcc) '设置起始速度、运行速度、停止速度、加速时间、减速时间
        ret = dmc_home_move(g_sCardId, usAxis) '//启动回零
    End Sub

    Private Sub radioButton6_CheckedChanged(sender As System.Object, e As System.EventArgs) Handles radioButton6.CheckedChanged

    End Sub
End Class
