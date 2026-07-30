Imports System.Threading
Public Class Form1

    Public g_sCardId As Short

    Private Sub Form1_Load(sender As Object, e As EventArgs) Handles MyBase.Load

        Dim sNum As Short
        Dim arr_nCardtypes(8) As UInt32
        Dim arr_nCardids(8) As UInt16
        sNum = dmc_board_init() '获取卡数量

        If (sNum <= 0) Or (sNum > 8) Then             '正常的卡数在1- 8之间
            MsgBox("初始化LTDMC卡失败！", vbOKOnly, "出错")
        End If
        MsgBox("初始化LTDMC卡成功！", vbOKOnly, "失败")
        dmc_get_CardInfList(sNum, arr_nCardtypes, arr_nCardids) '
        g_sCardId = arr_nCardids(0) '

        Dim axis As UShort
        axis = GetAxis() '
        nmc_set_axis_enable(g_sCardId, axis) '
        Timer1.Start() '

    End Sub

    Private Function GetAxis() As UShort

        Return Decimal.ToUInt16(numericUpDown_Axis.Value) '

    End Function

    Private Sub Timer1_Tick(sender As Object, e As EventArgs) Handles Timer1.Tick
        Dim usAxis As UShort
        Dim dCurrentVel As Double
        Dim dCurrentPos As Double
        Dim dCurrentEncoder As Double
        Dim errcode As Int32
        Dim Axis_State_machine As Int32

        usAxis = GetAxis() '
        dmc_read_current_speed_unit(g_sCardId, usAxis, dCurrentVel) '
        textBox1.Text = Str(dCurrentVel) '
        dmc_get_position_unit(g_sCardId, usAxis, dCurrentPos) '
        textBox2.Text = Str(dCurrentPos) '
        dmc_get_encoder_unit(g_sCardId, usAxis, dCurrentEncoder) '
        textBox3.Text = Str(dCurrentEncoder) '

        If (dmc_check_done(g_sCardId, usAxis) = 0) Then

            textBox4.Text = "运行中" '
        Else
            textBox4.Text = "停止中" '
        End If
        nmc_get_errcode(g_sCardId, 2, errcode)
        If (errcode = 0) Then
            TextBox6.Text = "EtherCAT总线正常"
            TextBox6.BackColor = Color.Green
        Else
            TextBox6.Text = "EtherCAT总线出错"
            TextBox6.BackColor = Color.Red
        End If

        nmc_get_axis_state_machine(g_sCardId, GetAxis(), Axis_State_machine)
        Select Case (Axis_State_machine) ' 读取指定轴状态机

            Case 0 : Machine.Text = "轴处于未启动状态"
                Machine.BackColor = Color.Red

            Case 1
                Machine.Text = "轴处于启动禁止状态"
                Machine.BackColor = Color.Red

            Case 2
                Machine.Text = "轴处于准备启动状态"
                Machine.BackColor = Color.Red

            Case 3
                Machine.Text = "轴处于启动状态"
                Machine.BackColor = Color.Red

            Case 4
                Machine.Text = "轴处于操作使能状态"
                Machine.BackColor = Color.Green

            Case 5
                Machine.Text = "轴处于停止状态"
                Machine.BackColor = Color.Red

            Case 6
                Machine.Text = "轴处于错误触发状态"
                Machine.BackColor = Color.Red

            Case 7
                Machine.Text = "轴处于错误状态"
                Machine.BackColor = Color.Red

        End Select

    End Sub

    Private Sub Btn_StartMove_Click(sender As System.Object, e As System.EventArgs) Handles Btn_StartMove.Click
        Dim usAxis As UShort
        Dim usDist As UShort
        usAxis = GetAxis() '
        dmc_set_profile_unit(g_sCardId,
                usAxis,
                Decimal.ToDouble(numericUpDown_StartSpeed.Value),
                Decimal.ToDouble(numericUpDown_MaxVel.Value),
                Decimal.ToDouble(numericUpDown_Tacc.Value),
                Decimal.ToDouble(numericUpDown_Tdcc.Value),
                Decimal.ToDouble(numericUpDown_StopTime.Value)) '
        dmc_set_s_profile(g_sCardId, usAxis, 0, Decimal.ToDouble(numericUpDown_STime.Value)) '
        usDist = Decimal.ToDouble(numericUpDown_Dist.Value) '
        dmc_pmove_unit(g_sCardId, usAxis, usDist, 0) '
    End Sub

    Private Sub Btn_StopMove_Click(sender As System.Object, e As System.EventArgs) Handles Btn_StopMove.Click
        dmc_stop(g_sCardId, GetAxis(), 1) '
    End Sub

    Private Sub Btn_ResetPos_Click(sender As System.Object, e As System.EventArgs) Handles Btn_ResetPos.Click
        dmc_set_position_unit(g_sCardId, GetAxis(), 0) '
    End Sub

    Private Sub Btn_SetEquiv_Click(sender As System.Object, e As System.EventArgs) Handles Btn_SetEquiv.Click
        Dim Rtn As Integer
        Rtn = dmc_set_equiv(g_sCardId, GetAxis(), Decimal.ToDouble(numericUpDown_Equiv.Value)) '
    End Sub

    Private Sub Btn_Close_Click(sender As System.Object, e As System.EventArgs) Handles Btn_Close.Click
        nmc_set_axis_disable(g_sCardId, 0)
        nmc_set_axis_disable(g_sCardId, 1)
        nmc_set_axis_disable(g_sCardId, 2)
        Me.Close() '
    End Sub

    Private Sub Btn_OnlineConjugation_Click(sender As System.Object, e As System.EventArgs) Handles Btn_OnlineConjugation.Click
        dmc_reset_target_position_unit(g_sCardId, GetAxis(), Decimal.ToDouble(numericUpDown_ChangDist.Value)) '
    End Sub

    Private Sub Btn_OnlineShifting_Click(sender As System.Object, e As System.EventArgs) Handles Btn_OnlineShifting.Click
        Dim Rtn As Integer
        Rtn = dmc_change_speed_unit(g_sCardId, GetAxis(), Decimal.ToDouble(numericUpDown_ChangeVel.Value), Decimal.ToDouble(numericUpDown_ChangeTime.Value)) '
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

    Private Sub Form1_FormClosed(sender As System.Object, e As System.Windows.Forms.FormClosedEventArgs) Handles MyBase.FormClosed
        dmc_board_close()
        Dim axis As UShort
        axis = GetAxis() '
        nmc_set_axis_disable(g_sCardId, axis) '
    End Sub


End Class
