Imports System.Threading

Public Class Form1
    Public g_sCardId As UShort
    Private Sub Form1_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        Dim nNum As Short
        Dim arr_nCardtypes(8) As UInteger
        Dim arr_nCardids(8) As UShort
        nNum = dmc_board_init() '获取卡数量

        If (nNum <= 0) Or (nNum > 8) Then             '正常的卡数在1- 8之间
            MsgBox("初始化LTDMC卡失败！", vbOKOnly, "出错")
        Else
            MsgBox("初始化LTDMC卡成功！", vbOKOnly, "OK")
        End If

        dmc_get_CardInfList(nNum, arr_nCardtypes, arr_nCardids) '

        g_sCardId = arr_nCardids(1) '


        Dim usTotalSlaves As UShort
        nmc_get_total_slaves(g_sCardId, 2, usTotalSlaves)
        Dim nItem As UInteger = 1
        If (usTotalSlaves > 1) Then
            For nItem = 1 To usTotalSlaves - 1
                ComboBox2.Items.Add(nItem)
            Next
        End If

        ComboBox1.SelectedIndex = 0
        ComboBox2.SelectedIndex = 0
        Timer1.Start() '

    End Sub
    Private Function GetAxis() As UShort
        Return Decimal.ToUInt16(ComboBox2.Text) '
    End Function

    Private Sub Button3_Click(sender As Object, e As EventArgs) Handles Button3.Click
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡硬件复位进行中……")
        Button3.Enabled = False
        Button4.Enabled = False

        dmc_board_reset()
        dmc_board_close()

        For i As Integer = 0 To 15 Step 1 '总线卡硬件复位耗时15s左右
            Application.DoEvents()
            Thread.Sleep(1000)
        Next

        dmc_board_init()
        RichTextBox1.AppendText("总线卡硬件复位完成,请确认总线状态")
        Button3.Enabled = True
        Button4.Enabled = True
        Button3.Focus()
    End Sub

    Private Sub Button4_Click(sender As Object, e As EventArgs) Handles Button4.Click
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡软件复位进行中……")
        Button3.Enabled = False
        Button4.Enabled = False

        Dim dt_start As DateTime
        Dim sRtn As Short
        Dim err As UShort
        dt_start = DateTime.Now
        sRtn = dmc_soft_reset(g_sCardId)
        If sRtn <> 0 Then
            MessageBox.Show("dmc_soft_reset == " + sRtn.ToString())
            Return
        End If
        While True
            If (DateTime.Now - dt_start).TotalMilliseconds >= 10000.0 Then '设置超时退出，防止死循环
                MessageBox.Show("总线复位失败!")
                Button3.Enabled = True
                Button4.Enabled = True
                Button4.Focus()
                Return
            End If

            sRtn = nmc_get_errcode(g_sCardId, 2, err)
            If sRtn = 0 Then
                If err = 0 Then '总线复位正常完成
                    Exit While
                End If
            Else
                MessageBox.Show("nmc_get_errcode == " + sRtn.ToString())
                Button3.Enabled = True
                Button4.Enabled = True
                Button4.Focus()
                Return
            End If
            Application.DoEvents()
        End While
        RichTextBox1.AppendText("总线卡软件复位完成,请确认总线状态")
        Button3.Enabled = True
        Button4.Enabled = True
        Button4.Focus()
    End Sub

    Private Sub Timer1_Tick(sender As Object, e As EventArgs) Handles Timer1.Tick
        Dim errcode As Int32
        Dim Axis_State_machine As Int32

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
            Case 0 : TextBox12.Text = "轴处于未启动状态"
                TextBox12.BackColor = Color.Red
            Case 1
                TextBox12.Text = "轴处于启动禁止状态"
                TextBox12.BackColor = Color.Red
            Case 2
                TextBox12.Text = "轴处于准备启动状态"
                TextBox12.BackColor = Color.Red
            Case 3
                TextBox12.Text = "轴处于启动状态"
                TextBox12.BackColor = Color.Red
            Case 4
                TextBox12.Text = "轴处于操作使能状态"
                TextBox12.BackColor = Color.Green
            Case 5
                TextBox12.Text = "轴处于停止状态"
                TextBox12.BackColor = Color.Red
            Case 6
                TextBox12.Text = "轴处于错误触发状态"
                TextBox12.BackColor = Color.Red
            Case 7
                TextBox12.Text = "轴处于错误状态"
                TextBox12.BackColor = Color.Red
        End Select

        Dim usTotalSlaves As UShort
        nmc_get_total_slaves(g_sCardId, 2, usTotalSlaves)
        TextBox16.Text = usTotalSlaves.ToString()

        Dim TotalAxises As UInt32
        nmc_get_total_axes(g_sCardId, TotalAxises)
        TextBox15.Text = TotalAxises.ToString()

        Dim ethIn As UShort
        Dim ethOut As UShort
        nmc_get_total_ionum(g_sCardId, ethIn, ethOut)
        TextBox17.Text = ethIn.ToString()
        TextBox18.Text = ethOut.ToString()

        Dim TotalADIn As UShort
        Dim TotalDAOut As UShort
        nmc_get_total_adcnum(g_sCardId, TotalADIn, TotalDAOut)
        TextBox19.Text = TotalADIn.ToString()
        TextBox20.Text = TotalDAOut.ToString()

        Dim TolIn As UShort
        Dim TolOut As UShort
        dmc_get_total_ionum(g_sCardId, TolIn, TolOut)
        TextBox14.Text = TolIn.ToString()
        TextBox13.Text = TolOut.ToString()
    End Sub

    Private Sub Button1_Click(sender As Object, e As EventArgs) Handles Button1.Click
        Dim Read_Value As Int32
        Dim usRtn As UShort
        Dim strStrStrsSrA As String
        usRtn = nmc_get_node_od(g_sCardId, 2, Convert.ToUInt16(TextBox1.Text), UInt16.Parse(TextBox2.Text, System.Globalization.NumberStyles.HexNumber), UInt16.Parse(TextBox3.Text, System.Globalization.NumberStyles.HexNumber), Convert.ToUInt16(ComboBox1.Text), Read_Value)
        If (usRtn = 0) Then
            strStrStrsSrA = Read_Value.ToString("x8") '10进制转化为16进制
            'int b = Int32.Parse(textBox8.Text, System.Globalization.NumberStyles.HexNumber);//16进制转化为10进制
            'String strStrStrsSrA = Read_Value.ToString("x8");//10进制转化为16进制
            TextBox4.Text = "0x" + strStrStrsSrA + "(" + Read_Value.ToString() + ")"
            MessageBox.Show("对象字典读取成功", "提示", MessageBoxButtons.OK)
        Else
            TextBox4.Text = "0"
            MessageBox.Show("对象字典读取失败！！！错误码是 " + usRtn.ToString(), "提示", MessageBoxButtons.OK)
        End If
    End Sub

    Private Sub Button2_Click(sender As Object, e As EventArgs) Handles Button2.Click
        Dim lWriteValue As Long
        Dim usRtn As UShort
        'Dim strStrStrsSrA As String
        lWriteValue = Convert.ToInt64(TextBox4.Text)
        usRtn = nmc_set_node_od(g_sCardId, 2, Convert.ToUInt16(TextBox1.Text), UInt16.Parse(TextBox2.Text, System.Globalization.NumberStyles.HexNumber), UInt16.Parse(TextBox3.Text, System.Globalization.NumberStyles.HexNumber), Convert.ToUInt16(ComboBox1.Text), lWriteValue)
        If (usRtn = 0) Then
            'int b = Int32.Parse(textBox8.Text, System.Globalization.NumberStyles.HexNumber);//16进制转化为10进制
            'String strStrStrsSrA = Read_Value.ToString("x8");//10进制转化为16进制                
            MessageBox.Show("对象字典设置成功", "提示", MessageBoxButtons.OK)
        Else
            MessageBox.Show("对象字典设置失败！！！错误码是 " + usRtn.ToString(), "提示", MessageBoxButtons.OK)
            'int b = Int32.Parse(textBox8.Text, System.Globalization.NumberStyles.HexNumber);//16进制转化为10进制
            'String strStrStrsSrA = Read_Value.ToString("x8");//10进制转化为16进制            
        End If
    End Sub

    Private Sub Form1_FormClosing(sender As Object, e As FormClosingEventArgs) Handles MyBase.FormClosing
        Timer1.Stop()
        dmc_board_close()
    End Sub

    Private Sub Button5_Click(sender As Object, e As EventArgs) Handles Button5.Click
        Dim usSlaveAddress As UShort
        Dim usSubSlaveAddress As UShort
        nmc_get_axis_node_address(g_sCardId, GetAxis(), usSlaveAddress, usSubSlaveAddress)
        TextBox6.Text = usSlaveAddress.ToString()
        TextBox7.Text = usSubSlaveAddress.ToString()
    End Sub

    Private Sub Button6_Click(sender As Object, e As EventArgs) Handles Button6.Click
        Dim usSlaveAddress As UShort
        Dim usSubSlaveAddress As UShort
        nmc_get_axis_node_address(g_sCardId, GetAxis(), usSlaveAddress, usSubSlaveAddress)
        TextBox6.Text = usSlaveAddress.ToString()
        TextBox7.Text = usSubSlaveAddress.ToString()
    End Sub

    Private Sub Button7_Click(sender As Object, e As EventArgs) Handles Button7.Click
        Dim usAxisType As UShort
        usAxisType = nmc_get_axis_type(g_sCardId, GetAxis(), usAxisType)
        Select Case (usAxisType)
            Case 0
                TextBox8.Text = GetAxis().ToString() + " 轴是虚拟轴"
            Case 1
                TextBox8.Text = GetAxis().ToString() + " 轴是EtherCAT轴"
            Case 2
                TextBox8.Text = GetAxis().ToString() + " 轴是CANopen轴"
            Case 3
                TextBox8.Text = GetAxis().ToString() + " 轴是脉冲轴"
            Case 4
                TextBox8.Text = GetAxis().ToString() + " 轴是未知类型轴"
        End Select
    End Sub

    Private Sub Button8_Click(sender As Object, e As EventArgs) Handles Button8.Click
        Dim strStrStrsSrA As String
        Dim usErrorCode As UShort
        nmc_get_axis_errcode(g_sCardId, GetAxis(), usErrorCode)
        strStrStrsSrA = usErrorCode.ToString("x8") '10进制转化为16进制        
        TextBox9.Text = "0x" + strStrStrsSrA + "(" + usErrorCode.ToString() + ")"
    End Sub

    Private Sub Button9_Click(sender As Object, e As EventArgs) Handles Button9.Click
        Dim StatusWord As Int32
        Dim strStrStrsSrA As String
        nmc_get_axis_statusword(g_sCardId, GetAxis(), StatusWord)
        strStrStrsSrA = StatusWord.ToString("x8") '10进制转化为16进制        
        TextBox10.Text = "0x" + strStrStrsSrA + "(" + StatusWord.ToString() + ")"
    End Sub

    Private Sub Button10_Click(sender As Object, e As EventArgs) Handles Button10.Click
        Dim ControlWord As Int32
        Dim strStrStrsSrA As String
        nmc_get_axis_contrlword(g_sCardId, GetAxis(), ControlWord)
        strStrStrsSrA = ControlWord.ToString("x8") '10进制转化为16进制       
        TextBox11.Text = "0x" + strStrStrsSrA + "(" + ControlWord.ToString() + ")"
    End Sub

    Private Sub Button12_Click(sender As Object, e As EventArgs) Handles Button12.Click
        Dim Rtn As Integer
        Rtn = nmc_set_axis_enable(g_sCardId, GetAxis())
    End Sub

    Private Sub Button13_Click(sender As Object, e As EventArgs) Handles Button13.Click
        nmc_set_axis_disable(g_sCardId, GetAxis())
    End Sub

    Private Sub Button14_Click(sender As Object, e As EventArgs) Handles Button14.Click
        Me.Close()
    End Sub

    Private Sub Button11_Click(sender As Object, e As EventArgs) Handles Button11.Click

    End Sub
End Class
