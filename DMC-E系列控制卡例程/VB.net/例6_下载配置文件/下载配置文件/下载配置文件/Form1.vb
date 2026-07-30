Imports System.Threading
Imports System.IO
Imports System.Text


Public Class Form1
    Public g_sCardID As Short
    Private Sub Button4_Click(ByVal sender As System.Object, ByVal e As System.EventArgs)
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡软件（热）复位进行中……")
        Btn_ClodReset.Enabled = False
        Btn_HoldReset.Enabled = False
        Dim dt_start As DateTime
        Dim sRtn As Short
        Dim usErr As UShort
        dt_start = DateTime.Now
        sRtn = dmc_soft_reset(g_sCardID)
        If sRtn <> 0 Then
            MessageBox.Show("dmc_soft_reset == " + sRtn.ToString())
            Return
        End If
        While True
            If (DateTime.Now - dt_start).TotalMilliseconds >= 10000.0 Then '设置超时退出，防止死循环
                MessageBox.Show("总线复位失败!")
                Btn_ClodReset.Enabled = True
                Btn_HoldReset.Enabled = True
                Btn_HoldReset.Focus()
                Return
            End If

            sRtn = nmc_get_errcode(g_sCardID, 2, usErr)
            If sRtn = 0 Then
                If usErr = 0 Then '总线复位正常完成
                    Exit While
                End If
            Else
                MessageBox.Show("nmc_get_errcode == " + sRtn.ToString())
                Btn_ClodReset.Enabled = True
                Btn_HoldReset.Enabled = True
                Btn_HoldReset.Focus()
                Return
            End If
            Application.DoEvents()
        End While
        RichTextBox1.AppendText("总线卡软件（热）复位完成,请确认总线状态")
        Btn_ClodReset.Enabled = True
        Btn_HoldReset.Enabled = True
        Btn_HoldReset.Focus()
    End Sub

    Private Sub Form1_Load(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles MyBase.Load
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
        g_sCardID = arr_usCardids(1) '
        Timer1.Start() '
    End Sub

    Private Sub Timer1_Tick(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Timer1.Tick
        Dim errcode As Int32
        nmc_get_errcode(g_sCardID, 2, errcode)
        If (errcode = 0) Then
            TextBox1.Text = "EtherCAT总线正常"
            TextBox1.BackColor = Color.Green
        Else
            TextBox1.Text = "EtherCAT总线出错"
            TextBox1.BackColor = Color.Red
        End If
    End Sub

  

    Private Sub Form1_FormClosing(ByVal sender As System.Object, ByVal e As System.Windows.Forms.FormClosingEventArgs) Handles MyBase.FormClosing
        Timer1.Stop()
    End Sub


    Private Sub Btn_DownINI_Click(sender As System.Object, e As System.EventArgs) Handles Btn_DownINI.Click
        Dim file2 = New OpenFileDialog()

        file2.Filter = "ini|*.ini"
        Dim sRtn = file2.ShowDialog()
        If sRtn = DialogResult.OK Then
            Dim str As String
            str = File.ReadAllText(file2.FileName)
            Dim buffer1() As Byte = Encoding.UTF8.GetBytes(str)
            Dim fileincontrol() As Byte = Encoding.UTF8.GetBytes("")
            Dim filetype2 As UShort = 201
            Dim ret As Short = dmc_download_memfile(g_sCardID, buffer1, buffer1.Length, fileincontrol, filetype2)

            If (ret = 0) Then
                MessageBox.Show("下载ENI配置文件成功！")
            End If

        End If
    End Sub

    Private Sub Btn_DownENI_Click(sender As System.Object, e As System.EventArgs) Handles Btn_DownENI.Click
        Dim file2 = New OpenFileDialog()

        file2.Filter = "eni|*.eni"
        Dim sRtn = file2.ShowDialog()
        If sRtn = DialogResult.OK Then
            Dim str As String
            str = File.ReadAllText(file2.FileName)
            Dim buffer1() As Byte = Encoding.UTF8.GetBytes(str)
            Dim fileincontrol() As Byte = Encoding.UTF8.GetBytes("")
            Dim filetype2 As UShort = 200
            Dim ret As Short = dmc_download_memfile(g_sCardID, buffer1, buffer1.Length, fileincontrol, filetype2)

            If (ret = 0) Then
                MessageBox.Show("下载ENI配置文件成功！")
            End If

        End If
    End Sub

    Private Sub Btn_ClodReset_Click(sender As System.Object, e As System.EventArgs) Handles Btn_ClodReset.Click
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡硬件（冷）复位进行中……")
        'Button3.Enabled = False
        'Button4.Enabled = False
        dmc_board_reset()
        dmc_board_close()
        For i As Integer = 0 To 15 Step 1 '总线卡硬件复位耗时15s左右
            Application.DoEvents()
            Thread.Sleep(1000)
        Next
        dmc_board_init()
        RichTextBox1.AppendText("总线卡硬件（冷）复位完成,请确认总线状态")
        'Button3.Enabled = True
        'Button4.Enabled = True
        'Button3.Focus()
    End Sub

    Private Sub Btn_HoldReset_Click(sender As System.Object, e As System.EventArgs) Handles Btn_HoldReset.Click
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡软件（热）复位进行中……")
        Btn_ClodReset.Enabled = False
        Btn_HoldReset.Enabled = False
        Dim dt_start As DateTime
        Dim sRtn As Short
        Dim usErr As UShort
        dt_start = DateTime.Now
        sRtn = dmc_soft_reset(g_sCardID)
        If sRtn <> 0 Then
            MessageBox.Show("dmc_soft_reset == " + sRtn.ToString())
            Return
        End If
        While True
            If (DateTime.Now - dt_start).TotalMilliseconds >= 10000.0 Then '设置超时退出，防止死循环
                MessageBox.Show("总线复位失败!")
                Btn_ClodReset.Enabled = True
                Btn_HoldReset.Enabled = True
                Btn_HoldReset.Focus()
                Return
            End If

            sRtn = nmc_get_errcode(g_sCardID, 2, usErr)
            If sRtn = 0 Then
                If usErr = 0 Then '总线复位正常完成
                    Exit While
                End If
            Else
                MessageBox.Show("nmc_get_errcode == " + sRtn.ToString())
                Btn_ClodReset.Enabled = True
                Btn_HoldReset.Enabled = True
                Btn_HoldReset.Focus()
                Return
            End If
            Application.DoEvents()
        End While
        RichTextBox1.AppendText("总线卡软件（热）复位完成,请确认总线状态")
        Btn_ClodReset.Enabled = True
        Btn_HoldReset.Enabled = True
        Btn_HoldReset.Focus()
    End Sub

    Private Sub Btn_InitialReset_Click(sender As System.Object, e As System.EventArgs) Handles Btn_InitialReset.Click
        Dim sRtn As Short
        RichTextBox1.Text = ""
        RichTextBox1.AppendText("请勿操作，总线卡初始值复位进行中……")
        '‘Button3.Enabled = False
        'Button4.Enabled = False
        '调用该函数只是删除了文件信息，此时需要，再调用一次冷复位或者热复位重启总线才可以完全清除系统的临时缓存信息。
        sRtn = dmc_original_reset(g_sCardID)
        If sRtn <> 0 Then
            MessageBox.Show("dmc_original_reset == " + sRtn.ToString())
            Return
        End If
        For i As Integer = 0 To 2 Step 1 '总线卡初始值复位耗时2s左右
            Application.DoEvents()
            Thread.Sleep(1000)
        Next
        sRtn = dmc_soft_reset(g_sCardID)
        If sRtn <> 0 Then
            MessageBox.Show("dmc_soft_reset == " + sRtn.ToString())
            Return
        End If
        'dmc_board_close()
        For i As Integer = 0 To 15 Step 1 '总线卡初始值复位耗时15s左右
            Application.DoEvents()
            Thread.Sleep(1000)
        Next
        'dmc_board_init()
        RichTextBox1.AppendText("总线卡初始值复位完成,请确认总线状态")
        ' Button3.Enabled = True
        ' Button4.Enabled = True
        ' Button4.Focus()
    End Sub

    Private Sub Btn_AxisEnable_Click(sender As System.Object, e As System.EventArgs) Handles Btn_AxisEnable.Click
        nmc_set_axis_enable(g_sCardID, 255)
    End Sub

    Private Sub Btn_AxisDisable_Click(sender As System.Object, e As System.EventArgs) Handles Btn_AxisDisable.Click
        nmc_set_axis_disable(g_sCardID, 255)
    End Sub

    Private Sub Btn_Close_Click(sender As System.Object, e As System.EventArgs) Handles Btn_Close.Click
        Me.Close()
    End Sub

    Private Sub Button1_Click(sender As System.Object, e As System.EventArgs) Handles Button1.Click
        Dim file1 = New OpenFileDialog()
        file1.Filter = "s6x|*.s6x"

        Dim sRtn = file1.ShowDialog()
        If sRtn = DialogResult.OK Then
            Dim str As String
            str = file1.FileName
            Dim ret As Short = dmc_download_firmware(g_sCardID, str)

            If (ret = 0) Then
                MessageBox.Show("下载固件文件成功！")
            End If

        End If
    End Sub
End Class
