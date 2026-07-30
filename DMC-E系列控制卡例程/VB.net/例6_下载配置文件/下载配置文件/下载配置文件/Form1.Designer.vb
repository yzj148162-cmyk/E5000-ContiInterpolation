<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class Form1
    Inherits System.Windows.Forms.Form

    'Form 重写 Dispose，以清理组件列表。
    <System.Diagnostics.DebuggerNonUserCode()> _
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Windows 窗体设计器所必需的
    Private components As System.ComponentModel.IContainer

    '注意: 以下过程是 Windows 窗体设计器所必需的
    '可以使用 Windows 窗体设计器修改它。
    '不要使用代码编辑器修改它。
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Me.components = New System.ComponentModel.Container()
        Me.Btn_DownINI = New System.Windows.Forms.Button()
        Me.Btn_DownENI = New System.Windows.Forms.Button()
        Me.Btn_ClodReset = New System.Windows.Forms.Button()
        Me.Btn_HoldReset = New System.Windows.Forms.Button()
        Me.Btn_InitialReset = New System.Windows.Forms.Button()
        Me.RichTextBox1 = New System.Windows.Forms.RichTextBox()
        Me.Btn_AxisEnable = New System.Windows.Forms.Button()
        Me.Btn_AxisDisable = New System.Windows.Forms.Button()
        Me.Btn_Close = New System.Windows.Forms.Button()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.TextBox1 = New System.Windows.Forms.TextBox()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.Label3 = New System.Windows.Forms.Label()
        Me.Timer1 = New System.Windows.Forms.Timer(Me.components)
        Me.Button1 = New System.Windows.Forms.Button()
        Me.SuspendLayout()
        '
        'Btn_DownINI
        '
        Me.Btn_DownINI.Location = New System.Drawing.Point(189, 32)
        Me.Btn_DownINI.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Btn_DownINI.Name = "Btn_DownINI"
        Me.Btn_DownINI.Size = New System.Drawing.Size(131, 35)
        Me.Btn_DownINI.TabIndex = 0
        Me.Btn_DownINI.Text = "1 下载INI文件"
        Me.Btn_DownINI.UseVisualStyleBackColor = True
        '
        'Btn_DownENI
        '
        Me.Btn_DownENI.Location = New System.Drawing.Point(421, 32)
        Me.Btn_DownENI.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Btn_DownENI.Name = "Btn_DownENI"
        Me.Btn_DownENI.Size = New System.Drawing.Size(121, 35)
        Me.Btn_DownENI.TabIndex = 1
        Me.Btn_DownENI.Text = "2 下载ENI文件"
        Me.Btn_DownENI.UseVisualStyleBackColor = True
        '
        'Btn_ClodReset
        '
        Me.Btn_ClodReset.Location = New System.Drawing.Point(67, 95)
        Me.Btn_ClodReset.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Btn_ClodReset.Name = "Btn_ClodReset"
        Me.Btn_ClodReset.Size = New System.Drawing.Size(117, 42)
        Me.Btn_ClodReset.TabIndex = 2
        Me.Btn_ClodReset.Text = "冷复位"
        Me.Btn_ClodReset.UseVisualStyleBackColor = True
        '
        'Btn_HoldReset
        '
        Me.Btn_HoldReset.Location = New System.Drawing.Point(235, 95)
        Me.Btn_HoldReset.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Btn_HoldReset.Name = "Btn_HoldReset"
        Me.Btn_HoldReset.Size = New System.Drawing.Size(117, 42)
        Me.Btn_HoldReset.TabIndex = 3
        Me.Btn_HoldReset.Text = "热复位"
        Me.Btn_HoldReset.UseVisualStyleBackColor = True
        '
        'Btn_InitialReset
        '
        Me.Btn_InitialReset.Location = New System.Drawing.Point(405, 95)
        Me.Btn_InitialReset.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Btn_InitialReset.Name = "Btn_InitialReset"
        Me.Btn_InitialReset.Size = New System.Drawing.Size(117, 42)
        Me.Btn_InitialReset.TabIndex = 4
        Me.Btn_InitialReset.Text = "初始值复位"
        Me.Btn_InitialReset.UseVisualStyleBackColor = True
        '
        'RichTextBox1
        '
        Me.RichTextBox1.Location = New System.Drawing.Point(21, 161)
        Me.RichTextBox1.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.RichTextBox1.Name = "RichTextBox1"
        Me.RichTextBox1.Size = New System.Drawing.Size(519, 145)
        Me.RichTextBox1.TabIndex = 5
        Me.RichTextBox1.Text = ""
        '
        'Btn_AxisEnable
        '
        Me.Btn_AxisEnable.Location = New System.Drawing.Point(77, 386)
        Me.Btn_AxisEnable.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Btn_AxisEnable.Name = "Btn_AxisEnable"
        Me.Btn_AxisEnable.Size = New System.Drawing.Size(117, 42)
        Me.Btn_AxisEnable.TabIndex = 6
        Me.Btn_AxisEnable.Text = "所有轴使能"
        Me.Btn_AxisEnable.UseVisualStyleBackColor = True
        '
        'Btn_AxisDisable
        '
        Me.Btn_AxisDisable.Location = New System.Drawing.Point(235, 386)
        Me.Btn_AxisDisable.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Btn_AxisDisable.Name = "Btn_AxisDisable"
        Me.Btn_AxisDisable.Size = New System.Drawing.Size(117, 42)
        Me.Btn_AxisDisable.TabIndex = 7
        Me.Btn_AxisDisable.Text = "所有轴失能"
        Me.Btn_AxisDisable.UseVisualStyleBackColor = True
        '
        'Btn_Close
        '
        Me.Btn_Close.Location = New System.Drawing.Point(395, 386)
        Me.Btn_Close.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Btn_Close.Name = "Btn_Close"
        Me.Btn_Close.Size = New System.Drawing.Size(117, 42)
        Me.Btn_Close.TabIndex = 8
        Me.Btn_Close.Text = "退出程序"
        Me.Btn_Close.UseVisualStyleBackColor = True
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Location = New System.Drawing.Point(85, 334)
        Me.Label1.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(82, 15)
        Me.Label1.TabIndex = 9
        Me.Label1.Text = "总线状态："
        '
        'TextBox1
        '
        Me.TextBox1.Location = New System.Drawing.Point(188, 328)
        Me.TextBox1.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.TextBox1.Name = "TextBox1"
        Me.TextBox1.Size = New System.Drawing.Size(261, 25)
        Me.TextBox1.TabIndex = 10
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.Location = New System.Drawing.Point(41, 42)
        Me.Label2.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(112, 15)
        Me.Label2.TabIndex = 11
        Me.Label2.Text = "下载配置文件："
        '
        'Label3
        '
        Me.Label3.AutoSize = True
        Me.Label3.Location = New System.Drawing.Point(361, 41)
        Me.Label3.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label3.Name = "Label3"
        Me.Label3.Size = New System.Drawing.Size(22, 15)
        Me.Label3.TabIndex = 12
        Me.Label3.Text = "→"
        '
        'Timer1
        '
        '
        'Button1
        '
        Me.Button1.Location = New System.Drawing.Point(767, 154)
        Me.Button1.Margin = New System.Windows.Forms.Padding(4)
        Me.Button1.Name = "Button1"
        Me.Button1.Size = New System.Drawing.Size(121, 35)
        Me.Button1.TabIndex = 13
        Me.Button1.Text = "下载固件"
        Me.Button1.UseVisualStyleBackColor = True
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(8.0!, 15.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(1007, 509)
        Me.Controls.Add(Me.Button1)
        Me.Controls.Add(Me.Label3)
        Me.Controls.Add(Me.Label2)
        Me.Controls.Add(Me.TextBox1)
        Me.Controls.Add(Me.Label1)
        Me.Controls.Add(Me.Btn_Close)
        Me.Controls.Add(Me.Btn_AxisDisable)
        Me.Controls.Add(Me.Btn_AxisEnable)
        Me.Controls.Add(Me.RichTextBox1)
        Me.Controls.Add(Me.Btn_InitialReset)
        Me.Controls.Add(Me.Btn_HoldReset)
        Me.Controls.Add(Me.Btn_ClodReset)
        Me.Controls.Add(Me.Btn_DownENI)
        Me.Controls.Add(Me.Btn_DownINI)
        Me.Margin = New System.Windows.Forms.Padding(4, 4, 4, 4)
        Me.Name = "Form1"
        Me.Text = "下载配置文件"
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents Btn_DownINI As System.Windows.Forms.Button
    Friend WithEvents Btn_DownENI As System.Windows.Forms.Button
    Friend WithEvents Btn_ClodReset As System.Windows.Forms.Button
    Friend WithEvents Btn_HoldReset As System.Windows.Forms.Button
    Friend WithEvents Btn_InitialReset As System.Windows.Forms.Button
    Friend WithEvents RichTextBox1 As System.Windows.Forms.RichTextBox
    Friend WithEvents Btn_AxisEnable As System.Windows.Forms.Button
    Friend WithEvents Btn_AxisDisable As System.Windows.Forms.Button
    Friend WithEvents Btn_Close As System.Windows.Forms.Button
    Friend WithEvents Label1 As System.Windows.Forms.Label
    Friend WithEvents TextBox1 As System.Windows.Forms.TextBox
    Friend WithEvents Label2 As System.Windows.Forms.Label
    Friend WithEvents Label3 As System.Windows.Forms.Label
    Friend WithEvents Timer1 As System.Windows.Forms.Timer
    Friend WithEvents Button1 As System.Windows.Forms.Button

End Class
