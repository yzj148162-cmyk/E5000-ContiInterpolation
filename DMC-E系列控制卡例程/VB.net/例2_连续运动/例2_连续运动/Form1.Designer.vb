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
        Me.groupBox3 = New System.Windows.Forms.GroupBox()
        Me.radioButton7 = New System.Windows.Forms.RadioButton()
        Me.radioButton8 = New System.Windows.Forms.RadioButton()
        Me.Btn_StartMove = New System.Windows.Forms.Button()
        Me.Btn_PosRest = New System.Windows.Forms.Button()
        Me.Btn_Close = New System.Windows.Forms.Button()
        Me.Btn_SetEquiv = New System.Windows.Forms.Button()
        Me.Btn_StopVel = New System.Windows.Forms.Button()
        Me.groupBox4 = New System.Windows.Forms.GroupBox()
        Me.Btn_changevel = New System.Windows.Forms.Button()
        Me.numericUpDown_ChangeVel = New System.Windows.Forms.NumericUpDown()
        Me.label25 = New System.Windows.Forms.Label()
        Me.numericUpDown_ChangeTime = New System.Windows.Forms.NumericUpDown()
        Me.label28 = New System.Windows.Forms.Label()
        Me.label26 = New System.Windows.Forms.Label()
        Me.label27 = New System.Windows.Forms.Label()
        Me.groupBox2 = New System.Windows.Forms.GroupBox()
        Me.TextBox5 = New System.Windows.Forms.TextBox()
        Me.Label29 = New System.Windows.Forms.Label()
        Me.textBox4 = New System.Windows.Forms.TextBox()
        Me.textBox3 = New System.Windows.Forms.TextBox()
        Me.label23 = New System.Windows.Forms.Label()
        Me.textBox2 = New System.Windows.Forms.TextBox()
        Me.label24 = New System.Windows.Forms.Label()
        Me.label21 = New System.Windows.Forms.Label()
        Me.label22 = New System.Windows.Forms.Label()
        Me.textBox1 = New System.Windows.Forms.TextBox()
        Me.label20 = New System.Windows.Forms.Label()
        Me.label19 = New System.Windows.Forms.Label()
        Me.label18 = New System.Windows.Forms.Label()
        Me.groupBox1 = New System.Windows.Forms.GroupBox()
        Me.numericUpDown_STime = New System.Windows.Forms.NumericUpDown()
        Me.label13 = New System.Windows.Forms.Label()
        Me.numericUpDown_Tdcc = New System.Windows.Forms.NumericUpDown()
        Me.label11 = New System.Windows.Forms.Label()
        Me.numericUpDown_Tacc = New System.Windows.Forms.NumericUpDown()
        Me.label9 = New System.Windows.Forms.Label()
        Me.numericUpDown_Dist = New System.Windows.Forms.NumericUpDown()
        Me.numericUpDown_StopTime = New System.Windows.Forms.NumericUpDown()
        Me.label17 = New System.Windows.Forms.Label()
        Me.numericUpDown_MaxVel = New System.Windows.Forms.NumericUpDown()
        Me.label15 = New System.Windows.Forms.Label()
        Me.label12 = New System.Windows.Forms.Label()
        Me.label7 = New System.Windows.Forms.Label()
        Me.label10 = New System.Windows.Forms.Label()
        Me.numericUpDown_StartSpeed = New System.Windows.Forms.NumericUpDown()
        Me.label16 = New System.Windows.Forms.Label()
        Me.label8 = New System.Windows.Forms.Label()
        Me.label14 = New System.Windows.Forms.Label()
        Me.label5 = New System.Windows.Forms.Label()
        Me.label6 = New System.Windows.Forms.Label()
        Me.numericUpDown_Equiv = New System.Windows.Forms.NumericUpDown()
        Me.label4 = New System.Windows.Forms.Label()
        Me.label3 = New System.Windows.Forms.Label()
        Me.label2 = New System.Windows.Forms.Label()
        Me.numericUpDown_Axis = New System.Windows.Forms.NumericUpDown()
        Me.label1 = New System.Windows.Forms.Label()
        Me.Timer1 = New System.Windows.Forms.Timer(Me.components)
        Me.GroupBox5 = New System.Windows.Forms.GroupBox()
        Me.RichTextBox1 = New System.Windows.Forms.RichTextBox()
        Me.Btn_SoftReset = New System.Windows.Forms.Button()
        Me.Btn_HardReset = New System.Windows.Forms.Button()
        Me.TextBox6 = New System.Windows.Forms.TextBox()
        Me.Label30 = New System.Windows.Forms.Label()
        Me.groupBox3.SuspendLayout()
        Me.groupBox4.SuspendLayout()
        CType(Me.numericUpDown_ChangeVel, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_ChangeTime, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.groupBox2.SuspendLayout()
        Me.groupBox1.SuspendLayout()
        CType(Me.numericUpDown_STime, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_Tdcc, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_Tacc, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_Dist, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_StopTime, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_MaxVel, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_StartSpeed, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_Equiv, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_Axis, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.GroupBox5.SuspendLayout()
        Me.SuspendLayout()
        '
        'groupBox3
        '
        Me.groupBox3.Controls.Add(Me.radioButton7)
        Me.groupBox3.Controls.Add(Me.radioButton8)
        Me.groupBox3.Location = New System.Drawing.Point(295, 131)
        Me.groupBox3.Name = "groupBox3"
        Me.groupBox3.Size = New System.Drawing.Size(224, 53)
        Me.groupBox3.TabIndex = 39
        Me.groupBox3.TabStop = False
        Me.groupBox3.Text = "运动方向"
        '
        'radioButton7
        '
        Me.radioButton7.AutoSize = True
        Me.radioButton7.Location = New System.Drawing.Point(120, 27)
        Me.radioButton7.Name = "radioButton7"
        Me.radioButton7.Size = New System.Drawing.Size(47, 16)
        Me.radioButton7.TabIndex = 0
        Me.radioButton7.Text = "反向"
        Me.radioButton7.UseVisualStyleBackColor = True
        '
        'radioButton8
        '
        Me.radioButton8.AutoSize = True
        Me.radioButton8.Checked = True
        Me.radioButton8.Location = New System.Drawing.Point(37, 27)
        Me.radioButton8.Name = "radioButton8"
        Me.radioButton8.Size = New System.Drawing.Size(47, 16)
        Me.radioButton8.TabIndex = 0
        Me.radioButton8.TabStop = True
        Me.radioButton8.Text = "正向"
        Me.radioButton8.UseVisualStyleBackColor = True
        '
        'Btn_StartMove
        '
        Me.Btn_StartMove.Location = New System.Drawing.Point(19, 353)
        Me.Btn_StartMove.Name = "Btn_StartMove"
        Me.Btn_StartMove.Size = New System.Drawing.Size(92, 37)
        Me.Btn_StartMove.TabIndex = 30
        Me.Btn_StartMove.Text = "执行运动"
        Me.Btn_StartMove.UseVisualStyleBackColor = True
        '
        'Btn_PosRest
        '
        Me.Btn_PosRest.Location = New System.Drawing.Point(505, 353)
        Me.Btn_PosRest.Name = "Btn_PosRest"
        Me.Btn_PosRest.Size = New System.Drawing.Size(93, 37)
        Me.Btn_PosRest.TabIndex = 33
        Me.Btn_PosRest.Text = "指令清零"
        Me.Btn_PosRest.UseVisualStyleBackColor = True
        '
        'Btn_Close
        '
        Me.Btn_Close.Location = New System.Drawing.Point(668, 353)
        Me.Btn_Close.Name = "Btn_Close"
        Me.Btn_Close.Size = New System.Drawing.Size(93, 37)
        Me.Btn_Close.TabIndex = 34
        Me.Btn_Close.Text = "退出程序"
        Me.Btn_Close.UseVisualStyleBackColor = True
        '
        'Btn_SetEquiv
        '
        Me.Btn_SetEquiv.Location = New System.Drawing.Point(343, 353)
        Me.Btn_SetEquiv.Name = "Btn_SetEquiv"
        Me.Btn_SetEquiv.Size = New System.Drawing.Size(92, 37)
        Me.Btn_SetEquiv.TabIndex = 35
        Me.Btn_SetEquiv.Text = "设置脉冲当量"
        Me.Btn_SetEquiv.UseVisualStyleBackColor = True
        '
        'Btn_StopVel
        '
        Me.Btn_StopVel.Location = New System.Drawing.Point(181, 353)
        Me.Btn_StopVel.Name = "Btn_StopVel"
        Me.Btn_StopVel.Size = New System.Drawing.Size(92, 37)
        Me.Btn_StopVel.TabIndex = 36
        Me.Btn_StopVel.Text = "停止运动"
        Me.Btn_StopVel.UseVisualStyleBackColor = True
        '
        'groupBox4
        '
        Me.groupBox4.Controls.Add(Me.Btn_changevel)
        Me.groupBox4.Controls.Add(Me.numericUpDown_ChangeVel)
        Me.groupBox4.Controls.Add(Me.label25)
        Me.groupBox4.Controls.Add(Me.numericUpDown_ChangeTime)
        Me.groupBox4.Controls.Add(Me.label28)
        Me.groupBox4.Controls.Add(Me.label26)
        Me.groupBox4.Controls.Add(Me.label27)
        Me.groupBox4.Location = New System.Drawing.Point(295, 12)
        Me.groupBox4.Name = "groupBox4"
        Me.groupBox4.Size = New System.Drawing.Size(224, 117)
        Me.groupBox4.TabIndex = 38
        Me.groupBox4.TabStop = False
        Me.groupBox4.Text = "在线变速"
        '
        'Btn_changevel
        '
        Me.Btn_changevel.Location = New System.Drawing.Point(81, 80)
        Me.Btn_changevel.Name = "Btn_changevel"
        Me.Btn_changevel.Size = New System.Drawing.Size(81, 31)
        Me.Btn_changevel.TabIndex = 1
        Me.Btn_changevel.Text = "在线变速"
        Me.Btn_changevel.UseVisualStyleBackColor = True
        '
        'numericUpDown_ChangeVel
        '
        Me.numericUpDown_ChangeVel.DecimalPlaces = 3
        Me.numericUpDown_ChangeVel.Location = New System.Drawing.Point(81, 23)
        Me.numericUpDown_ChangeVel.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_ChangeVel.Name = "numericUpDown_ChangeVel"
        Me.numericUpDown_ChangeVel.Size = New System.Drawing.Size(86, 21)
        Me.numericUpDown_ChangeVel.TabIndex = 1
        Me.numericUpDown_ChangeVel.Value = New Decimal(New Integer() {3500, 0, 0, 0})
        '
        'label25
        '
        Me.label25.AutoSize = True
        Me.label25.Location = New System.Drawing.Point(16, 23)
        Me.label25.Name = "label25"
        Me.label25.Size = New System.Drawing.Size(59, 12)
        Me.label25.TabIndex = 0
        Me.label25.Text = "运行速度:"
        '
        'numericUpDown_ChangeTime
        '
        Me.numericUpDown_ChangeTime.DecimalPlaces = 3
        Me.numericUpDown_ChangeTime.Location = New System.Drawing.Point(81, 50)
        Me.numericUpDown_ChangeTime.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_ChangeTime.Name = "numericUpDown_ChangeTime"
        Me.numericUpDown_ChangeTime.Size = New System.Drawing.Size(86, 21)
        Me.numericUpDown_ChangeTime.TabIndex = 1
        Me.numericUpDown_ChangeTime.Value = New Decimal(New Integer() {5, 0, 0, 65536})
        '
        'label28
        '
        Me.label28.AutoSize = True
        Me.label28.Location = New System.Drawing.Point(173, 52)
        Me.label28.Name = "label28"
        Me.label28.Size = New System.Drawing.Size(11, 12)
        Me.label28.TabIndex = 0
        Me.label28.Text = "s"
        '
        'label26
        '
        Me.label26.AutoSize = True
        Me.label26.Location = New System.Drawing.Point(173, 28)
        Me.label26.Name = "label26"
        Me.label26.Size = New System.Drawing.Size(41, 12)
        Me.label26.TabIndex = 0
        Me.label26.Text = "unit/s"
        '
        'label27
        '
        Me.label27.AutoSize = True
        Me.label27.Location = New System.Drawing.Point(16, 57)
        Me.label27.Name = "label27"
        Me.label27.Size = New System.Drawing.Size(59, 12)
        Me.label27.TabIndex = 0
        Me.label27.Text = "变速时间:"
        '
        'groupBox2
        '
        Me.groupBox2.Controls.Add(Me.TextBox5)
        Me.groupBox2.Controls.Add(Me.Label29)
        Me.groupBox2.Controls.Add(Me.textBox4)
        Me.groupBox2.Controls.Add(Me.textBox3)
        Me.groupBox2.Controls.Add(Me.label23)
        Me.groupBox2.Controls.Add(Me.textBox2)
        Me.groupBox2.Controls.Add(Me.label24)
        Me.groupBox2.Controls.Add(Me.label21)
        Me.groupBox2.Controls.Add(Me.label22)
        Me.groupBox2.Controls.Add(Me.textBox1)
        Me.groupBox2.Controls.Add(Me.label20)
        Me.groupBox2.Controls.Add(Me.label19)
        Me.groupBox2.Controls.Add(Me.label18)
        Me.groupBox2.Location = New System.Drawing.Point(528, 12)
        Me.groupBox2.Name = "groupBox2"
        Me.groupBox2.Size = New System.Drawing.Size(242, 172)
        Me.groupBox2.TabIndex = 37
        Me.groupBox2.TabStop = False
        Me.groupBox2.Text = "信息显示"
        '
        'TextBox5
        '
        Me.TextBox5.Location = New System.Drawing.Point(84, 123)
        Me.TextBox5.Name = "TextBox5"
        Me.TextBox5.ReadOnly = True
        Me.TextBox5.Size = New System.Drawing.Size(89, 21)
        Me.TextBox5.TabIndex = 3
        '
        'Label29
        '
        Me.Label29.AutoSize = True
        Me.Label29.Location = New System.Drawing.Point(21, 128)
        Me.Label29.Name = "Label29"
        Me.Label29.Size = New System.Drawing.Size(59, 12)
        Me.Label29.TabIndex = 2
        Me.Label29.Text = "轴状态机:"
        '
        'textBox4
        '
        Me.textBox4.Location = New System.Drawing.Point(84, 96)
        Me.textBox4.Name = "textBox4"
        Me.textBox4.ReadOnly = True
        Me.textBox4.Size = New System.Drawing.Size(89, 21)
        Me.textBox4.TabIndex = 1
        '
        'textBox3
        '
        Me.textBox3.Location = New System.Drawing.Point(85, 69)
        Me.textBox3.Name = "textBox3"
        Me.textBox3.ReadOnly = True
        Me.textBox3.Size = New System.Drawing.Size(88, 21)
        Me.textBox3.TabIndex = 1
        '
        'label23
        '
        Me.label23.AutoSize = True
        Me.label23.Location = New System.Drawing.Point(179, 73)
        Me.label23.Name = "label23"
        Me.label23.Size = New System.Drawing.Size(29, 12)
        Me.label23.TabIndex = 0
        Me.label23.Text = "unit"
        '
        'textBox2
        '
        Me.textBox2.Location = New System.Drawing.Point(85, 42)
        Me.textBox2.Name = "textBox2"
        Me.textBox2.ReadOnly = True
        Me.textBox2.Size = New System.Drawing.Size(88, 21)
        Me.textBox2.TabIndex = 1
        '
        'label24
        '
        Me.label24.AutoSize = True
        Me.label24.Location = New System.Drawing.Point(20, 100)
        Me.label24.Name = "label24"
        Me.label24.Size = New System.Drawing.Size(59, 12)
        Me.label24.TabIndex = 0
        Me.label24.Text = "运动状态:"
        '
        'label21
        '
        Me.label21.AutoSize = True
        Me.label21.Location = New System.Drawing.Point(179, 46)
        Me.label21.Name = "label21"
        Me.label21.Size = New System.Drawing.Size(29, 12)
        Me.label21.TabIndex = 0
        Me.label21.Text = "unit"
        '
        'label22
        '
        Me.label22.AutoSize = True
        Me.label22.Location = New System.Drawing.Point(20, 73)
        Me.label22.Name = "label22"
        Me.label22.Size = New System.Drawing.Size(59, 12)
        Me.label22.TabIndex = 0
        Me.label22.Text = "反馈位置:"
        '
        'textBox1
        '
        Me.textBox1.Location = New System.Drawing.Point(85, 18)
        Me.textBox1.Name = "textBox1"
        Me.textBox1.ReadOnly = True
        Me.textBox1.Size = New System.Drawing.Size(88, 21)
        Me.textBox1.TabIndex = 1
        '
        'label20
        '
        Me.label20.AutoSize = True
        Me.label20.Location = New System.Drawing.Point(20, 46)
        Me.label20.Name = "label20"
        Me.label20.Size = New System.Drawing.Size(59, 12)
        Me.label20.TabIndex = 0
        Me.label20.Text = "当前位置:"
        '
        'label19
        '
        Me.label19.AutoSize = True
        Me.label19.Location = New System.Drawing.Point(179, 22)
        Me.label19.Name = "label19"
        Me.label19.Size = New System.Drawing.Size(41, 12)
        Me.label19.TabIndex = 0
        Me.label19.Text = "unit/s"
        '
        'label18
        '
        Me.label18.AutoSize = True
        Me.label18.Location = New System.Drawing.Point(20, 22)
        Me.label18.Name = "label18"
        Me.label18.Size = New System.Drawing.Size(59, 12)
        Me.label18.TabIndex = 0
        Me.label18.Text = "当前速度:"
        '
        'groupBox1
        '
        Me.groupBox1.Controls.Add(Me.numericUpDown_STime)
        Me.groupBox1.Controls.Add(Me.label13)
        Me.groupBox1.Controls.Add(Me.numericUpDown_Tdcc)
        Me.groupBox1.Controls.Add(Me.label11)
        Me.groupBox1.Controls.Add(Me.numericUpDown_Tacc)
        Me.groupBox1.Controls.Add(Me.label9)
        Me.groupBox1.Controls.Add(Me.numericUpDown_Dist)
        Me.groupBox1.Controls.Add(Me.numericUpDown_StopTime)
        Me.groupBox1.Controls.Add(Me.label17)
        Me.groupBox1.Controls.Add(Me.numericUpDown_MaxVel)
        Me.groupBox1.Controls.Add(Me.label15)
        Me.groupBox1.Controls.Add(Me.label12)
        Me.groupBox1.Controls.Add(Me.label7)
        Me.groupBox1.Controls.Add(Me.label10)
        Me.groupBox1.Controls.Add(Me.numericUpDown_StartSpeed)
        Me.groupBox1.Controls.Add(Me.label16)
        Me.groupBox1.Controls.Add(Me.label8)
        Me.groupBox1.Controls.Add(Me.label14)
        Me.groupBox1.Controls.Add(Me.label5)
        Me.groupBox1.Controls.Add(Me.label6)
        Me.groupBox1.Controls.Add(Me.numericUpDown_Equiv)
        Me.groupBox1.Controls.Add(Me.label4)
        Me.groupBox1.Controls.Add(Me.label3)
        Me.groupBox1.Controls.Add(Me.label2)
        Me.groupBox1.Controls.Add(Me.numericUpDown_Axis)
        Me.groupBox1.Controls.Add(Me.label1)
        Me.groupBox1.Location = New System.Drawing.Point(12, 12)
        Me.groupBox1.Name = "groupBox1"
        Me.groupBox1.Size = New System.Drawing.Size(277, 325)
        Me.groupBox1.TabIndex = 32
        Me.groupBox1.TabStop = False
        Me.groupBox1.Text = "输入参数"
        '
        'numericUpDown_STime
        '
        Me.numericUpDown_STime.DecimalPlaces = 3
        Me.numericUpDown_STime.Location = New System.Drawing.Point(93, 228)
        Me.numericUpDown_STime.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_STime.Name = "numericUpDown_STime"
        Me.numericUpDown_STime.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_STime.TabIndex = 1
        '
        'label13
        '
        Me.label13.AutoSize = True
        Me.label13.Location = New System.Drawing.Point(192, 233)
        Me.label13.Name = "label13"
        Me.label13.Size = New System.Drawing.Size(11, 12)
        Me.label13.TabIndex = 0
        Me.label13.Text = "s"
        '
        'numericUpDown_Tdcc
        '
        Me.numericUpDown_Tdcc.DecimalPlaces = 3
        Me.numericUpDown_Tdcc.Location = New System.Drawing.Point(93, 193)
        Me.numericUpDown_Tdcc.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_Tdcc.Name = "numericUpDown_Tdcc"
        Me.numericUpDown_Tdcc.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_Tdcc.TabIndex = 1
        Me.numericUpDown_Tdcc.Value = New Decimal(New Integer() {1, 0, 0, 65536})
        '
        'label11
        '
        Me.label11.AutoSize = True
        Me.label11.Location = New System.Drawing.Point(192, 198)
        Me.label11.Name = "label11"
        Me.label11.Size = New System.Drawing.Size(11, 12)
        Me.label11.TabIndex = 0
        Me.label11.Text = "s"
        '
        'numericUpDown_Tacc
        '
        Me.numericUpDown_Tacc.DecimalPlaces = 3
        Me.numericUpDown_Tacc.Location = New System.Drawing.Point(93, 158)
        Me.numericUpDown_Tacc.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_Tacc.Name = "numericUpDown_Tacc"
        Me.numericUpDown_Tacc.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_Tacc.TabIndex = 1
        Me.numericUpDown_Tacc.Value = New Decimal(New Integer() {1, 0, 0, 65536})
        '
        'label9
        '
        Me.label9.AutoSize = True
        Me.label9.Location = New System.Drawing.Point(192, 163)
        Me.label9.Name = "label9"
        Me.label9.Size = New System.Drawing.Size(11, 12)
        Me.label9.TabIndex = 0
        Me.label9.Text = "s"
        '
        'numericUpDown_Dist
        '
        Me.numericUpDown_Dist.DecimalPlaces = 3
        Me.numericUpDown_Dist.Location = New System.Drawing.Point(93, 298)
        Me.numericUpDown_Dist.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_Dist.Name = "numericUpDown_Dist"
        Me.numericUpDown_Dist.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_Dist.TabIndex = 1
        Me.numericUpDown_Dist.Value = New Decimal(New Integer() {50000, 0, 0, 0})
        '
        'numericUpDown_StopTime
        '
        Me.numericUpDown_StopTime.DecimalPlaces = 3
        Me.numericUpDown_StopTime.Location = New System.Drawing.Point(93, 263)
        Me.numericUpDown_StopTime.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_StopTime.Name = "numericUpDown_StopTime"
        Me.numericUpDown_StopTime.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_StopTime.TabIndex = 1
        '
        'label17
        '
        Me.label17.AutoSize = True
        Me.label17.Location = New System.Drawing.Point(192, 303)
        Me.label17.Name = "label17"
        Me.label17.Size = New System.Drawing.Size(29, 12)
        Me.label17.TabIndex = 0
        Me.label17.Text = "unit"
        '
        'numericUpDown_MaxVel
        '
        Me.numericUpDown_MaxVel.DecimalPlaces = 3
        Me.numericUpDown_MaxVel.Location = New System.Drawing.Point(93, 123)
        Me.numericUpDown_MaxVel.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_MaxVel.Name = "numericUpDown_MaxVel"
        Me.numericUpDown_MaxVel.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_MaxVel.TabIndex = 1
        Me.numericUpDown_MaxVel.Value = New Decimal(New Integer() {10000, 0, 0, 0})
        '
        'label15
        '
        Me.label15.AutoSize = True
        Me.label15.Location = New System.Drawing.Point(192, 268)
        Me.label15.Name = "label15"
        Me.label15.Size = New System.Drawing.Size(41, 12)
        Me.label15.TabIndex = 0
        Me.label15.Text = "unit/s"
        '
        'label12
        '
        Me.label12.AutoSize = True
        Me.label12.Location = New System.Drawing.Point(27, 234)
        Me.label12.Name = "label12"
        Me.label12.Size = New System.Drawing.Size(53, 12)
        Me.label12.TabIndex = 0
        Me.label12.Text = "S段时间:"
        '
        'label7
        '
        Me.label7.AutoSize = True
        Me.label7.Location = New System.Drawing.Point(192, 128)
        Me.label7.Name = "label7"
        Me.label7.Size = New System.Drawing.Size(41, 12)
        Me.label7.TabIndex = 0
        Me.label7.Text = "unit/s"
        '
        'label10
        '
        Me.label10.AutoSize = True
        Me.label10.Location = New System.Drawing.Point(27, 199)
        Me.label10.Name = "label10"
        Me.label10.Size = New System.Drawing.Size(59, 12)
        Me.label10.TabIndex = 0
        Me.label10.Text = "减速时间:"
        '
        'numericUpDown_StartSpeed
        '
        Me.numericUpDown_StartSpeed.DecimalPlaces = 3
        Me.numericUpDown_StartSpeed.Location = New System.Drawing.Point(93, 88)
        Me.numericUpDown_StartSpeed.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_StartSpeed.Name = "numericUpDown_StartSpeed"
        Me.numericUpDown_StartSpeed.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_StartSpeed.TabIndex = 1
        '
        'label16
        '
        Me.label16.AutoSize = True
        Me.label16.Location = New System.Drawing.Point(27, 304)
        Me.label16.Name = "label16"
        Me.label16.Size = New System.Drawing.Size(59, 12)
        Me.label16.TabIndex = 0
        Me.label16.Text = "运行距离:"
        '
        'label8
        '
        Me.label8.AutoSize = True
        Me.label8.Location = New System.Drawing.Point(27, 164)
        Me.label8.Name = "label8"
        Me.label8.Size = New System.Drawing.Size(59, 12)
        Me.label8.TabIndex = 0
        Me.label8.Text = "加速时间:"
        '
        'label14
        '
        Me.label14.AutoSize = True
        Me.label14.Location = New System.Drawing.Point(27, 269)
        Me.label14.Name = "label14"
        Me.label14.Size = New System.Drawing.Size(59, 12)
        Me.label14.TabIndex = 0
        Me.label14.Text = "停止速度:"
        '
        'label5
        '
        Me.label5.AutoSize = True
        Me.label5.Location = New System.Drawing.Point(192, 93)
        Me.label5.Name = "label5"
        Me.label5.Size = New System.Drawing.Size(41, 12)
        Me.label5.TabIndex = 0
        Me.label5.Text = "unit/s"
        '
        'label6
        '
        Me.label6.AutoSize = True
        Me.label6.Location = New System.Drawing.Point(27, 129)
        Me.label6.Name = "label6"
        Me.label6.Size = New System.Drawing.Size(59, 12)
        Me.label6.TabIndex = 0
        Me.label6.Text = "运行速度:"
        '
        'numericUpDown_Equiv
        '
        Me.numericUpDown_Equiv.DecimalPlaces = 3
        Me.numericUpDown_Equiv.Location = New System.Drawing.Point(93, 53)
        Me.numericUpDown_Equiv.Maximum = New Decimal(New Integer() {1215752192, 23, 0, 0})
        Me.numericUpDown_Equiv.Name = "numericUpDown_Equiv"
        Me.numericUpDown_Equiv.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_Equiv.TabIndex = 1
        Me.numericUpDown_Equiv.Value = New Decimal(New Integer() {1, 0, 0, 0})
        '
        'label4
        '
        Me.label4.AutoSize = True
        Me.label4.Location = New System.Drawing.Point(27, 94)
        Me.label4.Name = "label4"
        Me.label4.Size = New System.Drawing.Size(59, 12)
        Me.label4.TabIndex = 0
        Me.label4.Text = "起始速度:"
        '
        'label3
        '
        Me.label3.AutoSize = True
        Me.label3.Location = New System.Drawing.Point(192, 58)
        Me.label3.Name = "label3"
        Me.label3.Size = New System.Drawing.Size(65, 12)
        Me.label3.TabIndex = 0
        Me.label3.Text = "pulse/unit"
        '
        'label2
        '
        Me.label2.AutoSize = True
        Me.label2.Location = New System.Drawing.Point(27, 59)
        Me.label2.Name = "label2"
        Me.label2.Size = New System.Drawing.Size(59, 12)
        Me.label2.TabIndex = 0
        Me.label2.Text = "脉冲当量:"
        '
        'numericUpDown_Axis
        '
        Me.numericUpDown_Axis.Location = New System.Drawing.Point(93, 18)
        Me.numericUpDown_Axis.Maximum = New Decimal(New Integer() {3, 0, 0, 0})
        Me.numericUpDown_Axis.Name = "numericUpDown_Axis"
        Me.numericUpDown_Axis.Size = New System.Drawing.Size(93, 21)
        Me.numericUpDown_Axis.TabIndex = 1
        '
        'label1
        '
        Me.label1.AutoSize = True
        Me.label1.Location = New System.Drawing.Point(27, 24)
        Me.label1.Name = "label1"
        Me.label1.Size = New System.Drawing.Size(59, 12)
        Me.label1.TabIndex = 0
        Me.label1.Text = "电机轴号:"
        '
        'Timer1
        '
        '
        'GroupBox5
        '
        Me.GroupBox5.Controls.Add(Me.RichTextBox1)
        Me.GroupBox5.Controls.Add(Me.Btn_SoftReset)
        Me.GroupBox5.Controls.Add(Me.Btn_HardReset)
        Me.GroupBox5.Controls.Add(Me.TextBox6)
        Me.GroupBox5.Controls.Add(Me.Label30)
        Me.GroupBox5.Location = New System.Drawing.Point(295, 191)
        Me.GroupBox5.Name = "GroupBox5"
        Me.GroupBox5.Size = New System.Drawing.Size(475, 146)
        Me.GroupBox5.TabIndex = 40
        Me.GroupBox5.TabStop = False
        Me.GroupBox5.Text = "复位操作及总线状态"
        '
        'RichTextBox1
        '
        Me.RichTextBox1.Location = New System.Drawing.Point(211, 20)
        Me.RichTextBox1.Name = "RichTextBox1"
        Me.RichTextBox1.Size = New System.Drawing.Size(250, 81)
        Me.RichTextBox1.TabIndex = 7
        Me.RichTextBox1.Text = ""
        '
        'Btn_SoftReset
        '
        Me.Btn_SoftReset.Location = New System.Drawing.Point(120, 89)
        Me.Btn_SoftReset.Name = "Btn_SoftReset"
        Me.Btn_SoftReset.Size = New System.Drawing.Size(80, 29)
        Me.Btn_SoftReset.TabIndex = 6
        Me.Btn_SoftReset.Text = "软件复位"
        Me.Btn_SoftReset.UseVisualStyleBackColor = True
        '
        'Btn_HardReset
        '
        Me.Btn_HardReset.Location = New System.Drawing.Point(13, 89)
        Me.Btn_HardReset.Name = "Btn_HardReset"
        Me.Btn_HardReset.Size = New System.Drawing.Size(85, 29)
        Me.Btn_HardReset.TabIndex = 5
        Me.Btn_HardReset.Text = "硬件复位"
        Me.Btn_HardReset.UseVisualStyleBackColor = True
        '
        'TextBox6
        '
        Me.TextBox6.Location = New System.Drawing.Point(83, 41)
        Me.TextBox6.Name = "TextBox6"
        Me.TextBox6.ReadOnly = True
        Me.TextBox6.Size = New System.Drawing.Size(117, 21)
        Me.TextBox6.TabIndex = 4
        '
        'Label30
        '
        Me.Label30.AutoSize = True
        Me.Label30.Location = New System.Drawing.Point(25, 44)
        Me.Label30.Name = "Label30"
        Me.Label30.Size = New System.Drawing.Size(59, 12)
        Me.Label30.TabIndex = 1
        Me.Label30.Text = "总线状态:"
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 12.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(782, 405)
        Me.Controls.Add(Me.GroupBox5)
        Me.Controls.Add(Me.groupBox3)
        Me.Controls.Add(Me.Btn_StartMove)
        Me.Controls.Add(Me.Btn_PosRest)
        Me.Controls.Add(Me.Btn_Close)
        Me.Controls.Add(Me.Btn_SetEquiv)
        Me.Controls.Add(Me.Btn_StopVel)
        Me.Controls.Add(Me.groupBox4)
        Me.Controls.Add(Me.groupBox2)
        Me.Controls.Add(Me.groupBox1)
        Me.Name = "Form1"
        Me.Text = "连续运动"
        Me.groupBox3.ResumeLayout(False)
        Me.groupBox3.PerformLayout()
        Me.groupBox4.ResumeLayout(False)
        Me.groupBox4.PerformLayout()
        CType(Me.numericUpDown_ChangeVel, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_ChangeTime, System.ComponentModel.ISupportInitialize).EndInit()
        Me.groupBox2.ResumeLayout(False)
        Me.groupBox2.PerformLayout()
        Me.groupBox1.ResumeLayout(False)
        Me.groupBox1.PerformLayout()
        CType(Me.numericUpDown_STime, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_Tdcc, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_Tacc, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_Dist, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_StopTime, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_MaxVel, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_StartSpeed, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_Equiv, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_Axis, System.ComponentModel.ISupportInitialize).EndInit()
        Me.GroupBox5.ResumeLayout(False)
        Me.GroupBox5.PerformLayout()
        Me.ResumeLayout(False)

    End Sub
    Private WithEvents groupBox3 As System.Windows.Forms.GroupBox
    Private WithEvents radioButton7 As System.Windows.Forms.RadioButton
    Private WithEvents radioButton8 As System.Windows.Forms.RadioButton
    Private WithEvents Btn_StartMove As System.Windows.Forms.Button
    Private WithEvents Btn_PosRest As System.Windows.Forms.Button
    Private WithEvents Btn_Close As System.Windows.Forms.Button
    Private WithEvents Btn_SetEquiv As System.Windows.Forms.Button
    Private WithEvents Btn_StopVel As System.Windows.Forms.Button
    Private WithEvents groupBox4 As System.Windows.Forms.GroupBox
    Private WithEvents Btn_changevel As System.Windows.Forms.Button
    Private WithEvents numericUpDown_ChangeVel As System.Windows.Forms.NumericUpDown
    Private WithEvents label25 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_ChangeTime As System.Windows.Forms.NumericUpDown
    Private WithEvents label28 As System.Windows.Forms.Label
    Private WithEvents label26 As System.Windows.Forms.Label
    Private WithEvents label27 As System.Windows.Forms.Label
    Private WithEvents groupBox2 As System.Windows.Forms.GroupBox
    Private WithEvents textBox4 As System.Windows.Forms.TextBox
    Private WithEvents textBox3 As System.Windows.Forms.TextBox
    Private WithEvents label23 As System.Windows.Forms.Label
    Private WithEvents textBox2 As System.Windows.Forms.TextBox
    Private WithEvents label24 As System.Windows.Forms.Label
    Private WithEvents label21 As System.Windows.Forms.Label
    Private WithEvents label22 As System.Windows.Forms.Label
    Private WithEvents textBox1 As System.Windows.Forms.TextBox
    Private WithEvents label20 As System.Windows.Forms.Label
    Private WithEvents label19 As System.Windows.Forms.Label
    Private WithEvents label18 As System.Windows.Forms.Label
    Private WithEvents groupBox1 As System.Windows.Forms.GroupBox
    Private WithEvents numericUpDown_STime As System.Windows.Forms.NumericUpDown
    Private WithEvents label13 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_Tdcc As System.Windows.Forms.NumericUpDown
    Private WithEvents label11 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_Tacc As System.Windows.Forms.NumericUpDown
    Private WithEvents label9 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_Dist As System.Windows.Forms.NumericUpDown
    Private WithEvents numericUpDown_StopTime As System.Windows.Forms.NumericUpDown
    Private WithEvents label17 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_MaxVel As System.Windows.Forms.NumericUpDown
    Private WithEvents label15 As System.Windows.Forms.Label
    Private WithEvents label12 As System.Windows.Forms.Label
    Private WithEvents label7 As System.Windows.Forms.Label
    Private WithEvents label10 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_StartSpeed As System.Windows.Forms.NumericUpDown
    Private WithEvents label16 As System.Windows.Forms.Label
    Private WithEvents label8 As System.Windows.Forms.Label
    Private WithEvents label14 As System.Windows.Forms.Label
    Private WithEvents label5 As System.Windows.Forms.Label
    Private WithEvents label6 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_Equiv As System.Windows.Forms.NumericUpDown
    Private WithEvents label4 As System.Windows.Forms.Label
    Private WithEvents label3 As System.Windows.Forms.Label
    Private WithEvents label2 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_Axis As System.Windows.Forms.NumericUpDown
    Private WithEvents label1 As System.Windows.Forms.Label
    Friend WithEvents Timer1 As System.Windows.Forms.Timer
    Private WithEvents TextBox5 As System.Windows.Forms.TextBox
    Private WithEvents Label29 As System.Windows.Forms.Label
    Friend WithEvents GroupBox5 As System.Windows.Forms.GroupBox
    Friend WithEvents RichTextBox1 As System.Windows.Forms.RichTextBox
    Friend WithEvents Btn_SoftReset As System.Windows.Forms.Button
    Friend WithEvents Btn_HardReset As System.Windows.Forms.Button
    Private WithEvents TextBox6 As System.Windows.Forms.TextBox
    Private WithEvents Label30 As System.Windows.Forms.Label

End Class
