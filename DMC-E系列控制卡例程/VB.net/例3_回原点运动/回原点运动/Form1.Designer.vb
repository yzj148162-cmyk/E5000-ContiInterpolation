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
        Me.Timer1 = New System.Windows.Forms.Timer(Me.components)
        Me.Btn_Close = New System.Windows.Forms.Button()
        Me.Btn_StopMove = New System.Windows.Forms.Button()
        Me.Btn_TdccStopMove = New System.Windows.Forms.Button()
        Me.Btn_ResetPos = New System.Windows.Forms.Button()
        Me.Btn_StartMove = New System.Windows.Forms.Button()
        Me.groupBox2 = New System.Windows.Forms.GroupBox()
        Me.Label16 = New System.Windows.Forms.Label()
        Me.NumericUpDown_Homeoffset = New System.Windows.Forms.NumericUpDown()
        Me.Label18 = New System.Windows.Forms.Label()
        Me.NumericUpDown_HomeMode = New System.Windows.Forms.NumericUpDown()
        Me.Label21 = New System.Windows.Forms.Label()
        Me.Label12 = New System.Windows.Forms.Label()
        Me.Label11 = New System.Windows.Forms.Label()
        Me.Label10 = New System.Windows.Forms.Label()
        Me.Label9 = New System.Windows.Forms.Label()
        Me.Label8 = New System.Windows.Forms.Label()
        Me.NumericUpDown_Axis = New System.Windows.Forms.NumericUpDown()
        Me.numericUpDown_Tdcc = New System.Windows.Forms.NumericUpDown()
        Me.Label7 = New System.Windows.Forms.Label()
        Me.label6 = New System.Windows.Forms.Label()
        Me.numericUpDown_Tacc = New System.Windows.Forms.NumericUpDown()
        Me.label5 = New System.Windows.Forms.Label()
        Me.numericUpDown_StopVel = New System.Windows.Forms.NumericUpDown()
        Me.label4 = New System.Windows.Forms.Label()
        Me.numericUpDown_HighVel = New System.Windows.Forms.NumericUpDown()
        Me.label3 = New System.Windows.Forms.Label()
        Me.numericUpDown_LowVel = New System.Windows.Forms.NumericUpDown()
        Me.label2 = New System.Windows.Forms.Label()
        Me.GroupBox1 = New System.Windows.Forms.GroupBox()
        Me.RichTextBox1 = New System.Windows.Forms.RichTextBox()
        Me.TextBox5 = New System.Windows.Forms.TextBox()
        Me.Label22 = New System.Windows.Forms.Label()
        Me.Btn_SoftReset = New System.Windows.Forms.Button()
        Me.Btn_HardReset = New System.Windows.Forms.Button()
        Me.label19 = New System.Windows.Forms.Label()
        Me.TextBox2 = New System.Windows.Forms.TextBox()
        Me.label14 = New System.Windows.Forms.Label()
        Me.label13 = New System.Windows.Forms.Label()
        Me.label20 = New System.Windows.Forms.Label()
        Me.textBox1 = New System.Windows.Forms.TextBox()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.textBox3 = New System.Windows.Forms.TextBox()
        Me.Label25 = New System.Windows.Forms.Label()
        Me.Label24 = New System.Windows.Forms.Label()
        Me.Label23 = New System.Windows.Forms.Label()
        Me.Button4 = New System.Windows.Forms.Button()
        Me.Button6 = New System.Windows.Forms.Button()
        Me.Button8 = New System.Windows.Forms.Button()
        Me.Label26 = New System.Windows.Forms.Label()
        Me.TextBox6 = New System.Windows.Forms.TextBox()
        Me.GroupBox6 = New System.Windows.Forms.GroupBox()
        Me.groupBox4 = New System.Windows.Forms.GroupBox()
        Me.radioButton5 = New System.Windows.Forms.RadioButton()
        Me.radioButton6 = New System.Windows.Forms.RadioButton()
        Me.groupBox5 = New System.Windows.Forms.GroupBox()
        Me.radioButton11 = New System.Windows.Forms.RadioButton()
        Me.radioButton9 = New System.Windows.Forms.RadioButton()
        Me.radioButton10 = New System.Windows.Forms.RadioButton()
        Me.groupBox3 = New System.Windows.Forms.GroupBox()
        Me.radioButton7 = New System.Windows.Forms.RadioButton()
        Me.radioButton8 = New System.Windows.Forms.RadioButton()
        Me.Button1 = New System.Windows.Forms.Button()
        Me.groupBox2.SuspendLayout()
        CType(Me.NumericUpDown_Homeoffset, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.NumericUpDown_HomeMode, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.NumericUpDown_Axis, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_Tdcc, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_Tacc, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_StopVel, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_HighVel, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.numericUpDown_LowVel, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.GroupBox1.SuspendLayout()
        Me.GroupBox6.SuspendLayout()
        Me.groupBox4.SuspendLayout()
        Me.groupBox5.SuspendLayout()
        Me.groupBox3.SuspendLayout()
        Me.SuspendLayout()
        '
        'Timer1
        '
        '
        'Btn_Close
        '
        Me.Btn_Close.Location = New System.Drawing.Point(822, 363)
        Me.Btn_Close.Margin = New System.Windows.Forms.Padding(4)
        Me.Btn_Close.Name = "Btn_Close"
        Me.Btn_Close.Size = New System.Drawing.Size(124, 39)
        Me.Btn_Close.TabIndex = 55
        Me.Btn_Close.Text = "退出程序"
        Me.Btn_Close.UseVisualStyleBackColor = True
        '
        'Btn_StopMove
        '
        Me.Btn_StopMove.Location = New System.Drawing.Point(518, 363)
        Me.Btn_StopMove.Margin = New System.Windows.Forms.Padding(4)
        Me.Btn_StopMove.Name = "Btn_StopMove"
        Me.Btn_StopMove.Size = New System.Drawing.Size(124, 39)
        Me.Btn_StopMove.TabIndex = 54
        Me.Btn_StopMove.Text = "立即停止"
        Me.Btn_StopMove.UseVisualStyleBackColor = True
        '
        'Btn_TdccStopMove
        '
        Me.Btn_TdccStopMove.Location = New System.Drawing.Point(366, 363)
        Me.Btn_TdccStopMove.Margin = New System.Windows.Forms.Padding(4)
        Me.Btn_TdccStopMove.Name = "Btn_TdccStopMove"
        Me.Btn_TdccStopMove.Size = New System.Drawing.Size(124, 39)
        Me.Btn_TdccStopMove.TabIndex = 48
        Me.Btn_TdccStopMove.Text = "减速停止"
        Me.Btn_TdccStopMove.UseVisualStyleBackColor = True
        '
        'Btn_ResetPos
        '
        Me.Btn_ResetPos.Location = New System.Drawing.Point(670, 363)
        Me.Btn_ResetPos.Margin = New System.Windows.Forms.Padding(4)
        Me.Btn_ResetPos.Name = "Btn_ResetPos"
        Me.Btn_ResetPos.Size = New System.Drawing.Size(124, 39)
        Me.Btn_ResetPos.TabIndex = 46
        Me.Btn_ResetPos.Text = "位置清零"
        Me.Btn_ResetPos.UseVisualStyleBackColor = True
        '
        'Btn_StartMove
        '
        Me.Btn_StartMove.Location = New System.Drawing.Point(214, 363)
        Me.Btn_StartMove.Margin = New System.Windows.Forms.Padding(4)
        Me.Btn_StartMove.Name = "Btn_StartMove"
        Me.Btn_StartMove.Size = New System.Drawing.Size(124, 39)
        Me.Btn_StartMove.TabIndex = 44
        Me.Btn_StartMove.Text = "总线轴回零"
        Me.Btn_StartMove.UseVisualStyleBackColor = True
        '
        'groupBox2
        '
        Me.groupBox2.Controls.Add(Me.Label16)
        Me.groupBox2.Controls.Add(Me.NumericUpDown_Homeoffset)
        Me.groupBox2.Controls.Add(Me.Label18)
        Me.groupBox2.Controls.Add(Me.NumericUpDown_HomeMode)
        Me.groupBox2.Controls.Add(Me.Label21)
        Me.groupBox2.Controls.Add(Me.Label12)
        Me.groupBox2.Controls.Add(Me.Label11)
        Me.groupBox2.Controls.Add(Me.Label10)
        Me.groupBox2.Controls.Add(Me.Label9)
        Me.groupBox2.Controls.Add(Me.Label8)
        Me.groupBox2.Controls.Add(Me.NumericUpDown_Axis)
        Me.groupBox2.Controls.Add(Me.numericUpDown_Tdcc)
        Me.groupBox2.Controls.Add(Me.Label7)
        Me.groupBox2.Controls.Add(Me.label6)
        Me.groupBox2.Controls.Add(Me.numericUpDown_Tacc)
        Me.groupBox2.Controls.Add(Me.label5)
        Me.groupBox2.Controls.Add(Me.numericUpDown_StopVel)
        Me.groupBox2.Controls.Add(Me.label4)
        Me.groupBox2.Controls.Add(Me.numericUpDown_HighVel)
        Me.groupBox2.Controls.Add(Me.label3)
        Me.groupBox2.Controls.Add(Me.numericUpDown_LowVel)
        Me.groupBox2.Controls.Add(Me.label2)
        Me.groupBox2.Location = New System.Drawing.Point(16, 6)
        Me.groupBox2.Margin = New System.Windows.Forms.Padding(4)
        Me.groupBox2.Name = "groupBox2"
        Me.groupBox2.Padding = New System.Windows.Forms.Padding(4)
        Me.groupBox2.Size = New System.Drawing.Size(273, 339)
        Me.groupBox2.TabIndex = 53
        Me.groupBox2.TabStop = False
        Me.groupBox2.Text = "运动参数"
        '
        'Label16
        '
        Me.Label16.AutoSize = True
        Me.Label16.Location = New System.Drawing.Point(216, 293)
        Me.Label16.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label16.Name = "Label16"
        Me.Label16.Size = New System.Drawing.Size(39, 15)
        Me.Label16.TabIndex = 76
        Me.Label16.Text = "unit"
        '
        'NumericUpDown_Homeoffset
        '
        Me.NumericUpDown_Homeoffset.DecimalPlaces = 2
        Me.NumericUpDown_Homeoffset.Location = New System.Drawing.Point(101, 290)
        Me.NumericUpDown_Homeoffset.Margin = New System.Windows.Forms.Padding(4)
        Me.NumericUpDown_Homeoffset.Maximum = New Decimal(New Integer() {10000, 0, 0, 0})
        Me.NumericUpDown_Homeoffset.Name = "NumericUpDown_Homeoffset"
        Me.NumericUpDown_Homeoffset.Size = New System.Drawing.Size(108, 25)
        Me.NumericUpDown_Homeoffset.TabIndex = 74
        '
        'Label18
        '
        Me.Label18.AutoSize = True
        Me.Label18.Location = New System.Drawing.Point(15, 294)
        Me.Label18.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label18.Name = "Label18"
        Me.Label18.Size = New System.Drawing.Size(75, 15)
        Me.Label18.TabIndex = 71
        Me.Label18.Text = "回零偏移:"
        '
        'NumericUpDown_HomeMode
        '
        Me.NumericUpDown_HomeMode.ForeColor = System.Drawing.SystemColors.WindowText
        Me.NumericUpDown_HomeMode.Location = New System.Drawing.Point(101, 254)
        Me.NumericUpDown_HomeMode.Margin = New System.Windows.Forms.Padding(4)
        Me.NumericUpDown_HomeMode.Maximum = New Decimal(New Integer() {10000, 0, 0, 0})
        Me.NumericUpDown_HomeMode.Name = "NumericUpDown_HomeMode"
        Me.NumericUpDown_HomeMode.Size = New System.Drawing.Size(108, 25)
        Me.NumericUpDown_HomeMode.TabIndex = 73
        Me.NumericUpDown_HomeMode.Value = New Decimal(New Integer() {1, 0, 0, 0})
        '
        'Label21
        '
        Me.Label21.AutoSize = True
        Me.Label21.Location = New System.Drawing.Point(15, 259)
        Me.Label21.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label21.Name = "Label21"
        Me.Label21.Size = New System.Drawing.Size(75, 15)
        Me.Label21.TabIndex = 72
        Me.Label21.Text = "回零模式:"
        '
        'Label12
        '
        Me.Label12.AutoSize = True
        Me.Label12.Location = New System.Drawing.Point(219, 221)
        Me.Label12.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label12.Name = "Label12"
        Me.Label12.Size = New System.Drawing.Size(15, 15)
        Me.Label12.TabIndex = 70
        Me.Label12.Text = "s"
        '
        'Label11
        '
        Me.Label11.AutoSize = True
        Me.Label11.Location = New System.Drawing.Point(219, 184)
        Me.Label11.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label11.Name = "Label11"
        Me.Label11.Size = New System.Drawing.Size(15, 15)
        Me.Label11.TabIndex = 69
        Me.Label11.Text = "s"
        '
        'Label10
        '
        Me.Label10.AutoSize = True
        Me.Label10.Location = New System.Drawing.Point(211, 152)
        Me.Label10.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label10.Name = "Label10"
        Me.Label10.Size = New System.Drawing.Size(55, 15)
        Me.Label10.TabIndex = 68
        Me.Label10.Text = "unit/s"
        '
        'Label9
        '
        Me.Label9.AutoSize = True
        Me.Label9.Location = New System.Drawing.Point(211, 116)
        Me.Label9.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label9.Name = "Label9"
        Me.Label9.Size = New System.Drawing.Size(55, 15)
        Me.Label9.TabIndex = 67
        Me.Label9.Text = "unit/s"
        '
        'Label8
        '
        Me.Label8.AutoSize = True
        Me.Label8.Location = New System.Drawing.Point(211, 78)
        Me.Label8.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label8.Name = "Label8"
        Me.Label8.Size = New System.Drawing.Size(55, 15)
        Me.Label8.TabIndex = 66
        Me.Label8.Text = "unit/s"
        '
        'NumericUpDown_Axis
        '
        Me.NumericUpDown_Axis.Location = New System.Drawing.Point(101, 38)
        Me.NumericUpDown_Axis.Margin = New System.Windows.Forms.Padding(4)
        Me.NumericUpDown_Axis.Maximum = New Decimal(New Integer() {-727379968, 232, 0, 0})
        Me.NumericUpDown_Axis.Name = "NumericUpDown_Axis"
        Me.NumericUpDown_Axis.Size = New System.Drawing.Size(108, 25)
        Me.NumericUpDown_Axis.TabIndex = 65
        '
        'numericUpDown_Tdcc
        '
        Me.numericUpDown_Tdcc.DecimalPlaces = 2
        Me.numericUpDown_Tdcc.Location = New System.Drawing.Point(101, 218)
        Me.numericUpDown_Tdcc.Margin = New System.Windows.Forms.Padding(4)
        Me.numericUpDown_Tdcc.Maximum = New Decimal(New Integer() {10000, 0, 0, 0})
        Me.numericUpDown_Tdcc.Name = "numericUpDown_Tdcc"
        Me.numericUpDown_Tdcc.Size = New System.Drawing.Size(108, 25)
        Me.numericUpDown_Tdcc.TabIndex = 1
        Me.numericUpDown_Tdcc.Value = New Decimal(New Integer() {1, 0, 0, 65536})
        '
        'Label7
        '
        Me.Label7.AutoSize = True
        Me.Label7.Location = New System.Drawing.Point(15, 46)
        Me.Label7.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label7.Name = "Label7"
        Me.Label7.Size = New System.Drawing.Size(75, 15)
        Me.Label7.TabIndex = 64
        Me.Label7.Text = "电机轴号:"
        '
        'label6
        '
        Me.label6.AutoSize = True
        Me.label6.Location = New System.Drawing.Point(15, 221)
        Me.label6.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label6.Name = "label6"
        Me.label6.Size = New System.Drawing.Size(75, 15)
        Me.label6.TabIndex = 0
        Me.label6.Text = "减速时间:"
        '
        'numericUpDown_Tacc
        '
        Me.numericUpDown_Tacc.DecimalPlaces = 2
        Me.numericUpDown_Tacc.Location = New System.Drawing.Point(101, 182)
        Me.numericUpDown_Tacc.Margin = New System.Windows.Forms.Padding(4)
        Me.numericUpDown_Tacc.Maximum = New Decimal(New Integer() {10000, 0, 0, 0})
        Me.numericUpDown_Tacc.Name = "numericUpDown_Tacc"
        Me.numericUpDown_Tacc.Size = New System.Drawing.Size(108, 25)
        Me.numericUpDown_Tacc.TabIndex = 1
        Me.numericUpDown_Tacc.Value = New Decimal(New Integer() {1, 0, 0, 65536})
        '
        'label5
        '
        Me.label5.AutoSize = True
        Me.label5.Location = New System.Drawing.Point(15, 186)
        Me.label5.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label5.Name = "label5"
        Me.label5.Size = New System.Drawing.Size(75, 15)
        Me.label5.TabIndex = 0
        Me.label5.Text = "加速时间:"
        '
        'numericUpDown_StopVel
        '
        Me.numericUpDown_StopVel.Location = New System.Drawing.Point(101, 146)
        Me.numericUpDown_StopVel.Margin = New System.Windows.Forms.Padding(4)
        Me.numericUpDown_StopVel.Maximum = New Decimal(New Integer() {-727379968, 232, 0, 0})
        Me.numericUpDown_StopVel.Name = "numericUpDown_StopVel"
        Me.numericUpDown_StopVel.Size = New System.Drawing.Size(108, 25)
        Me.numericUpDown_StopVel.TabIndex = 1
        Me.numericUpDown_StopVel.Value = New Decimal(New Integer() {100, 0, 0, 0})
        '
        'label4
        '
        Me.label4.AutoSize = True
        Me.label4.Location = New System.Drawing.Point(15, 149)
        Me.label4.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label4.Name = "label4"
        Me.label4.Size = New System.Drawing.Size(75, 15)
        Me.label4.TabIndex = 0
        Me.label4.Text = "停止速度:"
        '
        'numericUpDown_HighVel
        '
        Me.numericUpDown_HighVel.Location = New System.Drawing.Point(101, 110)
        Me.numericUpDown_HighVel.Margin = New System.Windows.Forms.Padding(4)
        Me.numericUpDown_HighVel.Maximum = New Decimal(New Integer() {-727379968, 232, 0, 0})
        Me.numericUpDown_HighVel.Name = "numericUpDown_HighVel"
        Me.numericUpDown_HighVel.Size = New System.Drawing.Size(108, 25)
        Me.numericUpDown_HighVel.TabIndex = 1
        Me.numericUpDown_HighVel.Value = New Decimal(New Integer() {1000, 0, 0, 0})
        '
        'label3
        '
        Me.label3.AutoSize = True
        Me.label3.Location = New System.Drawing.Point(15, 114)
        Me.label3.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label3.Name = "label3"
        Me.label3.Size = New System.Drawing.Size(75, 15)
        Me.label3.TabIndex = 0
        Me.label3.Text = "回零高速:"
        '
        'numericUpDown_LowVel
        '
        Me.numericUpDown_LowVel.Location = New System.Drawing.Point(101, 74)
        Me.numericUpDown_LowVel.Margin = New System.Windows.Forms.Padding(4)
        Me.numericUpDown_LowVel.Maximum = New Decimal(New Integer() {-727379968, 232, 0, 0})
        Me.numericUpDown_LowVel.Name = "numericUpDown_LowVel"
        Me.numericUpDown_LowVel.Size = New System.Drawing.Size(108, 25)
        Me.numericUpDown_LowVel.TabIndex = 1
        Me.numericUpDown_LowVel.Value = New Decimal(New Integer() {500, 0, 0, 0})
        '
        'label2
        '
        Me.label2.AutoSize = True
        Me.label2.Location = New System.Drawing.Point(15, 79)
        Me.label2.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label2.Name = "label2"
        Me.label2.Size = New System.Drawing.Size(75, 15)
        Me.label2.TabIndex = 0
        Me.label2.Text = "回零低速:"
        '
        'GroupBox1
        '
        Me.GroupBox1.Controls.Add(Me.RichTextBox1)
        Me.GroupBox1.Controls.Add(Me.TextBox5)
        Me.GroupBox1.Controls.Add(Me.Label22)
        Me.GroupBox1.Controls.Add(Me.Btn_SoftReset)
        Me.GroupBox1.Controls.Add(Me.Btn_HardReset)
        Me.GroupBox1.Location = New System.Drawing.Point(544, 205)
        Me.GroupBox1.Margin = New System.Windows.Forms.Padding(4)
        Me.GroupBox1.Name = "GroupBox1"
        Me.GroupBox1.Padding = New System.Windows.Forms.Padding(4)
        Me.GroupBox1.Size = New System.Drawing.Size(553, 140)
        Me.GroupBox1.TabIndex = 64
        Me.GroupBox1.TabStop = False
        Me.GroupBox1.Text = "复位操作及总线状态"
        '
        'RichTextBox1
        '
        Me.RichTextBox1.Location = New System.Drawing.Point(289, 45)
        Me.RichTextBox1.Margin = New System.Windows.Forms.Padding(4)
        Me.RichTextBox1.Name = "RichTextBox1"
        Me.RichTextBox1.Size = New System.Drawing.Size(255, 82)
        Me.RichTextBox1.TabIndex = 66
        Me.RichTextBox1.Text = ""
        '
        'TextBox5
        '
        Me.TextBox5.Location = New System.Drawing.Point(99, 48)
        Me.TextBox5.Margin = New System.Windows.Forms.Padding(4)
        Me.TextBox5.Name = "TextBox5"
        Me.TextBox5.ReadOnly = True
        Me.TextBox5.Size = New System.Drawing.Size(175, 25)
        Me.TextBox5.TabIndex = 65
        '
        'Label22
        '
        Me.Label22.AutoSize = True
        Me.Label22.Location = New System.Drawing.Point(19, 51)
        Me.Label22.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label22.Name = "Label22"
        Me.Label22.Size = New System.Drawing.Size(75, 15)
        Me.Label22.TabIndex = 64
        Me.Label22.Text = "总线状态:"
        '
        'Btn_SoftReset
        '
        Me.Btn_SoftReset.Location = New System.Drawing.Point(157, 86)
        Me.Btn_SoftReset.Margin = New System.Windows.Forms.Padding(4)
        Me.Btn_SoftReset.Name = "Btn_SoftReset"
        Me.Btn_SoftReset.Size = New System.Drawing.Size(115, 39)
        Me.Btn_SoftReset.TabIndex = 1
        Me.Btn_SoftReset.Text = "软件复位"
        Me.Btn_SoftReset.UseVisualStyleBackColor = True
        '
        'Btn_HardReset
        '
        Me.Btn_HardReset.Location = New System.Drawing.Point(21, 86)
        Me.Btn_HardReset.Margin = New System.Windows.Forms.Padding(4)
        Me.Btn_HardReset.Name = "Btn_HardReset"
        Me.Btn_HardReset.Size = New System.Drawing.Size(115, 39)
        Me.Btn_HardReset.TabIndex = 0
        Me.Btn_HardReset.Text = "硬件复位"
        Me.Btn_HardReset.UseVisualStyleBackColor = True
        '
        'label19
        '
        Me.label19.AutoSize = True
        Me.label19.Location = New System.Drawing.Point(17, 35)
        Me.label19.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label19.Name = "label19"
        Me.label19.Size = New System.Drawing.Size(75, 15)
        Me.label19.TabIndex = 56
        Me.label19.Text = "当前速度:"
        '
        'TextBox2
        '
        Me.TextBox2.Location = New System.Drawing.Point(104, 71)
        Me.TextBox2.Margin = New System.Windows.Forms.Padding(4)
        Me.TextBox2.Name = "TextBox2"
        Me.TextBox2.ReadOnly = True
        Me.TextBox2.Size = New System.Drawing.Size(104, 25)
        Me.TextBox2.TabIndex = 59
        '
        'label14
        '
        Me.label14.AutoSize = True
        Me.label14.Location = New System.Drawing.Point(217, 75)
        Me.label14.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label14.Name = "label14"
        Me.label14.Size = New System.Drawing.Size(39, 15)
        Me.label14.TabIndex = 61
        Me.label14.Text = "unit"
        '
        'label13
        '
        Me.label13.AutoSize = True
        Me.label13.Location = New System.Drawing.Point(17, 72)
        Me.label13.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label13.Name = "label13"
        Me.label13.Size = New System.Drawing.Size(75, 15)
        Me.label13.TabIndex = 57
        Me.label13.Text = "当前位置:"
        '
        'label20
        '
        Me.label20.AutoSize = True
        Me.label20.Location = New System.Drawing.Point(219, 35)
        Me.label20.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.label20.Name = "label20"
        Me.label20.Size = New System.Drawing.Size(55, 15)
        Me.label20.TabIndex = 60
        Me.label20.Text = "unit/s"
        '
        'textBox1
        '
        Me.textBox1.Location = New System.Drawing.Point(104, 111)
        Me.textBox1.Margin = New System.Windows.Forms.Padding(4)
        Me.textBox1.Name = "textBox1"
        Me.textBox1.ReadOnly = True
        Me.textBox1.Size = New System.Drawing.Size(104, 25)
        Me.textBox1.TabIndex = 47
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Location = New System.Drawing.Point(17, 115)
        Me.Label1.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(75, 15)
        Me.Label1.TabIndex = 62
        Me.Label1.Text = "运动状态:"
        '
        'textBox3
        '
        Me.textBox3.Location = New System.Drawing.Point(104, 31)
        Me.textBox3.Margin = New System.Windows.Forms.Padding(4)
        Me.textBox3.Name = "textBox3"
        Me.textBox3.ReadOnly = True
        Me.textBox3.Size = New System.Drawing.Size(104, 25)
        Me.textBox3.TabIndex = 58
        '
        'Label25
        '
        Me.Label25.AutoSize = True
        Me.Label25.Location = New System.Drawing.Point(295, 70)
        Me.Label25.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label25.Name = "Label25"
        Me.Label25.Size = New System.Drawing.Size(37, 15)
        Me.Label25.TabIndex = 68
        Me.Label25.Text = "原点"
        '
        'Label24
        '
        Me.Label24.AutoSize = True
        Me.Label24.Location = New System.Drawing.Point(367, 70)
        Me.Label24.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label24.Name = "Label24"
        Me.Label24.Size = New System.Drawing.Size(52, 15)
        Me.Label24.TabIndex = 69
        Me.Label24.Text = "正限位"
        '
        'Label23
        '
        Me.Label23.AutoSize = True
        Me.Label23.Location = New System.Drawing.Point(451, 70)
        Me.Label23.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label23.Name = "Label23"
        Me.Label23.Size = New System.Drawing.Size(52, 15)
        Me.Label23.TabIndex = 70
        Me.Label23.Text = "负限位"
        '
        'Button4
        '
        Me.Button4.Enabled = False
        Me.Button4.Location = New System.Drawing.Point(291, 89)
        Me.Button4.Margin = New System.Windows.Forms.Padding(4)
        Me.Button4.Name = "Button4"
        Me.Button4.Size = New System.Drawing.Size(52, 48)
        Me.Button4.TabIndex = 67
        Me.Button4.UseVisualStyleBackColor = True
        '
        'Button6
        '
        Me.Button6.Enabled = False
        Me.Button6.Location = New System.Drawing.Point(369, 89)
        Me.Button6.Margin = New System.Windows.Forms.Padding(4)
        Me.Button6.Name = "Button6"
        Me.Button6.Size = New System.Drawing.Size(52, 48)
        Me.Button6.TabIndex = 71
        Me.Button6.UseVisualStyleBackColor = True
        '
        'Button8
        '
        Me.Button8.Enabled = False
        Me.Button8.Location = New System.Drawing.Point(453, 89)
        Me.Button8.Margin = New System.Windows.Forms.Padding(4)
        Me.Button8.Name = "Button8"
        Me.Button8.Size = New System.Drawing.Size(52, 48)
        Me.Button8.TabIndex = 72
        Me.Button8.UseVisualStyleBackColor = True
        '
        'Label26
        '
        Me.Label26.AutoSize = True
        Me.Label26.Location = New System.Drawing.Point(17, 158)
        Me.Label26.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label26.Name = "Label26"
        Me.Label26.Size = New System.Drawing.Size(75, 15)
        Me.Label26.TabIndex = 73
        Me.Label26.Text = "轴状态机:"
        '
        'TextBox6
        '
        Me.TextBox6.Location = New System.Drawing.Point(104, 151)
        Me.TextBox6.Margin = New System.Windows.Forms.Padding(4)
        Me.TextBox6.Name = "TextBox6"
        Me.TextBox6.ReadOnly = True
        Me.TextBox6.Size = New System.Drawing.Size(169, 25)
        Me.TextBox6.TabIndex = 74
        '
        'GroupBox6
        '
        Me.GroupBox6.Controls.Add(Me.TextBox6)
        Me.GroupBox6.Controls.Add(Me.Label26)
        Me.GroupBox6.Controls.Add(Me.Button8)
        Me.GroupBox6.Controls.Add(Me.Button6)
        Me.GroupBox6.Controls.Add(Me.Button4)
        Me.GroupBox6.Controls.Add(Me.Label23)
        Me.GroupBox6.Controls.Add(Me.Label24)
        Me.GroupBox6.Controls.Add(Me.Label25)
        Me.GroupBox6.Controls.Add(Me.textBox3)
        Me.GroupBox6.Controls.Add(Me.Label1)
        Me.GroupBox6.Controls.Add(Me.textBox1)
        Me.GroupBox6.Controls.Add(Me.label20)
        Me.GroupBox6.Controls.Add(Me.label13)
        Me.GroupBox6.Controls.Add(Me.label14)
        Me.GroupBox6.Controls.Add(Me.TextBox2)
        Me.GroupBox6.Controls.Add(Me.label19)
        Me.GroupBox6.Location = New System.Drawing.Point(545, 6)
        Me.GroupBox6.Margin = New System.Windows.Forms.Padding(4)
        Me.GroupBox6.Name = "GroupBox6"
        Me.GroupBox6.Padding = New System.Windows.Forms.Padding(4)
        Me.GroupBox6.Size = New System.Drawing.Size(552, 191)
        Me.GroupBox6.TabIndex = 63
        Me.GroupBox6.TabStop = False
        Me.GroupBox6.Text = "信息显示"
        '
        'groupBox4
        '
        Me.groupBox4.Controls.Add(Me.radioButton5)
        Me.groupBox4.Controls.Add(Me.radioButton6)
        Me.groupBox4.Location = New System.Drawing.Point(304, 262)
        Me.groupBox4.Margin = New System.Windows.Forms.Padding(4)
        Me.groupBox4.Name = "groupBox4"
        Me.groupBox4.Padding = New System.Windows.Forms.Padding(4)
        Me.groupBox4.Size = New System.Drawing.Size(221, 83)
        Me.groupBox4.TabIndex = 65
        Me.groupBox4.TabStop = False
        Me.groupBox4.Text = "回原点速度"
        '
        'radioButton5
        '
        Me.radioButton5.AutoSize = True
        Me.radioButton5.Location = New System.Drawing.Point(121, 38)
        Me.radioButton5.Margin = New System.Windows.Forms.Padding(4)
        Me.radioButton5.Name = "radioButton5"
        Me.radioButton5.Size = New System.Drawing.Size(58, 19)
        Me.radioButton5.TabIndex = 0
        Me.radioButton5.Text = "高速"
        Me.radioButton5.UseVisualStyleBackColor = True
        '
        'radioButton6
        '
        Me.radioButton6.AutoSize = True
        Me.radioButton6.Checked = True
        Me.radioButton6.Location = New System.Drawing.Point(35, 38)
        Me.radioButton6.Margin = New System.Windows.Forms.Padding(4)
        Me.radioButton6.Name = "radioButton6"
        Me.radioButton6.Size = New System.Drawing.Size(58, 19)
        Me.radioButton6.TabIndex = 0
        Me.radioButton6.TabStop = True
        Me.radioButton6.Text = "低速"
        Me.radioButton6.UseVisualStyleBackColor = True
        '
        'groupBox5
        '
        Me.groupBox5.Controls.Add(Me.radioButton11)
        Me.groupBox5.Controls.Add(Me.radioButton9)
        Me.groupBox5.Controls.Add(Me.radioButton10)
        Me.groupBox5.Location = New System.Drawing.Point(308, 6)
        Me.groupBox5.Margin = New System.Windows.Forms.Padding(4)
        Me.groupBox5.Name = "groupBox5"
        Me.groupBox5.Padding = New System.Windows.Forms.Padding(4)
        Me.groupBox5.Size = New System.Drawing.Size(217, 164)
        Me.groupBox5.TabIndex = 66
        Me.groupBox5.TabStop = False
        Me.groupBox5.Text = "回原点方式"
        '
        'radioButton11
        '
        Me.radioButton11.AutoSize = True
        Me.radioButton11.Location = New System.Drawing.Point(35, 121)
        Me.radioButton11.Margin = New System.Windows.Forms.Padding(4)
        Me.radioButton11.Name = "radioButton11"
        Me.radioButton11.Size = New System.Drawing.Size(88, 19)
        Me.radioButton11.TabIndex = 0
        Me.radioButton11.Text = "两次回零"
        Me.radioButton11.UseVisualStyleBackColor = True
        '
        'radioButton9
        '
        Me.radioButton9.AutoSize = True
        Me.radioButton9.Location = New System.Drawing.Point(35, 79)
        Me.radioButton9.Margin = New System.Windows.Forms.Padding(4)
        Me.radioButton9.Name = "radioButton9"
        Me.radioButton9.Size = New System.Drawing.Size(126, 19)
        Me.radioButton9.TabIndex = 0
        Me.radioButton9.Text = "一次回零+反找"
        Me.radioButton9.UseVisualStyleBackColor = True
        '
        'radioButton10
        '
        Me.radioButton10.AutoSize = True
        Me.radioButton10.Checked = True
        Me.radioButton10.Location = New System.Drawing.Point(35, 37)
        Me.radioButton10.Margin = New System.Windows.Forms.Padding(4)
        Me.radioButton10.Name = "radioButton10"
        Me.radioButton10.Size = New System.Drawing.Size(88, 19)
        Me.radioButton10.TabIndex = 0
        Me.radioButton10.TabStop = True
        Me.radioButton10.Text = "一次回零"
        Me.radioButton10.UseVisualStyleBackColor = True
        '
        'groupBox3
        '
        Me.groupBox3.Controls.Add(Me.radioButton7)
        Me.groupBox3.Controls.Add(Me.radioButton8)
        Me.groupBox3.Location = New System.Drawing.Point(308, 182)
        Me.groupBox3.Margin = New System.Windows.Forms.Padding(4)
        Me.groupBox3.Name = "groupBox3"
        Me.groupBox3.Padding = New System.Windows.Forms.Padding(4)
        Me.groupBox3.Size = New System.Drawing.Size(217, 74)
        Me.groupBox3.TabIndex = 67
        Me.groupBox3.TabStop = False
        Me.groupBox3.Text = "回原点方向"
        '
        'radioButton7
        '
        Me.radioButton7.AutoSize = True
        Me.radioButton7.Checked = True
        Me.radioButton7.Location = New System.Drawing.Point(35, 33)
        Me.radioButton7.Margin = New System.Windows.Forms.Padding(4)
        Me.radioButton7.Name = "radioButton7"
        Me.radioButton7.Size = New System.Drawing.Size(58, 19)
        Me.radioButton7.TabIndex = 0
        Me.radioButton7.TabStop = True
        Me.radioButton7.Text = "正向"
        Me.radioButton7.UseVisualStyleBackColor = True
        '
        'radioButton8
        '
        Me.radioButton8.AutoSize = True
        Me.radioButton8.Location = New System.Drawing.Point(117, 32)
        Me.radioButton8.Margin = New System.Windows.Forms.Padding(4)
        Me.radioButton8.Name = "radioButton8"
        Me.radioButton8.Size = New System.Drawing.Size(58, 19)
        Me.radioButton8.TabIndex = 0
        Me.radioButton8.Text = "反向"
        Me.radioButton8.UseVisualStyleBackColor = True
        '
        'Button1
        '
        Me.Button1.Location = New System.Drawing.Point(62, 363)
        Me.Button1.Margin = New System.Windows.Forms.Padding(4)
        Me.Button1.Name = "Button1"
        Me.Button1.Size = New System.Drawing.Size(124, 39)
        Me.Button1.TabIndex = 68
        Me.Button1.Text = "脉冲轴回零"
        Me.Button1.UseVisualStyleBackColor = True
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(8.0!, 15.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(1111, 410)
        Me.Controls.Add(Me.Button1)
        Me.Controls.Add(Me.groupBox4)
        Me.Controls.Add(Me.groupBox5)
        Me.Controls.Add(Me.groupBox3)
        Me.Controls.Add(Me.GroupBox1)
        Me.Controls.Add(Me.GroupBox6)
        Me.Controls.Add(Me.Btn_Close)
        Me.Controls.Add(Me.Btn_StopMove)
        Me.Controls.Add(Me.Btn_TdccStopMove)
        Me.Controls.Add(Me.Btn_ResetPos)
        Me.Controls.Add(Me.Btn_StartMove)
        Me.Controls.Add(Me.groupBox2)
        Me.Margin = New System.Windows.Forms.Padding(4)
        Me.Name = "Form1"
        Me.Text = "回原点运动"
        Me.groupBox2.ResumeLayout(False)
        Me.groupBox2.PerformLayout()
        CType(Me.NumericUpDown_Homeoffset, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.NumericUpDown_HomeMode, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.NumericUpDown_Axis, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_Tdcc, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_Tacc, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_StopVel, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_HighVel, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.numericUpDown_LowVel, System.ComponentModel.ISupportInitialize).EndInit()
        Me.GroupBox1.ResumeLayout(False)
        Me.GroupBox1.PerformLayout()
        Me.GroupBox6.ResumeLayout(False)
        Me.GroupBox6.PerformLayout()
        Me.groupBox4.ResumeLayout(False)
        Me.groupBox4.PerformLayout()
        Me.groupBox5.ResumeLayout(False)
        Me.groupBox5.PerformLayout()
        Me.groupBox3.ResumeLayout(False)
        Me.groupBox3.PerformLayout()
        Me.ResumeLayout(False)

    End Sub
    Friend WithEvents Timer1 As System.Windows.Forms.Timer
    Private WithEvents Btn_Close As System.Windows.Forms.Button
    Private WithEvents Btn_StopMove As System.Windows.Forms.Button
    Private WithEvents Btn_TdccStopMove As System.Windows.Forms.Button
    Private WithEvents Btn_ResetPos As System.Windows.Forms.Button
    Private WithEvents Btn_StartMove As System.Windows.Forms.Button
    Private WithEvents groupBox2 As System.Windows.Forms.GroupBox
    Private WithEvents numericUpDown_Tdcc As System.Windows.Forms.NumericUpDown
    Private WithEvents label6 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_Tacc As System.Windows.Forms.NumericUpDown
    Private WithEvents label5 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_StopVel As System.Windows.Forms.NumericUpDown
    Private WithEvents label4 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_HighVel As System.Windows.Forms.NumericUpDown
    Private WithEvents label3 As System.Windows.Forms.Label
    Private WithEvents numericUpDown_LowVel As System.Windows.Forms.NumericUpDown
    Private WithEvents label2 As System.Windows.Forms.Label
    Private WithEvents Label12 As System.Windows.Forms.Label
    Private WithEvents Label11 As System.Windows.Forms.Label
    Private WithEvents Label10 As System.Windows.Forms.Label
    Private WithEvents Label9 As System.Windows.Forms.Label
    Private WithEvents Label8 As System.Windows.Forms.Label
    Private WithEvents NumericUpDown_Axis As System.Windows.Forms.NumericUpDown
    Private WithEvents Label7 As System.Windows.Forms.Label
    Private WithEvents Label16 As System.Windows.Forms.Label
    Private WithEvents NumericUpDown_Homeoffset As System.Windows.Forms.NumericUpDown
    Private WithEvents Label18 As System.Windows.Forms.Label
    Private WithEvents NumericUpDown_HomeMode As System.Windows.Forms.NumericUpDown
    Private WithEvents Label21 As System.Windows.Forms.Label
    Friend WithEvents GroupBox1 As System.Windows.Forms.GroupBox
    Friend WithEvents RichTextBox1 As System.Windows.Forms.RichTextBox
    Private WithEvents TextBox5 As System.Windows.Forms.TextBox
    Private WithEvents Label22 As System.Windows.Forms.Label
    Friend WithEvents Btn_SoftReset As System.Windows.Forms.Button
    Friend WithEvents Btn_HardReset As System.Windows.Forms.Button
    Private WithEvents label19 As System.Windows.Forms.Label
    Private WithEvents TextBox2 As System.Windows.Forms.TextBox
    Private WithEvents label14 As System.Windows.Forms.Label
    Private WithEvents label13 As System.Windows.Forms.Label
    Private WithEvents label20 As System.Windows.Forms.Label
    Private WithEvents textBox1 As System.Windows.Forms.TextBox
    Private WithEvents Label1 As System.Windows.Forms.Label
    Private WithEvents textBox3 As System.Windows.Forms.TextBox
    Private WithEvents Label25 As System.Windows.Forms.Label
    Private WithEvents Label24 As System.Windows.Forms.Label
    Private WithEvents Label23 As System.Windows.Forms.Label
    Friend WithEvents Button4 As System.Windows.Forms.Button
    Friend WithEvents Button6 As System.Windows.Forms.Button
    Friend WithEvents Button8 As System.Windows.Forms.Button
    Private WithEvents Label26 As System.Windows.Forms.Label
    Private WithEvents TextBox6 As System.Windows.Forms.TextBox
    Friend WithEvents GroupBox6 As System.Windows.Forms.GroupBox
    Private WithEvents groupBox4 As GroupBox
    Private WithEvents radioButton5 As RadioButton
    Private WithEvents radioButton6 As RadioButton
    Private WithEvents groupBox5 As GroupBox
    Private WithEvents radioButton11 As RadioButton
    Private WithEvents radioButton9 As RadioButton
    Private WithEvents radioButton10 As RadioButton
    Private WithEvents groupBox3 As GroupBox
    Private WithEvents radioButton7 As RadioButton
    Private WithEvents radioButton8 As RadioButton
    Private WithEvents Button1 As Button
End Class
