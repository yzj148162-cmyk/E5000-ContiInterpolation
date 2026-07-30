namespace 定长运动
{
    partial class Form1
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows 窗体设计器生成的代码

        /// <summary>
        /// 设计器支持所需的方法 - 不要
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.nud_Spara = new System.Windows.Forms.NumericUpDown();
            this.label13 = new System.Windows.Forms.Label();
            this.nud_Tdec = new System.Windows.Forms.NumericUpDown();
            this.label11 = new System.Windows.Forms.Label();
            this.nud_Tacc = new System.Windows.Forms.NumericUpDown();
            this.label9 = new System.Windows.Forms.Label();
            this.nud_Dist = new System.Windows.Forms.NumericUpDown();
            this.nud_StopVel = new System.Windows.Forms.NumericUpDown();
            this.label17 = new System.Windows.Forms.Label();
            this.nud_MaxVel = new System.Windows.Forms.NumericUpDown();
            this.label15 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.nud_StartVel = new System.Windows.Forms.NumericUpDown();
            this.label16 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.nud_PulseEquiv = new System.Windows.Forms.NumericUpDown();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.nud_AxisId = new System.Windows.Forms.NumericUpDown();
            this.label1 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label30 = new System.Windows.Forms.Label();
            this.textBox_StateMachine = new System.Windows.Forms.TextBox();
            this.tb_RunState = new System.Windows.Forms.TextBox();
            this.tb_Encoder = new System.Windows.Forms.TextBox();
            this.label23 = new System.Windows.Forms.Label();
            this.tb_CurrentPos = new System.Windows.Forms.TextBox();
            this.label24 = new System.Windows.Forms.Label();
            this.label21 = new System.Windows.Forms.Label();
            this.label22 = new System.Windows.Forms.Label();
            this.tb_CurrentVel = new System.Windows.Forms.TextBox();
            this.label20 = new System.Windows.Forms.Label();
            this.label19 = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.btn_ChangeVel = new System.Windows.Forms.Button();
            this.nud_NewVel = new System.Windows.Forms.NumericUpDown();
            this.label25 = new System.Windows.Forms.Label();
            this.nud_TaccDec = new System.Windows.Forms.NumericUpDown();
            this.label28 = new System.Windows.Forms.Label();
            this.label26 = new System.Windows.Forms.Label();
            this.label27 = new System.Windows.Forms.Label();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.btn_ChangePos = new System.Windows.Forms.Button();
            this.nud_NewPos = new System.Windows.Forms.NumericUpDown();
            this.label29 = new System.Windows.Forms.Label();
            this.label31 = new System.Windows.Forms.Label();
            this.btn_Start = new System.Windows.Forms.Button();
            this.btn_ClearPos = new System.Windows.Forms.Button();
            this.btn_Esc = new System.Windows.Forms.Button();
            this.btn_SetPulseEquiv = new System.Windows.Forms.Button();
            this.btn_Stop = new System.Windows.Forms.Button();
            this.richTextBox_Message = new System.Windows.Forms.RichTextBox();
            this.button_SoftwareReset = new System.Windows.Forms.Button();
            this.button_HardwareReset = new System.Windows.Forms.Button();
            this.textBox_EthercatState = new System.Windows.Forms.TextBox();
            this.label32 = new System.Windows.Forms.Label();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.groupBox1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nud_Spara)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_Tdec)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_Tacc)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_Dist)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_StopVel)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_MaxVel)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_StartVel)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_PulseEquiv)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_AxisId)).BeginInit();
            this.groupBox2.SuspendLayout();
            this.groupBox4.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nud_NewVel)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_TaccDec)).BeginInit();
            this.groupBox5.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nud_NewPos)).BeginInit();
            this.groupBox3.SuspendLayout();
            this.SuspendLayout();
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.nud_Spara);
            this.groupBox1.Controls.Add(this.label13);
            this.groupBox1.Controls.Add(this.nud_Tdec);
            this.groupBox1.Controls.Add(this.label11);
            this.groupBox1.Controls.Add(this.nud_Tacc);
            this.groupBox1.Controls.Add(this.label9);
            this.groupBox1.Controls.Add(this.nud_Dist);
            this.groupBox1.Controls.Add(this.nud_StopVel);
            this.groupBox1.Controls.Add(this.label17);
            this.groupBox1.Controls.Add(this.nud_MaxVel);
            this.groupBox1.Controls.Add(this.label15);
            this.groupBox1.Controls.Add(this.label12);
            this.groupBox1.Controls.Add(this.label7);
            this.groupBox1.Controls.Add(this.label10);
            this.groupBox1.Controls.Add(this.nud_StartVel);
            this.groupBox1.Controls.Add(this.label16);
            this.groupBox1.Controls.Add(this.label8);
            this.groupBox1.Controls.Add(this.label14);
            this.groupBox1.Controls.Add(this.label5);
            this.groupBox1.Controls.Add(this.label6);
            this.groupBox1.Controls.Add(this.nud_PulseEquiv);
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.nud_AxisId);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Location = new System.Drawing.Point(10, 10);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(280, 291);
            this.groupBox1.TabIndex = 5;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "输入参数";
            // 
            // nud_Spara
            // 
            this.nud_Spara.DecimalPlaces = 3;
            this.nud_Spara.Location = new System.Drawing.Point(93, 182);
            this.nud_Spara.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_Spara.Name = "nud_Spara";
            this.nud_Spara.Size = new System.Drawing.Size(93, 21);
            this.nud_Spara.TabIndex = 1;
            this.nud_Spara.Value = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(192, 186);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(11, 12);
            this.label13.TabIndex = 0;
            this.label13.Text = "s";
            // 
            // nud_Tdec
            // 
            this.nud_Tdec.DecimalPlaces = 3;
            this.nud_Tdec.Location = new System.Drawing.Point(93, 155);
            this.nud_Tdec.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_Tdec.Name = "nud_Tdec";
            this.nud_Tdec.Size = new System.Drawing.Size(93, 21);
            this.nud_Tdec.TabIndex = 1;
            this.nud_Tdec.Value = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(192, 159);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(11, 12);
            this.label11.TabIndex = 0;
            this.label11.Text = "s";
            // 
            // nud_Tacc
            // 
            this.nud_Tacc.DecimalPlaces = 3;
            this.nud_Tacc.Location = new System.Drawing.Point(93, 128);
            this.nud_Tacc.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_Tacc.Name = "nud_Tacc";
            this.nud_Tacc.Size = new System.Drawing.Size(93, 21);
            this.nud_Tacc.TabIndex = 1;
            this.nud_Tacc.Value = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(192, 132);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(11, 12);
            this.label9.TabIndex = 0;
            this.label9.Text = "s";
            // 
            // nud_Dist
            // 
            this.nud_Dist.DecimalPlaces = 3;
            this.nud_Dist.Location = new System.Drawing.Point(93, 236);
            this.nud_Dist.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_Dist.Name = "nud_Dist";
            this.nud_Dist.Size = new System.Drawing.Size(93, 21);
            this.nud_Dist.TabIndex = 1;
            this.nud_Dist.Value = new decimal(new int[] {
            15000,
            0,
            0,
            0});
            // 
            // nud_StopVel
            // 
            this.nud_StopVel.DecimalPlaces = 3;
            this.nud_StopVel.Location = new System.Drawing.Point(93, 209);
            this.nud_StopVel.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_StopVel.Name = "nud_StopVel";
            this.nud_StopVel.Size = new System.Drawing.Size(93, 21);
            this.nud_StopVel.TabIndex = 1;
            this.nud_StopVel.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // label17
            // 
            this.label17.AutoSize = true;
            this.label17.Location = new System.Drawing.Point(192, 240);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(29, 12);
            this.label17.TabIndex = 0;
            this.label17.Text = "unit";
            // 
            // nud_MaxVel
            // 
            this.nud_MaxVel.DecimalPlaces = 3;
            this.nud_MaxVel.Location = new System.Drawing.Point(93, 101);
            this.nud_MaxVel.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_MaxVel.Name = "nud_MaxVel";
            this.nud_MaxVel.Size = new System.Drawing.Size(93, 21);
            this.nud_MaxVel.TabIndex = 1;
            this.nud_MaxVel.Value = new decimal(new int[] {
            3000,
            0,
            0,
            0});
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(192, 213);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(41, 12);
            this.label15.TabIndex = 0;
            this.label15.Text = "unit/s";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(27, 186);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(53, 12);
            this.label12.TabIndex = 0;
            this.label12.Text = "S段时间:";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(192, 105);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(41, 12);
            this.label7.TabIndex = 0;
            this.label7.Text = "unit/s";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(27, 159);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(59, 12);
            this.label10.TabIndex = 0;
            this.label10.Text = "减速时间:";
            // 
            // nud_StartVel
            // 
            this.nud_StartVel.DecimalPlaces = 3;
            this.nud_StartVel.Location = new System.Drawing.Point(93, 74);
            this.nud_StartVel.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_StartVel.Name = "nud_StartVel";
            this.nud_StartVel.Size = new System.Drawing.Size(93, 21);
            this.nud_StartVel.TabIndex = 1;
            this.nud_StartVel.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Location = new System.Drawing.Point(27, 240);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(59, 12);
            this.label16.TabIndex = 0;
            this.label16.Text = "运行距离:";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(27, 132);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(59, 12);
            this.label8.TabIndex = 0;
            this.label8.Text = "加速时间:";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(27, 213);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(59, 12);
            this.label14.TabIndex = 0;
            this.label14.Text = "停止速度:";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(192, 78);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(41, 12);
            this.label5.TabIndex = 0;
            this.label5.Text = "unit/s";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(27, 105);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(59, 12);
            this.label6.TabIndex = 0;
            this.label6.Text = "运行速度:";
            // 
            // nud_PulseEquiv
            // 
            this.nud_PulseEquiv.DecimalPlaces = 3;
            this.nud_PulseEquiv.Location = new System.Drawing.Point(93, 47);
            this.nud_PulseEquiv.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_PulseEquiv.Name = "nud_PulseEquiv";
            this.nud_PulseEquiv.Size = new System.Drawing.Size(93, 21);
            this.nud_PulseEquiv.TabIndex = 1;
            this.nud_PulseEquiv.Value = new decimal(new int[] {
            100,
            0,
            0,
            0});
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(27, 78);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(59, 12);
            this.label4.TabIndex = 0;
            this.label4.Text = "起始速度:";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(192, 51);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(65, 12);
            this.label3.TabIndex = 0;
            this.label3.Text = "pulse/unit";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(27, 51);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(59, 12);
            this.label2.TabIndex = 0;
            this.label2.Text = "脉冲当量:";
            // 
            // nud_AxisId
            // 
            this.nud_AxisId.Location = new System.Drawing.Point(93, 20);
            this.nud_AxisId.Maximum = new decimal(new int[] {
            32,
            0,
            0,
            0});
            this.nud_AxisId.Name = "nud_AxisId";
            this.nud_AxisId.Size = new System.Drawing.Size(93, 21);
            this.nud_AxisId.TabIndex = 1;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(27, 24);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(59, 12);
            this.label1.TabIndex = 0;
            this.label1.Text = "电机轴号:";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.label30);
            this.groupBox2.Controls.Add(this.textBox_StateMachine);
            this.groupBox2.Controls.Add(this.tb_RunState);
            this.groupBox2.Controls.Add(this.tb_Encoder);
            this.groupBox2.Controls.Add(this.label23);
            this.groupBox2.Controls.Add(this.tb_CurrentPos);
            this.groupBox2.Controls.Add(this.label24);
            this.groupBox2.Controls.Add(this.label21);
            this.groupBox2.Controls.Add(this.label22);
            this.groupBox2.Controls.Add(this.tb_CurrentVel);
            this.groupBox2.Controls.Add(this.label20);
            this.groupBox2.Controls.Add(this.label19);
            this.groupBox2.Controls.Add(this.label18);
            this.groupBox2.Location = new System.Drawing.Point(522, 15);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(238, 193);
            this.groupBox2.TabIndex = 11;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "信息显示";
            // 
            // label30
            // 
            this.label30.AutoSize = true;
            this.label30.Location = new System.Drawing.Point(20, 150);
            this.label30.Name = "label30";
            this.label30.Size = new System.Drawing.Size(59, 12);
            this.label30.TabIndex = 6;
            this.label30.Text = "轴状态机:";
            // 
            // textBox_StateMachine
            // 
            this.textBox_StateMachine.Location = new System.Drawing.Point(85, 147);
            this.textBox_StateMachine.Name = "textBox_StateMachine";
            this.textBox_StateMachine.ReadOnly = true;
            this.textBox_StateMachine.Size = new System.Drawing.Size(121, 21);
            this.textBox_StateMachine.TabIndex = 5;
            // 
            // tb_RunState
            // 
            this.tb_RunState.Location = new System.Drawing.Point(85, 115);
            this.tb_RunState.Name = "tb_RunState";
            this.tb_RunState.ReadOnly = true;
            this.tb_RunState.Size = new System.Drawing.Size(88, 21);
            this.tb_RunState.TabIndex = 1;
            // 
            // tb_Encoder
            // 
            this.tb_Encoder.Location = new System.Drawing.Point(85, 88);
            this.tb_Encoder.Name = "tb_Encoder";
            this.tb_Encoder.ReadOnly = true;
            this.tb_Encoder.Size = new System.Drawing.Size(88, 21);
            this.tb_Encoder.TabIndex = 1;
            // 
            // label23
            // 
            this.label23.AutoSize = true;
            this.label23.Location = new System.Drawing.Point(179, 92);
            this.label23.Name = "label23";
            this.label23.Size = new System.Drawing.Size(29, 12);
            this.label23.TabIndex = 0;
            this.label23.Text = "unit";
            // 
            // tb_CurrentPos
            // 
            this.tb_CurrentPos.Location = new System.Drawing.Point(85, 61);
            this.tb_CurrentPos.Name = "tb_CurrentPos";
            this.tb_CurrentPos.ReadOnly = true;
            this.tb_CurrentPos.Size = new System.Drawing.Size(88, 21);
            this.tb_CurrentPos.TabIndex = 1;
            // 
            // label24
            // 
            this.label24.AutoSize = true;
            this.label24.Location = new System.Drawing.Point(20, 119);
            this.label24.Name = "label24";
            this.label24.Size = new System.Drawing.Size(59, 12);
            this.label24.TabIndex = 0;
            this.label24.Text = "运动状态:";
            // 
            // label21
            // 
            this.label21.AutoSize = true;
            this.label21.Location = new System.Drawing.Point(179, 65);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(29, 12);
            this.label21.TabIndex = 0;
            this.label21.Text = "unit";
            // 
            // label22
            // 
            this.label22.AutoSize = true;
            this.label22.Location = new System.Drawing.Point(20, 92);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(59, 12);
            this.label22.TabIndex = 0;
            this.label22.Text = "反馈位置:";
            // 
            // tb_CurrentVel
            // 
            this.tb_CurrentVel.Location = new System.Drawing.Point(85, 37);
            this.tb_CurrentVel.Name = "tb_CurrentVel";
            this.tb_CurrentVel.ReadOnly = true;
            this.tb_CurrentVel.Size = new System.Drawing.Size(88, 21);
            this.tb_CurrentVel.TabIndex = 1;
            // 
            // label20
            // 
            this.label20.AutoSize = true;
            this.label20.Location = new System.Drawing.Point(20, 65);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(59, 12);
            this.label20.TabIndex = 0;
            this.label20.Text = "当前位置:";
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.Location = new System.Drawing.Point(179, 41);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(41, 12);
            this.label19.TabIndex = 0;
            this.label19.Text = "unit/s";
            // 
            // label18
            // 
            this.label18.AutoSize = true;
            this.label18.Location = new System.Drawing.Point(20, 41);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(59, 12);
            this.label18.TabIndex = 0;
            this.label18.Text = "当前速度:";
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.btn_ChangeVel);
            this.groupBox4.Controls.Add(this.nud_NewVel);
            this.groupBox4.Controls.Add(this.label25);
            this.groupBox4.Controls.Add(this.nud_TaccDec);
            this.groupBox4.Controls.Add(this.label28);
            this.groupBox4.Controls.Add(this.label26);
            this.groupBox4.Controls.Add(this.label27);
            this.groupBox4.Location = new System.Drawing.Point(296, 15);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(220, 102);
            this.groupBox4.TabIndex = 13;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "在线变速";
            // 
            // btn_ChangeVel
            // 
            this.btn_ChangeVel.Location = new System.Drawing.Point(79, 68);
            this.btn_ChangeVel.Name = "btn_ChangeVel";
            this.btn_ChangeVel.Size = new System.Drawing.Size(75, 30);
            this.btn_ChangeVel.TabIndex = 1;
            this.btn_ChangeVel.Text = "在线变速";
            this.btn_ChangeVel.UseVisualStyleBackColor = true;
            this.btn_ChangeVel.Click += new System.EventHandler(this.btn_ChangeVel_Click);
            // 
            // nud_NewVel
            // 
            this.nud_NewVel.DecimalPlaces = 3;
            this.nud_NewVel.Location = new System.Drawing.Point(79, 16);
            this.nud_NewVel.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_NewVel.Name = "nud_NewVel";
            this.nud_NewVel.Size = new System.Drawing.Size(86, 21);
            this.nud_NewVel.TabIndex = 1;
            this.nud_NewVel.Value = new decimal(new int[] {
            3500,
            0,
            0,
            0});
            // 
            // label25
            // 
            this.label25.AutoSize = true;
            this.label25.Location = new System.Drawing.Point(13, 20);
            this.label25.Name = "label25";
            this.label25.Size = new System.Drawing.Size(59, 12);
            this.label25.TabIndex = 0;
            this.label25.Text = "运行速度:";
            // 
            // nud_TaccDec
            // 
            this.nud_TaccDec.DecimalPlaces = 3;
            this.nud_TaccDec.Location = new System.Drawing.Point(79, 43);
            this.nud_TaccDec.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_TaccDec.Name = "nud_TaccDec";
            this.nud_TaccDec.Size = new System.Drawing.Size(86, 21);
            this.nud_TaccDec.TabIndex = 1;
            this.nud_TaccDec.Value = new decimal(new int[] {
            5,
            0,
            0,
            65536});
            // 
            // label28
            // 
            this.label28.AutoSize = true;
            this.label28.Location = new System.Drawing.Point(172, 47);
            this.label28.Name = "label28";
            this.label28.Size = new System.Drawing.Size(11, 12);
            this.label28.TabIndex = 0;
            this.label28.Text = "s";
            // 
            // label26
            // 
            this.label26.AutoSize = true;
            this.label26.Location = new System.Drawing.Point(171, 18);
            this.label26.Name = "label26";
            this.label26.Size = new System.Drawing.Size(41, 12);
            this.label26.TabIndex = 0;
            this.label26.Text = "unit/s";
            // 
            // label27
            // 
            this.label27.AutoSize = true;
            this.label27.Location = new System.Drawing.Point(13, 47);
            this.label27.Name = "label27";
            this.label27.Size = new System.Drawing.Size(59, 12);
            this.label27.TabIndex = 0;
            this.label27.Text = "变速时间:";
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.btn_ChangePos);
            this.groupBox5.Controls.Add(this.nud_NewPos);
            this.groupBox5.Controls.Add(this.label29);
            this.groupBox5.Controls.Add(this.label31);
            this.groupBox5.Location = new System.Drawing.Point(296, 123);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(220, 85);
            this.groupBox5.TabIndex = 14;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "在线变位";
            // 
            // btn_ChangePos
            // 
            this.btn_ChangePos.Location = new System.Drawing.Point(77, 42);
            this.btn_ChangePos.Name = "btn_ChangePos";
            this.btn_ChangePos.Size = new System.Drawing.Size(75, 30);
            this.btn_ChangePos.TabIndex = 1;
            this.btn_ChangePos.Text = "在线变位";
            this.btn_ChangePos.UseVisualStyleBackColor = true;
            this.btn_ChangePos.Click += new System.EventHandler(this.btn_ChangePos_Click);
            // 
            // nud_NewPos
            // 
            this.nud_NewPos.DecimalPlaces = 3;
            this.nud_NewPos.Location = new System.Drawing.Point(77, 15);
            this.nud_NewPos.Maximum = new decimal(new int[] {
            1215752192,
            23,
            0,
            0});
            this.nud_NewPos.Name = "nud_NewPos";
            this.nud_NewPos.Size = new System.Drawing.Size(86, 21);
            this.nud_NewPos.TabIndex = 1;
            this.nud_NewPos.Value = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            // 
            // label29
            // 
            this.label29.AutoSize = true;
            this.label29.Location = new System.Drawing.Point(11, 19);
            this.label29.Name = "label29";
            this.label29.Size = new System.Drawing.Size(59, 12);
            this.label29.TabIndex = 0;
            this.label29.Text = "目标位置:";
            // 
            // label31
            // 
            this.label31.AutoSize = true;
            this.label31.Location = new System.Drawing.Point(169, 17);
            this.label31.Name = "label31";
            this.label31.Size = new System.Drawing.Size(29, 12);
            this.label31.TabIndex = 0;
            this.label31.Text = "unit";
            // 
            // btn_Start
            // 
            this.btn_Start.Location = new System.Drawing.Point(32, 331);
            this.btn_Start.Name = "btn_Start";
            this.btn_Start.Size = new System.Drawing.Size(75, 37);
            this.btn_Start.TabIndex = 15;
            this.btn_Start.Text = "执行运动";
            this.btn_Start.UseVisualStyleBackColor = true;
            this.btn_Start.Click += new System.EventHandler(this.btn_Start_Click);
            // 
            // btn_ClearPos
            // 
            this.btn_ClearPos.Location = new System.Drawing.Point(303, 331);
            this.btn_ClearPos.Name = "btn_ClearPos";
            this.btn_ClearPos.Size = new System.Drawing.Size(93, 37);
            this.btn_ClearPos.TabIndex = 16;
            this.btn_ClearPos.Text = "指令清零";
            this.btn_ClearPos.UseVisualStyleBackColor = true;
            this.btn_ClearPos.Click += new System.EventHandler(this.btn_ClearPos_Click);
            // 
            // btn_Esc
            // 
            this.btn_Esc.Location = new System.Drawing.Point(592, 331);
            this.btn_Esc.Name = "btn_Esc";
            this.btn_Esc.Size = new System.Drawing.Size(93, 37);
            this.btn_Esc.TabIndex = 17;
            this.btn_Esc.Text = "退出程序";
            this.btn_Esc.UseVisualStyleBackColor = true;
            this.btn_Esc.Click += new System.EventHandler(this.btn_Esc_Click);
            // 
            // btn_SetPulseEquiv
            // 
            this.btn_SetPulseEquiv.Location = new System.Drawing.Point(448, 331);
            this.btn_SetPulseEquiv.Name = "btn_SetPulseEquiv";
            this.btn_SetPulseEquiv.Size = new System.Drawing.Size(92, 37);
            this.btn_SetPulseEquiv.TabIndex = 18;
            this.btn_SetPulseEquiv.Text = "设置脉冲当量";
            this.btn_SetPulseEquiv.UseVisualStyleBackColor = true;
            this.btn_SetPulseEquiv.Click += new System.EventHandler(this.btn_SetPulseEquiv_Click);
            // 
            // btn_Stop
            // 
            this.btn_Stop.Location = new System.Drawing.Point(159, 331);
            this.btn_Stop.Name = "btn_Stop";
            this.btn_Stop.Size = new System.Drawing.Size(92, 37);
            this.btn_Stop.TabIndex = 19;
            this.btn_Stop.Text = "停止运动";
            this.btn_Stop.UseVisualStyleBackColor = true;
            this.btn_Stop.Click += new System.EventHandler(this.btn_Stop_Click);
            // 
            // richTextBox_Message
            // 
            this.richTextBox_Message.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.richTextBox_Message.Location = new System.Drawing.Point(245, 20);
            this.richTextBox_Message.Name = "richTextBox_Message";
            this.richTextBox_Message.ReadOnly = true;
            this.richTextBox_Message.Size = new System.Drawing.Size(192, 53);
            this.richTextBox_Message.TabIndex = 24;
            this.richTextBox_Message.Text = "";
            // 
            // button_SoftwareReset
            // 
            this.button_SoftwareReset.Location = new System.Drawing.Point(135, 47);
            this.button_SoftwareReset.Name = "button_SoftwareReset";
            this.button_SoftwareReset.Size = new System.Drawing.Size(75, 30);
            this.button_SoftwareReset.TabIndex = 23;
            this.button_SoftwareReset.Text = "软件复位";
            this.button_SoftwareReset.UseVisualStyleBackColor = true;
            this.button_SoftwareReset.Click += new System.EventHandler(this.button_SoftwareReset_Click);
            // 
            // button_HardwareReset
            // 
            this.button_HardwareReset.Location = new System.Drawing.Point(40, 47);
            this.button_HardwareReset.Name = "button_HardwareReset";
            this.button_HardwareReset.Size = new System.Drawing.Size(75, 30);
            this.button_HardwareReset.TabIndex = 22;
            this.button_HardwareReset.Text = "硬件复位";
            this.button_HardwareReset.UseVisualStyleBackColor = true;
            this.button_HardwareReset.Click += new System.EventHandler(this.button_HardwareReset_Click);
            // 
            // textBox_EthercatState
            // 
            this.textBox_EthercatState.Location = new System.Drawing.Point(88, 20);
            this.textBox_EthercatState.Name = "textBox_EthercatState";
            this.textBox_EthercatState.ReadOnly = true;
            this.textBox_EthercatState.Size = new System.Drawing.Size(121, 21);
            this.textBox_EthercatState.TabIndex = 21;
            // 
            // label32
            // 
            this.label32.AutoSize = true;
            this.label32.Location = new System.Drawing.Point(23, 25);
            this.label32.Name = "label32";
            this.label32.Size = new System.Drawing.Size(59, 12);
            this.label32.TabIndex = 20;
            this.label32.Text = "总线状态:";
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.richTextBox_Message);
            this.groupBox3.Controls.Add(this.label32);
            this.groupBox3.Controls.Add(this.button_SoftwareReset);
            this.groupBox3.Controls.Add(this.textBox_EthercatState);
            this.groupBox3.Controls.Add(this.button_HardwareReset);
            this.groupBox3.Location = new System.Drawing.Point(299, 214);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(460, 87);
            this.groupBox3.TabIndex = 25;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "复位及总线操作";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(771, 398);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.btn_Start);
            this.Controls.Add(this.btn_ClearPos);
            this.Controls.Add(this.btn_Esc);
            this.Controls.Add(this.btn_SetPulseEquiv);
            this.Controls.Add(this.btn_Stop);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Margin = new System.Windows.Forms.Padding(2);
            this.Name = "Form1";
            this.Text = "定长运动";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.Form1_FormClosed);
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nud_Spara)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_Tdec)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_Tacc)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_Dist)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_StopVel)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_MaxVel)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_StartVel)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_PulseEquiv)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_AxisId)).EndInit();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nud_NewVel)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nud_TaccDec)).EndInit();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nud_NewPos)).EndInit();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.NumericUpDown nud_Spara;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.NumericUpDown nud_Tdec;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.NumericUpDown nud_Tacc;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.NumericUpDown nud_Dist;
        private System.Windows.Forms.NumericUpDown nud_StopVel;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.NumericUpDown nud_MaxVel;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.NumericUpDown nud_StartVel;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.NumericUpDown nud_PulseEquiv;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.NumericUpDown nud_AxisId;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.TextBox tb_RunState;
        private System.Windows.Forms.TextBox tb_Encoder;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.TextBox tb_CurrentPos;
        private System.Windows.Forms.Label label24;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.TextBox tb_CurrentVel;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Button btn_ChangeVel;
        private System.Windows.Forms.NumericUpDown nud_NewVel;
        private System.Windows.Forms.Label label25;
        private System.Windows.Forms.NumericUpDown nud_TaccDec;
        private System.Windows.Forms.Label label28;
        private System.Windows.Forms.Label label26;
        private System.Windows.Forms.Label label27;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.Button btn_ChangePos;
        private System.Windows.Forms.NumericUpDown nud_NewPos;
        private System.Windows.Forms.Label label29;
        private System.Windows.Forms.Label label31;
        private System.Windows.Forms.Button btn_Start;
        private System.Windows.Forms.Button btn_ClearPos;
        private System.Windows.Forms.Button btn_Esc;
        private System.Windows.Forms.Button btn_SetPulseEquiv;
        private System.Windows.Forms.Button btn_Stop;
        private System.Windows.Forms.Label label30;
        private System.Windows.Forms.TextBox textBox_StateMachine;
        private System.Windows.Forms.RichTextBox richTextBox_Message;
        private System.Windows.Forms.Button button_SoftwareReset;
        private System.Windows.Forms.Button button_HardwareReset;
        private System.Windows.Forms.TextBox textBox_EthercatState;
        private System.Windows.Forms.Label label32;
        private System.Windows.Forms.GroupBox groupBox3;
    }
}

