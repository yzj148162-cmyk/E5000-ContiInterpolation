namespace 回原点运动
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
            this.textBox_RunStatus = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.numericUpDown_AxisNum = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown_DecTime = new System.Windows.Forms.NumericUpDown();
            this.label7 = new System.Windows.Forms.Label();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.numericUpDown_HighSpeed = new System.Windows.Forms.NumericUpDown();
            this.label3 = new System.Windows.Forms.Label();
            this.numericUpDown_LowSpeed = new System.Windows.Forms.NumericUpDown();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.panel_LimtP = new System.Windows.Forms.Panel();
            this.panel_LimtN = new System.Windows.Forms.Panel();
            this.panel_Home = new System.Windows.Forms.Panel();
            this.label4 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label30 = new System.Windows.Forms.Label();
            this.textBox_StateMachine = new System.Windows.Forms.TextBox();
            this.texBox_CurrentPos = new System.Windows.Forms.TextBox();
            this.label21 = new System.Windows.Forms.Label();
            this.texBox_CurrentVel = new System.Windows.Forms.TextBox();
            this.label20 = new System.Windows.Forms.Label();
            this.label19 = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.textBox_HomeStatus = new System.Windows.Forms.TextBox();
            this.label12 = new System.Windows.Forms.Label();
            this.numericUpDown_AccTime = new System.Windows.Forms.NumericUpDown();
            this.label2 = new System.Windows.Forms.Label();
            this.button_EmgStop = new System.Windows.Forms.Button();
            this.button_DecStop = new System.Windows.Forms.Button();
            this.button_ClearPos = new System.Windows.Forms.Button();
            this.button_Start = new System.Windows.Forms.Button();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label14 = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.numericUpDown_HomeOffset = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown_HomeMode = new System.Windows.Forms.NumericUpDown();
            this.button_Exit = new System.Windows.Forms.Button();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.richTextBox_Message = new System.Windows.Forms.RichTextBox();
            this.label32 = new System.Windows.Forms.Label();
            this.button_SoftwareReset = new System.Windows.Forms.Button();
            this.textBox_EthercatState = new System.Windows.Forms.TextBox();
            this.button_HardwareReset = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.radioButton5 = new System.Windows.Forms.RadioButton();
            this.radioButton6 = new System.Windows.Forms.RadioButton();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.radioButton11 = new System.Windows.Forms.RadioButton();
            this.radioButton9 = new System.Windows.Forms.RadioButton();
            this.radioButton10 = new System.Windows.Forms.RadioButton();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.radioButton7 = new System.Windows.Forms.RadioButton();
            this.radioButton8 = new System.Windows.Forms.RadioButton();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_AxisNum)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_DecTime)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_HighSpeed)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_LowSpeed)).BeginInit();
            this.groupBox6.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_AccTime)).BeginInit();
            this.groupBox2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_HomeOffset)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_HomeMode)).BeginInit();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // textBox_RunStatus
            // 
            this.textBox_RunStatus.Location = new System.Drawing.Point(85, 79);
            this.textBox_RunStatus.Name = "textBox_RunStatus";
            this.textBox_RunStatus.ReadOnly = true;
            this.textBox_RunStatus.Size = new System.Drawing.Size(88, 21);
            this.textBox_RunStatus.TabIndex = 34;
            this.textBox_RunStatus.Text = "停止中";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(20, 83);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(59, 12);
            this.label1.TabIndex = 32;
            this.label1.Text = "运动状态:";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(166, 160);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(11, 12);
            this.label11.TabIndex = 133;
            this.label11.Text = "s";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(166, 131);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(11, 12);
            this.label10.TabIndex = 133;
            this.label10.Text = "s";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(166, 98);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(41, 12);
            this.label8.TabIndex = 132;
            this.label8.Text = "unit/s";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(166, 67);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(41, 12);
            this.label13.TabIndex = 132;
            this.label13.Text = "unit/s";
            // 
            // numericUpDown_AxisNum
            // 
            this.numericUpDown_AxisNum.Location = new System.Drawing.Point(79, 31);
            this.numericUpDown_AxisNum.Maximum = new decimal(new int[] {
            3,
            0,
            0,
            0});
            this.numericUpDown_AxisNum.Name = "numericUpDown_AxisNum";
            this.numericUpDown_AxisNum.Size = new System.Drawing.Size(81, 21);
            this.numericUpDown_AxisNum.TabIndex = 46;
            // 
            // numericUpDown_DecTime
            // 
            this.numericUpDown_DecTime.DecimalPlaces = 2;
            this.numericUpDown_DecTime.Location = new System.Drawing.Point(79, 156);
            this.numericUpDown_DecTime.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.numericUpDown_DecTime.Name = "numericUpDown_DecTime";
            this.numericUpDown_DecTime.Size = new System.Drawing.Size(81, 21);
            this.numericUpDown_DecTime.TabIndex = 1;
            this.numericUpDown_DecTime.Value = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(13, 35);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(59, 12);
            this.label7.TabIndex = 45;
            this.label7.Text = "电机轴号:";
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(13, 160);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(59, 12);
            this.label6.TabIndex = 0;
            this.label6.Text = "减速时间:";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(13, 126);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(59, 12);
            this.label5.TabIndex = 0;
            this.label5.Text = "加速时间:";
            // 
            // numericUpDown_HighSpeed
            // 
            this.numericUpDown_HighSpeed.Location = new System.Drawing.Point(79, 94);
            this.numericUpDown_HighSpeed.Maximum = new decimal(new int[] {
            -727379968,
            232,
            0,
            0});
            this.numericUpDown_HighSpeed.Name = "numericUpDown_HighSpeed";
            this.numericUpDown_HighSpeed.Size = new System.Drawing.Size(81, 21);
            this.numericUpDown_HighSpeed.TabIndex = 1;
            this.numericUpDown_HighSpeed.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(13, 98);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(59, 12);
            this.label3.TabIndex = 0;
            this.label3.Text = "回零高速:";
            // 
            // numericUpDown_LowSpeed
            // 
            this.numericUpDown_LowSpeed.Location = new System.Drawing.Point(79, 63);
            this.numericUpDown_LowSpeed.Maximum = new decimal(new int[] {
            -727379968,
            232,
            0,
            0});
            this.numericUpDown_LowSpeed.Name = "numericUpDown_LowSpeed";
            this.numericUpDown_LowSpeed.Size = new System.Drawing.Size(81, 21);
            this.numericUpDown_LowSpeed.TabIndex = 1;
            this.numericUpDown_LowSpeed.Value = new decimal(new int[] {
            500,
            0,
            0,
            0});
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.panel_LimtP);
            this.groupBox6.Controls.Add(this.panel_LimtN);
            this.groupBox6.Controls.Add(this.panel_Home);
            this.groupBox6.Controls.Add(this.label4);
            this.groupBox6.Controls.Add(this.label16);
            this.groupBox6.Controls.Add(this.label9);
            this.groupBox6.Controls.Add(this.label30);
            this.groupBox6.Controls.Add(this.textBox_StateMachine);
            this.groupBox6.Controls.Add(this.texBox_CurrentPos);
            this.groupBox6.Controls.Add(this.label21);
            this.groupBox6.Controls.Add(this.texBox_CurrentVel);
            this.groupBox6.Controls.Add(this.label20);
            this.groupBox6.Controls.Add(this.label19);
            this.groupBox6.Controls.Add(this.label18);
            this.groupBox6.Controls.Add(this.textBox_HomeStatus);
            this.groupBox6.Controls.Add(this.label12);
            this.groupBox6.Controls.Add(this.textBox_RunStatus);
            this.groupBox6.Controls.Add(this.label1);
            this.groupBox6.Location = new System.Drawing.Point(406, 12);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(460, 172);
            this.groupBox6.TabIndex = 55;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "信息显示";
            // 
            // panel_LimtP
            // 
            this.panel_LimtP.BackColor = System.Drawing.Color.Red;
            this.panel_LimtP.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.panel_LimtP.Location = new System.Drawing.Point(312, 73);
            this.panel_LimtP.Name = "panel_LimtP";
            this.panel_LimtP.Size = new System.Drawing.Size(45, 30);
            this.panel_LimtP.TabIndex = 39;
            // 
            // panel_LimtN
            // 
            this.panel_LimtN.BackColor = System.Drawing.Color.Red;
            this.panel_LimtN.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.panel_LimtN.Location = new System.Drawing.Point(384, 72);
            this.panel_LimtN.Name = "panel_LimtN";
            this.panel_LimtN.Size = new System.Drawing.Size(45, 30);
            this.panel_LimtN.TabIndex = 40;
            // 
            // panel_Home
            // 
            this.panel_Home.BackColor = System.Drawing.Color.Red;
            this.panel_Home.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.panel_Home.Location = new System.Drawing.Point(241, 71);
            this.panel_Home.Name = "panel_Home";
            this.panel_Home.Size = new System.Drawing.Size(45, 30);
            this.panel_Home.TabIndex = 40;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(310, 54);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(41, 12);
            this.label4.TabIndex = 37;
            this.label4.Text = "正限位";
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Location = new System.Drawing.Point(384, 53);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(41, 12);
            this.label16.TabIndex = 38;
            this.label16.Text = "负限位";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(244, 53);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(29, 12);
            this.label9.TabIndex = 38;
            this.label9.Text = "原点";
            // 
            // label30
            // 
            this.label30.AutoSize = true;
            this.label30.Location = new System.Drawing.Point(20, 136);
            this.label30.Name = "label30";
            this.label30.Size = new System.Drawing.Size(59, 12);
            this.label30.TabIndex = 36;
            this.label30.Text = "轴状态机:";
            // 
            // textBox_StateMachine
            // 
            this.textBox_StateMachine.Location = new System.Drawing.Point(85, 133);
            this.textBox_StateMachine.Name = "textBox_StateMachine";
            this.textBox_StateMachine.ReadOnly = true;
            this.textBox_StateMachine.Size = new System.Drawing.Size(121, 21);
            this.textBox_StateMachine.TabIndex = 35;
            // 
            // texBox_CurrentPos
            // 
            this.texBox_CurrentPos.Location = new System.Drawing.Point(85, 46);
            this.texBox_CurrentPos.Name = "texBox_CurrentPos";
            this.texBox_CurrentPos.ReadOnly = true;
            this.texBox_CurrentPos.Size = new System.Drawing.Size(88, 21);
            this.texBox_CurrentPos.TabIndex = 1;
            // 
            // label21
            // 
            this.label21.AutoSize = true;
            this.label21.Location = new System.Drawing.Point(179, 50);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(29, 12);
            this.label21.TabIndex = 0;
            this.label21.Text = "unit";
            // 
            // texBox_CurrentVel
            // 
            this.texBox_CurrentVel.Location = new System.Drawing.Point(85, 18);
            this.texBox_CurrentVel.Name = "texBox_CurrentVel";
            this.texBox_CurrentVel.ReadOnly = true;
            this.texBox_CurrentVel.Size = new System.Drawing.Size(88, 21);
            this.texBox_CurrentVel.TabIndex = 1;
            // 
            // label20
            // 
            this.label20.AutoSize = true;
            this.label20.Location = new System.Drawing.Point(20, 50);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(59, 12);
            this.label20.TabIndex = 0;
            this.label20.Text = "当前位置:";
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.Location = new System.Drawing.Point(179, 22);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(41, 12);
            this.label19.TabIndex = 0;
            this.label19.Text = "unit/s";
            // 
            // label18
            // 
            this.label18.AutoSize = true;
            this.label18.Location = new System.Drawing.Point(20, 22);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(59, 12);
            this.label18.TabIndex = 0;
            this.label18.Text = "当前速度:";
            // 
            // textBox_HomeStatus
            // 
            this.textBox_HomeStatus.Location = new System.Drawing.Point(85, 106);
            this.textBox_HomeStatus.Name = "textBox_HomeStatus";
            this.textBox_HomeStatus.ReadOnly = true;
            this.textBox_HomeStatus.Size = new System.Drawing.Size(88, 21);
            this.textBox_HomeStatus.TabIndex = 34;
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(20, 110);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(59, 12);
            this.label12.TabIndex = 32;
            this.label12.Text = "回零状态:";
            // 
            // numericUpDown_AccTime
            // 
            this.numericUpDown_AccTime.DecimalPlaces = 2;
            this.numericUpDown_AccTime.Location = new System.Drawing.Point(79, 122);
            this.numericUpDown_AccTime.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.numericUpDown_AccTime.Name = "numericUpDown_AccTime";
            this.numericUpDown_AccTime.Size = new System.Drawing.Size(81, 21);
            this.numericUpDown_AccTime.TabIndex = 1;
            this.numericUpDown_AccTime.Value = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(13, 67);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(59, 12);
            this.label2.TabIndex = 0;
            this.label2.Text = "回零低速:";
            // 
            // button_EmgStop
            // 
            this.button_EmgStop.Location = new System.Drawing.Point(417, 291);
            this.button_EmgStop.Name = "button_EmgStop";
            this.button_EmgStop.Size = new System.Drawing.Size(93, 37);
            this.button_EmgStop.TabIndex = 53;
            this.button_EmgStop.Text = "立即停止";
            this.button_EmgStop.UseVisualStyleBackColor = true;
            this.button_EmgStop.Click += new System.EventHandler(this.button_EmgStop_Click);
            // 
            // button_DecStop
            // 
            this.button_DecStop.Location = new System.Drawing.Point(299, 291);
            this.button_DecStop.Name = "button_DecStop";
            this.button_DecStop.Size = new System.Drawing.Size(93, 37);
            this.button_DecStop.TabIndex = 48;
            this.button_DecStop.Text = "减速停止";
            this.button_DecStop.UseVisualStyleBackColor = true;
            this.button_DecStop.Click += new System.EventHandler(this.button_DecStop_Click);
            // 
            // button_ClearPos
            // 
            this.button_ClearPos.Location = new System.Drawing.Point(535, 291);
            this.button_ClearPos.Name = "button_ClearPos";
            this.button_ClearPos.Size = new System.Drawing.Size(93, 37);
            this.button_ClearPos.TabIndex = 47;
            this.button_ClearPos.Text = "位置清零";
            this.button_ClearPos.UseVisualStyleBackColor = true;
            this.button_ClearPos.Click += new System.EventHandler(this.button_ClearPos_Click);
            // 
            // button_Start
            // 
            this.button_Start.Location = new System.Drawing.Point(181, 291);
            this.button_Start.Name = "button_Start";
            this.button_Start.Size = new System.Drawing.Size(93, 37);
            this.button_Start.TabIndex = 46;
            this.button_Start.Text = "总线轴回零";
            this.button_Start.UseVisualStyleBackColor = true;
            this.button_Start.Click += new System.EventHandler(this.button_Start_Click);
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.label14);
            this.groupBox2.Controls.Add(this.label15);
            this.groupBox2.Controls.Add(this.label11);
            this.groupBox2.Controls.Add(this.label10);
            this.groupBox2.Controls.Add(this.label8);
            this.groupBox2.Controls.Add(this.label13);
            this.groupBox2.Controls.Add(this.numericUpDown_AxisNum);
            this.groupBox2.Controls.Add(this.numericUpDown_HomeOffset);
            this.groupBox2.Controls.Add(this.numericUpDown_HomeMode);
            this.groupBox2.Controls.Add(this.numericUpDown_DecTime);
            this.groupBox2.Controls.Add(this.label7);
            this.groupBox2.Controls.Add(this.label6);
            this.groupBox2.Controls.Add(this.numericUpDown_AccTime);
            this.groupBox2.Controls.Add(this.label5);
            this.groupBox2.Controls.Add(this.numericUpDown_HighSpeed);
            this.groupBox2.Controls.Add(this.label3);
            this.groupBox2.Controls.Add(this.numericUpDown_LowSpeed);
            this.groupBox2.Controls.Add(this.label2);
            this.groupBox2.Location = new System.Drawing.Point(12, 12);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(219, 273);
            this.groupBox2.TabIndex = 52;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "运动参数";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(13, 230);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(59, 12);
            this.label14.TabIndex = 134;
            this.label14.Text = "回零偏移:";
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(13, 196);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(59, 12);
            this.label15.TabIndex = 135;
            this.label15.Text = "回零模式:";
            // 
            // numericUpDown_HomeOffset
            // 
            this.numericUpDown_HomeOffset.DecimalPlaces = 2;
            this.numericUpDown_HomeOffset.Location = new System.Drawing.Point(79, 228);
            this.numericUpDown_HomeOffset.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.numericUpDown_HomeOffset.Name = "numericUpDown_HomeOffset";
            this.numericUpDown_HomeOffset.Size = new System.Drawing.Size(81, 21);
            this.numericUpDown_HomeOffset.TabIndex = 1;
            // 
            // numericUpDown_HomeMode
            // 
            this.numericUpDown_HomeMode.Location = new System.Drawing.Point(79, 194);
            this.numericUpDown_HomeMode.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.numericUpDown_HomeMode.Name = "numericUpDown_HomeMode";
            this.numericUpDown_HomeMode.Size = new System.Drawing.Size(81, 21);
            this.numericUpDown_HomeMode.TabIndex = 1;
            this.numericUpDown_HomeMode.Value = new decimal(new int[] {
            28,
            0,
            0,
            0});
            // 
            // button_Exit
            // 
            this.button_Exit.Location = new System.Drawing.Point(653, 291);
            this.button_Exit.Name = "button_Exit";
            this.button_Exit.Size = new System.Drawing.Size(93, 37);
            this.button_Exit.TabIndex = 54;
            this.button_Exit.Text = "退出程序";
            this.button_Exit.UseVisualStyleBackColor = true;
            this.button_Exit.Click += new System.EventHandler(this.button_Exit_Click);
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.richTextBox_Message);
            this.groupBox3.Controls.Add(this.label32);
            this.groupBox3.Controls.Add(this.button_SoftwareReset);
            this.groupBox3.Controls.Add(this.textBox_EthercatState);
            this.groupBox3.Controls.Add(this.button_HardwareReset);
            this.groupBox3.Location = new System.Drawing.Point(406, 198);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(460, 87);
            this.groupBox3.TabIndex = 56;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "复位及总线操作";
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
            // label32
            // 
            this.label32.AutoSize = true;
            this.label32.Location = new System.Drawing.Point(23, 25);
            this.label32.Name = "label32";
            this.label32.Size = new System.Drawing.Size(59, 12);
            this.label32.TabIndex = 20;
            this.label32.Text = "总线状态:";
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
            // textBox_EthercatState
            // 
            this.textBox_EthercatState.Location = new System.Drawing.Point(88, 20);
            this.textBox_EthercatState.Name = "textBox_EthercatState";
            this.textBox_EthercatState.ReadOnly = true;
            this.textBox_EthercatState.Size = new System.Drawing.Size(121, 21);
            this.textBox_EthercatState.TabIndex = 21;
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
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(63, 291);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(93, 37);
            this.button1.TabIndex = 57;
            this.button1.Text = "脉冲轴回零";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.radioButton5);
            this.groupBox4.Controls.Add(this.radioButton6);
            this.groupBox4.Location = new System.Drawing.Point(234, 217);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(166, 66);
            this.groupBox4.TabIndex = 68;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "回原点速度";
            // 
            // radioButton5
            // 
            this.radioButton5.AutoSize = true;
            this.radioButton5.Location = new System.Drawing.Point(91, 30);
            this.radioButton5.Name = "radioButton5";
            this.radioButton5.Size = new System.Drawing.Size(47, 16);
            this.radioButton5.TabIndex = 0;
            this.radioButton5.Text = "高速";
            this.radioButton5.UseVisualStyleBackColor = true;
            // 
            // radioButton6
            // 
            this.radioButton6.AutoSize = true;
            this.radioButton6.Checked = true;
            this.radioButton6.Location = new System.Drawing.Point(26, 30);
            this.radioButton6.Name = "radioButton6";
            this.radioButton6.Size = new System.Drawing.Size(47, 16);
            this.radioButton6.TabIndex = 0;
            this.radioButton6.TabStop = true;
            this.radioButton6.Text = "低速";
            this.radioButton6.UseVisualStyleBackColor = true;
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.radioButton11);
            this.groupBox5.Controls.Add(this.radioButton9);
            this.groupBox5.Controls.Add(this.radioButton10);
            this.groupBox5.Location = new System.Drawing.Point(237, 12);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(163, 131);
            this.groupBox5.TabIndex = 69;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "回原点方式";
            // 
            // radioButton11
            // 
            this.radioButton11.AutoSize = true;
            this.radioButton11.Location = new System.Drawing.Point(26, 97);
            this.radioButton11.Name = "radioButton11";
            this.radioButton11.Size = new System.Drawing.Size(71, 16);
            this.radioButton11.TabIndex = 0;
            this.radioButton11.Text = "两次回零";
            this.radioButton11.UseVisualStyleBackColor = true;
            // 
            // radioButton9
            // 
            this.radioButton9.AutoSize = true;
            this.radioButton9.Location = new System.Drawing.Point(26, 63);
            this.radioButton9.Name = "radioButton9";
            this.radioButton9.Size = new System.Drawing.Size(101, 16);
            this.radioButton9.TabIndex = 0;
            this.radioButton9.Text = "一次回零+反找";
            this.radioButton9.UseVisualStyleBackColor = true;
            // 
            // radioButton10
            // 
            this.radioButton10.AutoSize = true;
            this.radioButton10.Checked = true;
            this.radioButton10.Location = new System.Drawing.Point(26, 30);
            this.radioButton10.Name = "radioButton10";
            this.radioButton10.Size = new System.Drawing.Size(71, 16);
            this.radioButton10.TabIndex = 0;
            this.radioButton10.TabStop = true;
            this.radioButton10.Text = "一次回零";
            this.radioButton10.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.radioButton7);
            this.groupBox1.Controls.Add(this.radioButton8);
            this.groupBox1.Location = new System.Drawing.Point(237, 153);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(163, 59);
            this.groupBox1.TabIndex = 70;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "回原点方向";
            // 
            // radioButton7
            // 
            this.radioButton7.AutoSize = true;
            this.radioButton7.Checked = true;
            this.radioButton7.Location = new System.Drawing.Point(26, 26);
            this.radioButton7.Name = "radioButton7";
            this.radioButton7.Size = new System.Drawing.Size(47, 16);
            this.radioButton7.TabIndex = 0;
            this.radioButton7.TabStop = true;
            this.radioButton7.Text = "正向";
            this.radioButton7.UseVisualStyleBackColor = true;
            // 
            // radioButton8
            // 
            this.radioButton8.AutoSize = true;
            this.radioButton8.Location = new System.Drawing.Point(88, 26);
            this.radioButton8.Name = "radioButton8";
            this.radioButton8.Size = new System.Drawing.Size(47, 16);
            this.radioButton8.TabIndex = 0;
            this.radioButton8.Text = "反向";
            this.radioButton8.UseVisualStyleBackColor = true;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(882, 341);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.button_EmgStop);
            this.Controls.Add(this.button_DecStop);
            this.Controls.Add(this.button_ClearPos);
            this.Controls.Add(this.button_Start);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.button_Exit);
            this.Name = "Form1";
            this.Text = "回原点运动";
            this.Load += new System.EventHandler(this.Form1_Load);
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_AxisNum)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_DecTime)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_HighSpeed)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_LowSpeed)).EndInit();
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_AccTime)).EndInit();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_HomeOffset)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_HomeMode)).EndInit();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TextBox textBox_RunStatus;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.NumericUpDown numericUpDown_AxisNum;
        private System.Windows.Forms.NumericUpDown numericUpDown_DecTime;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.NumericUpDown numericUpDown_HighSpeed;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.NumericUpDown numericUpDown_LowSpeed;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.TextBox texBox_CurrentPos;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.TextBox texBox_CurrentVel;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.NumericUpDown numericUpDown_AccTime;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button button_EmgStop;
        private System.Windows.Forms.Button button_DecStop;
        private System.Windows.Forms.Button button_ClearPos;
        private System.Windows.Forms.Button button_Start;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Button button_Exit;
        private System.Windows.Forms.TextBox textBox_HomeStatus;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Panel panel_LimtP;
        private System.Windows.Forms.Panel panel_LimtN;
        private System.Windows.Forms.Panel panel_Home;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label30;
        private System.Windows.Forms.TextBox textBox_StateMachine;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.NumericUpDown numericUpDown_HomeOffset;
        private System.Windows.Forms.NumericUpDown numericUpDown_HomeMode;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.RichTextBox richTextBox_Message;
        private System.Windows.Forms.Label label32;
        private System.Windows.Forms.Button button_SoftwareReset;
        private System.Windows.Forms.TextBox textBox_EthercatState;
        private System.Windows.Forms.Button button_HardwareReset;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.RadioButton radioButton5;
        private System.Windows.Forms.RadioButton radioButton6;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.RadioButton radioButton11;
        private System.Windows.Forms.RadioButton radioButton9;
        private System.Windows.Forms.RadioButton radioButton10;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.RadioButton radioButton7;
        private System.Windows.Forms.RadioButton radioButton8;

    }
}

