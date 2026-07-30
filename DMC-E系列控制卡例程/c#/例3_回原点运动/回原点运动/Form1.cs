using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using csLTDMC;
using System.Threading;

namespace 回原点运动
{
    public partial class Form1 : Form
    {
        private ushort usCardNum = 0;
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            short sNum = LTDMC.dmc_board_init();//获取卡数量
            if (sNum <= 0 || sNum > 8)
            {
                MessageBox.Show("初始卡失败!", "出错");
            }
            ushort usNum = 0;
            ushort[] arrusCardList = new ushort[8];
            uint[] arrunCardTypes = new uint[8];
            short sRtn = LTDMC.dmc_get_CardInfList(ref usNum, arrunCardTypes, arrusCardList);
            if (sRtn != 0)
            {
                MessageBox.Show("获取卡信息失败!");
            }
            usCardNum = arrusCardList[1];
            timer1.Start();
            // 使能对应轴    
            sRtn = LTDMC.nmc_set_axis_enable(usCardNum, GetAxis());
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            // 失能对应轴
            short sRtn = 0;
            sRtn = LTDMC.nmc_set_axis_disable(usCardNum, GetAxis());
            timer1.Stop();
            LTDMC.dmc_board_close();
        }

        private ushort GetHomeMode()
        {
            ushort HomeMode = 0;

            if (radioButton10.Checked)
            {
                HomeMode = 0;
            }
            else if (radioButton9.Checked)
            {
                HomeMode = 1;
            }
            else if (radioButton11.Checked)
            {
                HomeMode = 2;
            }
            return HomeMode;
        }

        private ushort GetHomeDir()
        {
            ushort HomeDir = 0;

            if (radioButton7.Checked)
            {
                HomeDir = 1;
            }
            else if (radioButton8.Checked)
            {
                HomeDir = 0;
            }
            return HomeDir;
        }

        private ushort GetVelMode()
        {
            ushort VelMode = 0;

            if (radioButton6.Checked)
            {
                VelMode = 0;
            }
            else if (radioButton5.Checked)
            {
                VelMode = 1;
            }
            return VelMode;
        }

        private ushort GetAxis()
        {
            return decimal.ToUInt16(numericUpDown_AxisNum.Value);
        }
              
       
        private void button_Start_Click(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis();
            double dHomeLowSpeed = decimal.ToDouble(numericUpDown_LowSpeed.Value);
            double dHomeHighSpeed = decimal.ToDouble(numericUpDown_HighSpeed.Value);      
            double dAccTime = decimal.ToDouble(numericUpDown_AccTime.Value);
            double dDecTime = decimal.ToDouble(numericUpDown_DecTime.Value);
            ushort usHomeMode = decimal.ToUInt16(numericUpDown_HomeMode.Value);
            double dHomeOffset = decimal.ToDouble(numericUpDown_HomeOffset.Value);    
            
            short sRtn = 0;

            sRtn = LTDMC.nmc_set_home_profile(usCardNum, usAxis, usHomeMode, dHomeLowSpeed, dHomeHighSpeed, dAccTime, dDecTime, dHomeOffset);
            sRtn = LTDMC.nmc_home_move(usCardNum, usAxis);//启动回零   
        }

        private void button_DecStop_Click(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis();
            LTDMC.dmc_stop(usCardNum, usAxis, 0);
        }

        private void button_EmgStop_Click(object sender, EventArgs e)
        {
            LTDMC.dmc_emg_stop(usCardNum);
        }

        private void button_ClearPos_Click(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis();
            LTDMC.dmc_set_position_unit(usCardNum, usAxis, 0);//位置清零
        }

        private void button_Exit_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //获取当前位置
            double dPos = 0;
            LTDMC.dmc_get_position_unit(usCardNum, GetAxis(), ref dPos);
            texBox_CurrentPos.Text = dPos.ToString();
                   
            //获取当前速度
            double dSpeed = 0;
            LTDMC.dmc_read_current_speed_unit(usCardNum, GetAxis(), ref dSpeed);
            texBox_CurrentVel.Text = dSpeed.ToString();

            short sState = LTDMC.dmc_check_done(usCardNum, GetAxis());
            if (sState == 0)
                textBox_RunStatus.Text = "运动中";
            else
                textBox_RunStatus.Text = "停止";
            

            UInt16 result = 0;
            LTDMC.dmc_get_home_result(usCardNum, GetAxis(), ref result);
            if (result == 1)
            {
                textBox_HomeStatus.Text = "回零完成";
            }
            else
            {
                textBox_HomeStatus.Text = "回零未完成";
            }

            // 读取指定轴状态机
            ushort usAxisStateMachine = 0;
            LTDMC.nmc_get_axis_state_machine(usCardNum, GetAxis(), ref usAxisStateMachine);
            switch (usAxisStateMachine)
            {
                case 0: textBox_StateMachine.Text = "轴处于未启动状态";
                    textBox_StateMachine.BackColor = Color.Red;
                    break;
                case 1:
                    textBox_StateMachine.Text = "轴处于启动禁止状态";
                    textBox_StateMachine.BackColor = Color.Red;
                    break;
                case 2:
                    textBox_StateMachine.Text = "轴处于准备启动状态";
                    textBox_StateMachine.BackColor = Color.Red;
                    break;
                case 3:
                    textBox_StateMachine.Text = "轴处于启动状态";
                    textBox_StateMachine.BackColor = Color.Red;
                    break;
                case 4:
                    textBox_StateMachine.Text = "轴处于操作使能状态";
                    textBox_StateMachine.BackColor = Color.Green;
                    break;
                case 5:
                    textBox_StateMachine.Text = "轴处于停止状态";
                    textBox_StateMachine.BackColor = Color.Red;
                    break;
                case 6:
                    textBox_StateMachine.Text = "轴处于错误触发状态";
                    textBox_StateMachine.BackColor = Color.Red;
                    break;
                case 7:
                    textBox_StateMachine.Text = "轴处于错误状态";
                    textBox_StateMachine.BackColor = Color.Red;
                    break;
            };

            //读取总线状态
            ushort usErrorCode = 0;
            LTDMC.nmc_get_errcode(usCardNum, 2, ref usErrorCode);
            if (usErrorCode == 0)
            {
                textBox_EthercatState.Text = "EtherCAT总线正常";
                textBox_EthercatState.BackColor = Color.Green;
            }
            else
            {
                textBox_EthercatState.Text = "EtherCAT总线出错";
                textBox_EthercatState.BackColor = Color.Red;
            }


            //读取限位原点状态
            uint unIoState = LTDMC.dmc_axis_io_status(usCardNum, GetAxis());
            if ((unIoState & 2) == 2)//检测正限位信号
            {
                panel_LimtP.BackColor = Color.Green;
            }
            else
            {
                panel_LimtP.BackColor = Color.Red;
            }
            if ((unIoState & 4) == 4)//检测负限位信号
            {
                panel_LimtN.BackColor = Color.Green;
            }
            else
            {
                panel_LimtN.BackColor = Color.Red;
            }
            if ((unIoState & 16) == 16)//检测原点信号
            {
                panel_Home.BackColor = Color.Green;
            }
            else
            {
                panel_Home.BackColor = Color.Red;
            }
        }

        private void button_HardwareReset_Click(object sender, EventArgs e)
        {
            richTextBox_Message.Text = "";
            richTextBox_Message.AppendText("请勿操作，总线卡硬件复位进行中……" + "\n");
            button_HardwareReset.Enabled = false;
            button_SoftwareReset.Enabled = false;
            //Application.DoEvents();
            LTDMC.dmc_board_reset();
            LTDMC.dmc_board_close();
            for (int i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
            {
                Application.DoEvents();
                Thread.Sleep(1000);
            }
            LTDMC.dmc_board_init();
            richTextBox_Message.AppendText("总线卡硬件复位完成,请确认总线状态");
            button_HardwareReset.Enabled = true;
            button_SoftwareReset.Enabled = true;
            button_HardwareReset.Focus();
        }

        private void button_SoftwareReset_Click(object sender, EventArgs e)
        {
            richTextBox_Message.Text = "";
            richTextBox_Message.AppendText("请勿操作，总线卡软件复位进行中……" + "\n");
            button_HardwareReset.Enabled = false;
            button_SoftwareReset.Enabled = false;
            //Application.DoEvents();
            LTDMC.dmc_soft_reset(usCardNum);
            LTDMC.dmc_board_close();
            for (int i = 0; i < 15; i++)//总线卡软件复位耗时15s左右
            {
                Application.DoEvents();
                Thread.Sleep(1000);
            }
            LTDMC.dmc_board_init();
            richTextBox_Message.AppendText("总线卡软件复位完成,请确认总线状态");
            button_SoftwareReset.Enabled = true;
            button_HardwareReset.Enabled = true;
            button_HardwareReset.Focus();

        }

        private void button1_Click(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis();
            ushort HomeMode = GetHomeMode();
            ushort HomeDir = GetHomeDir();
            ushort vel_mode = GetVelMode();
            double dHomeLowSpeed = decimal.ToDouble(numericUpDown_LowSpeed.Value);
            double dHomeHighSpeed = decimal.ToDouble(numericUpDown_HighSpeed.Value);      
            double dAccTime = decimal.ToDouble(numericUpDown_AccTime.Value);
            double dDecTime = decimal.ToDouble(numericUpDown_DecTime.Value);
            ushort usHomeMode = decimal.ToUInt16(numericUpDown_HomeMode.Value);

            ushort usAxisIoInMsg_PEL = 0;   //正限位信号
            ushort usAxisIoInMsg_NEL = 1;   //负限位信号
            ushort usAxisIoInMsg_ORG = 2;   //原点信号

            //将正限位信号、负限位信号和原点信号分别映射到通用输入口0、1、2
            LTDMC.dmc_set_axis_io_map(usCardNum, usAxis, usAxisIoInMsg_PEL, 6, 0, 0);
            LTDMC.dmc_set_axis_io_map(usCardNum, usAxis, usAxisIoInMsg_NEL, 6, 1, 0);
            LTDMC.dmc_set_axis_io_map(usCardNum, usAxis, usAxisIoInMsg_ORG, 6, 2, 0);
            LTDMC.dmc_set_el_mode(usCardNum, usAxis, 1, 0, 0); //设置正、负限位信号低电平有效且遇限位立即停止
            LTDMC.dmc_set_home_pin_logic(usCardNum, usAxis, 0, 0);//设置原点低电平有效

            LTDMC.dmc_set_home_profile_unit(usCardNum, usAxis, dHomeLowSpeed, dHomeHighSpeed, dAccTime, dDecTime);
            LTDMC.dmc_set_homemode(usCardNum, usAxis, HomeDir, vel_mode, HomeMode, 0);
            LTDMC.dmc_home_move(usCardNum, usAxis);
        }
    }
}
