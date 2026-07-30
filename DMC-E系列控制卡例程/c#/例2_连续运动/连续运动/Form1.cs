using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using csLTDMC;
using System.Threading;

namespace 连续运动
{
    public partial class Form1 : Form
    {
        private ushort usCardId = 0;
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
            usCardId = arrusCardList[0];

            timer1.Start();
            // 使能对应轴    
            sRtn = LTDMC.nmc_set_axis_enable(usCardId, GetAxis());

        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            // 失能对应轴
            short sRtn = 0;
            sRtn = LTDMC.nmc_set_axis_disable(usCardId, GetAxis());
            LTDMC.dmc_board_close();
        }
        private ushort GetAxis()
        {
            return decimal.ToUInt16(nud_AxisId.Value);
        }
        private void btn_ChangeVel_Click(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis(); //轴号
            double dNewVel = decimal.ToDouble(nud_NewVel.Value);                // 新的运行速度
            double dTaccDec = decimal.ToDouble(nud_TaccDec.Value);              //变速时间
            LTDMC.dmc_change_speed_unit(usCardId, usAxis, dNewVel, dTaccDec);  //在线变速
        }

        private void btn_Start_Click(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis(); //轴号
            double dStartVel;           //起始速度
            double dMaxVel;             //运行速度
            double dTacc;               //加速时间
            double dTdec;               //减速时间
            double dStopVel;            //停止速度
            double dSpara;              //S段时间           
            ushort usDir;               //运动方向

            dStartVel = decimal.ToDouble(nud_StartVel.Value);
            dMaxVel = decimal.ToDouble(nud_MaxVel.Value);
            dTacc = decimal.ToDouble(nud_Tacc.Value);
            dTdec = decimal.ToDouble(nud_Tdec.Value);
            dStopVel = decimal.ToDouble(nud_StopVel.Value);
            dSpara = decimal.ToDouble(nud_Spara.Value);           
            if (radioButton_DirP.Checked)
            {
                usDir = 1;
            }
            else
            {
                usDir = 0;
            }
            LTDMC.dmc_set_profile_unit(usCardId, usAxis, dStartVel, dMaxVel, dTacc, dTdec, dStopVel);//设置速度参数
            LTDMC.dmc_set_s_profile(usCardId, usAxis, 0, dSpara);                                    //设置S段速度参数
            LTDMC.dmc_vmove(usCardId, usAxis, usDir);                                                 //定速运动
        }

        private void btn_Stop_Click(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis();                             //轴号
            ushort usStopMode = 0;                                 //制动方式，0：减速停止，1：紧急停止
            double dTdec = decimal.ToDouble(nud_Tdec.Value);       //减速时间         
            LTDMC.dmc_set_dec_stop_time(usCardId, usAxis, dTdec);  //设置减速停止时间
            LTDMC.dmc_stop(usCardId, usAxis, usStopMode);
        }

        private void btn_SetPulseEquiv_Click(object sender, EventArgs e)
        {            
            ushort usAxis = GetAxis();
            double dEquiv = decimal.ToDouble(nud_PulseEquiv.Value);
            LTDMC.dmc_set_equiv(usCardId, usAxis, dEquiv);      //设置脉冲当量
        }

        private void btn_ClearPos_Click(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis();                           //轴号
            double dPos = 0;                                     // 当前位置
            LTDMC.dmc_set_position_unit(usCardId, usAxis, dPos); //设置指定轴的当前指令位置值
        }

        private void btn_Esc_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            ushort usAxis = GetAxis();  //轴号
            double dCurrentVel = 0;     //速度
            double dCmdPos = 0;         //指令位置
            double dEnPos = 0;          //编码器反馈位置

            LTDMC.dmc_read_current_speed_unit(usCardId, usAxis, ref dCurrentVel); // 读取轴当前速度
            tb_CurrentVel.Text = dCurrentVel.ToString();
            LTDMC.dmc_get_position_unit(usCardId, usAxis, ref dCmdPos);           //读取指定轴指令位置值
            tb_CurrentPos.Text = dCmdPos.ToString();
            LTDMC.dmc_get_encoder_unit(usCardId, usAxis, ref dEnPos);             //读取指定轴的编码器反馈值
            tb_Encoder.Text = dEnPos.ToString();

            if (LTDMC.dmc_check_done(usCardId, usAxis) == 0)                      //读取指定轴运动状态
            {
                tb_RunState.Text = "运行中";
            }
            else
            {
                tb_RunState.Text = "停止中";
            }


            // 读取指定轴状态机
            ushort usAxisStateMachine = 0;
            LTDMC.nmc_get_axis_state_machine(usCardId, usAxis, ref usAxisStateMachine);
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
            LTDMC.nmc_get_errcode(usAxis, 2, ref usErrorCode);
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
            LTDMC.dmc_soft_reset(usCardId);
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


    }
}
