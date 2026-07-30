using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;
using csLTDMC;

namespace IO
{
    public partial class Form1 : Form
    {
        private ushort usCardNum = 1;
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
            usCardNum = arrusCardList[0];

            timer1.Start();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            timer1.Stop();
            LTDMC.dmc_board_close();
        }

        private Label GetINLabel(int nIndex)
        {
            string stStr = nIndex.ToString();
            foreach (Label _label in groupBox1.Controls)
            {
                if (_label != null && _label.Text == stStr)
                {
                    return _label;
                }
            }
            return null;
        }
        private Label GetOUTLabel(int nIndex)
        {
            string stStr = nIndex.ToString();
            foreach (Label _label in groupBox2.Controls)
            {
                if (_label != null && _label.Text == stStr)
                {
                    return _label;
                }
            }
            return null;
        }

        private Label GetEtherCATINLabel(int nIndex)
        {
            string stStr = nIndex.ToString();
            foreach (Label _label in groupBox5.Controls)
            {
                if (_label != null && _label.Text == stStr)
                {
                    return _label;
                }
            }
            return null;
        }

        private Label GetEtherCATOUTLabel(int nIndex)
        {
            string stStr = nIndex.ToString();
            foreach (Label _label in groupBox6.Controls)
            {
                if (_label != null && _label.Text == stStr)
                {
                    return _label;
                }
            }
            return null;
        }

        private void SetLabel(Label label, bool status)
        {
            if (label != null)
            {
                if (status)
                {
                    label.BackColor = Color.Red;
                }
                else
                {
                    label.BackColor = Color.Green;
                }
            }
        }

        private void label_Click(object sender, EventArgs e)
        {
            int nIndex = -1;
            foreach (Label _label in groupBox2.Controls)
            {
                if (_label == sender)
                {
                    nIndex = Convert.ToInt32(_label.Text);
                    break;
                }
            }
            if (nIndex > -1)
            {
                short s = LTDMC.dmc_read_outbit(usCardNum, (ushort)nIndex);
                if (s == 1)
                {
                    LTDMC.dmc_write_outbit(usCardNum, (ushort)nIndex, 0); // 设置 输出点
                }
                else
                {
                    LTDMC.dmc_write_outbit(usCardNum, (ushort)nIndex, 1);// 设置 输出点
                }
            }

            int nEtherCATIndex = -1;
            foreach (Label _label in groupBox6.Controls)
            {
                if (_label == sender)
                {
                    nEtherCATIndex = Convert.ToInt32(_label.Text) + 16;
                    break;
                }
            }

            if (nEtherCATIndex > -1)
            {
                short s = LTDMC.dmc_read_outbit(usCardNum, (ushort)nEtherCATIndex);
                if (s == 1)
                {
                    LTDMC.dmc_write_outbit(usCardNum, (ushort)nEtherCATIndex, 0); // 设置 输出点
                }
                else
                {
                    LTDMC.dmc_write_outbit(usCardNum, (ushort)nEtherCATIndex, 1);// 设置 输出点
                }
            }

        }

        private void button_Exit_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //输入口
            uint unN = LTDMC.dmc_read_inport(usCardNum, 0);//读本地输入口的值
            for (int i = 0; i < 16; i++)
            {
                Label label = GetINLabel(i);
                if (label != null)
                {
                    SetLabel(label, (unN & 1) == 1);
                }
                unN = unN >> 1;//unN变成unN向右移一位的那个数
            }
            //输出口
            unN = LTDMC.dmc_read_outport(usCardNum, 0);//读本地输出口的值
            for (int i = 0; i < 32; i++)
            {
                Label label = GetOUTLabel(i);
                if (label != null)
                {
                    SetLabel(label, (unN & 1) == 1);
                }

                unN = unN >> 1;
            }

            //输入口
            uint unNE = LTDMC.dmc_read_inport(usCardNum, 0);//读本地输入口的值
            unNE = unNE >> 16;
            for (int i = 0; i < 16; i++)
            {
                Label label = GetEtherCATINLabel(i);
                if (label != null)
                {
                    SetLabel(label, (unNE & 1) == 1);
                }
                unNE = unNE >> 1;//unN变成unN向右移一位的那个数
            }
            //输出口
            unNE = LTDMC.dmc_read_outport(usCardNum, 0);//读本地输出口的值
            unNE = unNE >> 16;
            for (int i = 0; i < 16; i++)
            {
                Label label = GetEtherCATOUTLabel(i);
                if (label != null)
                {
                    SetLabel(label, (unNE & 1) == 1);
                }
                unNE = unNE >> 1;
            }

            //IO数量
            ushort usTotalIn = 0;
            ushort usTotalOut = 0;
            short sRtn = LTDMC.nmc_get_total_ionum(usCardNum, ref usTotalIn, ref usTotalOut);
            textBox_InNum.Text = usTotalIn.ToString();
            textBox_OutNum.Text = usTotalOut.ToString();

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
    }
}
