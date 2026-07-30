using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using csLTDMC;
using System.Threading;

namespace 下载EtherCAT配置文件
{
    public partial class Form1 : Form
    {
        private ushort _CardNo = 0;
        public Form1()
        {
            InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            OpenFileDialog f = new OpenFileDialog();
            f.Filter = "ini|*.ini";
            DialogResult res = f.ShowDialog();
            if (res == DialogResult.OK)
            {
                FileStream fs = File.Open(f.FileName, FileMode.Open, FileAccess.Read, FileShare.ReadWrite);
                StreamReader sr = new StreamReader(fs);
                string str = sr.ReadToEnd();
                sr.Close();
                fs.Close();
                byte[] buffer = Encoding.UTF8.GetBytes(str);
                byte[] fileincontrol = Encoding.UTF8.GetBytes("");
                ushort filetype = 201;
                short ret = -1;
                ret = LTDMC.nmc_set_cycletime(_CardNo, 2, 1000);      //设置总线周期
                ret = LTDMC.dmc_download_memfile(_CardNo, buffer, (uint)buffer.Length, fileincontrol, filetype);     //下载INI文件
                if (ret == 0)
                {
                    MessageBox.Show("下载成功！");
                }
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            ushort errcode = 0;
            LTDMC.nmc_get_errcode(_CardNo, 2, ref errcode);
            if (errcode == 0)
            {
                textBox5.Text = "总线正常";
                textBox5.BackColor = Color.Green;
            }
            else
            {
                textBox5.Text = "总线错误";
                textBox5.BackColor = Color.Red;
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            short num = -1;
            num = LTDMC.dmc_board_init();
            if (num <= 0 || num >= 8)
            {
                MessageBox.Show("初始化控制卡失败！", "出错");
            }
            ushort _num = 0;
            ushort[] cardids = new ushort[8];
            uint[] cardtypes = new uint[8];
            short res = LTDMC.dmc_get_CardInfList(ref _num, cardtypes, cardids);
            if (res != 0)
            {
                MessageBox.Show("获取控制卡信息失败！");
            }
            _CardNo = cardids[0];
            timer1.Start();
        }

        private void button3_Click(object sender, EventArgs e)
        {
            LTDMC.nmc_set_axis_enable(_CardNo, 255);
        }

        private void button4_Click(object sender, EventArgs e)
        {
            LTDMC.nmc_set_axis_disable(_CardNo, 255);
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            LTDMC.dmc_board_close();
        }

        private void button6_Click(object sender, EventArgs e)
        {
            OpenFileDialog f = new OpenFileDialog();
            f.Filter = "eni|*.eni";
            DialogResult res = f.ShowDialog();
            if (res == DialogResult.OK)
            {
                FileStream fs = File.Open(f.FileName, FileMode.Open, FileAccess.Read, FileShare.ReadWrite);
                StreamReader sr = new StreamReader(fs);
                string str = sr.ReadToEnd();
                sr.Close();
                fs.Close();
                byte[] buffer = Encoding.UTF8.GetBytes(str);
                byte[] fileincontrol = Encoding.UTF8.GetBytes("");
                ushort filetype = 200;
                short ret = -1;
                ret = LTDMC.dmc_download_memfile(_CardNo, buffer, (uint)buffer.Length, fileincontrol, filetype);     //下载ENI文件
                if (ret == 0)
                {
                    MessageBox.Show("下载成功！");
                }
            }
        }

        private void button7_Click(object sender, EventArgs e)
        {
            richTextBox1.Text = "";
            richTextBox1.AppendText("请勿操作，总线卡软件复位进行中……" + "\n");
            button1.Enabled = false;
            button2.Enabled = false;
            button3.Enabled = false;
            button4.Enabled = false;
            button5.Enabled = false;
            button6.Enabled = false;
            button7.Enabled = false;
            LTDMC.dmc_soft_reset(_CardNo);
            LTDMC.dmc_board_close();

            for (int i = 0; i < 15; i++)//总线卡软件复位耗时15s左右
            {
                Application.DoEvents();
                Thread.Sleep(1000);
            }

            LTDMC.dmc_board_init();
            richTextBox1.AppendText("总线卡软件复位完成,请确认总线状态");
            button1.Enabled = true;
            button2.Enabled = true;
            button3.Enabled = true;
            button4.Enabled = true;
            button5.Enabled = true;
            button6.Enabled = true;
            button7.Enabled = true;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            richTextBox1.Text = "";
            richTextBox1.AppendText("请勿操作，总线卡硬件复位进行中……" + "\n");
            button1.Enabled = false;
            button2.Enabled = false;
            button3.Enabled = false;
            button4.Enabled = false;
            button5.Enabled = false;
            button6.Enabled = false;
            button7.Enabled = false;
            LTDMC.dmc_board_reset();
            LTDMC.dmc_board_close();

            for (int i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
            {
                Application.DoEvents();
                Thread.Sleep(1000);
            }

            LTDMC.dmc_board_init();
            richTextBox1.AppendText("总线卡硬件复位完成,请确认总线状态");
            button1.Enabled = true;
            button2.Enabled = true;
            button3.Enabled = true;
            button4.Enabled = true;
            button5.Enabled = true;
            button6.Enabled = true;
            button7.Enabled = true;
        }

        private void button5_Click(object sender, EventArgs e)
        {
            richTextBox1.Text = "";
            richTextBox1.AppendText("请勿操作，总线卡初始值复位进行中……" + "\n");
            button1.Enabled = false;
            button2.Enabled = false;
            button3.Enabled = false;
            button4.Enabled = false;
            button5.Enabled = false;
            button6.Enabled = false;
            button7.Enabled = false;
            LTDMC.dmc_original_reset(_CardNo);
            LTDMC.dmc_board_close();

            for (int i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
            {
                Application.DoEvents();
                Thread.Sleep(1000);
            }

            LTDMC.dmc_board_init();
            richTextBox1.AppendText("总线卡初始值复位完成,请确认总线状态");
            button1.Enabled = true;
            button2.Enabled = true;
            button3.Enabled = true;
            button4.Enabled = true;
            button5.Enabled = true;
            button6.Enabled = true;
            button7.Enabled = true;
        }
    }
}
