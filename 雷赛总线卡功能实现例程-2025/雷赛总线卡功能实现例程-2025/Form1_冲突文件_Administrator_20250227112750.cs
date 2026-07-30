using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using csLTDMC;

namespace 雷赛总线卡功能实现例程_2025
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private const ushort AxisNumMax = 256;  //控制卡的最大轴数量
        private const ushort AdNumMax = 256;    //控制卡的最大AD数量
        private const ushort DaNumMax = 256;    //控制卡的最大DA数量
        private bool ExcuteConnect = false;       //连接控制卡
        private bool ExcuteDisConnect = false;  //断开连接控制卡
        private bool ExcuteInit = false;        //初始化控制卡
        private bool ExcuteSoftReset = false;   //热复位控制卡
        private bool IsConnectDone = false;     //控制卡连接状态
        private bool IsInitDone = false;        //控制卡初始化状态
        private static List<string> ListInfo = new List<string>();      //日志信息的集合

        private void Form1_Load(object sender, EventArgs e)
        {
            Thread MainThread = new Thread(() =>
            {

            });
            MainThread.IsBackground = true;
            MainThread.Start();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            DialogResult res = MessageBox.Show("是否关闭软件", "询问", MessageBoxButtons.OKCancel, MessageBoxIcon.Question);
            if (res == DialogResult.Cancel)
            {
                e.Cancel = true;//不关闭窗体
            }
            else
            {
                btn_disConnect.PerformClick();
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {

        }

        private void btn_Connect_Click(object sender, EventArgs e)
        {

        }

        private void btn_DisConnect_Click(object sender, EventArgs e)
        {

        }

        private void btn_SoftReset_Click(object sender, EventArgs e)
        {

        }

        private void btn_EMG_Click(object sender, EventArgs e)
        {

        }
    }
}
