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

namespace 总线操作
{
    public partial class Form1 : Form
    {
        private ushort _CardID = 0;
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            short num = LTDMC.dmc_board_init();//获取卡数量
            if (num <= 0 || num > 8)
            {
                MessageBox.Show("初始卡失败!", "出错");
            }
            ushort _num = 0;
            ushort[] cardids = new ushort[8];
            uint[] cardtypes = new uint[8];
            short res = LTDMC.dmc_get_CardInfList(ref _num, cardtypes, cardids);
            if (res != 0)
            {
                MessageBox.Show("获取卡信息失败!");
            }
            comboBox1.SelectedIndex = 0;
            comboBox2.SelectedIndex = 0;

            _CardID = cardids[0];
            //
            timer1.Start();
            uint TotalAxises = 0;
            LTDMC.nmc_get_total_axes(_CardID, ref TotalAxises);
            textBox12.Text = TotalAxises.ToString();
            for (uint item = 1; item < TotalAxises; item++)
                comboBox2.Items.Add(item);
        }

        private ushort GetAxis()
        {
            return Convert.ToUInt16(comboBox2.Text);
        }

        private void timer1_Tick(object sender, EventArgs e)
        {            
            ushort Axis_State_machine=0;   
            LTDMC.nmc_get_axis_state_machine(_CardID, GetAxis(), ref Axis_State_machine);
            switch (Axis_State_machine)// 读取指定轴状态机
            {
                case 0: textBox5.Text = GetAxis()+" 轴处于未启动状态";
                        textBox5.BackColor = Color.Red;
                        break;
                case 1:
                        textBox5.Text = GetAxis() + " 轴处于启动禁止状态";
                        textBox5.BackColor = Color.Red;
                    break;
                case 2:
                    textBox5.Text = GetAxis() + " 轴处于准备启动状态";
                        textBox5.BackColor = Color.Red;
                    break;
                case 3:
                    textBox5.Text = GetAxis() + " 轴处于启动状态";
                        textBox5.BackColor = Color.Red; 
                    break;
                case 4:
                    textBox5.Text = GetAxis() + " 轴处于操作使能状态";
                        textBox5.BackColor = Color.Green;
                    break;
                case 5:
                    textBox5.Text = GetAxis() + " 轴处于停止状态";
                        textBox5.BackColor = Color.Red;
                    break;
                case 6:
                    textBox5.Text = GetAxis() + " 轴处于错误触发状态";
                        textBox5.BackColor = Color.Red;
                    break;
                case 7:
                    textBox5.Text = GetAxis() + " 轴处于错误状态";
                        textBox5.BackColor = Color.Red;
                    break;
            };

            ushort errcode = 0;
            LTDMC.nmc_get_errcode(_CardID,2,ref errcode);
            if (errcode == 0)
            {
                textBox6.Text = "EtherCAT总线正常";
                textBox6.BackColor = Color.Green;
            }
            else
            {
                textBox6.Text = "EtherCAT总线出错";
                textBox6.BackColor = Color.Red;
            }
            ushort TotalSlaves=0;
            LTDMC.nmc_get_total_slaves(_CardID, 2, ref TotalSlaves);
            textBox10.Text = TotalSlaves.ToString();

            uint TotalAxises = 0;
            LTDMC.nmc_get_total_axes(_CardID,ref TotalAxises);
            textBox12.Text = TotalAxises.ToString();
            //for (uint item = 0; item < TotalAxises; item++)
            //    comboBox2.Items.Add(item);

            ushort TotalIn=0,TotalOut=0;
            LTDMC.nmc_get_total_ionum(_CardID, ref TotalIn, ref TotalOut);
            textBox13.Text = TotalIn.ToString();
            textBox14.Text = TotalOut.ToString();
            //获取总线AD/DA数
            ushort TotalADIn = 0, TotalDAOut = 0;
            LTDMC.nmc_get_total_adcnum(_CardID, ref TotalADIn, ref TotalDAOut);
            textBox15.Text = TotalADIn.ToString();
            textBox16.Text = TotalDAOut.ToString();

            textBox20.Text = 8.ToString();
            textBox21.Text = 8.ToString();
        }

        private void button8_Click(object sender, EventArgs e)
        {
            ushort axis = GetAxis(); //轴号
            LTDMC.nmc_set_axis_enable(_CardID, GetAxis());// 使能对应轴
        }

        private void button9_Click(object sender, EventArgs e)
        {
            ushort axis = GetAxis(); //轴号
            LTDMC.nmc_set_axis_disable(_CardID, GetAxis());// 失能对应轴
        }

        private void button10_Click(object sender, EventArgs e)
        {
            richTextBox1.Text = "";
            richTextBox1.AppendText("请勿操作，总线卡硬件复位进行中……" + "\n");
            button10.Enabled = false;
            button11.Enabled = false;
            //Application.DoEvents();
            LTDMC.dmc_board_reset();
            LTDMC.dmc_board_close();

            for (int i = 0; i < 15; i++)//总线卡硬件复位耗时15s左右
            {
                Application.DoEvents();
                Thread.Sleep(1000);
            }
           
            LTDMC.dmc_board_init();
            richTextBox1.AppendText("总线卡硬件复位完成,请确认总线状态");
            button10.Enabled = true;
            button11.Enabled = true;            
            button10.Focus();
        }

        private void button11_Click(object sender, EventArgs e)
        {
            richTextBox1.Text = "";
            richTextBox1.AppendText("请勿操作，总线卡软件复位进行中……"+"\n");
            button11.Enabled = false;
            button10.Enabled = false;
            //Application.DoEvents();
            LTDMC.dmc_soft_reset(_CardID);
            LTDMC.dmc_board_close();

            for (int i = 0; i < 15; i++)//总线卡软件复位耗时15s左右
            {
                Application.DoEvents();
                Thread.Sleep(1000);
            }
            LTDMC.dmc_board_init();
            richTextBox1.AppendText("总线卡软件复位完成,请确认总线状态");
            button10.Enabled = true;
            button11.Enabled = true;
            button11.Focus();
            
        }

        private void button5_Click_1(object sender, EventArgs e)
        {
            ushort SlaveAddress=0,Sub_SlaveAddress=0;
            LTDMC.nmc_get_axis_node_address(_CardID, GetAxis(), ref SlaveAddress, ref Sub_SlaveAddress);
            textBox17.Text = SlaveAddress.ToString();
            textBox18.Text = Sub_SlaveAddress.ToString();
        }

        private void button6_Click_1(object sender, EventArgs e)
        {
            ushort SlaveAddress = 0, Sub_SlaveAddress = 0;
            LTDMC.nmc_get_axis_node_address(_CardID, GetAxis(), ref SlaveAddress, ref Sub_SlaveAddress);
            textBox17.Text = SlaveAddress.ToString();
            textBox18.Text = Sub_SlaveAddress.ToString();
        }

        private void button14_Click(object sender, EventArgs e)
        {
            ushort Axis_Type = 0;
            LTDMC.nmc_get_axis_type(_CardID, GetAxis(), ref Axis_Type);
            switch(Axis_Type)
            {
                case 0: textBox19.Text = GetAxis()+ " 轴是虚拟轴";                        
                        break;
                case 1:
                        textBox19.Text = GetAxis() + " 轴是EtherCAT轴";                   
                    break;
                case 2:
                        textBox19.Text = GetAxis() + " 轴是CANopen轴";                 
                    break;
                case 3:
                        textBox19.Text = GetAxis() + " 轴是脉冲轴";                   
                    break;
                case 4:
                        textBox19.Text = GetAxis() + " 轴是未知类型轴";           
                    break;               
            }           
            
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            ushort ErrorCode=0;
            LTDMC.nmc_get_axis_errcode(_CardID, GetAxis(), ref ErrorCode);
            textBox2.Text = ErrorCode.ToString();    
        }

        private void button2_Click_1(object sender, EventArgs e)
        {
            int StatusWord = 0;
            LTDMC.nmc_get_axis_statusword(_CardID, GetAxis(), ref StatusWord);
            textBox3.Text = StatusWord.ToString();      
        }

        private void button3_Click_1(object sender, EventArgs e)
        {
            int ControlWord = 0;
            LTDMC.nmc_get_axis_contrlword(_CardID, GetAxis(), ref ControlWord);
            textBox4.Text = ControlWord.ToString(); 
        }

        private void button7_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void button12_Click(object sender, EventArgs e)
        {
            int Read_Value=0;            
            short ret1 = LTDMC.nmc_get_node_od(_CardID, 2, Convert.ToUInt16(textBox7.Text), UInt16.Parse(textBox8.Text,System.Globalization.NumberStyles.HexNumber),UInt16.Parse(textBox9.Text, System.Globalization.NumberStyles.HexNumber), Convert.ToUInt16(comboBox1.Text), ref Read_Value);
            if(ret1==0)
            {
                String strA = Read_Value.ToString("x8");//10进制转化为16进制
                //int b = Int32.Parse(textBox8.Text, System.Globalization.NumberStyles.HexNumber);//16进制转化为10进制
                //String strA = Read_Value.ToString("x8");//10进制转化为16进制
                textBox11.Text = "0x" + strA + "(" + Read_Value+")";
                MessageBox.Show("对象字典读取成功","提示",MessageBoxButtons.OK);
            }
            else
            {
                textBox11.Text = "0";
                MessageBox.Show("对象字典读取失败！！！错误码是 " + ret1, "提示", MessageBoxButtons.OK);
            }               
        }

        private void button13_Click(object sender, EventArgs e)
        {
            int Write_Value = Convert.ToUInt16(textBox11.Text);
            short ret1 = LTDMC.nmc_set_node_od(_CardID, 2, Convert.ToUInt16(textBox7.Text), UInt16.Parse(textBox8.Text, System.Globalization.NumberStyles.HexNumber), UInt16.Parse(textBox9.Text, System.Globalization.NumberStyles.HexNumber), Convert.ToUInt16(comboBox1.Text), Write_Value);
            if (ret1 == 0)
            {                
                //int b = Int32.Parse(textBox8.Text, System.Globalization.NumberStyles.HexNumber);//16进制转化为10进制
                //String strA = Read_Value.ToString("x8");//10进制转化为16进制                
                MessageBox.Show("对象字典设置成功", "提示", MessageBoxButtons.OK);
            }
            else
            {
                MessageBox.Show("对象字典设置失败！！！错误码是 " + ret1, "提示", MessageBoxButtons.OK);
            }
            //int b = Int32.Parse(textBox8.Text, System.Globalization.NumberStyles.HexNumber);//16进制转化为10进制
            //String strA = Read_Value.ToString("x8");//10进制转化为16进制            
        }
       
      }
}
