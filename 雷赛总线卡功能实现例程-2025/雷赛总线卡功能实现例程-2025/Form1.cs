using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using csLTDMC;
using Leadshine;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Button;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using System.Diagnostics;
using System.Reflection.Emit;
using System.Runtime.InteropServices;
using System.Dynamic;
using System.Security.Cryptography;

namespace 雷赛总线卡功能实现例程_2025
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        Form2 form2;
        MC_Leadshine mc_Leadshine = new MC_Leadshine();
        structCommands Commands = new structCommands();
        uint WindowsPage = 0;//UI操作界面ID号
        ushort usCurrentOutID = 0;//当前操作的输出口ID号
        long lMaxUsedTimes = 50000;//指令最大耗时，单位ms
        private static List<string> ListInfo = new List<string>();          //日志信息的集合
        private static List<string> TraceFileInfo = new List<string>();     //采样文件信息的集合
        private static double[,] arrayTargetPos = new double[5000, 256];    //插补运动中，轴的目标位置
        private static uint uiRowNum = 0;           //日志文件读出的：行数量
        private static uint uiColumnNum = 0;        //日志文件读出的：列数量
        Thread MainThread;
        Thread FileThread;
        enum enumMotionControllerType { DMC_E, EMC_E, BAC_E };//枚举的声明，控制器类型选择
        enumMotionControllerType enumControllerType;

        /// <summary>
        /// 打开软件窗体
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Load(object sender, EventArgs e)
        {
            #region 主界面UI的一些初始化设置
            foreach (Control item in groupBox9.Controls)
            {
                if (item is System.Windows.Forms.ComboBox comboBox)
                {
                    comboBox.SelectedIndex = 0;
                }
            }
            foreach (Control item in groupBox7.Controls)
            {
                if (item is System.Windows.Forms.ComboBox comboBox)
                {
                    comboBox.SelectedIndex = 0;
                }
            }
            comboBox1.SelectedIndex = 4;
            comboBox8.SelectedIndex = 2;
            radioButton7.Checked = true;
            #endregion

            MainThread = new Thread(() =>
            {
                Stopwatch stopWatch = Stopwatch.StartNew();
                while (true)
                {
                    stopWatch.Start();

                    #region 启动控制卡连接
                    if (Commands.ExcuteConnect)
                    {
                        while (!mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            if (radioButton1.Checked)
                            {
                                enumControllerType = enumMotionControllerType.DMC_E;
                            }
                            else if (radioButton2.Checked)
                            {
                                enumControllerType = enumMotionControllerType.EMC_E;
                                this.Invoke((MethodInvoker)delegate { numericUpDown10.Maximum = 32; });
                            }
                            else if (radioButton11.Checked)
                            {
                                enumControllerType = enumMotionControllerType.BAC_E;
                            }


                            if (enumControllerType == enumMotionControllerType.EMC_E)//连接EMC控制卡
                            {
                                mc_Leadshine.CardID = 8;
                                short res = LTDMC.dmc_board_init_eth(mc_Leadshine.CardID, textBox1.Text.Trim());
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：连接EMC控制器失败,dmc_board_init_eth({mc_Leadshine.CardID},{textBox1.Text.Trim()})={res}");
                                    break;
                                }
                                AddListInfo(ListInfo, $"连接EMC控制器成功,dmc_board_init_eth({mc_Leadshine.CardID},{textBox1.Text.Trim()})={res}");
                            }
                            else if (enumControllerType == enumMotionControllerType.DMC_E)//连接DMC控制卡
                            {
                                short res = LTDMC.dmc_board_init();
                                if (res <= 0 || res > 8)
                                {
                                    AddListInfo(ListInfo, $"报警：连接DMC控制卡失败，dmc_board_init()={res}");
                                    break;
                                }
                                AddListInfo(ListInfo, $"连接DMC控制卡成功，dmc_board_init()={res}");
                                ushort MyCardNum = 0;                                    //初始化成功的卡数量
                                uint[] MyCardTypeList = new uint[8];                     //控制卡固件类型数组
                                ushort[] MyCardIDList = new ushort[8];                   //控制卡硬件ID号数组，卡号按从小到大顺序排列
                                res = LTDMC.dmc_get_CardInfList(ref MyCardNum, MyCardTypeList, MyCardIDList);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：获取DMC卡信息失败，dmc_get_CardInfList() ={res}");
                                    break;
                                }
                                mc_Leadshine.CardID = MyCardIDList[0];
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                mc_Leadshine.CardID = 8;
                                short res = LTSMC.smc_board_init(mc_Leadshine.CardID, 2, textBox1.Text.Trim(), 115200);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：连接BAC控制器失败,smc_board_init({mc_Leadshine.CardID},2,{textBox1.Text.Trim()},115200)={res}");
                                    break;
                                }
                                AddListInfo(ListInfo, $"连接BAC控制器成功,smc_board_init({mc_Leadshine.CardID},2,{textBox1.Text.Trim()},115200)={res}");
                            }
                            mc_Leadshine.IsConnectDone = true;
                            WindowsPage = 0;

                            #region 下载总线配置文件(.eni+.ini)
                            //主要用于防止控制卡的总线配置被其他人更改，强烈建议使用。(注意文件信息需要与实际网络拓扑中的从站保持一致)
                            if (checkBox1.Checked == false)
                            {
                                Commands.ExcuteInit = true;//开启控制卡初始设置
                            }
                            else
                            {
                                string strFileName_eni = textBox9.Text.Trim();
                                string strFileName_ini = textBox10.Text.Trim();
                                if (!File.Exists(strFileName_ini) || !File.Exists(strFileName_eni))
                                {
                                    AddListInfo(ListInfo, $"报警：控制卡的EtherCAT总线配置文件不存在");
                                    break;
                                }
                                //设置总线周期，单位us (注意要保持和雷赛软件上配置总线时一致)，默认是1000us
                                short res = LTDMC.nmc_set_cycletime(mc_Leadshine.CardID, 2, 1000);      //设置总线周期，单位us
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_set_cycletime({mc_Leadshine.CardID},{2},{1000})={res}");
                                    break;
                                }
                                //下载总线配置文件.ini
                                FileStream fs = new FileStream(strFileName_ini, FileMode.Open, FileAccess.Read);
                                StreamReader sr = new StreamReader(fs);
                                string str = sr.ReadToEnd();
                                sr.Close();
                                fs.Close();
                                byte[] buffer = Encoding.UTF8.GetBytes(str);
                                byte[] fileincontrol = Encoding.UTF8.GetBytes("");
                                ushort filetype = 201;
                                res = LTDMC.dmc_download_memfile(mc_Leadshine.CardID, buffer, (uint)buffer.Length, fileincontrol, filetype); //下载ini文件
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_download_memfile({mc_Leadshine.CardID},{filetype})={res}");
                                    break;
                                }
                                //下载总线配置文件.eni
                                FileStream _fs = new FileStream(strFileName_eni, FileMode.Open, FileAccess.Read);
                                StreamReader _sr = new StreamReader(_fs);
                                string _str = _sr.ReadToEnd();
                                _sr.Close();
                                _fs.Close();
                                byte[] _buffer = Encoding.UTF8.GetBytes(_str);
                                byte[] _fileincontrol = Encoding.UTF8.GetBytes("");
                                ushort _filetype = 200;
                                res = LTDMC.dmc_download_memfile(mc_Leadshine.CardID, _buffer, (uint)_buffer.Length, _fileincontrol, _filetype); //下载ENI文件
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_download_memfile({mc_Leadshine.CardID},{_filetype})={res}");
                                    break;
                                }
                                Commands.ExcuteSoftReset = true;//开启控制卡热复位
                            }
                            #endregion
                            break;
                        }
                        Commands.ExcuteConnect = false;
                    }
                    #endregion

                    #region 启动控制卡关闭
                    if (Commands.ExcuteDisConnect)
                    {
                        //while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        while (!mc_Leadshine.IsSoftReset)
                        {
                            mc_Leadshine.IsConnectDone = false;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_trace_data_stop(mc_Leadshine.CardID);//停止采样
                                res = LTDMC.dmc_emg_stop(mc_Leadshine.CardID); //所有轴立即停止
                                res = LTDMC.dmc_board_close();
                                if (res == 0)
                                {
                                    AddListInfo(ListInfo, $"关闭控制卡成功,dmc_board_close()={res}");
                                }
                                else
                                {
                                    AddListInfo(ListInfo, $"报警：关闭控制卡失败,dmc_board_close()={res}");
                                }
                                //ushort usTotalIn_EtherCat = 0;
                                //ushort usTotalOut_EtherCat = 0;
                                //short res0 = LTDMC.nmc_get_total_ionum(mc_Leadshine.CardID, ref usTotalIn_EtherCat, ref usTotalOut_EtherCat);
                                //short res1 = LTDMC.nmc_read_inbit_extern(mc_Leadshine.CardID,2,1001,0,ref usTotalIn_EtherCat);
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                //short res = LTDMC.dmc_trace_data_stop(mc_Leadshine.CardID);//停止采样
                                short res = LTSMC.smc_emg_stop(mc_Leadshine.CardID); //所有轴立即停止
                                res = LTSMC.smc_board_close(mc_Leadshine.CardID);
                                if (res == 0)
                                {
                                    AddListInfo(ListInfo, $"关闭控制卡成功,smc_board_close(mc_Leadshine.CardID)={res}");
                                }
                                else
                                {
                                    AddListInfo(ListInfo, $"报警：关闭控制卡失败,smc_board_close(mc_Leadshine.CardID)={res}");
                                }
                            }
                            break;
                        }
                        Commands.ExcuteDisConnect = false;
                    }
                    #endregion

                    #region 启动控制卡初始化
                    if (Commands.ExcuteInit)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            short res = -1;
                            #region 获取：控制器发布版本号
                            byte[] byReleaseVersion = new byte[50];
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.dmc_get_release_version(mc_Leadshine.CardID, byReleaseVersion);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_release_version({mc_Leadshine.CardID})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.smc_get_release_version(mc_Leadshine.CardID, byReleaseVersion);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_release_version({mc_Leadshine.CardID})={res}");
                                    break;
                                }
                            }
                            mc_Leadshine.ReleaseVersion = Encoding.UTF8.GetString(byReleaseVersion).TrimEnd('\0');
                            #endregion

                            #region 获取：控制器硬件版本号
                            uint uiCardVersion = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.dmc_get_card_version(mc_Leadshine.CardID, ref uiCardVersion);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_card_version({mc_Leadshine.CardID},{uiCardVersion})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.smc_get_card_version(mc_Leadshine.CardID, ref uiCardVersion);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_card_version({mc_Leadshine.CardID},{uiCardVersion})={res}");
                                    break;
                                }
                            }
                            mc_Leadshine.CardVersion = uiCardVersion;
                            #endregion

                            #region 获取：控制器固件版本号
                            uint uiFirmID = 0;
                            uint uiSubFirmID = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.dmc_get_card_soft_version(mc_Leadshine.CardID, ref uiFirmID, ref uiSubFirmID);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_card_soft_version({mc_Leadshine.CardID},{uiFirmID},{uiSubFirmID})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.smc_get_card_soft_version(mc_Leadshine.CardID, ref uiFirmID, ref uiSubFirmID);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_card_soft_version({mc_Leadshine.CardID},{uiFirmID},{uiSubFirmID})={res}");
                                    break;
                                }
                            }
                            mc_Leadshine.FirmID = uiFirmID;
                            mc_Leadshine.SubFirmID = uiSubFirmID;
                            #endregion

                            #region 获取：当前控制卡本体的IO总数
                            ushort usTotalIn = 0;
                            ushort usTotalOut = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.dmc_get_total_ionum(mc_Leadshine.CardID, ref usTotalIn, ref usTotalOut);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_total_ionum({mc_Leadshine.CardID},{usTotalIn},{usTotalOut})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.smc_get_total_ionum(mc_Leadshine.CardID, ref usTotalIn, ref usTotalOut);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_total_ionum({mc_Leadshine.CardID},{usTotalIn},{usTotalOut})={res}");
                                    break;
                                }
                            }
                            #endregion

                            #region 获取：当前控制卡从站的总数
                            ushort usTotalSlaves = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.nmc_get_total_slaves(mc_Leadshine.CardID, 2, ref usTotalSlaves);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_get_total_slaves({mc_Leadshine.CardID},{2},{usTotalSlaves})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.nmcs_get_total_slaves(mc_Leadshine.CardID, 2, ref usTotalSlaves);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_get_total_slaves({mc_Leadshine.CardID},{2},{usTotalSlaves})={res}");
                                    break;
                                }
                            }
                            mc_Leadshine.SlaveNum = usTotalSlaves;
                            #endregion

                            #region 获取：EtherCAT总线扩展IO输入输出口总数
                            ushort usTotalIn_EtherCat = 0;
                            ushort usTotalOut_EtherCat = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.nmc_get_total_ionum(mc_Leadshine.CardID, ref usTotalIn_EtherCat, ref usTotalOut_EtherCat);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_get_total_ionum({mc_Leadshine.CardID},{usTotalIn_EtherCat},{usTotalOut_EtherCat})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.nmcs_get_total_ionum(mc_Leadshine.CardID, ref usTotalIn_EtherCat, ref usTotalOut_EtherCat);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_get_total_ionum({mc_Leadshine.CardID},{usTotalIn_EtherCat},{usTotalOut_EtherCat})={res}");
                                    break;
                                }
                            }
                            mc_Leadshine.InputNum = (uint)usTotalIn + usTotalIn_EtherCat;
                            mc_Leadshine.OutputNum = (uint)usTotalOut + usTotalOut_EtherCat;
                            #endregion

                            #region 获取：当前控制卡本体的轴总数
                            uint uTotalAxisNum = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.dmc_get_total_axes(mc_Leadshine.CardID, ref uTotalAxisNum);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_total_axes({mc_Leadshine.CardID},{uTotalAxisNum})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.smc_get_total_axes(mc_Leadshine.CardID, ref uTotalAxisNum);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_total_axes({mc_Leadshine.CardID},{uTotalAxisNum})={res}");
                                    break;
                                }
                            }
                            #endregion

                            #region 获取：当前控制卡总线的轴总数
                            uint uiTotalAxisNum_EtherCat = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.nmc_get_total_axes(mc_Leadshine.CardID, ref uiTotalAxisNum_EtherCat);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_get_total_axes({mc_Leadshine.CardID},{uiTotalAxisNum_EtherCat})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.nmcs_get_total_axes(mc_Leadshine.CardID, ref uiTotalAxisNum_EtherCat);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_get_total_axes({mc_Leadshine.CardID},{uiTotalAxisNum_EtherCat})={res}");
                                    break;
                                }
                            }
                            mc_Leadshine.AxisNum = uTotalAxisNum + uiTotalAxisNum_EtherCat;
                            #endregion

                            #region 获取：当前控制卡总线的模拟量通道数量
                            ushort usTotalAD_EtherCat = 0;
                            ushort usTotalDA_EtherCat = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.nmc_get_total_adcnum(mc_Leadshine.CardID, ref usTotalAD_EtherCat, ref usTotalDA_EtherCat);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_get_total_adcnum({mc_Leadshine.CardID},{usTotalAD_EtherCat},{usTotalDA_EtherCat})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.nmcs_get_total_adcnum(mc_Leadshine.CardID, ref usTotalAD_EtherCat, ref usTotalDA_EtherCat);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_get_total_adcnum({mc_Leadshine.CardID},{usTotalAD_EtherCat},{usTotalDA_EtherCat})={res}");
                                    break;
                                }
                            }
                            mc_Leadshine.AdNum = usTotalAD_EtherCat;
                            mc_Leadshine.DaNum = usTotalDA_EtherCat;
                            #endregion

                            #region 设置：轴的脉冲当量
                            //if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            //{
                            //    for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                            //    {
                            //        res = LTDMC.dmc_set_equiv(mc_Leadshine.CardID, i, 1000);
                            //        if (res != 0)
                            //        {
                            //            AddListInfo(ListInfo, $"报警：dmc_set_equiv({mc_Leadshine.CardID},{i},{1000})={res}");
                            //            break;
                            //        }
                            //    }
                            //}
                            //else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            //{
                            //    for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                            //    {
                            //        res = LTSMC.smc_set_equiv(mc_Leadshine.CardID, i, 1000);
                            //        if (res != 0)
                            //        {
                            //            AddListInfo(ListInfo, $"报警：smc_set_equiv({mc_Leadshine.CardID},{i},{1000})={res}");
                            //            break;
                            //        }
                            //    }
                            //}
                            //if (res != 0)
                            //{
                            //    break;
                            //}
                            #endregion

                            #region 设置：总线掉线后能继续操作从站
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                //res = LTDMC.nmc_set_fieldbus_error_switch(mc_Leadshine.CardID, 2, 0);//1:使能，0:不使能。控制卡默认是0不使能
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"报警：nmc_set_fieldbus_error_switch({mc_Leadshine.CardID},{2},{1})={res}");
                                //    break;
                                //}
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                //res = LTSMC.nmcs_set_fieldbus_error_switch(mc_Leadshine.CardID, 2, 0);//1:使能，0:不使能。控制卡默认是0不使能
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"报警：nmcs_set_fieldbus_error_switch({mc_Leadshine.CardID},{2},{1})={res}");
                                //    break;
                                //}
                            }
                            #endregion

                            #region 设置：总线复位时输出口状态保持（此功能需扩展IO模块支持才行）
                            //if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            //{
                            //    res = LTDMC.nmc_set_slave_output_retain(mc_Leadshine.CardID, 0);
                            //    if (res != 0)
                            //    {
                            //        AddListInfo(ListInfo, $"报警：nmc_set_slave_output_retain({mc_Leadshine.CardID},{0})={res}");
                            //        break;
                            //    }
                            //}
                            //else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            //{
                            //    res = LTSMC.nmcs_set_slave_output_retain(mc_Leadshine.CardID, 0);
                            //    if (res != 0)
                            //    {
                            //        AddListInfo(ListInfo, $"报警：nmcs_set_slave_output_retain({mc_Leadshine.CardID},{0})={res}");
                            //        break;
                            //    }
                            //}
                            #endregion

                            #region 设置：一些特殊设置（需和雷赛技术沟通后再使用）
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                //res = LTDMC.nmc_set_dc_mode(mc_Leadshine.CardID, 2, 1);//0:master shift，1:bus shift
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"报警：nmc_set_dc_mode({mc_Leadshine.CardID},{2},{1})={res}");
                                //    break;
                                //}
                                //res = LTDMC.nmc_set_data_offset_time(mc_Leadshine.CardID, 200);//设置控制卡的总线同步偏移，单位us
                                ////以上指令执行后，还需要热复位控制卡后才会生效
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                //res = LTSMC.nmcs_set_dc_mode(mc_Leadshine.CardID, 2, 1);//0:master shift，1:bus shift
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"报警：nmc_set_dc_mode({mc_Leadshine.CardID},{2},{1})={res}");
                                //    break;
                                //}

                                res = LTSMC.nmcs_set_data_offset_time(mc_Leadshine.CardID, 200);//设置控制卡的总线同步偏移，单位us
                                //以上指令执行后，还需要热复位控制卡后才会生效
                            }


                            #endregion

                            #region 获取：EtherCAT总线状态
                            ushort usEtherCATState = 1;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.nmc_get_errcode(mc_Leadshine.CardID, 2, ref usEtherCATState);
                                if (res != 0 || usEtherCATState != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：总线通讯异常nmc_get_errcode({mc_Leadshine.CardID},{2},{usEtherCATState})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                res = LTSMC.nmcs_get_errcode(mc_Leadshine.CardID, 2, ref usEtherCATState);
                                if (res != 0 || usEtherCATState != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：总线通讯异常nmcs_get_errcode({mc_Leadshine.CardID},{2},{usEtherCATState})={res}");
                                    break;
                                }
                            }
                            #endregion

                            #region 设置：下载轴参数文件,文件格式(.ini)。注意：需要在总线通讯状态正常时执行
                            if (checkBox2.Checked == true)
                            {
                                //string strFileName = "AxisParameters.ini";
                                string strFileName = textBox11.Text.Trim();
                                if (File.Exists(strFileName))
                                {
                                    if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                                    {
                                        res = LTDMC.dmc_download_configfile(mc_Leadshine.CardID, strFileName);//下载轴参数文件.ini
                                        if (res != 0)
                                        {
                                            AddListInfo(ListInfo, $"报警：nmc_set_offset_pos({mc_Leadshine.CardID},{strFileName})={res}");
                                            break;
                                        }
                                    }
                                    else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                                    {

                                        res = LTSMC.smc_download_parafile(mc_Leadshine.CardID, strFileName);//下载轴参数文件.cfg进控制器
                                        if (res != 0)
                                        {
                                            AddListInfo(ListInfo, $"报警：smc_download_parafile({mc_Leadshine.CardID},{strFileName})={res}");
                                            break;
                                        }
                                    }
                                }
                                else
                                {
                                    AddListInfo(ListInfo, $"报警：轴参数文件{strFileName}不存在");
                                    break;
                                }
                            }
                            #endregion

                            #region 设置：控制卡轴位置和驱动器位置0x6064同步，一般用于绝对值编码器。注意：需要在总线通讯状态正常时执行
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                //for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                                //{
                                //    res = LTDMC.nmc_set_offset_pos(mc_Leadshine.CardID, i, 0);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"报警：nmc_set_offset_pos({mc_Leadshine.CardID},{i},{0})={res}");
                                //        break;
                                //    }
                                //}
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                //for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                                //{
                                //    res = LTSMC.nmcs_set_offset_pos(mc_Leadshine.CardID, i, 0);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"报警：nmcs_set_offset_pos({mc_Leadshine.CardID},{i},{0})={res}");
                                //        break;
                                //    }
                                //}
                            }
                            if (res != 0)
                            {
                                break;
                            }
                            #endregion

                            #region 设置：总线轴遇到限位减速停止
                            /*
                            for (ushort i = 0; i < mc_Leadshine.AxisNum; i++)
                            {
                                res = LTDMC.dmc_set_el_mode(mc_Leadshine.CardID, i, 1, 1, 1);//开启轴遇限位：减速停止功能（冷复位控制卡才能关闭）
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_set_el_mode({mc_Leadshine.CardID}, {i},{1},{1},{1})={res}");
                                    break;
                                }
                                res = LTDMC.dmc_set_dec_stop_time(mc_Leadshine.CardID, i, 0.02);//设置轴遇限位时减速停止时间为0.02秒
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_set_dec_stop_time({mc_Leadshine.CardID}, {i},{0.02})={res}");
                                    break;
                                }
                                //res = LTDMC.nmc_set_etc_el_stop_mode(mc_Leadshine.CardID, i, 3, 0, 0);//设置轴遇到限位的停止模式3,即按照设置的减速停止时间来规划停止0x607A
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"nmc_set_etc_el_stop_mode({mc_Leadshine.CardID}, {i},{3},{0},{0})={res}");
                                //    break;
                                //}
                            }
                            if (res != 0)
                            {
                                break;
                            }
                            */
                            #endregion

                            #region 设置：轴速度前馈设置（需要专用版本才支持）
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                //for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                                //{
                                //    res = LTDMC.dmc_set_feedforward_profile(mc_Leadshine.CardID, i, 1, 1);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"报警：dmc_set_feedforward_profile({mc_Leadshine.CardID},{i},{1},{1})={res}");
                                //        break;
                                //    }
                                //}
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                            {
                                //for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                                //{
                                    //res = LTSMC.smc_set_feedforward_profile(mc_Leadshine.CardID, i, decimal.ToDouble(numericUpDown51.Value), 1);
                                    //if (res != 0)
                                    //{
                                    //    AddListInfo(ListInfo, $"报警：smc_set_feedforward_profile({mc_Leadshine.CardID},{i},{decimal.ToDouble(numericUpDown51.Value)},{1})={res}");
                                    //    break;
                                    //}

                                    //AddListInfo(ListInfo, $"提示：smc_set_feedforward_profile({mc_Leadshine.CardID},{i},{decimal.ToDouble(numericUpDown51.Value)},{1})={res}");

                                //}
                            }
                            if (res != 0)
                            {
                                break;
                            }
                            #endregion
                            break;
                        }
                        Commands.ExcuteInit = false;
                    }
                    #endregion

                    #region 启动控制卡热复位
                    if (Commands.ExcuteSoftReset)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            short res = 1;
                            ushort State = 1;
                            mc_Leadshine.IsSoftReset = true;
                            stopWatch.Restart();
                            AddListInfo(ListInfo, $"控制卡开始热复位......");
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.dmc_soft_reset(mc_Leadshine.CardID);//热复位控制卡
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：控制卡热复位失败，dmc_soft_reset({mc_Leadshine.CardID})={res}");
                                    break;
                                }
                                res = LTDMC.dmc_board_close();//断开控制卡的连接
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：控制卡热复位失败，dmc_board_close()={res}");
                                    break;
                                }
                                Thread.Sleep(5000);//延时5s
                                if (enumControllerType == enumMotionControllerType.EMC_E)
                                {
                                    res = LTDMC.dmc_board_init_eth(mc_Leadshine.CardID, textBox1.Text.Trim());//连接EMC控制卡
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：控制卡热复位失败，dmc_board_init_eth({mc_Leadshine.CardID}, {textBox6.Text.Trim()})={res}");
                                        break;
                                    }
                                }
                                else
                                {
                                    res = LTDMC.dmc_board_init();//连接DMC控制卡
                                    if (res <= 0 || res > 8)
                                    {
                                        AddListInfo(ListInfo, $"报警：控制卡热复位失败，dmc_board_init()={res}");
                                        break;
                                    }
                                }
                                while (true)
                                {
                                    res = LTDMC.nmc_get_errcode(mc_Leadshine.CardID, 2, ref State);//读取EtherCAT总线状态
                                    if ((res == 0 && State == 0) || res != 0 || stopWatch.ElapsedMilliseconds > 18000)//超时限制18秒
                                    {
                                        break;
                                    }
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                res = LTSMC.smc_soft_reset(mc_Leadshine.CardID);//热复位控制卡
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：控制卡热复位失败，smc_soft_reset({mc_Leadshine.CardID})={res}");
                                    break;
                                }
                                res = LTSMC.smc_board_close(mc_Leadshine.CardID);//断开控制卡的连接
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：控制卡热复位失败，smc_board_close({mc_Leadshine.CardID})={res}");
                                    break;
                                }
                                Thread.Sleep(5000);//延时5s
                                res = LTSMC.smc_board_init(mc_Leadshine.CardID, 2, textBox1.Text.Trim(), 115200);//连接BAC控制卡
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：控制卡热复位失败，smc_board_init({mc_Leadshine.CardID},2,{textBox1.Text.Trim()},115200)={res}");
                                    break;
                                }
                                while (true)
                                {
                                    res = LTSMC.nmcs_get_errcode(mc_Leadshine.CardID, 2, ref State);//读取EtherCAT总线状态
                                    if ((res == 0 && State == 0) || res != 0 || stopWatch.ElapsedMilliseconds > 18000)//超时限制18秒
                                    {
                                        break;
                                    }
                                }
                            }
                            if (res == 0 && State == 0)
                            {
                                AddListInfo(ListInfo, $"控制卡热复位成功，耗时{(float)stopWatch.ElapsedMilliseconds / 1000}s");
                                Commands.ExcuteInit = true;//控制卡初始化设置
                            }
                            else
                            {
                                AddListInfo(ListInfo, $"报警：控制卡热复位失败，nmc_get_errcode({mc_Leadshine.CardID}, {2}, {State})={res},耗时{(float)stopWatch.ElapsedMilliseconds / 1000}s");
                            }
                            break;
                        }
                        mc_Leadshine.IsSoftReset = false;
                        this.Invoke((MethodInvoker)delegate
                        {
                            form2.Close();
                        });
                        Commands.ExcuteSoftReset = false;
                    }
                    #endregion

                    #region 启动-控制卡急停
                    if (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.IsEMG)
                    {
                        if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                        {
                            short res = LTDMC.dmc_emg_stop(mc_Leadshine.CardID); //所有轴立即停止
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"dmc_emg_stop({mc_Leadshine.CardID})={res}");
                                break;
                            }
                        }
                        else if (enumControllerType == enumMotionControllerType.BAC_E)//连接BAC控制卡
                        {
                            short res = LTSMC.smc_emg_stop(mc_Leadshine.CardID); //所有轴立即停止
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"smc_emg_stop({mc_Leadshine.CardID})={res}");
                                break;
                            }
                        }

                    }
                    #endregion

                    #region 启动轴开关使能
                    if (Commands.ExcuteAxisPowerOn)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);
                            ushort temp = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.nmc_get_axis_state_machine(mc_Leadshine.CardID, usAxisID, ref temp);
                                if (temp != 4)
                                {
                                    res = LTDMC.nmc_set_axis_enable(mc_Leadshine.CardID, usAxisID);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：nmc_set_axis_enable({mc_Leadshine.CardID},{usAxisID})={res}");
                                        break;
                                    }
                                }
                                else
                                {
                                    res = LTDMC.nmc_set_axis_disable(mc_Leadshine.CardID, usAxisID);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：nmc_set_axis_disable({mc_Leadshine.CardID},{usAxisID})={res}");
                                        break;
                                    }
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.nmcs_get_axis_state_machine(mc_Leadshine.CardID, usAxisID, ref temp);
                                if (temp != 4)
                                {
                                    res = LTSMC.nmcs_set_axis_enable(mc_Leadshine.CardID, usAxisID);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：nmcs_set_axis_enable({mc_Leadshine.CardID},{usAxisID})={res}");
                                        break;
                                    }
                                }
                                else
                                {
                                    res = LTSMC.nmcs_set_axis_disable(mc_Leadshine.CardID, usAxisID);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：nmcs_set_axis_disable({mc_Leadshine.CardID},{usAxisID})={res}");
                                        break;
                                    }
                                }
                            }

                            break;
                        }
                        Commands.ExcuteAxisPowerOn = false;
                    }
                    #endregion

                    #region 启动-清零轴位置
                    if (Commands.ExcuteClearAxisPos)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                for (ushort i = 0; i < mc_Leadshine.AxisNum; i++)
                                {
                                    short res = LTDMC.dmc_set_position_unit(mc_Leadshine.CardID, i, 0); //强制设置轴指令位置
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"dmc_set_position_unit({mc_Leadshine.CardID},{i}, {0})={res}");
                                        break;
                                    }
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                for (ushort i = 0; i < mc_Leadshine.AxisNum; i++)
                                {
                                    short res = LTSMC.smc_set_position_unit(mc_Leadshine.CardID, i, 0); //强制设置轴指令位置
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"dmc_set_position_unit({mc_Leadshine.CardID},{i}, {0})={res}");
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                        Commands.ExcuteClearAxisPos = false;
                    }
                    #endregion

                    #region 启动单轴点位运动
                    if (Commands.ExcuteMovePos)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            short res = 0;
                            ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);
                            mc_Leadshine.AxisStartVel[usAxisID] = 0;
                            mc_Leadshine.AxisMaxVel[usAxisID] = decimal.ToDouble(numericUpDown16.Value);
                            mc_Leadshine.AxisTimeAcc[usAxisID] = mc_Leadshine.AxisMaxVel[usAxisID] / decimal.ToDouble(numericUpDown15.Value);
                            mc_Leadshine.AxisTimeDec[usAxisID] = mc_Leadshine.AxisMaxVel[usAxisID] / decimal.ToDouble(numericUpDown15.Value);
                            mc_Leadshine.AxisStopVel[usAxisID] = 0;
                            mc_Leadshine.AxisDistancePos[usAxisID] = decimal.ToDouble(numericUpDown14.Value);
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                if (!checkBox3.Checked)
                                {
                                    res = LTDMC.nmc_set_axis_run_mode(mc_Leadshine.CardID, usAxisID, (ushort)(radioButton12.Checked ? 1 : 0));//设置单轴的点位运动模式，模式：0 CSP模式，1 PP模式
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：nmc_set_axis_run_mode({mc_Leadshine.CardID},{usAxisID}, {(radioButton12.Checked ? 1 : 0)})={res}");
                                        break;
                                    }
                                    res = LTDMC.dmc_set_profile_unit(mc_Leadshine.CardID, usAxisID, mc_Leadshine.AxisStartVel[usAxisID], mc_Leadshine.AxisMaxVel[usAxisID], mc_Leadshine.AxisTimeAcc[usAxisID], mc_Leadshine.AxisTimeDec[usAxisID], mc_Leadshine.AxisStopVel[usAxisID]);
                                    //res = LTDMC.dmc_set_plan_mode(mc_Leadshine.CardID, usAxisID, 2);//速度模式0指梯形，2指S-plus
                                    //res = LTDMC.dmc_set_profile_extern(mc_Leadshine.CardID, usAxisID, mc_Leadshine.AxisStartVel[usAxisID], mc_Leadshine.AxisMaxVel[usAxisID], decimal.ToDouble(numericUpDown15.Value), decimal.ToDouble(numericUpDown15.Value), decimal.ToDouble(numericUpDown15.Value) * 20, decimal.ToDouble(numericUpDown15.Value) * 20, mc_Leadshine.AxisStopVel[usAxisID]);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_set_profile_unit({mc_Leadshine.CardID},{usAxisID}, {mc_Leadshine.AxisStartVel[usAxisID]}, {mc_Leadshine.AxisMaxVel[usAxisID]}, {mc_Leadshine.AxisTimeAcc[usAxisID]}, {mc_Leadshine.AxisTimeDec[usAxisID]}, {mc_Leadshine.AxisStopVel[usAxisID]})={res}");
                                        break;
                                    }
                                }
                                else
                                {
                                    //速度规划模式，0：T型，1：保留，2：Splus 型，3：Spro型，4：Time规划(3阶)，5-9保留，10：4阶S型，11：4阶正余弦型，12：Time规划（4阶），13：加加速限制的4阶S型，14-19保留：
                                    #region 模式11
                                    //ushort usPlanMode = 11;
                                    //structAdvancPlanParam_t advancPlanParam = new structAdvancPlanParam_t();//速度参数，以结构体的方式描述
                                    //ushort usAccMode = 1;//加速参数描述方式，0：加减速时间，1：加减速度
                                    //advancPlanParam.fs = 0;//轴起始速度，单位：unit/s
                                    //advancPlanParam.f = decimal.ToDouble(numericUpDown16.Value);//轴运行速度，单位：unit/s
                                    //advancPlanParam.fe = 0;//轴停止速度，单位：unit/s
                                    //advancPlanParam.avg_acc = decimal.ToDouble(numericUpDown15.Value); //最大加速度，单位：unit/s^2
                                    //advancPlanParam.avg_dec = advancPlanParam.avg_acc;//最大减速度，单位：unit/s^2
                                    //advancPlanParam.JaRatio = 1;//加加速度比率
                                    //advancPlanParam.JdRatio = 1;//加减速度比率
                                    //IntPtr Pointer = StructToIntPtr(advancPlanParam);
                                    #endregion

                                    #region 模式13
                                    ushort usPlanMode = 2;
                                    structSplusePlanParam_t sPlusePlanParam = new structSplusePlanParam_t();//速度参数，以结构体的方式描述
                                    ushort usAccMode = 1;//加速参数描述方式，0：加减速时间，1：加减速度
                                    sPlusePlanParam.fs = 0;//轴起始速度，单位：unit/s
                                    sPlusePlanParam.f = decimal.ToDouble(numericUpDown16.Value);//轴运行速度，单位：unit/s
                                    sPlusePlanParam.fe = 0;//轴停止速度，单位：unit/s
                                    sPlusePlanParam.avg_acc = decimal.ToDouble(numericUpDown15.Value); //最大加速度，单位：unit/s^2
                                    sPlusePlanParam.avg_dec = sPlusePlanParam.avg_acc;//最大减速度，单位：unit/s^2
                                    sPlusePlanParam.Ja = sPlusePlanParam.avg_acc * 20;//最大加加速度，单位：unit / s ^ 3
                                    sPlusePlanParam.Jd = sPlusePlanParam.avg_acc * 20;//最大减减速度，单位：unit / s ^ 3
                                    IntPtr Pointer = StructToIntPtr(sPlusePlanParam);
                                    #endregion

                                    res = LTDMC.dmc_set_ultra_profile_unit(mc_Leadshine.CardID, usAxisID, usPlanMode, Pointer, usAccMode);//高阶S速度曲线设置
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_set_ultra_profile_unit({mc_Leadshine.CardID}, {usAxisID},{usPlanMode}, {Pointer}, {usAccMode})={res}");
                                        break;
                                    }
                                }
                                res = LTDMC.dmc_clear_stop_reason(mc_Leadshine.CardID, usAxisID);//清除轴停止原因
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_clear_stop_reason({mc_Leadshine.CardID},{usAxisID})={res}");
                                    break;
                                }
                                res = LTDMC.dmc_pmove_unit(mc_Leadshine.CardID, usAxisID, decimal.ToDouble(numericUpDown14.Value), (ushort)(radioButton5.Checked ? 0 : 1));//单轴点位运动
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_pmove_unit({mc_Leadshine.CardID}, {usAxisID}, {decimal.ToDouble(numericUpDown14.Value)},{(ushort)(radioButton5.Checked ? 0 : 1)})={res}");
                                    break;
                                }
                                #region 软着陆功能
                                //res = LTDMC.dmc_set_t_pmove_extern_dectime(mc_Leadshine.CardID, usAxisID, 50);//设置软着陆低速段减速时间，单位 ms，范围：0~100000
                                //res = LTDMC.dmc_t_pmove_extern_unit(mc_Leadshine.CardID, usAxisID, -125500, -135500, 0 * 1000, 355000, 355000 * 0.2, 0.1, 0.1, 1);
                                #endregion
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                //res = LTSMC.nmcs_set_axis_run_mode(mc_Leadshine.CardID, usAxisID, (ushort)(radioButton12.Checked ? 1 : 0));//设置单轴的点位运动模式，模式：0 CSP模式，1 PP模式
                                res = LTSMC.smc_set_profile_unit(mc_Leadshine.CardID, usAxisID, mc_Leadshine.AxisStartVel[usAxisID], mc_Leadshine.AxisMaxVel[usAxisID], mc_Leadshine.AxisTimeAcc[usAxisID], mc_Leadshine.AxisTimeDec[usAxisID], mc_Leadshine.AxisStopVel[usAxisID]);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_profile_unit({mc_Leadshine.CardID},{usAxisID}, {mc_Leadshine.AxisStartVel[usAxisID]}, {mc_Leadshine.AxisMaxVel[usAxisID]}, {mc_Leadshine.AxisTimeAcc[usAxisID]}, {mc_Leadshine.AxisTimeDec[usAxisID]}, {mc_Leadshine.AxisStopVel[usAxisID]})={res}");
                                    break;
                                }
                                res = LTSMC.smc_clear_stop_reason(mc_Leadshine.CardID, usAxisID);//清除轴停止原因
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_clear_stop_reason({mc_Leadshine.CardID}, {usAxisID})={res}");
                                    break;
                                }
                                res = LTSMC.smc_pmove_unit(mc_Leadshine.CardID, usAxisID, decimal.ToDouble(numericUpDown14.Value), (ushort)(radioButton5.Checked ? 0 : 1));//单轴点位运动
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_pmove_unit({mc_Leadshine.CardID}, {usAxisID}, {decimal.ToDouble(numericUpDown14.Value)},{(ushort)(radioButton5.Checked ? 0 : 1)})={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteMovePos = false;
                    }
                    #endregion

                    #region 启动单轴CST转矩运动
                    if (Commands.ExcuteMoveTorque)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState == 0 && mc_Leadshine.AxisNum >= 1)
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                ushort usAxisState = 0;//轴状态
                                ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);//轴号
                                short res = LTDMC.dmc_check_done_ex(mc_Leadshine.CardID, usAxisID, ref usAxisState);//判断轴状态是否停止
                                if (res != 0 || usAxisState != 1)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_check_done_ex({mc_Leadshine.CardID}, {usAxisID}, {usAxisState})={res}");
                                    break;
                                }
                                ushort usSlaveAddr = 0;//驱动器轴对应的的ID号，如1001,1002
                                ushort usSubSlaveAddr = 0;
                                res = LTDMC.nmc_get_axis_node_address(mc_Leadshine.CardID, usAxisID, ref usSlaveAddr, ref usSubSlaveAddr);//获取轴号对应的从站ID
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_get_axis_node_address({mc_Leadshine.CardID}, {usSlaveAddr}, {usSubSlaveAddr})={res}");
                                    break;
                                }
                                //设置转矩运动时的最大速度限制，实际是把值写入驱动器地址0x6080（有些厂商的驱动器可能是另外的地址0x607F，建议咨询驱动器厂商）
                                //一定记得不用转矩运动时，要把最大速度限制恢复默认，不然会影响点位运动CSP，PP和回零运动HM的最大运行速度。
                                int uiValue = decimal.ToInt32(numericUpDown50.Value);
                                res = LTDMC.nmc_set_node_od(mc_Leadshine.CardID, 2, usSlaveAddr, 0x6080, 0, 32, uiValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_set_node_od({mc_Leadshine.CardID},{2}, {usSlaveAddr}, {0x6080},{0},{32},{uiValue})={res}");
                                    break;
                                }
                                //启动转矩运动并设置目标转矩,实际是把值写入驱动器地址0x6071,配置总线时一定要把0x6071添加到RxPDO里面
                                int iTargetTorque = decimal.ToInt32(numericUpDown51.Value);
                                res = LTDMC.nmc_torque_move(mc_Leadshine.CardID, usAxisID, iTargetTorque, 0, 0x80, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_torque_move({mc_Leadshine.CardID},{usAxisID}, {iTargetTorque}, {0},{0x80},{0})={res}");
                                    break;
                                }
                                //其他相关指令：
                                //在线调整当前轴的转矩值
                                //LTDMC.nmc_change_torque(mc_Leadshine.CardID, usAxisID, iTargetTorque);
                                //获取当前轴的转矩值,实际是读取驱动器地址0x6077的值,配置总线时一定要把0x6077添加到TxPDO里面
                                //LTDMC.nmc_get_torque(mc_Leadshine.CardID, usAxisID, iTargetTorque);
                                //停止转矩CST运动时可以调用dmc_stop指令，注意停止后轴会自动切换为点位CSP模式。
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                ushort usAxisState = 0;//轴状态
                                ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);//轴号
                                short res = LTSMC.smc_check_done_ex(mc_Leadshine.CardID, usAxisID, ref usAxisState);//判断轴状态是否停止
                                if (res != 0 || usAxisState != 1)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_check_done_ex({mc_Leadshine.CardID}, {usAxisID}, {usAxisState})={res}");
                                    break;
                                }
                                ushort usSlaveAddr = 0;//驱动器轴对应的的ID号，如1001,1002
                                ushort usSubSlaveAddr = 0;
                                res = LTSMC.nmcs_get_axis_node_address(mc_Leadshine.CardID, usAxisID, ref usSlaveAddr, ref usSubSlaveAddr);//获取轴号对应的从站ID
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_get_axis_node_address({mc_Leadshine.CardID}, {usSlaveAddr}, {usSubSlaveAddr})={res}");
                                    break;
                                }
                                //设置转矩运动时的最大速度限制，实际是把值写入驱动器地址0x6080（有些厂商的驱动器可能是另外的地址0x607F，建议咨询驱动器厂商）
                                //一定记得不用转矩运动时，要把最大速度限制恢复默认，不然会影响点位运动CSP，PP和回零运动HM的最大运行速度。
                                uint uiValue = decimal.ToUInt32(numericUpDown50.Value);
                                res = LTSMC.nmcs_set_node_od(mc_Leadshine.CardID, 2, usSlaveAddr, 0x6080, 0, 32, uiValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_set_node_od({mc_Leadshine.CardID},{2}, {usSlaveAddr}, {0x6080},{0},{32},{uiValue})={res}");
                                    break;
                                }
                                //启动转矩运动并设置目标转矩,实际是把值写入驱动器地址0x6071,配置总线时一定要把0x6071添加到RxPDO里面
                                int iTargetTorque = decimal.ToInt32(numericUpDown51.Value);
                                res = LTSMC.nmcs_torque_move(mc_Leadshine.CardID, usAxisID, iTargetTorque, 0, 0x80, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_torque_move({mc_Leadshine.CardID},{usAxisID}, {iTargetTorque}, 0,0x80,0)={res}");
                                    break;
                                }
                                //其他相关指令：
                                //在线调整当前轴的转矩值
                                //LTSMC.nmcs_change_torque(mc_Leadshine.CardID, usAxisID, iTargetTorque);
                                //获取当前轴的转矩值,实际是读取驱动器地址0x6077的值,配置总线时一定要把0x6077添加到TxPDO里面
                                //LTSMC.nmcs_get_torque(mc_Leadshine.CardID, usAxisID, iTargetTorque);
                                //停止转矩CST运动时可以调用smc_stop指令，注意停止后轴会自动切换为点位CSP模式。
                            }
                            break;
                        }
                        Commands.ExcuteMoveTorque = false;
                    }
                    #endregion

                    #region 启动单轴正余弦振荡运动
                    if (Commands.ExcuteMoveOscillate)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState == 0 && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usAxisState = 0;//轴状态
                            ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);//轴号
                            double fAmplitude = decimal.ToDouble(numericUpDown53.Value);//正余弦曲线振幅 
                            double fFrequency = decimal.ToDouble(numericUpDown54.Value);//正余弦曲线频率
                            uint uiCycleNum = decimal.ToUInt32(numericUpDown55.Value);//振荡次数

                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_check_done_ex(mc_Leadshine.CardID, usAxisID, ref usAxisState);//判断轴状态是否停止
                                if (res != 0 || usAxisState != 1)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_check_done_ex({mc_Leadshine.CardID}, {usAxisID}, {usAxisState})={res}");
                                    break;
                                }
                                //正弦振荡模式设置：0-位置时间为正弦曲线，1-位置时间为带偏移的余弦曲线。
                                //建议使用余弦运动，因为正弦运动启动时会有个快速加速到终点的动作
                                res = LTDMC.dmc_sine_oscillate_set_mode(mc_Leadshine.CardID, usAxisID, 1);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_sine_oscillate_set_mode({mc_Leadshine.CardID}, {usAxisID}, {1})={res}");
                                    break;
                                }
                                if (enumControllerType == enumMotionControllerType.DMC_E)
                                {
                                    res = LTDMC.dmc_sine_oscillate_set_cycle_num(mc_Leadshine.CardID, usAxisID, uiCycleNum);//振荡次数设置
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_sine_oscillate_set_cycle_num({mc_Leadshine.CardID},{usAxisID}, {uiCycleNum})={res}");
                                        break;
                                    }
                                }
                                res = LTDMC.dmc_sine_oscillate_unit(mc_Leadshine.CardID, usAxisID, fAmplitude, fFrequency);//启动振荡运动
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_sine_oscillate_unit({mc_Leadshine.CardID},{usAxisID}, {uiCycleNum})={res}");
                                    break;
                                }
                                //其他相关指令：LTDMC.dmc_sine_oscillate_stop(usConnectNo, usAxisID);停止正余弦振荡运动（运动完当前周期才停止）
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_check_done_ex(mc_Leadshine.CardID, usAxisID, ref usAxisState);//判断轴状态是否停止
                                if (res != 0 || usAxisState != 1)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_check_done_ex({mc_Leadshine.CardID}, {usAxisID}, {usAxisState})={res}");
                                    break;
                                }
                                //正弦振荡模式设置：0-位置时间为正弦曲线，1-位置时间为带偏移的余弦曲线。
                                //建议使用余弦运动，因为正弦运动启动时会有个快速加速到终点的动作
                                res = LTSMC.smc_sine_oscillate_set_mode(mc_Leadshine.CardID, usAxisID, 1);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_sine_oscillate_set_mode({mc_Leadshine.CardID}, {usAxisID}, {1})={res}");
                                    break;
                                }
                                res = LTSMC.smc_sine_oscillate_set_cycle_num(mc_Leadshine.CardID, usAxisID, uiCycleNum);//振荡次数设置
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_sine_oscillate_set_cycle_num({mc_Leadshine.CardID},{usAxisID}, {uiCycleNum})={res}");
                                    break;
                                }
                                res = LTSMC.smc_sine_oscillate_unit(mc_Leadshine.CardID, usAxisID, fAmplitude, fFrequency);//启动振荡运动
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_sine_oscillate_unit({mc_Leadshine.CardID},{usAxisID}, {uiCycleNum})={res}");
                                    break;
                                }
                                //其他相关指令：LTSMC.smc_sine_oscillate_stop(usConnectNo, usAxisID);停止正余弦振荡运动（运动完当前周期才停止）
                            }
                            break;
                        }
                        Commands.ExcuteMoveOscillate = false;
                    }
                    #endregion

                    #region 启动-正常停止单轴正余弦运动
                    if (Commands.ExcuteStopOscillate)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_sine_oscillate_stop(mc_Leadshine.CardID, usAxisID);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_sine_oscillate_stop({mc_Leadshine.CardID}, {usAxisID})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_sine_oscillate_stop(mc_Leadshine.CardID, usAxisID);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_sine_oscillate_stop({mc_Leadshine.CardID}, {usAxisID})={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteStopOscillate = false;
                    }
                    #endregion

                    #region 启动单轴减速停止
                    if (Commands.ExcuteStopMove)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);
                            mc_Leadshine.AxisTimeDec[usAxisID] = 0.1;// decimal.ToDouble(numericUpDown15.Value);
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_set_dec_stop_time(mc_Leadshine.CardID, usAxisID, mc_Leadshine.AxisTimeDec[usAxisID]);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_dec_stop_time({mc_Leadshine.CardID},{usAxisID}, {mc_Leadshine.AxisTimeAcc[usAxisID]})={res}");
                                    break;
                                }
                                res = LTDMC.dmc_stop(mc_Leadshine.CardID, usAxisID, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_stop({mc_Leadshine.CardID}, {usAxisID}, 0)={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_set_dec_stop_time(mc_Leadshine.CardID, usAxisID, mc_Leadshine.AxisTimeDec[usAxisID]);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_dec_stop_time({mc_Leadshine.CardID},{usAxisID}, {mc_Leadshine.AxisTimeAcc[usAxisID]})={res}");
                                    break;
                                }
                                res = LTSMC.smc_stop(mc_Leadshine.CardID, usAxisID, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_stop({mc_Leadshine.CardID}, {usAxisID}, 0)={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteStopMove = false;
                    }
                    #endregion

                    #region 启动插补运动
                    if (Commands.ExcuteVectorMove)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usCrd = decimal.ToUInt16(numericUpDown18.Value);//声明一个插补系号变量
                            ushort[] arrayAxisList = new ushort[5] { 0, 1, 2, 3, 4 };   //参与运动的轴号
                            double[] fTargetPos = new double[arrayAxisList.Length];     //轴的目标位置
                            double fSpeed = 375;                                       //插补速度,单位unit/s
                            double AccRatio = 10;                                       //加速度与 插补速度 的倍率
                            double JerkRatio = 20;                                      //加加速度与 加速度 的倍率
                            ushort usCameraOutNo = 2;                                   //相机输出口ID号

                            short res = LTDMC.dmc_conti_get_run_state(mc_Leadshine.CardID, usCrd);//读取指定坐标系的插补运动状态运动状态，0：运动中，1：暂停中，2：正常停止，3：未启动，4：空闲
                            if (res != 4)
                            {
                                AddListInfo(ListInfo, $"报警：插补系状态忙dmc_conti_get_run_state({mc_Leadshine.CardID},{usCrd})={res}");
                                break;
                            }
                            res = LTDMC.dmc_hcmp_clear_points(mc_Leadshine.CardID, usCameraOutNo);
                            res = LTDMC.dmc_hcmp_set_mode(mc_Leadshine.CardID, usCameraOutNo, 0);
                            if (checkBox4.Checked)
                            {
                                this.Invoke((MethodInvoker)delegate
                                {
                                    ushort usPin = Convert.ToUInt16(comboBox6.SelectedItem); //高速输出口号
                                    ushort usLogic = Convert.ToUInt16(comboBox3.SelectedItem);//输出电平：0 低电平，1 高电平；
                                    ushort usMode = Convert.ToUInt16(comboBox7.SelectedItem); ;//0:分段等间距;1:全局等间距，2:轨迹段单独设置距离模式，3:轨迹段单独设置点数模式(头输出尾不输出)，4:轨
                                    ushort usVirtualAxis = Convert.ToUInt16(comboBox4.SelectedItem); ;//辅助轴号0-1
                                    ushort usSource = Convert.ToUInt16(comboBox5.SelectedItem); ;//比较源，底层默认为指令位置（保留参数）
                                    uint uiRevTime = decimal.ToUInt32(numericUpDown22.Value);//输出持续时间，单位：us
                                    double[] fGap = new double[] { decimal.ToDouble(numericUpDown21.Value), 30, 5 };//比较点间距数据，3个元素依次为直线路径间距、圆弧路径间距、样条路径间距,
                                    res = LTDMC.dmc_set_vector_profile_unit(mc_Leadshine.CardID, 0, 0, 10000, 0.1, 0.1, 0);
                                    res = LTDMC.dmc_conti_open_list(mc_Leadshine.CardID, 0, 2, new ushort[] { 0, 1 });//打开连续插补缓存区
                                    res = LTDMC.dmc_set_gap_cmp_param(mc_Leadshine.CardID, 0, usPin, usLogic, usMode, usVirtualAxis, usSource, uiRevTime, fGap); //设置 PSO比较参数
                                    res = LTDMC.dmc_set_gap_cmp_enable(mc_Leadshine.CardID, 0, (ushort)(checkBox4.Checked ? 1 : 0));
                                    res = LTDMC.dmc_axis_follow_line_enable(mc_Leadshine.CardID, 0, 0);//0：最多只有3个轴参与直线插补，其它轴跟随运动,1：坐标系所有轴都参与直线插
                                    res = LTDMC.dmc_conti_line_unit(mc_Leadshine.CardID, 0, 2, new ushort[] { 0, 1 }, new double[] { 10000, 10000 }, 1, 0);
                                    res = LTDMC.dmc_conti_start_list(mc_Leadshine.CardID, 0); //开始连续插补运动                                                                
                                    res = LTDMC.dmc_conti_close_list(mc_Leadshine.CardID, 0); //关闭连续插补缓冲
                                });
                            }
                            else
                            {
                                for (int i = 0; i < arrayAxisList.Length; i++)
                                {
                                    res = LTDMC.dmc_compare_clear_points(mc_Leadshine.CardID, arrayAxisList[i]);//清除已添加的所有一维位置比较点
                                }
                                res = LTDMC.dmc_set_profile_limit_unit(mc_Leadshine.CardID, arrayAxisList[0], fSpeed, fSpeed, 0);//X单轴限速，设置每个轴允许的最大运行速度和加速度
                                res = LTDMC.dmc_set_profile_limit_unit(mc_Leadshine.CardID, arrayAxisList[1], fSpeed, fSpeed, 0);//Y单轴限速，设置每个轴允许的最大运行速度和加速度
                                res = LTDMC.dmc_set_profile_limit_unit(mc_Leadshine.CardID, arrayAxisList[2], fSpeed, fSpeed, 0);//Z单轴限速，设置每个轴允许的最大运行速度和加速度
                                res = LTDMC.dmc_set_profile_limit_unit(mc_Leadshine.CardID, arrayAxisList[3], fSpeed * 20, fSpeed * 20 * AccRatio, 0);//R1单轴限速，设置每个轴允许的最大运行速度和加速度(可以尽量设置大一些)
                                res = LTDMC.dmc_set_profile_limit_unit(mc_Leadshine.CardID, arrayAxisList[4], fSpeed * 20, fSpeed * 20 * AccRatio, 0);//R2单轴限速，设置每个轴允许的最大运行速度和加速度(可以尽量设置大一些)
                                res = LTDMC.dmc_set_vector_profile_limit_by_axis(mc_Leadshine.CardID, usCrd, 2);//启用限速功能
                                res = LTDMC.dmc_set_traj_min_travel_time(mc_Leadshine.CardID, usCrd, 0);        //设置连续插补单个轨迹段最小运行时间限制，单位：秒，设置成0时表示不启用
                                res = LTDMC.dmc_set_vector_plan_mode(mc_Leadshine.CardID, usCrd, 2);            //速度曲线规划:0-T 形,1-T+s平滑,2-标准S
                                res = LTDMC.dmc_set_vector_s_profile(mc_Leadshine.CardID, usCrd, 0, 0);         //插补运动速度曲线的平滑时间，单位秒
                                res = LTDMC.dmc_set_vector_profile_extern(mc_Leadshine.CardID, usCrd, 0, fSpeed, fSpeed * AccRatio, fSpeed * AccRatio, fSpeed * AccRatio * JerkRatio, fSpeed * AccRatio * JerkRatio, 0);//插补运动速度曲线设置

                                res = LTDMC.dmc_conti_set_lookahead_mode(mc_Leadshine.CardID, usCrd, 4, 1000, 0, fSpeed * AccRatio); //使用AOI专用功能，前瞻模式需要设置为4，允许误差需要设置为0
                                res = LTDMC.dmc_conti_open_list(mc_Leadshine.CardID, usCrd, (ushort)arrayAxisList.Length, arrayAxisList);//打开连续插补缓存区
                                //res = LTDMC.dmc_conti_start_list(mc_Leadshine.CardID, usCrd);//开始连续插补运动                                                                                                       
                                for (int i = 0; i < uiRowNum; i++)
                                {
                                    for (int j = 0; j < uiColumnNum; j++)
                                    {
                                        fTargetPos[j] = arrayTargetPos[i, j];
                                    }
                                    if (i >= 1 && i < 93)
                                    {
                                        //注意：滞后时间与翻转时间之和必须≤单个轨迹段最小运行时间限制          
                                        //res = LTDMC.dmc_conti_accurate_outbit_unit(mc_Leadshine.CardID, usCrd, usCameraOutNo, 0, 0XFF, 1800, 5, 8);//连续插补中精确位置 CMP 输出控制
                                        //res = LTDMC.dmc_conti_delay_outbit_to_start(mc_Leadshine.CardID, usCrd, usCameraOutNo, 0, 100000, 0, 5000);//连续插补中相对于轨迹段起点IO滞后输出（段内执行）,时间单位us
                                    }
                                    res = LTDMC.dmc_conti_line_unit(mc_Leadshine.CardID, usCrd, (ushort)arrayAxisList.Length, arrayAxisList, fTargetPos, 1, 0);
                                }
                                res = LTDMC.dmc_conti_start_list(mc_Leadshine.CardID, usCrd);//开始连续插补运动 
                                res = LTDMC.dmc_conti_close_list(mc_Leadshine.CardID, usCrd);//关闭插补缓存区，不再允许写入坐标数据
                            }
                            break;
                        }
                        Commands.ExcuteVectorMove = false;
                    }
                    #endregion

                    #region 启动-停止插补运动
                    if (Commands.ExcuteStopVectorMove)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usCrd = decimal.ToUInt16(numericUpDown18.Value);//声明一个插补系号
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_conti_stop_list(mc_Leadshine.CardID, usCrd, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_conti_stop_list({mc_Leadshine.CardID},{mc_Leadshine.CardID},{0})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_conti_stop_list(mc_Leadshine.CardID, usCrd, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_conti_stop_list({mc_Leadshine.CardID},{mc_Leadshine.CardID},{0})={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteStopVectorMove = false;
                    }
                    #endregion

                    #region 启动-指令缓存运动
                    if (Commands.ExcuteGroupMove)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usGroupID = decimal.ToUInt16(numericUpDown32.Value);//组号，范围 0~1
                            ushort usXaxisID = decimal.ToUInt16(numericUpDown35.Value);//X轴号
                            ushort usYaxisID = decimal.ToUInt16(numericUpDown34.Value);//Y轴号
                            ushort usZaxisID = decimal.ToUInt16(numericUpDown33.Value);//Z轴号
                            ushort[] GroupAxisList = new ushort[1] { usXaxisID };//参与运动的轴号
                            if (mc_Leadshine.usGroupStste == 0)
                            {
                                /*
                                short res = LTDMC.dmc_m_set_factor_error(mc_Leadshine.CardID, usXaxisID, 1, 10, 10, 0);//设置位置误差带
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_set_factor_error({mc_Leadshine.CardID},{usXaxisID},{1},{10},{10},{0})={res}");
                                    break;
                                }
                                */
                                short res = LTDMC.dmc_m_open_list(mc_Leadshine.CardID, usGroupID, (ushort)GroupAxisList.Length, GroupAxisList);//打开缓存区
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_open_list({mc_Leadshine.CardID},{usGroupID}, {(ushort)GroupAxisList.Length}, {GroupAxisList})={res}");
                                    break;
                                }
                                /*
                                ushort[] usPlanMode = new ushort[] { 1, 1, 1 };//规划模式0-T规划；1-S规划
                                double[] fStartVel = new double[] { 0, 0, 0 };//轴起始速度
                                double[] fMaxVel = new double[] { decimal.ToDouble(numericUpDown28.Value), decimal.ToDouble(numericUpDown27.Value), decimal.ToDouble(numericUpDown26.Value) };//轴运行速度
                                double[] fStopVel = new double[] { 0, 0, 0 };//轴停止速度
                                double[] fAcc = new double[] { fMaxVel[0] * 10, fMaxVel[1] * 10, fMaxVel[2] * 10 };//轴加速度
                                double[] fDec = new double[] { fMaxVel[0] * 10, fMaxVel[1] * 10, fMaxVel[2] * 10 };//轴减速度
                                double[] fAccJerk = new double[] { fMaxVel[0] * 100, fMaxVel[1] * 100, fMaxVel[2] * 100 };//轴加加速度
                                double[] fDecJerk = new double[] { fMaxVel[0] * 100, fMaxVel[1] * 100, fMaxVel[2] * 100 };//轴减减速度

                                
                                //设置单轴 / 多轴 S - Plus 速度模式
                                res = LTDMC.dmc_m_cmd_buf_set_profile_extern(mc_Leadshine.CardID, usGroupID, (ushort)GroupAxisList.Length, GroupAxisList, usPlanMode, fStartVel, fMaxVel, fStopVel, fAcc, fDec, fAccJerk, fDecJerk);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_cmd_buf_set_axis_profile_ext({mc_Leadshine.CardID},{usGroupID},{GroupAxisList.Length},{GroupAxisList},{usPlanMode},{fStartVel},{fMaxVel},{fStopVel}, {fAcc}, {fDec}, {fAccJerk}, {fDecJerk})={res}");
                                    break;
                                }
                                //res = LTDMC.dmc_m_set_profile_unit(mc_Leadshine.CardID, usGroupID, usXaxisID, 0, fMaxVel[0], 0.1, 0.1, 0);
                                //res = LTDMC.dmc_m_set_profile_unit(mc_Leadshine.CardID, usGroupID, usYaxisID, 0, fMaxVel[1], 0.1, 0.1, 0);
                                //res = LTDMC.dmc_m_set_profile_unit(mc_Leadshine.CardID, usGroupID, usZaxisID, 0, fMaxVel[2], 0.1, 0.1, 0);


                                ////添加Z轴运动
                                double MTargetPos = decimal.ToDouble(numericUpDown29.Value);
                                res = LTDMC.dmc_m_add_sigaxis_moveseg_data_ex(mc_Leadshine.CardID, usGroupID, usZaxisID, MTargetPos, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_add_sigaxis_moveseg_data_ex({mc_Leadshine.CardID},{usGroupID},{usZaxisID},{MTargetPos},{0})={res}");
                                    break;
                                }

                                //等待事件配置
                                double fTargetValue = decimal.ToDouble(numericUpDown23.Value);
                                res = LTDMC.dmc_m_add_wait_event_data(mc_Leadshine.CardID, usGroupID, 1, usZaxisID, 2, fTargetValue, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_add_wait_event_data({mc_Leadshine.CardID},{usGroupID},{1},{usZaxisID},{0}{fTargetValue},{0})={res}");
                                    break;
                                }

                                //// 添加Y轴运动
                                //MTargetPos = decimal.ToDouble(numericUpDown30.Value);
                                //res = LTDMC.dmc_m_add_sigaxis_moveseg_data_ex(mc_Leadshine.CardID, usGroupID, usYaxisID, MTargetPos, 0);
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"dmc_m_add_sigaxis_moveseg_data_ex({mc_Leadshine.CardID},{usGroupID},{usYaxisID},{MTargetPos},{0})={res}");
                                //    break;
                                //}

                                ////添加X轴运动
                                //MTargetPos = decimal.ToDouble(numericUpDown31.Value);
                                //res = LTDMC.dmc_m_add_sigaxis_moveseg_data_ex(mc_Leadshine.CardID, usGroupID, usXaxisID, MTargetPos, 0);
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"dmc_m_add_sigaxis_moveseg_data_ex({mc_Leadshine.CardID},{usGroupID},{usXaxisID},{MTargetPos},{0})={res}");
                                //    break;
                                //}

                                //添加  X Y 轴运动
                                res = LTDMC.dmc_m_add_sigaxis_moveseg_data_multi(mc_Leadshine.CardID, usGroupID, 2, new ushort[] { usXaxisID, usYaxisID }, new double[] { decimal.ToDouble(numericUpDown31.Value), decimal.ToDouble(numericUpDown30.Value) }, new uint[] { 0, 0 });
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_add_sigaxis_moveseg_data_multi({mc_Leadshine.CardID},{usGroupID},{2},{usXaxisID},{decimal.ToDouble(numericUpDown11.Value)},{0})={res}");
                                    break;
                                }
                                //等待事件配置
                                fTargetValue = decimal.ToDouble(numericUpDown25.Value);
                                res = LTDMC.dmc_m_add_wait_event_data(mc_Leadshine.CardID, usGroupID, 1, usXaxisID, 0, fTargetValue, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_add_wait_event_data({mc_Leadshine.CardID},{usGroupID},{1},{usXaxisID},{0}{fTargetValue},{0})={res}");
                                    break;
                                }
                                //等待事件配置
                                fTargetValue = decimal.ToDouble(numericUpDown24.Value);
                                res = LTDMC.dmc_m_add_wait_event_data(mc_Leadshine.CardID, usGroupID, 1, usYaxisID, 0, fTargetValue, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_add_wait_event_data({mc_Leadshine.CardID},{usGroupID},{1},{usYaxisID},{0}{fTargetValue},{0})={res}");
                                    break;
                                }
                                //添加Z轴运动
                                res = LTDMC.dmc_m_add_sigaxis_moveseg_data_ex(mc_Leadshine.CardID, usGroupID, usZaxisID, 0, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_add_sigaxis_moveseg_data_ex({mc_Leadshine.CardID},{usGroupID},{usZaxisID},{0},{0})={res}");
                                    break;
                                }
                                */

                                ushort usAxisID = 0;//轴号
                                double fAxisSpeed = 5000;//轴速度
                                double fAxisTarget = 5000;//轴目标位置
                                ushort usOutputID = 2;//输出口号
                                ushort usInputID = 6;//输入口号
                                double fInputLogic = 0; //输入电平逻辑，0：低电平；1：高电平
                                //res = LTDMC.dmc_m_add_wait_event_data(mc_Leadshine.CardID, usGroupID, 3, usInputID, 0, fInputLogic, 0);//等待输入口触发
                                //res = LTDMC.dmc_m_add_time_delay(mc_Leadshine.CardID, usGroupID, 1000, 0);//延时，单位ms
                                //res = LTDMC.dmc_m_add_trigger_data(mc_Leadshine.CardID, usGroupID, 0, usOutputID, 0, 0);//添加触发动作：打开输出口2
                                res = LTDMC.dmc_m_set_profile_unit(mc_Leadshine.CardID, usGroupID, usAxisID, 0, fAxisSpeed, 0.01, 0.01, 0);//单轴速度
                                res = LTDMC.dmc_m_add_sigaxis_moveseg_data_ex(mc_Leadshine.CardID, usGroupID, usAxisID, fAxisTarget, 0);//轴开始朝目标位置运动(绝对坐标)
                                //res = LTDMC.dmc_m_add_wait_event_data(mc_Leadshine.CardID, usGroupID, 3, (ushort)(usInputID + 1), 0, fInputLogic, 0);//等待输入口触发
                                //res = LTDMC.dmc_m_add_trigger_data(mc_Leadshine.CardID, usGroupID, 0, usOutputID, 1, 0);//添加触发动作：关闭输出口2
                                //res = LTDMC.dmc_m_add_trigger_data(mc_Leadshine.CardID, usGroupID, 2, usAxisID, 1, 0);//添加触发动作：轴减速停止运动
                                res = LTDMC.dmc_m_add_sigaxis_moveseg_data_extern(mc_Leadshine.CardID, usGroupID, usAxisID, 2300, 0, 0);
                                //res = LTDMC.dmc_m_add_sigaxis_moveseg_data_ex(mc_Leadshine.CardID, usGroupID, usAxisID, fAxisTarget, 0);//轴开始朝目标位置运动(绝对坐标)

                                res = LTDMC.dmc_m_start_list(mc_Leadshine.CardID, usGroupID);//启动缓存区运动
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_start_list({mc_Leadshine.CardID},{usGroupID})={res}");
                                    break;
                                }
                                res = LTDMC.dmc_m_close_list(mc_Leadshine.CardID, usGroupID);//关闭缓存区
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_close_list({mc_Leadshine.CardID},{usGroupID})={res}");
                                    break;
                                }
                            }
                            else if (mc_Leadshine.usGroupStste == 99 && mc_Leadshine.usGroupEnable == 1)
                            {
                                short res = LTDMC.dmc_m_start_list(mc_Leadshine.CardID, usGroupID);//启动缓存区运动
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_start_list({mc_Leadshine.CardID},{usGroupID})={res}");
                                    break;
                                }
                            }
                            else
                            {
                                AddListInfo(ListInfo, $"缓存区状态不处于空闲");
                                break;
                            }
                            break;
                        }
                        Commands.ExcuteGroupMove = false;
                    }
                    #endregion

                    #region 启动-暂停指令缓存功能
                    if (Commands.ExcuteGroupPauseMove)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usGroupID = decimal.ToUInt16(numericUpDown32.Value);//组号，范围 0~1
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_m_pause_list(mc_Leadshine.CardID, usGroupID, 0);//暂停缓存区运动
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_pause_list({mc_Leadshine.CardID},{usGroupID},{0})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                //short res = LTSMC.smc_m_pause_list(mc_Leadshine.CardID, usGroupID, 0);//暂停缓存区运动
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"dmc_m_pause_list({mc_Leadshine.CardID},{usGroupID},{0})={res}");
                                //    break;
                                //}
                            }
                            break;
                        }
                        Commands.ExcuteGroupPauseMove = false;
                    }
                    #endregion

                    #region 启动-停止指令缓存功能
                    if (Commands.ExcuteGroupStopMove)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usGroupID = decimal.ToUInt16(numericUpDown32.Value);//组号，范围 0~1
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_m_stop_list(mc_Leadshine.CardID, usGroupID, 0);//停止缓存区运动
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"dmc_m_stop_list({mc_Leadshine.CardID},{usGroupID},{0})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                //short res = LTSMC.smc_m_stop_list(mc_Leadshine.CardID, usGroupID, 0);//停止缓存区运动
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"smc_m_stop_list({mc_Leadshine.CardID},{usGroupID},{0})={res}");
                                //    break;
                                //}
                            }
                            break;
                        }
                        Commands.ExcuteGroupStopMove = false;
                    }
                    #endregion

                    #region 启动-龙门跟随运动
                    if (Commands.ExcuteGearMove)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usMasterAxisID = decimal.ToUInt16(numericUpDown41.Value);//主轴轴号
                            ushort usSlaveAxisID = decimal.ToUInt16(numericUpDown42.Value);//跟随轴轴号
                            double fRatio = decimal.ToDouble(numericUpDown43.Value);//比率(从轴分子，主轴分母)
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_set_gear_follow_profile(mc_Leadshine.CardID, usSlaveAxisID, 1, usMasterAxisID, fRatio);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_gear_follow_profile({mc_Leadshine.CardID},{usSlaveAxisID}, {1}, {usMasterAxisID}, {fRatio})={res}");
                                    break;
                                }
                                double fProtectError = decimal.ToDouble(numericUpDown44.Value);
                                //设置龙门模式主从轴编码器跟随误差停止阀值
                                res = LTDMC.dmc_set_grant_error_protect_unit(mc_Leadshine.CardID, usMasterAxisID, 0, fProtectError, fProtectError * 10);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_grant_error_protect_unit({mc_Leadshine.CardID}, {usMasterAxisID}, {1},{fProtectError},{fProtectError * 10})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_set_gear_follow_profile(mc_Leadshine.CardID, usSlaveAxisID, 1, usMasterAxisID, fRatio);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_gear_follow_profile({mc_Leadshine.CardID},{usSlaveAxisID}, {1}, {usMasterAxisID}, {fRatio})={res}");
                                    break;
                                }
                                double fProtectError = decimal.ToDouble(numericUpDown44.Value);
                                //设置龙门模式主从轴编码器跟随误差停止阀值
                                res = LTSMC.smc_set_grant_error_protect_unit(mc_Leadshine.CardID, usMasterAxisID, 1, fProtectError, fProtectError * 10);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_grant_error_protect_unit({mc_Leadshine.CardID}, {usMasterAxisID}, {1},{fProtectError},{fProtectError * 10})={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteGearMove = false;
                    }
                    #endregion

                    #region 启动-关闭龙门跟随运动
                    if (Commands.ExcuteStopGearMove)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usMasterAxisID = decimal.ToUInt16(numericUpDown41.Value);//主轴轴号
                            ushort usSlaveAxisID = decimal.ToUInt16(numericUpDown42.Value);//跟随轴轴号
                            double fRatio = decimal.ToDouble(numericUpDown43.Value);//比率(从轴分子，主轴分母)
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_set_gear_follow_profile(mc_Leadshine.CardID, usSlaveAxisID, 0, usMasterAxisID, fRatio);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_gear_follow_profile({mc_Leadshine.CardID},{usSlaveAxisID}, {1}, {usMasterAxisID}, {fRatio})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_set_gear_follow_profile(mc_Leadshine.CardID, usSlaveAxisID, 0, usMasterAxisID, fRatio);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_gear_follow_profile({mc_Leadshine.CardID},{usSlaveAxisID}, {1}, {usMasterAxisID}, {fRatio})={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteStopGearMove = false;
                    }
                    #endregion

                    #region 启动-轴清除报警
                    if (Commands.ExcuteClearAxisError)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
                        {
                            ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);

                            if (mc_Leadshine.AxisAlarm[usAxisID] == true)
                            {
                                if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                                {
                                    short res = LTDMC.nmc_clear_axis_errcode(mc_Leadshine.CardID, usAxisID);//清除轴报警
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：nmc_clear_axis_errcode({mc_Leadshine.CardID},{usAxisID})={res}");
                                        break;
                                    }
                                }
                                else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                                {
                                    short res = LTSMC.nmcs_clear_axis_errcode(mc_Leadshine.CardID, usAxisID);//清除轴报警
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：nmcs_clear_axis_errcode({mc_Leadshine.CardID},{usAxisID})={res}");
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                        Commands.ExcuteClearAxisError = false;
                    }
                    #endregion

                    #region 启动-切换输出口的状态
                    if (Commands.ExcuteTurnOnOffOutbit)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            ushort OutValue = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_read_outbit_ex(mc_Leadshine.CardID, usCurrentOutID, ref OutValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_read_outbit_ex({mc_Leadshine.CardID}, {usCurrentOutID}, {OutValue})={res}");
                                    break;
                                }
                                res = LTDMC.dmc_write_outbit(mc_Leadshine.CardID, usCurrentOutID, (ushort)(OutValue == 0 ? 1 : 0));
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_write_outbit({mc_Leadshine.CardID}, {usCurrentOutID}, {(ushort)(OutValue == 0 ? 1 : 0)})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_read_outbit_ex(mc_Leadshine.CardID, usCurrentOutID, ref OutValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_read_outbit_ex({mc_Leadshine.CardID}, {usCurrentOutID}, {OutValue})={res}");
                                    break;
                                }
                                res = LTSMC.smc_write_outbit(mc_Leadshine.CardID, usCurrentOutID, (ushort)(OutValue == 0 ? 1 : 0));
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_write_outbit({mc_Leadshine.CardID}, {usCurrentOutID}, {(ushort)(OutValue == 0 ? 1 : 0)})={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteTurnOnOffOutbit = false;
                    }
                    #endregion

                    #region 启动-清除总线通讯报警
                    if (Commands.ExcuteClearEtherCATError)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState != 0 && mc_Leadshine.AxisNum >= 1)
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.nmc_clear_errcode(mc_Leadshine.CardID, 2);//清除 EtherCAT 总线错误码
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_clear_errcode({mc_Leadshine.CardID},2)={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.nmcs_clear_errcode(mc_Leadshine.CardID, 2);//清除 EtherCAT 总线错误码
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_clear_errcode({mc_Leadshine.CardID},2)={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteClearEtherCATError = false;
                    }
                    #endregion

                    #region 启动-设置模拟量输出
                    if (Commands.ExcuteSetDaValue)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.DaNum > 0)
                        {
                            double DaValue = 0;
                            ushort channelId = decimal.ToUInt16(numericUpDown6.Value);
                            double.TryParse(this.textBox12.Text.Trim(), out DaValue);
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_set_da_output(mc_Leadshine.CardID, channelId, DaValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_da_output({mc_Leadshine.CardID}, {channelId}, {DaValue})={res}");
                                    break;
                                }
                                channelId = decimal.ToUInt16(numericUpDown7.Value);
                                double.TryParse(this.textBox13.Text, out DaValue);
                                res = LTDMC.dmc_set_da_output(mc_Leadshine.CardID, channelId, DaValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_da_output({mc_Leadshine.CardID}, {channelId}, {DaValue})={res}");
                                    break;
                                }
                                channelId = decimal.ToUInt16(numericUpDown8.Value);
                                double.TryParse(this.textBox14.Text, out DaValue);
                                res = LTDMC.dmc_set_da_output(mc_Leadshine.CardID, channelId, DaValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_da_output({mc_Leadshine.CardID}, {channelId}, {DaValue})={res}");
                                    break;
                                }
                                channelId = decimal.ToUInt16(numericUpDown9.Value);
                                double.TryParse(this.textBox15.Text, out DaValue);
                                res = LTDMC.dmc_set_da_output(mc_Leadshine.CardID, channelId, DaValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_da_output({mc_Leadshine.CardID}, {channelId}, {DaValue})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_set_da_output(mc_Leadshine.CardID, channelId, DaValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_da_output({mc_Leadshine.CardID}, {channelId}, {DaValue})={res}");
                                    break;
                                }
                                channelId = decimal.ToUInt16(numericUpDown7.Value);
                                double.TryParse(this.textBox13.Text, out DaValue);
                                res = LTSMC.smc_set_da_output(mc_Leadshine.CardID, channelId, DaValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_da_output({mc_Leadshine.CardID}, {channelId}, {DaValue})={res}");
                                    break;
                                }
                                channelId = decimal.ToUInt16(numericUpDown8.Value);
                                double.TryParse(this.textBox14.Text, out DaValue);
                                res = LTSMC.smc_set_da_output(mc_Leadshine.CardID, channelId, DaValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_da_output({mc_Leadshine.CardID}, {channelId}, {DaValue})={res}");
                                    break;
                                }
                                channelId = decimal.ToUInt16(numericUpDown9.Value);
                                double.TryParse(this.textBox15.Text, out DaValue);
                                res = LTSMC.smc_set_da_output(mc_Leadshine.CardID, channelId, DaValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_da_output({mc_Leadshine.CardID}, {channelId}, {DaValue})={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteSetDaValue = false;
                    }
                    #endregion

                    #region 启动-清除辅助编码器的值
                    if (Commands.ExcuteClearExtraEncoder)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_set_extra_encoder(mc_Leadshine.CardID, decimal.ToUInt16(numericUpDown19.Value), 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_extra_encoder({mc_Leadshine.CardID}, {decimal.ToUInt16(numericUpDown19.Value)}, {0})={res}");
                                    break;
                                }
                                res = LTDMC.dmc_set_extra_encoder(mc_Leadshine.CardID, decimal.ToUInt16(numericUpDown20.Value), 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_set_extra_encoder({mc_Leadshine.CardID}, {decimal.ToUInt16(numericUpDown20.Value)}, {0})={res}");
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_set_extra_encoder(mc_Leadshine.CardID, decimal.ToUInt16(numericUpDown19.Value), 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_extra_encoder({mc_Leadshine.CardID}, {decimal.ToUInt16(numericUpDown19.Value)}, {0})={res}");
                                    break;
                                }
                                res = LTSMC.smc_set_extra_encoder(mc_Leadshine.CardID, decimal.ToUInt16(numericUpDown20.Value), 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_set_extra_encoder({mc_Leadshine.CardID}, {decimal.ToUInt16(numericUpDown20.Value)}, {0})={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteClearExtraEncoder = false;
                    }
                    #endregion

                    #region 启动-读取PDO值
                    if (Commands.ExcuteReadPDO)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            ushort usSlaveAddress = decimal.ToUInt16(numericUpDown47.Value);//从站ID号
                            ushort usPdoIndex = decimal.ToUInt16(numericUpDown48.Value); //Convert.ToUInt16(numericUpDown48.Value.ToString(),16);//PDO地址主索引
                            ushort usPdoSubIndex = decimal.ToUInt16(numericUpDown49.Value);//Convert.ToUInt16(numericUpDown49.Value.ToString(), 16);//PDO地址子索引
                            ushort usPdoValueLength = 0;//PDO地址长度
                            byte[] PdoValue = new byte[4];//PDO地址的值
                            this.Invoke((MethodInvoker)delegate { usPdoValueLength = Convert.ToUInt16(comboBox8.Text); });
                            if (radioButton9.Checked)
                            {
                                short res = LTDMC.nmc_read_txpdo(mc_Leadshine.CardID, 2, usSlaveAddress, usPdoIndex, usPdoSubIndex, usPdoValueLength, PdoValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_read_txpdo({mc_Leadshine.CardID}, {2}, {usSlaveAddress}, {usPdoIndex},{usPdoSubIndex} ,{usPdoValueLength},{PdoValue})={res}");
                                    this.Invoke((MethodInvoker)delegate { textBox51.Text = string.Empty; });
                                    break;
                                }
                                this.Invoke((MethodInvoker)delegate { textBox51.Text = BitConverter.ToInt32(PdoValue, 0).ToString(); });
                            }
                            else
                            {
                                short res = LTDMC.nmc_read_rxpdo(mc_Leadshine.CardID, 2, usSlaveAddress, usPdoIndex, usPdoSubIndex, usPdoValueLength, PdoValue);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_read_rxpdo({mc_Leadshine.CardID}, {2}, {usSlaveAddress}, {usPdoIndex},{usPdoSubIndex} ,{usPdoValueLength},{PdoValue})={res}");
                                    this.Invoke((MethodInvoker)delegate { textBox51.Text = string.Empty; });
                                    break;
                                }
                                this.Invoke((MethodInvoker)delegate { textBox51.Text = BitConverter.ToInt32(PdoValue, 0).ToString(); });
                            }
                            break;
                        }
                        Commands.ExcuteReadPDO = false;
                    }
                    #endregion

                    #region 启动-设置PDO值
                    if (Commands.ExcuteWritePDO)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            ushort usSlaveAddress = decimal.ToUInt16(numericUpDown47.Value);//从站ID号
                            ushort usPdoIndex = decimal.ToUInt16(numericUpDown48.Value); //Convert.ToUInt16(numericUpDown48.Value.ToString(),16);//PDO地址主索引
                            ushort usPdoSubIndex = decimal.ToUInt16(numericUpDown49.Value);//Convert.ToUInt16(numericUpDown49.Value.ToString(), 16);//PDO地址子索引
                            ushort usPdoValueLength = 0;//PDO地址长度
                            int iTempValue = 0;//临时变量
                            int.TryParse(textBox51.Text, out iTempValue);
                            byte[] PdoValue = BitConverter.GetBytes(iTempValue);//PDO地址的值
                            //byte[] PdoValue =BitConverter.GetBytes( Convert.ToInt32(textBox51.Text)) ;//System.Text.Encoding.UTF8.GetBytes(textBox51.Text);
                            this.Invoke((MethodInvoker)delegate { usPdoValueLength = Convert.ToUInt16(comboBox8.Text); });
                            short res = LTDMC.nmc_write_rxpdo(mc_Leadshine.CardID, 2, usSlaveAddress, usPdoIndex, usPdoSubIndex, usPdoValueLength, PdoValue);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_write_rxpdo({mc_Leadshine.CardID}, {2}, {usSlaveAddress}, {usPdoIndex},{usPdoSubIndex} ,{usPdoValueLength},{PdoValue})={res}");
                                break;
                            }
                            break;
                        }
                        Commands.ExcuteWritePDO = false;
                    }
                    #endregion

                    #region 启动-读取扩展PDO值
                    if (Commands.ExcuteReadExtraPDO)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            //连续读取扩展TxPDO地址的值（单字WORD，如果是双字需要自己组合，低位在前高位在后）
                            ushort[] PdoValue = new ushort[2];//添加扩展TxPDO地址的最大长度
                            short res = LTDMC.nmc_read_txpdo_extra_short(mc_Leadshine.CardID, 2, decimal.ToUInt16(numericUpDown45.Value), (ushort)PdoValue.Length, PdoValue);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_read_txpdo_extra_short({mc_Leadshine.CardID},{2},{decimal.ToUInt16(numericUpDown45.Value)},{(ushort)PdoValue.Length},{PdoValue})={res}");
                                break;
                            }
                            this.Invoke((MethodInvoker)delegate { textBox45.Text = string.Join(",", PdoValue); });
                            Int32 aa = Convert.ToInt32((UInt32)(PdoValue[0] + PdoValue[1] << 16));//将2个单字转换一个带符号的双字



                            //short res = LTDMC.nmc_read_txpdo_extra_extend(mc_Leadshine.CardID, 2, 4,, decimal.ToUInt16(numericUpDown45.Value), 8, PdoValue);
                            //if (res != 0)
                            //{
                            //    AddListInfo(ListInfo, $"报警：nmc_read_txpdo_extra_short({mc_Leadshine.CardID}, {2}, {decimal.ToUInt16(numericUpDown45.Value)}, {8}, {PdoValue})={res}");
                            //    break;
                            //}
                            //this.Invoke((MethodInvoker)delegate { textBox45.Text = string.Join(",", PdoValue); });



                            //res = LTDMC.nmc_read_rxpdo_extra_short(mc_Leadshine.CardID, 2, decimal.ToUInt16(numericUpDown46.Value), 8, PdoValue);
                            //if (res != 0)
                            //{
                            //    AddListInfo(ListInfo, $"报警：nmc_read_rxpdo_extra_short({mc_Leadshine.CardID}, {2}, {decimal.ToUInt16(numericUpDown46.Value)}, {8}, {PdoValue})={res}");
                            //    break;
                            //}
                            ////this.Invoke((MethodInvoker)delegate { textBox46.Text = string.Join(",", PdoValue); });
                            //this.Invoke((MethodInvoker)delegate { textBox46.Text = PdoValue[0].ToString(); });
                            break;
                        }
                        Commands.ExcuteReadExtraPDO = false;
                    }
                    #endregion

                    #region 启动-设置扩展PDO值
                    if (Commands.ExcuteWriteExtraPDO)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            ushort[] PdoValue = new ushort[8];
                            PdoValue[0] = Convert.ToUInt16(textBox46.Text);
                            short res = LTDMC.nmc_write_rxpdo_extra_short(mc_Leadshine.CardID, 2, decimal.ToUInt16(numericUpDown46.Value), 1, PdoValue);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_write_rxpdo_extra_short({mc_Leadshine.CardID}, {2}, {decimal.ToUInt16(numericUpDown45.Value)}, {1}, {PdoValue})={res}");
                                break;
                            }
                            break;
                        }
                        Commands.ExcuteWriteExtraPDO = false;
                    }
                    #endregion

                    #region 启动-高速位置比较功能
                    if (Commands.ExcuteHcmpEnable)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            ushort hcmpId = decimal.ToUInt16(numericUpDown10.Value);    //比较器号
                            ushort hcmpMode = 4;// 比较模式，0：禁止（默认值）1：等于2：小于3：大于4：队列，提供2500个点比较空间，采用先添加先比较，比较完可追加比较点，也可一次性添加多个比较点5：线性，提供起始比较点，位置增量，比较次数
                            ushort hcmpAxis = decimal.ToUInt16(numericUpDown11.Value);  //辅助编码器通道号,取值范围：0~扩展编码器通道数-1
                            ushort hcmpSource = 1;                                      //比较位置源，固定值 1：辅助编码器计数器
                            ushort hcmpLogic = 0;                                       //有效电平： 0：低电平， 1：高电平
                            int hcmpTime = decimal.ToInt32(numericUpDown13.Value);     //脉冲宽度，单位： us，取值范围： 1us~1s
                            int hcmpPos = decimal.ToInt32(numericUpDown1.Value);        //添加比较位置
                            this.Invoke((MethodInvoker)delegate
                            {
                                hcmpMode = (ushort)comboBox1.SelectedIndex;
                                hcmpLogic = (ushort)comboBox2.SelectedIndex;
                            });
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                if (radioButton3.Checked == true)//选择一维位置比较
                                {
                                    short res = LTDMC.dmc_hcmp_2d_set_enable(mc_Leadshine.CardID, hcmpId, 0);//关闭二维高速比较使能
                                    res = LTDMC.dmc_hcmp_set_mode(mc_Leadshine.CardID, hcmpId, hcmpMode);//设置对应比较器的比较模式
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_set_mode({mc_Leadshine.CardID},{hcmpId},{hcmpMode})={res}");
                                        break;
                                    }
                                    res = LTDMC.dmc_hcmp_get_mode(mc_Leadshine.CardID, hcmpId, ref hcmpMode);//读取对应比较器的比较模式
                                    res = LTDMC.dmc_hcmp_set_config(mc_Leadshine.CardID, hcmpId, hcmpAxis, hcmpSource, hcmpLogic, hcmpTime);//配置比较器关联的编码器通道号，比较源，辅助编码器计数器
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_set_config({mc_Leadshine.CardID},{hcmpId},{hcmpAxis},{hcmpSource},{hcmpLogic},{hcmpTime})={res}");
                                        break;
                                    }
                                    res = LTDMC.dmc_hcmp_clear_points(mc_Leadshine.CardID, hcmpId);//清除已添加的所有一维高速位置比较点
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_clear_points({mc_Leadshine.CardID},{hcmpId})={res}");
                                        break;
                                    }
                                    if (enumControllerType == enumMotionControllerType.DMC_E)//是否启用FIFO模式
                                    {
                                        res = LTDMC.dmc_hcmp_fifo_set_mode(mc_Leadshine.CardID, hcmpId, (ushort)(checkBox5.Checked ? 1 : 0));//启用缓存FIFO方式添加比较位置,0不启用，1启用
                                        if (res != 0)
                                        {
                                            AddListInfo(ListInfo, $"报警：dmc_hcmp_fifo_set_mode({mc_Leadshine.CardID},{hcmpId},(ushort) (checkBox5.Checked ? 1:0))={res}");
                                            break;
                                        }
                                        if (checkBox5.Checked)//选择启用缓存FIFO
                                        {
                                            res = LTDMC.dmc_hcmp_fifo_clear_points(mc_Leadshine.CardID, hcmpId);//清除缓存FIFO里面的比较位置,也会把 FPGA 的位置同步清除掉
                                            if (res != 0)
                                            {
                                                AddListInfo(ListInfo, $"报警：dmc_hcmp_fifo_clear_points({mc_Leadshine.CardID},{hcmpId})={res}");
                                                break;
                                            }
                                        }
                                    }
                                    res = LTDMC.dmc_hcmp_add_point(mc_Leadshine.CardID, hcmpId, hcmpPos);//添加/更新一维高速比较位置
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_add_point({mc_Leadshine.CardID},{hcmpId},{hcmpPos})={res}");
                                        break;
                                    }
                                    //for (int i = 0; i < 1000; i++)
                                    //{
                                    //    res = LTDMC.dmc_hcmp_add_point(mc_Leadshine.CardID, hcmpId, hcmpPos+i);//添加/更新一维高速比较位置
                                    //}
                                    //AddListInfo(ListInfo, $"高速位置比较添加1000个位置耗时{(float)swTempTimer.ElapsedMilliseconds / 1000}s");
                                }
                                else
                                {
                                    double hcmpError = decimal.ToDouble(numericUpDown12.Value);
                                    double[] x_Pos = new double[1] { hcmpPos };
                                    double[] y_Pos = new double[1] { hcmpPos };
                                    short res = LTDMC.dmc_hcmp_set_mode(mc_Leadshine.CardID, hcmpId, 0);//关闭一维比较功能
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_set_mode({mc_Leadshine.CardID},{hcmpId},0)={res}");
                                        break;
                                    }
                                    res = LTDMC.dmc_hcmp_2d_set_enable(mc_Leadshine.CardID, hcmpId, 1);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_2d_set_enable({mc_Leadshine.CardID},{hcmpId},1)={res}");
                                        break;
                                    }
                                    res = LTDMC.dmc_hcmp_2d_set_config_unit(mc_Leadshine.CardID, hcmpId, 0, 0, 1, hcmpError, 1, 1, hcmpError, hcmpLogic, hcmpTime);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_2d_set_config_unit({mc_Leadshine.CardID},{hcmpId},0,0,1,{hcmpError},1,1,{hcmpError},{hcmpLogic},{hcmpTime})={res}");
                                        break;
                                    }
                                    res = LTDMC.dmc_hcmp_2d_clear_points(mc_Leadshine.CardID, hcmpId);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_2d_clear_points({mc_Leadshine.CardID},{hcmpId})={res}");
                                        break;
                                    }
                                    res = LTDMC.dmc_hcmp_2d_add_point_unit(mc_Leadshine.CardID, hcmpId, hcmpPos, hcmpPos, (ushort)(hcmpId + 2));
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_2d_add_point_unit({mc_Leadshine.CardID},{hcmpId},{hcmpPos},{hcmpPos},{(ushort)(hcmpId + 2)})={res}");
                                        break;
                                    }
                                    //res = LTDMC.dmc_hcmp_2d_fifo_set_mode(mc_Leadshine.CardID, hcmpId, 1);//启用缓存方式添加二维高速比较位置，fifo缓存模式，0：不启用；1：启用
                                    //res = LTDMC.dmc_hcmp_2d_fifo_add_table_unit(mc_Leadshine.CardID, hcmpId, 1, x_Pos, y_Pos, new ushort[1] { 2 });
                                    //res = LTDMC.dmc_hcmp_2d_fifo_clear_points(mc_Leadshine.CardID, hcmpId);
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                ;
                            }
                            break;
                        }
                        Commands.ExcuteHcmpEnable = false;
                    }
                    #endregion

                    #region 启动-关闭高速位置比较功能
                    if (Commands.ExcuteHcmpDisable)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            ushort hcmpId = decimal.ToUInt16(numericUpDown10.Value);    //比较器号
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                short res = LTDMC.dmc_hcmp_2d_set_enable(mc_Leadshine.CardID, hcmpId, 0);//关闭二维高速比较使能
                                res = LTDMC.dmc_hcmp_set_mode(mc_Leadshine.CardID, hcmpId, 0);//关闭一维比较功能
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_hcmp_set_mode({mc_Leadshine.CardID},{hcmpId},0)={res}");
                                    break;
                                }
                                if (enumControllerType == enumMotionControllerType.DMC_E)
                                {
                                    res = LTDMC.dmc_hcmp_fifo_set_mode(mc_Leadshine.CardID, hcmpId, 0);//启用缓存方式添加比较位置,0不启用，1启用
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_fifo_set_mode({mc_Leadshine.CardID},{hcmpId},0)={res}");
                                        break;
                                    }
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                short res = LTSMC.smc_hcmp_2d_set_enable(mc_Leadshine.CardID, hcmpId, 0);//关闭二维高速比较使能
                                res = LTSMC.smc_hcmp_set_mode(mc_Leadshine.CardID, hcmpId, 0);//关闭一维比较功能
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_hcmp_set_mode({mc_Leadshine.CardID},{hcmpId},0)={res}");
                                    break;
                                }
                            }
                            break;
                        }
                        Commands.ExcuteHcmpDisable = false;
                    }
                    #endregion

                    #region 启动-采样跟踪
                    if (Commands.ExcuteTraceData)
                    {
                        while (!mc_Leadshine.IsTracing && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState == 0)
                        {
                            int iCycleTime = 0;
                            //获取当前的采样周期时间，单位us（其实获取的是总线周期时间）
                            short res = LTDMC.nmc_get_cycletime(mc_Leadshine.CardID, 2, ref iCycleTime);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_get_cycletime({mc_Leadshine.CardID}, {2}, {iCycleTime})={res}");
                                break;
                            }
                            mc_Leadshine.EthercatTime = iCycleTime;
                            //强制停止采样
                            res = LTDMC.dmc_trace_data_stop(mc_Leadshine.CardID);
                            //复位trace采样，停止采样的时候才能调用，会清除溢出标志位和采集到的数据
                            res = LTDMC.dmc_trace_data_reset(mc_Leadshine.CardID);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_trace_data_reset({mc_Leadshine.CardID})={res}");
                                break;
                            }

                            //采样数据的配置信息，参数如下：
                            short shTraceCycle = 1;//采样周期数，每shTraceCycle个总线周期采样一次
                            short shLostHandle = 1;//溢出后处理方式 0-停止采集，1-覆盖采集
                            short shTraceType = 0;// 采集方式 0 - 连续追踪， 1 - 条件触发追踪
                            short shTriggerObjectIndex = 0;//触发对象索引
                            short shTriggerType = 0; //触发方式：0 - 等于，1 - 大于，2 - 小于
                            int iTriggerMask = 0; //按位触发时的掩码
                            long lTriggerCondition = 0;//触发采集的数值，double类型对象请用内存拷贝memcpy方式赋值
                            res = LTDMC.dmc_trace_set_config(mc_Leadshine.CardID, shTraceCycle, shLostHandle, shTraceType, shTriggerObjectIndex, shTriggerType, iTriggerMask, lTriggerCondition);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_trace_set_config({mc_Leadshine.CardID}, {shTraceCycle}, {shLostHandle}, {shTraceType}, {shTriggerObjectIndex}, {shTriggerType}, {iTriggerMask}, {lTriggerCondition})={res}");
                                break;
                            }
                            //清空所有采集对象，再次启动采集之前需要重新添加好对象
                            res = LTDMC.dmc_trace_reset_config_object(mc_Leadshine.CardID);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_trace_reset_config_object({mc_Leadshine.CardID})={res}");
                                break;
                            }

                            ushort usAxisID = decimal.ToUInt16(numericUpDown52.Value);//轴号
                            ushort usSlaveAddr = 0;//驱动器轴对应的的ID号，如1001,1002
                            ushort usSubSlaveAddr = 0;
                            res = LTDMC.nmc_get_axis_node_address(mc_Leadshine.CardID, usAxisID, ref usSlaveAddr, ref usSubSlaveAddr);//获取轴号对应的从站ID
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_get_axis_node_address({mc_Leadshine.CardID}, {usSlaveAddr}, {usSubSlaveAddr})={res}");
                                break;
                            }

                            //配置采集对象，一次可以添加500个采集对象
                            short shDateType = 2;//数据的类型，见采集对象说明表。
                            short shDateIndex = 0;//数据的主索引，如果采集对象是轴相关，则是轴序号；如果采集对象是按位IO，则是IO序号；如果是按组采集IO，则表示IO起始的序号；如果采集对象是PDO，则是对象字典的索引号。
                            short shDateSubindex = 0;//数据的子索引，如果采集对象是轴相关，则是0；如果采集对象是按组采集IO，则表示IO结束的序号；如果采集对象是PDO，则是对象字典的子索引号。
                            short shSlaveId = (short)usSlaveAddr;//从站ID号，采集PDO对象的时候使用
                            short shDateBytes = 0;//对象字节数，现有采集类型会自动匹配，固定为0，预留后续扩展功能.
                            mc_Leadshine.ObjectBytes[0] = 4;//周期数占用的字节数，默认4字节

                            mc_Leadshine.ObjectBytes[1] = 1;//添加PDO地址0x6060，数据类型sint
                            res = LTDMC.dmc_trace_add_config_object(mc_Leadshine.CardID, 19, 0x6060, 0, shSlaveId, mc_Leadshine.ObjectBytes[1]);

                            mc_Leadshine.ObjectBytes[2] = 1;//添加PDO地址0x6061，数据类型sint
                            res = LTDMC.dmc_trace_add_config_object(mc_Leadshine.CardID, 19, 0x6061, 0, shSlaveId, mc_Leadshine.ObjectBytes[2]);

                            mc_Leadshine.ObjectBytes[3] = 2;//添加PDO地址0x6040，数据类型uint
                            res = LTDMC.dmc_trace_add_config_object(mc_Leadshine.CardID, 19, 0x6040, 0, shSlaveId, mc_Leadshine.ObjectBytes[3]);

                            mc_Leadshine.ObjectBytes[4] = 2;//添加PDO地址0x6041，数据类型uint
                            res = LTDMC.dmc_trace_add_config_object(mc_Leadshine.CardID, 19, 0x6041, 0, shSlaveId, mc_Leadshine.ObjectBytes[4]);

                            mc_Leadshine.ObjectBytes[5] = 4;//添加PDO地址0x607A，数据类型Dint
                            res = LTDMC.dmc_trace_add_config_object(mc_Leadshine.CardID, 19, 0x607A, 0, shSlaveId, mc_Leadshine.ObjectBytes[5]);

                            mc_Leadshine.ObjectBytes[6] = 4;//添加PDO地址0x6064，数据类型Dint
                            res = LTDMC.dmc_trace_add_config_object(mc_Leadshine.CardID, 19, 0x6064, 0, shSlaveId, mc_Leadshine.ObjectBytes[6]);

                            mc_Leadshine.ObjectBytes[7] = 4;//添加PDO地址0x60FD，数据类型UDint
                            res = LTDMC.dmc_trace_add_config_object(mc_Leadshine.CardID, 19, 0x60FD, 0, shSlaveId, mc_Leadshine.ObjectBytes[7]);
                            this.Invoke((MethodInvoker)delegate
                            {
                                textBox60.Text = "TraceLog_" + DateTime.Now.Year.ToString() + DateTime.Now.Month.ToString() + DateTime.Now.Day.ToString() + DateTime.Now.Hour.ToString() + DateTime.Now.Minute.ToString() + DateTime.Now.Second.ToString() + ".csv";
                            });
                            FileStream fs = new FileStream(textBox60.Text, FileMode.Create, FileAccess.Write);
                            StreamWriter sw = new StreamWriter(fs);
                            sw.Write("Time(ms)" + "," + "0x6060" + "," + "0x6061" + "," + "0x6040" + "," + "0x6041" + "," + "0x607A" + "," + "0x6064" + "," + "0x60FD" + "\n");
                            sw.Close();
                            fs.Close();
                            res = LTDMC.dmc_trace_data_start(mc_Leadshine.CardID);//启动采样
                            mc_Leadshine.IsTracing = true;
                            break;
                        }
                        Commands.ExcuteTraceData = false;
                    }
                    #endregion

                    #region 启动-停止采样跟踪
                    if (Commands.ExcuteStopTraceData)
                    {
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            mc_Leadshine.IsTracing = false;
                            short res = LTDMC.dmc_trace_data_stop(mc_Leadshine.CardID);//停止采样
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_trace_data_stop({mc_Leadshine.CardID})={res}");
                                break;
                            }
                            res = LTDMC.dmc_trace_data_reset(mc_Leadshine.CardID);//复位trace采样，停止采样的时候才能调用，会清除溢出标志位和采集到的数据
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_trace_data_reset({mc_Leadshine.CardID})={res}");
                                break;
                            }
                            break;
                        }
                        Commands.ExcuteStopTraceData = false;
                    }
                    #endregion

                    #region 启动自动运行
                    if (Commands.ExcuteAutoRun)
                    {
                        if (!mc_Leadshine.IsAutoRunning && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            mc_Leadshine.IsAutoRunning = true;
                            Thread threadAutoRun = new Thread(() =>
                            {
                                while (mc_Leadshine.IsAutoRunning && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                                {
                                    //LTDMC.dmc_change_speed_unit(mc_Leadshine.CardID, 2, 10357.088867, 0.03);
                                    //LTDMC.dmc_update_target_position_unit(mc_Leadshine.CardID, 2, -688251.250000);
                                    //Thread.Sleep(20);
                                    //LTDMC.dmc_change_speed_unit(mc_Leadshine.CardID, 2, 12609.384766, 0.03);
                                    //LTDMC.dmc_update_target_position_unit(mc_Leadshine.CardID, 2, -688308.000000);
                                    //Thread.Sleep(20);
                                    Random rand = new Random();


                                    short res = 0;
                                    res = LTDMC.nmc_set_axis_run_mode(mc_Leadshine.CardID, 9, 1);//设置轴的运行模式 1为pp模式，6为回零模式，8为csp模式
                                    res = LTDMC.dmc_set_profile_unit(mc_Leadshine.CardID, 9, 3000, 3000, 0.006, 0.006, 3000);
                                    res = LTDMC.dmc_pmove_unit(mc_Leadshine.CardID, 9, 1000000, 1);
                                    int randomIntRange = rand.Next(1, 6);// 生成50到100之间的随机整数（包括50，不包括100）
                                    Thread.Sleep(randomIntRange * 1000);
                                    LTDMC.dmc_write_outbit(mc_Leadshine.CardID, 29, 0);
                                    uint usAxisStatus = 2;
                                    while (true)
                                    {
                                        LTDMC.dmc_axis_io_status_ex(mc_Leadshine.CardID, 9, ref usAxisStatus);
                                        if ((usAxisStatus & (1 << 1)) != 0)
                                        {
                                            break;
                                        }
                                    }
                                    ushort usAxisState = 2;
                                    do
                                    {
                                        LTDMC.dmc_check_done_ex(mc_Leadshine.CardID, 9, ref usAxisState);
                                    } while (usAxisState == 0);



                                    res = LTDMC.nmc_set_axis_run_mode(mc_Leadshine.CardID, 9, 1);//设置轴的运行模式 1为pp模式，6为回零模式，8为csp模式
                                    res = LTDMC.dmc_set_profile_unit(mc_Leadshine.CardID, 9, 3000, 3000, 0.006, 0.006, 3000);
                                    res = LTDMC.dmc_pmove_unit(mc_Leadshine.CardID, 9, -5000, 1);
                                    randomIntRange = rand.Next(1, 3);// 生成50到100之间的随机整数（包括50，不包括100）
                                    Thread.Sleep(randomIntRange * 1000);
                                    LTDMC.dmc_write_outbit(mc_Leadshine.CardID, 29, 1);
                                    while (true)
                                    {
                                        LTDMC.dmc_axis_io_status_ex(mc_Leadshine.CardID, 9, ref usAxisStatus);
                                        if ((usAxisStatus & (1 << 1)) == 0)
                                        {
                                            break;
                                        }
                                    }
                                    //LTDMC.dmc_stop(mc_Leadshine.CardID, 9,1);
                                    do
                                    {
                                        LTDMC.dmc_check_done_ex(mc_Leadshine.CardID, 9, ref usAxisState);
                                    } while (usAxisState == 0);
                                }
                                mc_Leadshine.IsAutoRunning = false;
                            });
                            threadAutoRun.IsBackground = true;
                            threadAutoRun.Start();
                        }
                        Commands.ExcuteAutoRun = false;
                    }
                    #endregion

                    #region 获取：轮询卡的各种状态
                    while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                    {
                        #region 获取：总线通讯状态
                        short res = 1;
                        long lUsedTimes = 0;
                        if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                        {
                            ushort state1 = 1;
                            stopWatch.Restart();
                            res = LTDMC.nmc_get_errcode(mc_Leadshine.CardID, 2, ref state1);//读取EtherCAT总线状态
                            lUsedTimes = stopWatch.ElapsedMilliseconds;
                            if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                            {
                                AddListInfo(ListInfo, $"报警：总线通讯异常nmc_get_errcode({mc_Leadshine.CardID},{2},{state1})={res},耗时{lUsedTimes}");
                                if (res != 0)
                                {
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                            }
                            mc_Leadshine.EtherCATState = state1;
                            if (enumControllerType == enumMotionControllerType.EMC_E)
                            {
                                stopWatch.Restart();
                                res = LTDMC.nmc_get_errcode(mc_Leadshine.CardID, 6, ref state1);//读取控制器背板总线状态
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：背板通讯异常nmc_get_errcode({mc_Leadshine.CardID},{6},{state1})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                mc_Leadshine.LocalBusState = state1;
                            }
                        }
                        else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                        {
                            ushort state1 = 1;
                            stopWatch.Restart();
                            res = LTSMC.nmcs_get_errcode(mc_Leadshine.CardID, 2, ref state1);//读取EtherCAT总线状态
                            lUsedTimes = stopWatch.ElapsedMilliseconds;
                            if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                            {
                                AddListInfo(ListInfo, $"报警：总线通讯异常nmcs_get_errcode({mc_Leadshine.CardID},{2},{state1})={res},耗时{lUsedTimes}");
                                if (res != 0)
                                {
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                            }
                            mc_Leadshine.EtherCATState = state1;
                        }
                        #endregion

                        #region 获取：批量读取所有轴的属性：轴专用IO，轴使能，轴运行状态，轴指令位置，轴反馈位置，轴当前速度
                        if ((WindowsPage == 1 | WindowsPage == 6) && mc_Leadshine.AxisNum >= 1)
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                uint[] uiAxisState = new uint[mc_Leadshine.AxisNum];
                                stopWatch.Restart();
                                res = LTDMC.dmc_axis_io_status_ex(mc_Leadshine.CardID, 255, ref uiAxisState[0]);//读取所有轴有关运动信号的状态,包括限位，原点，报警等
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_axis_io_status_ex({mc_Leadshine.CardID},{255},{uiAxisState[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                for (uint i = 0; i < uiAxisState.Length; i++)
                                {
                                    mc_Leadshine.AxisAlarm[i] = ((uiAxisState[i] & (1 << 0)) == 0) ? false : true;//伺服报警
                                    mc_Leadshine.AxisELP[i] = ((uiAxisState[i] & (1 << 1)) == 0) ? false : true;//正限位ELP
                                    mc_Leadshine.AxisELN[i] = ((uiAxisState[i] & (1 << 2)) == 0) ? false : true;//正限位ELN
                                    mc_Leadshine.AxisEMG[i] = ((uiAxisState[i] & (1 << 3)) == 0) ? false : true;//急停EMG
                                    mc_Leadshine.AxisORG[i] = ((uiAxisState[i] & (1 << 4)) == 0) ? false : true;//原点ORG
                                }

                                ushort[] usStateMachine = new ushort[mc_Leadshine.AxisNum];
                                stopWatch.Restart();
                                res = LTDMC.nmc_get_axis_state_machine(mc_Leadshine.CardID, 255, ref usStateMachine[0]);//获取所有轴的使能状态，4为使能，非4未使能
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_get_axis_state_machine({mc_Leadshine.CardID},{255},{usStateMachine[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                for (uint i = 0; i < usStateMachine.Length; i++)
                                {
                                    mc_Leadshine.AxisPowerOn[i] = (usStateMachine[i] == 4) ? true : false;
                                }

                                stopWatch.Restart();
                                res = LTDMC.dmc_check_done_ex(mc_Leadshine.CardID, 255, ref usStateMachine[0]);//获取所有轴运行状态，0运行，1停止
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_check_done_ex({mc_Leadshine.CardID},{255},{usStateMachine[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                for (uint i = 0; i < usStateMachine.Length; i++)
                                {
                                    mc_Leadshine.AxisBusy[i] = (usStateMachine[i] == 0) ? true : false;
                                }

                                stopWatch.Restart();
                                res = LTDMC.dmc_get_home_result(mc_Leadshine.CardID, 255, ref usStateMachine[0]);//获取回零结果
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_home_result({mc_Leadshine.CardID},{255},{usStateMachine[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(usStateMachine, mc_Leadshine.AxisHomeResult, usStateMachine.Length);

                                stopWatch.Restart();
                                res = LTDMC.dmc_get_axis_run_mode(mc_Leadshine.CardID, 255, ref usStateMachine[0]);//获取主轴的运动模式
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_axis_run_mode({mc_Leadshine.CardID},{255},{usStateMachine[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(usStateMachine, mc_Leadshine.AxisRunMode, usStateMachine.Length);

                                int[] iAxisStopReason = new int[mc_Leadshine.AxisNum];
                                stopWatch.Restart();
                                res = LTDMC.dmc_get_stop_reason(mc_Leadshine.CardID, 255, ref iAxisStopReason[0]);//获取轴的停止原因
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_stop_reason({mc_Leadshine.CardID},{255},{iAxisStopReason[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(iAxisStopReason, mc_Leadshine.AxisStopReason, iAxisStopReason.Length);

                                stopWatch.Restart();
                                res = LTDMC.nmc_get_torque(mc_Leadshine.CardID, 255, ref iAxisStopReason[0]);// 获取所有轴的当前转矩值
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_get_torque({mc_Leadshine.CardID},{255},{iAxisStopReason[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(iAxisStopReason, mc_Leadshine.AxisCurrentTorque, iAxisStopReason.Length);

                                double[] temDouble = new double[mc_Leadshine.AxisNum];//临时变量
                                stopWatch.Restart();
                                res = LTDMC.dmc_get_position_unit(mc_Leadshine.CardID, 255, ref temDouble[0]);//获取所有轴的指令位置
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_position_unit({mc_Leadshine.CardID},{255},{temDouble[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(temDouble, mc_Leadshine.AxisCommandPos, temDouble.Length);

                                stopWatch.Restart();
                                res = LTDMC.dmc_get_encoder_unit(mc_Leadshine.CardID, 255, ref temDouble[0]);// 获取所有轴的反馈位置
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_encoder_unit({mc_Leadshine.CardID},{255},{temDouble[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(temDouble, mc_Leadshine.AxisActualPos, temDouble.Length);

                                stopWatch.Restart();
                                res = LTDMC.dmc_read_current_speed_unit(mc_Leadshine.CardID, 255, ref temDouble[0]);// 获取所有轴的当前速度
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_read_current_speed_unit({mc_Leadshine.CardID},{255},{temDouble[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(temDouble, mc_Leadshine.AxisCurrentSpeed, temDouble.Length);



                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                uint[] uiAxisState = new uint[mc_Leadshine.AxisNum];
                                stopWatch.Restart();
                                res = LTSMC.smc_axis_io_status_ex(mc_Leadshine.CardID, 255, ref uiAxisState[0]);//读取所有轴有关运动信号的状态,包括限位，原点，报警等
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_axis_io_status_ex({mc_Leadshine.CardID},{255},{uiAxisState[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                for (uint i = 0; i < uiAxisState.Length; i++)
                                {
                                    mc_Leadshine.AxisAlarm[i] = ((uiAxisState[i] & (1 << 0)) == 0) ? false : true;//伺服报警
                                    mc_Leadshine.AxisELP[i] = ((uiAxisState[i] & (1 << 1)) == 0) ? false : true;//正限位ELP
                                    mc_Leadshine.AxisELN[i] = ((uiAxisState[i] & (1 << 2)) == 0) ? false : true;//正限位ELN
                                    mc_Leadshine.AxisEMG[i] = ((uiAxisState[i] & (1 << 3)) == 0) ? false : true;//急停EMG
                                    mc_Leadshine.AxisORG[i] = ((uiAxisState[i] & (1 << 4)) == 0) ? false : true;//原点ORG
                                }

                                ushort[] usStateMachine = new ushort[mc_Leadshine.AxisNum];
                                stopWatch.Restart();
                                res = LTSMC.nmcs_get_axis_state_machine(mc_Leadshine.CardID, 255, ref usStateMachine[0]);//获取所有轴的使能状态，4为使能，非4未使能
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：nmcs_get_axis_state_machine({mc_Leadshine.CardID},{255},{usStateMachine[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                for (uint i = 0; i < usStateMachine.Length; i++)
                                {
                                    mc_Leadshine.AxisPowerOn[i] = (usStateMachine[i] == 4) ? true : false;
                                }
                                stopWatch.Restart();
                                res = LTSMC.smc_check_done_ex(mc_Leadshine.CardID, 255, ref usStateMachine[0]);//获取所有轴运行状态，0运行，1停止
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_check_done_ex({mc_Leadshine.CardID},{255},{usStateMachine[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                for (uint i = 0; i < usStateMachine.Length; i++)
                                {
                                    mc_Leadshine.AxisBusy[i] = (usStateMachine[i] == 0) ? true : false;
                                }

                                double[] temDouble = new double[mc_Leadshine.AxisNum];//临时变量
                                stopWatch.Restart();
                                res = LTSMC.smc_get_position_unit(mc_Leadshine.CardID, 255, ref temDouble[0]);//获取所有轴的指令位置
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_position_unit({mc_Leadshine.CardID},{255},{temDouble[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(temDouble, mc_Leadshine.AxisCommandPos, temDouble.Length);

                                stopWatch.Restart();
                                res = LTSMC.smc_get_encoder_unit(mc_Leadshine.CardID, 255, ref temDouble[0]);// 获取所有轴的反馈位置
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_encoder_unit({mc_Leadshine.CardID},{255},{temDouble[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(temDouble, mc_Leadshine.AxisActualPos, temDouble.Length);

                                stopWatch.Restart();
                                res = LTSMC.smc_read_current_speed_unit(mc_Leadshine.CardID, 255, ref temDouble[0]);// 获取所有轴的当前速度
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_read_current_speed_unit({mc_Leadshine.CardID},{255},{temDouble[0]})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                Array.Copy(temDouble, mc_Leadshine.AxisCurrentSpeed, temDouble.Length);
                            }
                        }
                        #endregion

                        #region  获取：单轴的属性，包含回零结果，停止原因
                        if ((WindowsPage == 1 | WindowsPage == 6) && mc_Leadshine.AxisNum > 0)
                        {
                            ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                //ushort usAxisHomeRes = 2;
                                //stopWatch.Restart();
                                //res = LTDMC.dmc_get_home_result(mc_Leadshine.CardID, usAxisID, ref usAxisHomeRes);//获取回零结果
                                //lUsedTimes = stopWatch.ElapsedMilliseconds;
                                //if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                //{
                                //    AddListInfo(ListInfo, $"报警：dmc_get_home_result({mc_Leadshine.CardID},{usAxisID},{usAxisHomeRes})={res},耗时{lUsedTimes}");
                                //    if (res != 0)
                                //    {
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //mc_Leadshine.AxisHomeResult[usAxisID] = usAxisHomeRes;

                                //double fCurrentSpeed = 0;
                                //stopWatch.Restart();
                                //res = LTDMC.dmc_read_current_speed_unit(mc_Leadshine.CardID, usAxisID, ref fCurrentSpeed);// 获取所有轴的当前速度
                                //lUsedTimes = stopWatch.ElapsedMilliseconds;
                                //if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                //{
                                //    AddListInfo(ListInfo, $"报警：dmc_read_current_speed_unit({mc_Leadshine.CardID},{255},{fCurrentSpeed})={res},耗时{lUsedTimes}");
                                //    if (res != 0)
                                //    {
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //mc_Leadshine.AxisCurrentSpeed[usAxisID] = fCurrentSpeed;

                                //int iAxisStopReason = 2;
                                //stopWatch.Restart();
                                //res = LTDMC.dmc_get_stop_reason(mc_Leadshine.CardID, usAxisID, ref iAxisStopReason);//获取轴的停止原因
                                //lUsedTimes = stopWatch.ElapsedMilliseconds;
                                //if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                //{
                                //    AddListInfo(ListInfo, $"报警：dmc_get_stop_reason({mc_Leadshine.CardID},{usAxisID},{iAxisStopReason})={res},耗时{lUsedTimes}");
                                //    if (res != 0)
                                //    {
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //mc_Leadshine.AxisStopReason[usAxisID] = iAxisStopReason;

                                //usAxisID = decimal.ToUInt16(numericUpDown41.Value);
                                //ushort usAxisRunMode = 0;
                                //stopWatch.Restart();
                                //res = LTDMC.dmc_get_axis_run_mode(mc_Leadshine.CardID, usAxisID, ref usAxisRunMode);//获取主轴的运动模式
                                //lUsedTimes = stopWatch.ElapsedMilliseconds;
                                //if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                //{
                                //    AddListInfo(ListInfo, $"报警：dmc_get_axis_run_mode({mc_Leadshine.CardID},{usAxisID},{usAxisRunMode})={res},耗时{lUsedTimes}");
                                //    if (res != 0)
                                //    {
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //mc_Leadshine.AxisRunMode[usAxisID] = usAxisRunMode;

                                //usAxisID = decimal.ToUInt16(numericUpDown42.Value);
                                //stopWatch.Restart();
                                //res = LTDMC.dmc_get_axis_run_mode(mc_Leadshine.CardID, usAxisID, ref usAxisRunMode);//获取从轴的运动模式
                                //lUsedTimes = stopWatch.ElapsedMilliseconds;
                                //if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                //{
                                //    AddListInfo(ListInfo, $"报警：dmc_get_axis_run_mode({mc_Leadshine.CardID},{usAxisID},{usAxisRunMode})={res},耗时{lUsedTimes}");
                                //    if (res != 0)
                                //    {
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //mc_Leadshine.AxisRunMode[usAxisID] = usAxisRunMode;
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                ushort usAxisHomeRes = 2;
                                stopWatch.Restart();
                                res = LTSMC.smc_get_home_result(mc_Leadshine.CardID, usAxisID, ref usAxisHomeRes);//获取回零结果
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_home_result({mc_Leadshine.CardID},{usAxisID},{usAxisHomeRes})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                mc_Leadshine.AxisHomeResult[usAxisID] = usAxisHomeRes;

                                //double fCurrentSpeed = 0;
                                //stopWatch.Restart();
                                //res = LTDMC.dmc_read_current_speed_unit(mc_Leadshine.CardID, usAxisID, ref fCurrentSpeed);// 获取所有轴的当前速度
                                //lUsedTimes = stopWatch.ElapsedMilliseconds;
                                //if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                //{
                                //    AddListInfo(ListInfo, $"报警：dmc_read_current_speed_unit({mc_Leadshine.CardID},{255},{fCurrentSpeed})={res},耗时{lUsedTimes}");
                                //    if (res != 0)
                                //    {
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //mc_Leadshine.AxisCurrentSpeed[usAxisID] = fCurrentSpeed;

                                int iAxisStopReason = 2;
                                stopWatch.Restart();
                                res = LTSMC.smc_get_stop_reason(mc_Leadshine.CardID, usAxisID, ref iAxisStopReason);//获取轴的停止原因
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_stop_reason({mc_Leadshine.CardID},{usAxisID},{iAxisStopReason})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                mc_Leadshine.AxisStopReason[usAxisID] = iAxisStopReason;

                                usAxisID = decimal.ToUInt16(numericUpDown41.Value);
                                ushort usAxisRunMode = 0;
                                stopWatch.Restart();
                                res = LTSMC.smc_get_axis_run_mode(mc_Leadshine.CardID, usAxisID, ref usAxisRunMode);//获取主轴的运动模式
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_axis_run_mode({mc_Leadshine.CardID},{usAxisID},{usAxisRunMode})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                mc_Leadshine.AxisRunMode[usAxisID] = usAxisRunMode;

                                usAxisID = decimal.ToUInt16(numericUpDown42.Value);
                                stopWatch.Restart();
                                res = LTSMC.smc_get_axis_run_mode(mc_Leadshine.CardID, usAxisID, ref usAxisRunMode);//获取从轴的运动模式
                                lUsedTimes = stopWatch.ElapsedMilliseconds;
                                if (res != 0 || lUsedTimes >= lMaxUsedTimes)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_axis_run_mode({mc_Leadshine.CardID},{usAxisID},{usAxisRunMode})={res},耗时{lUsedTimes}");
                                    if (res != 0)
                                    {
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                mc_Leadshine.AxisRunMode[usAxisID] = usAxisRunMode;
                            }
                        }
                        #endregion

                        #region 获取：模拟量输入值
                        if (WindowsPage == 3 && (mc_Leadshine.AdNum > 0 | mc_Leadshine.DaNum > 0))
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E)
                            {
                                double[] temDouble1 = new double[255];//临时变量
                                res = LTDMC.dmc_get_ad_input(mc_Leadshine.CardID, 255, ref temDouble1[0]);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_ad_input({mc_Leadshine.CardID}, {255}, {temDouble1[0]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                Array.Copy(temDouble1, mc_Leadshine.ADValue, temDouble1.Length);

                                double[] temDouble2 = new double[255];//临时变量
                                res = LTDMC.dmc_get_da_output(mc_Leadshine.CardID, 255, ref temDouble2[0]);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_da_output({mc_Leadshine.CardID}, {255}, {temDouble2[0]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                Array.Copy(temDouble2, mc_Leadshine.DAValue, temDouble2.Length);

                                //double AdValue = 0;
                                //ushort channelId = 0;
                                //for (int i = 0; i < 4; i++)
                                //{
                                //    switch (i)
                                //    {
                                //        case 0:
                                //            channelId = decimal.ToUInt16(numericUpDown2.Value); break;
                                //        case 1:
                                //            channelId = decimal.ToUInt16(numericUpDown3.Value); break;
                                //        case 2:
                                //            channelId = decimal.ToUInt16(numericUpDown4.Value); break;
                                //        case 3:
                                //            channelId = decimal.ToUInt16(numericUpDown5.Value); break;
                                //        default:
                                //            break;
                                //    }
                                //    res = LTDMC.dmc_get_ad_input(mc_Leadshine.CardID, channelId, ref AdValue);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"报警：dmc_get_ad_input({mc_Leadshine.CardID}, {channelId}, {AdValue})={res}");
                                //        break;
                                //    }
                                //    mc_Leadshine.ADValue[channelId] = AdValue;
                                //}
                                //if (res != 0)
                                //{
                                //    Commands.ExcuteDisConnect = true;
                                //    break;
                                //}
                                //for (int i = 0; i < 4; i++)
                                //{
                                //    switch (i)
                                //    {
                                //        case 0:
                                //            channelId = decimal.ToUInt16(numericUpDown6.Value); break;
                                //        case 1:
                                //            channelId = decimal.ToUInt16(numericUpDown7.Value); break;
                                //        case 2:
                                //            channelId = decimal.ToUInt16(numericUpDown8.Value); break;
                                //        case 3:
                                //            channelId = decimal.ToUInt16(numericUpDown9.Value); break;
                                //        default:
                                //            break;
                                //    }
                                //    res = LTDMC.dmc_get_da_output(mc_Leadshine.CardID, channelId, ref AdValue);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"报警：dmc_get_da_output({mc_Leadshine.CardID}, {channelId}, {AdValue})={res}");
                                //        break;
                                //    }
                                //    mc_Leadshine.DAValue[channelId] = AdValue;
                                //}
                                //if (res != 0)
                                //{
                                //    Commands.ExcuteDisConnect = true;
                                //    break;
                                //}
                            }
                            else if (enumControllerType == enumMotionControllerType.EMC_E)
                            {
                                double[] temDouble1 = new double[255];//临时变量
                                res = LTDMC.dmc_get_ad_input(mc_Leadshine.CardID, 255, ref temDouble1[0]);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_ad_input({mc_Leadshine.CardID}, {255}, {temDouble1[0]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                Array.Copy(temDouble1, mc_Leadshine.ADValue, temDouble1.Length);

                                double[] temDouble2 = new double[255];//临时变量
                                res = LTDMC.dmc_get_da_output(mc_Leadshine.CardID, 255, ref temDouble2[0]);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_da_output({mc_Leadshine.CardID}, {255}, {temDouble2[0]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                Array.Copy(temDouble2, mc_Leadshine.DAValue, temDouble2.Length);
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                double AdValue = 0;
                                ushort channelId = 0;
                                for (int i = 0; i < 4; i++)
                                {
                                    switch (i)
                                    {
                                        case 0:
                                            channelId = decimal.ToUInt16(numericUpDown2.Value); break;
                                        case 1:
                                            channelId = decimal.ToUInt16(numericUpDown3.Value); break;
                                        case 2:
                                            channelId = decimal.ToUInt16(numericUpDown4.Value); break;
                                        case 3:
                                            channelId = decimal.ToUInt16(numericUpDown5.Value); break;
                                        default:
                                            break;
                                    }
                                    res = LTSMC.smc_get_ad_input(mc_Leadshine.CardID, channelId, ref AdValue);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：smc_get_ad_input({mc_Leadshine.CardID}, {channelId}, {AdValue})={res}");
                                        break;
                                    }
                                    mc_Leadshine.ADValue[channelId] = AdValue;
                                }
                                if (res != 0)
                                {
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                for (int i = 0; i < 4; i++)
                                {
                                    switch (i)
                                    {
                                        case 0:
                                            channelId = decimal.ToUInt16(numericUpDown6.Value); break;
                                        case 1:
                                            channelId = decimal.ToUInt16(numericUpDown7.Value); break;
                                        case 2:
                                            channelId = decimal.ToUInt16(numericUpDown8.Value); break;
                                        case 3:
                                            channelId = decimal.ToUInt16(numericUpDown9.Value); break;
                                        default:
                                            break;
                                    }
                                    res = LTSMC.smc_get_da_output(mc_Leadshine.CardID, channelId, ref AdValue);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：smc_get_da_output({mc_Leadshine.CardID}, {channelId}, {AdValue})={res}");
                                        break;
                                    }
                                    mc_Leadshine.DAValue[channelId] = AdValue;
                                }
                                if (res != 0)
                                {
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                            }
                        }
                        #endregion

                        #region 获取：辅助编码器的值
                        if (WindowsPage == 4)
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                ushort channelId = decimal.ToUInt16(numericUpDown19.Value);
                                res = LTDMC.dmc_get_extra_encoder(mc_Leadshine.CardID, channelId, ref mc_Leadshine.ExtraEncoder[channelId]);//读取辅助编码器计数值
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_extra_encoder({mc_Leadshine.CardID},{channelId},{mc_Leadshine.ExtraEncoder[channelId]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                channelId = decimal.ToUInt16(numericUpDown20.Value);
                                res = LTDMC.dmc_get_extra_encoder(mc_Leadshine.CardID, channelId, ref mc_Leadshine.ExtraEncoder[channelId]);//读取辅助编码器计数值
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_extra_encoder({mc_Leadshine.CardID},{channelId},{mc_Leadshine.ExtraEncoder[channelId]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                ushort channelId = decimal.ToUInt16(numericUpDown19.Value);
                                res = LTSMC.smc_get_extra_encoder(mc_Leadshine.CardID, channelId, ref mc_Leadshine.ExtraEncoder[channelId]);//读取辅助编码器计数值
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_extra_encoder({mc_Leadshine.CardID},{channelId},{mc_Leadshine.ExtraEncoder[channelId]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                channelId = decimal.ToUInt16(numericUpDown20.Value);
                                res = LTSMC.smc_get_extra_encoder(mc_Leadshine.CardID, channelId, ref mc_Leadshine.ExtraEncoder[channelId]);//读取辅助编码器计数值
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：smc_get_extra_encoder({mc_Leadshine.CardID},{channelId},{mc_Leadshine.ExtraEncoder[channelId]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                            }
                        }
                        #endregion

                        #region 获取：高速比较器的状态
                        if (WindowsPage == 4)
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                ushort hcmpId = decimal.ToUInt16(numericUpDown10.Value);    //比较器号
                                if (radioButton3.Checked == true)//选择一维高速位置比较
                                {
                                    int remainedPoints = 0;             //返回可添加比较点数
                                    double currentPoint = 0;            //返回当前比较点位置，单位：pluse
                                    int runnedPoints = 0;               //返回已比较点数
                                    res = LTDMC.dmc_hcmp_get_current_state_unit(mc_Leadshine.CardID, hcmpId, ref remainedPoints, ref currentPoint, ref runnedPoints);//读取一维高速比较参数
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_get_current_state_unit({mc_Leadshine.CardID},{hcmpId},{remainedPoints},{currentPoint},{runnedPoints})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                    long remainedFifoPoints = 0;//返回FIFO区间可添加比较点数
                                    if (enumControllerType == enumMotionControllerType.DMC_E)
                                    {
                                        ushort usFifoMde = 2;
                                        res = LTDMC.dmc_hcmp_fifo_get_mode(mc_Leadshine.CardID, hcmpId, ref usFifoMde);//获取缓存方式添加比较位置,0不启用，1启用
                                        if (res != 0)
                                        {
                                            AddListInfo(ListInfo, $"报警：dmc_hcmp_fifo_get_mode({mc_Leadshine.CardID},{hcmpId},{usFifoMde})={res}");
                                            break;
                                        }
                                        if (usFifoMde == 1)
                                        {
                                            res = LTDMC.dmc_hcmp_fifo_get_state(mc_Leadshine.CardID, hcmpId, ref remainedFifoPoints);
                                            if (res != 0)
                                            {
                                                AddListInfo(ListInfo, $"报警：dmc_hcmp_fifo_get_state({mc_Leadshine.CardID},{hcmpId},{remainedFifoPoints})={res}");
                                                Commands.ExcuteDisConnect = true;
                                                break;
                                            }
                                        }
                                    }
                                    mc_Leadshine.remainedPoints = remainedPoints + (int)remainedFifoPoints;
                                    mc_Leadshine.currentPoint = currentPoint;
                                    mc_Leadshine.runnedPoints = runnedPoints;
                                }
                                else//选择二维高速位置比较
                                {
                                    int remainedPoints = 0;             //返回可添加比较点数
                                    int runnedPoints = 0;               //返回已比较点数
                                    double x_currentPoint = 0;          //返回当前X轴比较点位置，单位：pluse
                                    double y_currentPoint = 0;          //返回当前X轴比较点位置，单位：pluse
                                    ushort currentState = 0;            //比较器状态 1 正在输出 0 输出完成
                                    ushort currentOutbit = 0;           //返回当前输出口
                                    res = LTDMC.dmc_hcmp_2d_get_current_state_unit(mc_Leadshine.CardID, hcmpId, ref remainedPoints, ref x_currentPoint, ref y_currentPoint, ref runnedPoints, ref currentState, ref currentOutbit);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_2d_get_current_state_unit({mc_Leadshine.CardID},{hcmpId},{remainedPoints},{x_currentPoint},{y_currentPoint},{runnedPoints},{currentState},{currentOutbit})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                    ushort usFifoMde_2d = 2;
                                    res = LTDMC.dmc_hcmp_2d_fifo_get_mode(mc_Leadshine.CardID, hcmpId, ref usFifoMde_2d);//获取缓存方式添加比较位置,0不启用，1启用
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_hcmp_2d_fifo_get_mode({mc_Leadshine.CardID},{hcmpId},{usFifoMde_2d})={res}");
                                        break;
                                    }
                                    long remainedPoints_2d = 0;//返回FIFO区间可添加比较点数
                                    if (usFifoMde_2d == 1)
                                    {
                                        res = LTDMC.dmc_hcmp_2d_fifo_get_state(mc_Leadshine.CardID, hcmpId, ref remainedPoints_2d);
                                        if (res != 0)
                                        {
                                            AddListInfo(ListInfo, $"报警：dmc_hcmp_2d_fifo_get_state({mc_Leadshine.CardID},{hcmpId},{remainedPoints_2d})={res}");
                                            Commands.ExcuteDisConnect = true;
                                            break;
                                        }
                                    }
                                    mc_Leadshine.remainedPoints = (int)(remainedPoints + remainedPoints_2d);
                                    mc_Leadshine.currentPoint = x_currentPoint;
                                    mc_Leadshine.runnedPoints = runnedPoints;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                ushort hcmpId = decimal.ToUInt16(numericUpDown10.Value);    //比较器号
                                if (radioButton3.Checked == true)
                                {
                                    int remainedPoints = 0;             //返回可添加比较点数
                                    double currentPoint = 0;            //返回当前比较点位置，单位：pluse
                                    int runnedPoints = 0;               //返回已比较点数
                                    res = LTSMC.smc_hcmp_get_current_state_unit(mc_Leadshine.CardID, hcmpId, ref remainedPoints, ref currentPoint, ref runnedPoints);//读取一维高速比较参数
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：smc_hcmp_get_current_state_unit({mc_Leadshine.CardID},{hcmpId},{remainedPoints},{currentPoint},{runnedPoints})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                    mc_Leadshine.remainedPoints = remainedPoints;
                                    mc_Leadshine.currentPoint = currentPoint;
                                    mc_Leadshine.runnedPoints = runnedPoints;
                                }
                                else
                                {
                                    int remainedPoints = 0;             //返回可添加比较点数
                                    long remainedPoints_2d = 0;         //返回可添加比较点数
                                    int runnedPoints = 0;               //返回已比较点数
                                    double x_currentPoint = 0;          //返回当前X轴比较点位置，单位：pluse
                                    double y_currentPoint = 0;          //返回当前X轴比较点位置，单位：pluse
                                    ushort currentState = 0;            //比较器状态 1 正在输出 0 输出完成
                                    ushort currentOutbit = 0;           //返回当前输出口
                                    res = LTSMC.smc_hcmp_2d_get_current_state_unit(mc_Leadshine.CardID, hcmpId, ref remainedPoints, ref x_currentPoint, ref y_currentPoint, ref runnedPoints, ref currentState, ref currentOutbit);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：smc_hcmp_2d_get_current_state_unit({mc_Leadshine.CardID},{hcmpId},{remainedPoints},{x_currentPoint},{y_currentPoint},{runnedPoints},{currentState},{currentOutbit})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                    res = LTSMC.smc_hcmp_2d_fifo_get_state(mc_Leadshine.CardID, hcmpId, ref remainedPoints_2d);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：smc_hcmp_2d_fifo_get_state({mc_Leadshine.CardID},{hcmpId},{remainedPoints_2d})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                    mc_Leadshine.remainedPoints = (int)(remainedPoints + remainedPoints_2d);
                                    mc_Leadshine.currentPoint = x_currentPoint;
                                    mc_Leadshine.runnedPoints = runnedPoints;
                                }
                            }
                        }
                        #endregion

                        #region 获取：通用IO状态
                        if (WindowsPage == 5)
                        {
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                #region 一条指令获取所有通用输入口状态
                                if (mc_Leadshine.InputNum > 0)
                                {
                                    ushort usInPortNum = 0;//输出口组的数量，32个输出口为一组
                                    usInPortNum = (ushort)Math.Ceiling((double)mc_Leadshine.InputNum / 32);
                                    uint[] uiInStatus = new uint[usInPortNum];//所有输出口的状态
                                    res = LTDMC.dmc_read_inport_array(mc_Leadshine.CardID, usInPortNum, uiInStatus);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"dmc_read_outport_array({mc_Leadshine.CardID},{usInPortNum},{uiInStatus})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                    int i = groupBox5.Controls.Count;
                                    foreach (Control item in groupBox5.Controls)
                                    {
                                        i--;
                                        if (i < mc_Leadshine.InputNum)
                                        {
                                            long BitState = 0;
                                            if (i < 32)
                                            {
                                                BitState = uiInStatus[0] & (1 << i);
                                            }
                                            else if (i < 64)
                                            {
                                                BitState = uiInStatus[1] & (1 << i);
                                            }
                                            item.BackColor = (BitState == 0) ? Color.GreenYellow : Control.DefaultBackColor;
                                        }
                                    }
                                }
                                #endregion
                                //int i = groupBox5.Controls.Count;
                                //uint InportState_0 = 0;
                                //uint InportState_1 = 0;
                                //if (mc_Leadshine.InputNum > 0)
                                //{
                                //    res = LTDMC.dmc_read_inport_ex(mc_Leadshine.CardID, 0, ref InportState_0);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"dmc_read_inport_ex({mc_Leadshine.CardID},{0}, {InportState_0})={res}");
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //if (mc_Leadshine.InputNum > 32)
                                //{
                                //    res = LTDMC.dmc_read_inport_ex(mc_Leadshine.CardID, 1, ref InportState_1);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"dmc_read_inport_ex({mc_Leadshine.CardID},{1}, {InportState_1})={res}");
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //foreach (Control item in groupBox5.Controls)
                                //{
                                //    i--;
                                //    if (i < mc_Leadshine.InputNum)
                                //    {
                                //        long BitState = 0;
                                //        if (i < 32)
                                //        {
                                //            BitState = InportState_0 & (1 << i);
                                //        }
                                //        else if (i < 64)
                                //        {
                                //            BitState = InportState_1 & (1 << i);
                                //        }
                                //        item.BackColor = (BitState == 0) ? Color.GreenYellow : Control.DefaultBackColor;
                                //    }
                                //}
                                //foreach (Control item in groupBox5.Controls)
                                //{
                                //    i--;
                                //    if (i < 32 && i < mc_Leadshine.InputNum)
                                //    {
                                //        long BitState = InportState_0 & (1 << i);
                                //        if (BitState == 0)//输入口低电平，有效了
                                //        {
                                //            item.BackColor = Color.GreenYellow;
                                //        }
                                //        else//输入口高电平，无效了
                                //        {
                                //            item.BackColor = Control.DefaultBackColor;
                                //        }
                                //    }
                                //}

                                #region 一条指令获取所有通用输出口状态
                                if (mc_Leadshine.OutputNum > 0)
                                {
                                    ushort usOutPortNum = 0;//输出口组的数量，32个输出口为一组
                                    usOutPortNum = (ushort)Math.Ceiling((double)mc_Leadshine.OutputNum / 32);
                                    uint[] uiOutStatus = new uint[usOutPortNum];//所有输出口的状态
                                    res = LTDMC.dmc_read_outport_array(mc_Leadshine.CardID, usOutPortNum, uiOutStatus);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"dmc_read_outport_array({mc_Leadshine.CardID},{usOutPortNum},{uiOutStatus})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                    int j = groupBox6.Controls.Count;
                                    foreach (Control item in groupBox6.Controls)
                                    {
                                        j--;
                                        if (j < mc_Leadshine.OutputNum)
                                        {
                                            long BitState = 0;
                                            if (j < 32)
                                            {
                                                BitState = uiOutStatus[0] & (1 << j);
                                            }
                                            else if (j < 64)
                                            {
                                                BitState = uiOutStatus[1] & (1 << j);
                                            }
                                            item.BackColor = (BitState == 0) ? Color.GreenYellow : Control.DefaultBackColor;
                                        }
                                    }
                                }
                                #endregion
                                //int j = groupBox6.Controls.Count;
                                //uint OutportState_0 = 0;
                                //uint OutportState_1 = 0;
                                //if (mc_Leadshine.OutputNum > 0)
                                //{
                                //    res = LTDMC.dmc_read_outport_ex(mc_Leadshine.CardID, 0, ref OutportState_0);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"dmc_read_outport_ex({mc_Leadshine.CardID},{0}, {OutportState_0})={res}");
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //if (mc_Leadshine.OutputNum > 32)
                                //{
                                //    res = LTDMC.dmc_read_outport_ex(mc_Leadshine.CardID, 1, ref OutportState_1);
                                //    if (res != 0)
                                //    {
                                //        AddListInfo(ListInfo, $"dmc_read_outport_ex({mc_Leadshine.CardID},{1}, {OutportState_1})={res}");
                                //        Commands.ExcuteDisConnect = true;
                                //        break;
                                //    }
                                //}
                                //foreach (Control item in groupBox6.Controls)
                                //{
                                //    j--;
                                //    if (j < mc_Leadshine.OutputNum)
                                //    {
                                //        long BitState = 0;
                                //        if (j < 32)
                                //        {
                                //            BitState = OutportState_0 & (1 << j);
                                //        }
                                //        else if (j < 64)
                                //        {
                                //            BitState = OutportState_1 & (1 << j);
                                //        }
                                //        item.BackColor = (BitState == 0) ? Color.GreenYellow : Control.DefaultBackColor;
                                //    }
                                //}
                                //foreach (Control item in groupBox6.Controls)
                                //{
                                //    j--;
                                //    if (j < 32 && j < mc_Leadshine.OutputNum)
                                //    {
                                //        long BitState = OutportState_0 & (1 << j);
                                //        if (BitState == 0)//输出口低电平，打开了
                                //        {
                                //            item.BackColor = Color.GreenYellow;
                                //        }
                                //        else//输出口高电平，关闭了
                                //        {
                                //            item.BackColor = Control.DefaultBackColor;
                                //        }
                                //    }
                                //}

                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                int i = groupBox5.Controls.Count;
                                uint InportState_0 = 0;
                                uint InportState_1 = 0;
                                if (mc_Leadshine.InputNum > 0)
                                {
                                    res = LTSMC.smc_read_inport_ex(mc_Leadshine.CardID, 0, ref InportState_0);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"smc_read_inport_ex({mc_Leadshine.CardID},{0}, {InportState_0})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                if (mc_Leadshine.InputNum > 32)
                                {
                                    res = LTSMC.smc_read_inport_ex(mc_Leadshine.CardID, 1, ref InportState_1);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"smc_read_inport_ex({mc_Leadshine.CardID},{1}, {InportState_1})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                foreach (Control item in groupBox5.Controls)
                                {
                                    i--;
                                    if (i < mc_Leadshine.InputNum)
                                    {
                                        long BitState = 0;
                                        if (i < 32)
                                        {
                                            BitState = InportState_0 & (1 << i);
                                        }
                                        else if (i < 64)
                                        {
                                            BitState = InportState_1 & (1 << i);
                                        }
                                        item.BackColor = (BitState == 0) ? Color.GreenYellow : Control.DefaultBackColor;
                                    }
                                }
                                int j = groupBox6.Controls.Count;
                                uint OutportState_0 = 0;
                                uint OutportState_1 = 0;
                                if (mc_Leadshine.OutputNum > 0)
                                {
                                    res = LTSMC.smc_read_outport_ex(mc_Leadshine.CardID, 0, ref OutportState_0);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"smc_read_outport_ex({mc_Leadshine.CardID},{0}, {OutportState_0})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                if (mc_Leadshine.OutputNum > 32)
                                {
                                    res = LTSMC.smc_read_outport_ex(mc_Leadshine.CardID, 1, ref OutportState_1);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"smc_read_outport_ex({mc_Leadshine.CardID},{1}, {OutportState_1})={res}");
                                        Commands.ExcuteDisConnect = true;
                                        break;
                                    }
                                }
                                foreach (Control item in groupBox6.Controls)
                                {
                                    j--;
                                    if (j < mc_Leadshine.OutputNum)
                                    {
                                        long BitState = 0;
                                        if (j < 32)
                                        {
                                            BitState = OutportState_0 & (1 << j);
                                        }
                                        else if (j < 64)
                                        {
                                            BitState = OutportState_1 & (1 << j);
                                        }
                                        item.BackColor = (BitState == 0) ? Color.GreenYellow : Control.DefaultBackColor;
                                    }
                                }
                            }
                        }
                        #endregion

                        #region 获取：指令缓存的运行状态
                        if (WindowsPage == 6)
                        {
                            ushort usGroupID = decimal.ToUInt16(numericUpDown32.Value);//指令缓存区组号
                            uint uiGroupRemainSpace = 0;//缓存区剩余空间
                            ushort usGroupStste = 0;//当前指令的运行状态
                            ushort usGroupEnable = 0;//Group打开，0表示未使能；1表示使能
                            uint uiGroupStopReason = 0;//停止原因
                            ushort usGroupTrigPhase = 0;//当前指令等待触发的过程
                            uint uiGroupMark = 0;//当前执行段的段号 
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.dmc_m_remain_space(mc_Leadshine.CardID, usGroupID, ref uiGroupRemainSpace);//读取缓存区剩余空间
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_m_remain_space({mc_Leadshine.CardID},{usGroupID},{uiGroupRemainSpace})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                mc_Leadshine.uiGroupRemainSpace = uiGroupRemainSpace;
                                res = LTDMC.dmc_m_get_run_state(mc_Leadshine.CardID, 0, ref usGroupStste, ref usGroupEnable, ref uiGroupStopReason, ref usGroupTrigPhase, ref uiGroupMark);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_m_get_run_state({mc_Leadshine.CardID},{usGroupID},{usGroupStste},{usGroupEnable},{uiGroupStopReason},{usGroupTrigPhase},{uiGroupMark})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                //res = LTSMC.smc_m_remain_space(mc_Leadshine.CardID, usGroupID, ref uiGroupRemainSpace);//读取缓存区剩余空间
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"报警：smc_m_remain_space({mc_Leadshine.CardID},{usGroupID},{uiGroupRemainSpace})={res}");
                                //    Commands.ExcuteDisConnect = true;
                                //    break;
                                //}
                                //mc_Leadshine.uiGroupRemainSpace = uiGroupRemainSpace;
                                //res = LTSMC.smc_m_get_run_state(mc_Leadshine.CardID, 0, ref usGroupStste, ref usGroupEnable, ref uiGroupStopReason, ref usGroupTrigPhase, ref uiGroupMark);
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"报警：smc_m_get_run_state({mc_Leadshine.CardID},{usGroupID},{usGroupStste},{usGroupEnable},{uiGroupStopReason},{usGroupTrigPhase},{uiGroupMark})={res}");
                                //    Commands.ExcuteDisConnect = true;
                                //    break;
                                //}
                            }
                            mc_Leadshine.usGroupStste = usGroupStste;
                            mc_Leadshine.usGroupEnable = usGroupEnable;
                            mc_Leadshine.uiGroupStopReason = uiGroupStopReason;
                            mc_Leadshine.usGroupTrigPhase = usGroupTrigPhase;
                            mc_Leadshine.uiGroupMark = uiGroupMark;
                        }
                        #endregion

                        #region 获取：插补系的状态
                        if (WindowsPage == 7)
                        {
                            ushort usCrd = decimal.ToUInt16(numericUpDown18.Value);//声明一个插补系号变量
                            double tempVectorCurrentSpeed = 0;
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                mc_Leadshine.runState = LTDMC.dmc_conti_get_run_state(mc_Leadshine.CardID, usCrd);//读取指定坐标系的插补运动状态运动状态，0：运动中，1：暂停中，2：正常停止，3：未启动，4：空闲
                                mc_Leadshine.remainSpace = LTDMC.dmc_conti_remain_space(mc_Leadshine.CardID, usCrd);//查询插补缓冲区剩余插补空间
                                mc_Leadshine.currentMark = LTDMC.dmc_conti_read_current_mark(mc_Leadshine.CardID, usCrd);//读连续插补缓冲区当前插补段号
                                res = LTDMC.dmc_read_vector_speed_unit(mc_Leadshine.CardID, usCrd, ref tempVectorCurrentSpeed);//获取插补系的合速度值
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_read_vector_speed_unit({mc_Leadshine.CardID},{usCrd},{tempVectorCurrentSpeed})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                mc_Leadshine.vectorCurrentSpeed = tempVectorCurrentSpeed;
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                mc_Leadshine.runState = LTDMC.dmc_conti_get_run_state(mc_Leadshine.CardID, usCrd);//读取指定坐标系的插补运动状态运动状态，0：运动中，1：暂停中，2：正常停止，3：未启动，4：空闲
                                mc_Leadshine.remainSpace = LTDMC.dmc_conti_remain_space(mc_Leadshine.CardID, usCrd);//查询插补缓冲区剩余插补空间
                                mc_Leadshine.currentMark = LTDMC.dmc_conti_read_current_mark(mc_Leadshine.CardID, usCrd);//读连续插补缓冲区当前插补段号
                                //res = LTSMC.smc_read_vector_speed_unit(mc_Leadshine.CardID, usCrd, ref tempVectorCurrentSpeed);//获取插补系的合速度值
                                //if (res != 0)
                                //{
                                //    AddListInfo(ListInfo, $"报警：smc_read_vector_speed_unit({mc_Leadshine.CardID},{usCrd},{tempVectorCurrentSpeed})={res}");
                                //    Commands.ExcuteDisConnect = true;
                                //    break;
                                //}
                                mc_Leadshine.vectorCurrentSpeed = tempVectorCurrentSpeed;
                            }
                        }
                        #endregion

                        #region 获取：Trace采集的各种状态

                        #region 读取Trace采集标志：启动采集标志，触发标志，溢出标志
                        short shStartFlag = 0;
                        short shTriggeredFlag = 0;
                        short shLostflag = 0;
                        if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                        {
                            res = LTDMC.dmc_trace_get_flag(mc_Leadshine.CardID, ref shStartFlag, ref shTriggeredFlag, ref shLostflag);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_trace_get_flag({mc_Leadshine.CardID},{shStartFlag},{shTriggeredFlag},{shLostflag})={res}");
                                Commands.ExcuteDisConnect = true;
                                break;
                            }
                        }
                        else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                        {
                            ;
                        }
                        mc_Leadshine.shStartFlag = shStartFlag;
                        mc_Leadshine.shTriggeredFlag = shTriggeredFlag;
                        mc_Leadshine.shLostflag = shLostflag;
                        #endregion

                        #region 读取Trace采集状态，最大存储5M
                        int iValidNum = 0;                  //已采集但未被读取的数据个数
                        int iFreeNum = 0;                   //剩余可用于保存采集数据的个数
                        int iObjectTotalBytes = 0;          //采集对象总字节数
                        int iValidObjectTotalNum = 0;       //采集对象总个数
                        if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                        {
                            res = LTDMC.dmc_trace_get_state(mc_Leadshine.CardID, ref iValidNum, ref iFreeNum, ref iObjectTotalBytes, ref iValidObjectTotalNum);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_trace_get_state({mc_Leadshine.CardID},{iValidNum},{iFreeNum},{iObjectTotalBytes},{iValidObjectTotalNum})={res}");
                                Commands.ExcuteDisConnect = true;
                                break;
                            }
                        }
                        else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                        {
                            ;
                        }
                        mc_Leadshine.iValidNum = iValidNum;
                        mc_Leadshine.iFreeNum = iFreeNum;
                        mc_Leadshine.iObjectTotalBytes = iObjectTotalBytes;
                        mc_Leadshine.iValidObjectTotalNum = iValidObjectTotalNum;
                        #endregion

                        #region 读取Trace采集数据，
                        if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                        {
                            if (mc_Leadshine.IsTracing && mc_Leadshine.shStartFlag != 0 && iValidNum > 0)
                            {
                                int bufsize = mc_Leadshine.iObjectTotalBytes;//数据缓冲区字节数
                                byte[] data = new byte[bufsize]; //数据缓冲区，
                                int iByteSize = 0; //读取数据的字节数
                                res = LTDMC.dmc_trace_get_data(mc_Leadshine.CardID, bufsize, data, ref iByteSize);
                                if (res != 0 || iByteSize != bufsize)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_trace_get_data({mc_Leadshine.CardID},{bufsize},{data},{iByteSize})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                string strLog = string.Empty;
                                for (int i = 0; i < mc_Leadshine.iValidObjectTotalNum + 1; i++)
                                {
                                    switch (i)
                                    {
                                        case 0://采集次数
                                            strLog += (BitConverter.ToUInt32(data, 0) * (float)mc_Leadshine.EthercatTime / 1000).ToString() + ",";
                                            break;
                                        case 1:
                                            strLog += data[4].ToString() + ",";
                                            break;
                                        case 2:
                                            strLog += data[5].ToString() + ","; ;
                                            break;
                                        case 3:
                                            strLog += BitConverter.ToUInt16(data, 6).ToString() + ",";
                                            break;
                                        case 4:
                                            strLog += BitConverter.ToUInt16(data, 8).ToString() + ",";
                                            break;
                                        case 5:
                                            strLog += BitConverter.ToInt32(data, 10).ToString() + ",";
                                            break;
                                        case 6:
                                            strLog += BitConverter.ToInt32(data, 14).ToString() + ",";
                                            break;
                                        case 7:
                                            strLog += BitConverter.ToInt32(data, 18).ToString() + "\n";
                                            break;
                                        default:
                                            break;
                                    }
                                }
                                TraceFileInfo.Add(strLog);
                            }
                        }
                        else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                        {
                            ;
                        }
                        #endregion

                        #endregion

                        #region 获取：各个从站的状态，是否在线，是否OP
                        if (WindowsPage == 10 && mc_Leadshine.SlaveNum > 0)
                        {
                            byte[] usSlavePresent = new byte[mc_Leadshine.SlaveNum];
                            ushort[] usSlaveState = new ushort[mc_Leadshine.SlaveNum];
                            if (enumControllerType == enumMotionControllerType.DMC_E || enumControllerType == enumMotionControllerType.EMC_E)//连接DMC/EMC控制卡
                            {
                                res = LTDMC.nmc_get_slave_present(mc_Leadshine.CardID, 0xFFFF, usSlavePresent);//获取所有从站在线状态，0不在线，1在线，2其他
                                res = LTDMC.nmc_get_slave_state(mc_Leadshine.CardID, 0xFFFF, usSlaveState);//获取所有从站，8为OP
                                int i = groupBox13.Controls.Count;
                                foreach (Control item in groupBox13.Controls)
                                {
                                    i--;
                                    if (i < 64 && i < mc_Leadshine.SlaveNum)
                                    {
                                        if (usSlavePresent[i] == 1)
                                        {
                                            item.BackColor = (usSlaveState[i] == 8) ? Color.GreenYellow : Color.Yellow;//绿色OP，黄色非OP
                                        }
                                        else
                                        {
                                            item.BackColor = Color.OrangeRed;//从站不在线
                                        }
                                    }
                                }
                                //int i = groupBox13.Controls.Count;
                                //foreach (Control item in groupBox13.Controls)
                                //{
                                //    i--;
                                //    if (i < 64 && i < mc_Leadshine.SlaveNum)
                                //    {
                                //        ushort usSlaveId = (ushort)(i + 1001);
                                //        ushort usSlaveState = 0;
                                //        //res = LTDMC.nmc_get_slave_state(mc_Leadshine.CardID, usSlaveId, ref usSlaveState);//8为OP
                                //        //if (res == 0)
                                //        //{
                                //        //    if (usSlaveState == 8)
                                //        //    {
                                //        //        item.BackColor = Color.GreenYellow;//从站OP
                                //        //    }
                                //        //    else
                                //        //    {
                                //        //        item.BackColor = Color.Yellow;//从站非OP
                                //        //    }
                                //        //}
                                //        //else
                                //        //{
                                //        //    item.BackColor = Color.OrangeRed;//从站不在线
                                //        //}

                                //        res = LTDMC.nmc_get_slave_present(mc_Leadshine.CardID, usSlaveId, ref usSlaveState);//获取从站在线状态，0不在线，1在线，2其他
                                //        if (res == 0 && usSlaveState == 1)
                                //        {
                                //            res = LTDMC.nmc_get_slave_state(mc_Leadshine.CardID, usSlaveId, ref usSlaveState);//8为OP
                                //            if (res == 0 && usSlaveState == 8)
                                //            {
                                //                item.BackColor = Color.GreenYellow;//从站OP
                                //            }
                                //            else
                                //            {
                                //                item.BackColor = Color.Yellow;//从站非OP
                                //            }
                                //        }
                                //        else
                                //        {
                                //            item.BackColor = Color.OrangeRed;//从站不在线
                                //        }
                                //    }
                                //    else
                                //    {
                                //        ;//item.Visible = false;
                                //    }
                                //}
                            }
                            else if (enumControllerType == enumMotionControllerType.BAC_E)//选择BAC控制卡
                            {
                                //res = LTSMC.nmcs_get_slave_present(mc_Leadshine.CardID, 0xFFFF, usSlavePresent);//获取所有从站在线状态，0不在线，1在线，2其他
                                res = LTSMC.nmcs_get_slave_state(mc_Leadshine.CardID, 0xFFFF, ref usSlaveState[0]);//获取所有从站，8为OP
                                int i = groupBox13.Controls.Count;
                                foreach (Control item in groupBox13.Controls)
                                {
                                    i--;
                                    if (i < 64 && i < mc_Leadshine.SlaveNum)
                                    {
                                        if (usSlavePresent[i] == 1)
                                        {
                                            item.BackColor = (usSlaveState[i] == 8) ? Color.GreenYellow : Color.Yellow;//绿色OP，黄色非OP
                                        }
                                        else
                                        {
                                            item.BackColor = Color.OrangeRed;//从站不在线
                                        }
                                    }
                                }
                            }
                        }
                        #endregion
                        break;
                    }
                    #endregion

                    stopWatch.Stop();
                }
            });
            MainThread.IsBackground = true;
            MainThread.Start();

            FileThread = new Thread(() =>
            {
                try
                {
                    while (true)
                    {
                        if (TraceFileInfo.Count > 1000)
                        {
                            FileStream fs = new FileStream(textBox60.Text, FileMode.Append, FileAccess.Write);
                            StreamWriter sw = new StreamWriter(fs);
                            for (int i = 0; i < 1000; i++)
                            {
                                sw.Write(TraceFileInfo[0]);
                                TraceFileInfo.RemoveAt(0);
                            }
                            sw.Close();
                            fs.Close();
                        }
                        else if (TraceFileInfo.Count > 100)
                        {
                            FileStream fs = new FileStream(textBox60.Text, FileMode.Append, FileAccess.Write);
                            StreamWriter sw = new StreamWriter(fs);
                            for (int i = 0; i < 100; i++)
                            {
                                sw.Write(TraceFileInfo[0]);
                                TraceFileInfo.RemoveAt(0);
                            }
                            sw.Close();
                            fs.Close();
                        }
                        else if (TraceFileInfo.Count > 0)
                        {
                            FileStream fs = new FileStream(textBox60.Text, FileMode.Append, FileAccess.Write);
                            StreamWriter sw = new StreamWriter(fs);
                            sw.Write(TraceFileInfo[0]);
                            TraceFileInfo.RemoveAt(0);
                            sw.Close();
                            fs.Close();
                        }
                    }
                }
                catch (Exception)
                {

                    throw;
                }

            });
            FileThread.IsBackground = true;
            FileThread.Start();
        }

        /// <summary>
        /// 关闭软件窗体
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            DialogResult res = MessageBox.Show("是否关闭软件", "询问", MessageBoxButtons.OKCancel, MessageBoxIcon.Question);
            if (res == DialogResult.Cancel)
            {
                e.Cancel = true;//不关闭窗体
            }
            else
            {
                //LTDMC.dmc_board_close();
                btn_DisConnect.PerformClick();
                while (mc_Leadshine.IsConnectDone)
                {
                    ;
                }
                Thread.Sleep(500);
                MainThread.Abort();
                FileThread.Abort();
            }
        }

        /// <summary>
        /// 时钟刷新
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void timer1_Tick(object sender, EventArgs e)
        {
            this.btn_Connect.Enabled = radioButton1.Enabled = radioButton2.Enabled = radioButton11.Enabled = !mc_Leadshine.IsConnectDone;
            this.btn_EMG.BackColor = mc_Leadshine.IsEMG ? Color.OrangeRed : Control.DefaultBackColor;
            textBox8.Visible = label8.Visible = (radioButton1.Checked | radioButton11.Checked) ? false : true;
            label76.Visible = label77.Visible = numericUpDown12.Visible = radioButton4.Checked;
            if (mc_Leadshine.IsConnectDone)
            {
                textBox2.Text = mc_Leadshine.AxisNum.ToString();
                textBox3.Text = mc_Leadshine.InputNum.ToString();
                textBox4.Text = mc_Leadshine.OutputNum.ToString();
                textBox5.Text = mc_Leadshine.AdNum.ToString();
                textBox6.Text = mc_Leadshine.DaNum.ToString();
                textBox7.Text = (mc_Leadshine.EtherCATState == 0) ? ($"正常") : ($"报警:0x" + Convert.ToString(mc_Leadshine.EtherCATState, 16));
                textBox7.BackColor = (mc_Leadshine.EtherCATState == 0) ? Control.DefaultBackColor : Color.OrangeRed;
                textBox8.Text = (mc_Leadshine.LocalBusState == 0) ? ($"正常") : ($"报警:0x" + Convert.ToString(mc_Leadshine.LocalBusState, 16));
                textBox8.BackColor = (mc_Leadshine.LocalBusState == 0) ? Control.DefaultBackColor : Color.OrangeRed;
                toolStripStatusLabel2.Text = Convert.ToString(mc_Leadshine.FirmID, 16).ToUpper();
                toolStripStatusLabel4.Text = Convert.ToString(mc_Leadshine.SubFirmID).ToUpper();
                toolStripStatusLabel6.Text = Convert.ToString(mc_Leadshine.CardVersion, 16).ToUpper();
                toolStripStatusLabel8.Text = mc_Leadshine.ReleaseVersion;
                #region 功能页面选择
                if (this.tabControl1.SelectedTab == tabPage1 && mc_Leadshine.AxisNum > 0)
                {
                    WindowsPage = 1;
                    numericUpDown17.Maximum = numericUpDown41.Maximum = numericUpDown42.Maximum = mc_Leadshine.AxisNum - 1;
                    ushort usAxisID = decimal.ToUInt16(numericUpDown17.Value);
                    label93.BackColor = mc_Leadshine.AxisAlarm[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//伺服报警
                    label89.BackColor = mc_Leadshine.AxisELP[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//正限位ELP
                    label87.BackColor = mc_Leadshine.AxisELN[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//正限位ELN
                    label82.BackColor = mc_Leadshine.AxisEMG[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//急停EMG
                    label84.BackColor = mc_Leadshine.AxisORG[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//原点ORG
                    label81.BackColor = mc_Leadshine.AxisPowerOn[usAxisID] ? Color.GreenYellow : Control.DefaultBackColor;//轴使能
                    textBox28.Text = mc_Leadshine.AxisCommandPos[usAxisID].ToString();
                    textBox29.Text = mc_Leadshine.AxisActualPos[usAxisID].ToString();
                    textBox27.Text = mc_Leadshine.AxisCurrentSpeed[usAxisID].ToString();
                    textBox26.Text = (mc_Leadshine.AxisBusy[usAxisID] == false) ? "停止" : "运行";
                    textBox26.BackColor = (mc_Leadshine.AxisBusy[usAxisID] == false) ? Control.DefaultBackColor : Color.GreenYellow;
                    textBox25.Text = (mc_Leadshine.AxisStopReason[usAxisID] == 0) ? "正常停止" : $"非正常停止:{mc_Leadshine.AxisStopReason[usAxisID]}";
                    textBox62.Text = mc_Leadshine.AxisCurrentTorque[usAxisID].ToString();
                    textBox63.Text = (mc_Leadshine.AxisHomeResult[usAxisID] == 1) ? "完成" : "未完成";
                    //textBox63.Text = (mc_Leadshine.AxisHomeResult[usAxisID] == 1 && mc_Leadshine.AxisStopReason[usAxisID] == 0) ? "完成" : "未完成";

                    for (int i = 0; i < 2; i++)
                    {
                        string strRunMode = string.Empty;
                        if (i == 0)
                        {
                            usAxisID = decimal.ToUInt16(numericUpDown41.Value);
                        }
                        else
                        {
                            usAxisID = decimal.ToUInt16(numericUpDown42.Value);
                        }
                        switch (mc_Leadshine.AxisRunMode[usAxisID])
                        {
                            case 0:
                                strRunMode = "0,空闲模式";
                                break;
                            case 1:
                                strRunMode = "1,定长模式";
                                break;
                            case 2:
                                strRunMode = "2,定速模式";
                                break;
                            case 3:
                                strRunMode = "3,回零模式";
                                break;
                            case 4:
                                strRunMode = "4,手轮模式";
                                break;
                            case 5:
                                strRunMode = "5,PT模式";
                                break;
                            case 6:
                                strRunMode = "6,PVT模式";
                                break;
                            case 7:
                                strRunMode = "7,电子齿轮模式";
                                break;
                            case 8:
                                strRunMode = "8,电子凸轮模式";
                                break;
                            case 9:
                                strRunMode = "9,单段直线模式";
                                break;
                            case 10:
                                strRunMode = "10,连续插补模式";
                                break;
                            case 11:
                                strRunMode = "11,停止模式";
                                break;
                            case 12:
                                strRunMode = "12,限位模式";
                                break;
                            case 13:
                                strRunMode = "13,驱动器PP模式";
                                break;
                            case 14:
                                strRunMode = "14,龙门模式";
                                break;
                            case 17:
                                strRunMode = "17,退出龙门模式";
                                break;
                            case 18:
                                strRunMode = "18,门型运动";
                                break;
                            case 19:
                                strRunMode = "19,插补速度跟随模式";
                                break;
                            case 20:
                                strRunMode = "20,正弦振荡曲线模式";
                                break;
                            case 21:
                                strRunMode = "21,插补速度跟随模式";
                                break;
                            case 22:
                                strRunMode = "22,转矩控制模式";
                                break;
                            case 23:
                                strRunMode = "23,PDO缓存运动模式";
                                break;
                            case 24:
                                strRunMode = "24,驱动器PV模式";
                                break;
                            case 25:
                                strRunMode = "25,插补暂停定长运动模式";
                                break;
                            default:
                                break;
                        }
                        if (i == 0)
                        {
                            textBox43.Text = strRunMode;
                        }
                        else
                        {
                            textBox44.Text = strRunMode;
                        }
                    }
                }
                else if (this.tabControl1.SelectedTab == tabPage2)
                {
                    WindowsPage = 2;
                }
                else if (this.tabControl1.SelectedTab == tabPage3)
                {
                    WindowsPage = 3;
                    textBox21.Text = mc_Leadshine.ADValue[decimal.ToUInt16(numericUpDown2.Value)].ToString();
                    textBox20.Text = mc_Leadshine.ADValue[decimal.ToUInt16(numericUpDown3.Value)].ToString();
                    textBox17.Text = mc_Leadshine.ADValue[decimal.ToUInt16(numericUpDown4.Value)].ToString();
                    textBox16.Text = mc_Leadshine.ADValue[decimal.ToUInt16(numericUpDown5.Value)].ToString();
                    textBox47.Text = mc_Leadshine.DAValue[decimal.ToUInt16(numericUpDown6.Value)].ToString();
                    textBox48.Text = mc_Leadshine.DAValue[decimal.ToUInt16(numericUpDown7.Value)].ToString();
                    textBox49.Text = mc_Leadshine.DAValue[decimal.ToUInt16(numericUpDown8.Value)].ToString();
                    textBox50.Text = mc_Leadshine.DAValue[decimal.ToUInt16(numericUpDown9.Value)].ToString();
                }
                else if (this.tabControl1.SelectedTab == tabPage4)
                {
                    WindowsPage = 4;
                    textBox18.Text = mc_Leadshine.ExtraEncoder[decimal.ToUInt16(numericUpDown19.Value)].ToString();
                    textBox19.Text = mc_Leadshine.ExtraEncoder[decimal.ToUInt16(numericUpDown20.Value)].ToString();
                    textBox24.Text = mc_Leadshine.remainedPoints.ToString();
                    textBox23.Text = radioButton3.Checked == true ? mc_Leadshine.currentPoint.ToString() : $"{mc_Leadshine.currentPoint},{mc_Leadshine.currentPoint}";
                    textBox22.Text = mc_Leadshine.runnedPoints.ToString();
                    //label76.Visible = label77.Visible = numericUpDown12.Visible = radioButton4.Checked;
                }
                else if (this.tabControl1.SelectedTab == tabPage5)
                {
                    WindowsPage = 5;
                    int i = groupBox5.Controls.Count;
                    foreach (Control item in groupBox5.Controls)
                    {
                        i--;
                        if (i < 64 && i < mc_Leadshine.InputNum)
                        {
                            ;
                        }
                        else
                        {
                            item.Visible = false;
                        }
                    }
                    int j = groupBox6.Controls.Count;
                    foreach (Control item in groupBox6.Controls)
                    {
                        j--;
                        if (j < 64 && j < mc_Leadshine.OutputNum)
                        {
                            ;
                        }
                        else
                        {
                            item.Visible = false;
                        }
                    }
                }
                else if (this.tabControl1.SelectedTab == tabPage6)
                {
                    WindowsPage = 6;
                    textBox37.Text = mc_Leadshine.AxisCommandPos[decimal.ToUInt16(numericUpDown35.Value)].ToString();
                    textBox36.Text = mc_Leadshine.AxisCommandPos[decimal.ToUInt16(numericUpDown34.Value)].ToString();
                    textBox35.Text = mc_Leadshine.AxisCommandPos[decimal.ToUInt16(numericUpDown33.Value)].ToString();
                    textBox34.Text = mc_Leadshine.uiGroupRemainSpace.ToString();
                    switch (mc_Leadshine.usGroupStste)
                    {
                        case 0:
                            textBox42.Text = "空闲状态";
                            break;
                        case 4:
                            textBox42.Text = "缓存指令执行中，处理延时指令";
                            break;
                        case 10:
                            textBox42.Text = "指令等待阶段，且缓存区中的指令运动全部结束";
                            break;
                        case 11:
                            textBox42.Text = "等待事件触发过程中";
                            break;
                        case 12:
                            textBox42.Text = "减速停止过程中";
                            break;
                        case 13:
                            textBox42.Text = "指令等待阶段，且缓存区中的指令还在运动中";
                            break;
                        case 99:
                            textBox42.Text = "暂停状态";
                            break;
                        case 100:
                            textBox42.Text = "因错误原因停止";
                            break;
                        default:
                            break;
                    }
                    textBox41.Text = mc_Leadshine.usGroupEnable == 0 ? "未使能" : "已使能";
                    switch (mc_Leadshine.uiGroupStopReason)
                    {
                        case 0:
                            textBox40.Text = "正常执行完成";
                            break;
                        case 1:
                            textBox40.Text = "外部指令减速停止";
                            break;
                        case 2:
                            textBox40.Text = "外部指令急停";
                            break;
                        case 3:
                            textBox40.Text = "触发异常停止";
                            break;
                        case 10:
                            textBox40.Text = "指令减速停止";
                            break;
                        case 11:
                            textBox40.Text = "指令立即停止";
                            break;
                        default:
                            break;
                    }
                    switch (mc_Leadshine.usGroupTrigPhase)
                    {
                        case 0:
                            textBox39.Text = "等待指定轴checkdone结束";
                            break;
                        case 1:
                            textBox39.Text = "等待指令位置到达指定位置";
                            break;
                        case 2:
                            textBox39.Text = "等待编码器位置到达指定位置";
                            break;
                        case 3:
                            textBox39.Text = "等待输入IO符合设定条件";
                            break;
                        case 4:
                            textBox39.Text = "等待模拟量输入符合设定条件";
                            break;
                        default:
                            break;
                    }
                    textBox38.Text = mc_Leadshine.uiGroupMark.ToString();
                }
                else if (this.tabControl1.SelectedTab == tabPage7)
                {
                    WindowsPage = 7;
                    switch (mc_Leadshine.runState)
                    {
                        case 0:
                            textBox30.Text = string.Format("{0},运动中", mc_Leadshine.runState);
                            break;
                        case 1:
                            textBox30.Text = string.Format("{0},暂停中", mc_Leadshine.runState);
                            break;
                        case 2:
                            textBox30.Text = string.Format("{0},正常停止", mc_Leadshine.runState);
                            break;
                        case 3:
                            textBox30.Text = string.Format("{0},未启动", mc_Leadshine.runState);
                            break;
                        case 4:
                            textBox30.Text = string.Format("{0},空闲", mc_Leadshine.runState);
                            break;
                        default:
                            textBox30.Text = string.Format("{0},错误码", mc_Leadshine.runState);
                            break;
                    }
                    textBox31.Text = mc_Leadshine.remainSpace.ToString();
                    textBox32.Text = mc_Leadshine.currentMark.ToString();
                    textBox33.Text = mc_Leadshine.vectorCurrentSpeed.ToString();
                }
                else if (this.tabControl1.SelectedTab == tabPage8)
                {
                    WindowsPage = 8;
                }
                else if (this.tabControl1.SelectedTab == tabPage9)
                {
                    WindowsPage = 9;
                }
                else if (this.tabControl1.SelectedTab == tabPage10)
                {
                    WindowsPage = 10;
                    int i = groupBox13.Controls.Count;
                    foreach (Control item in groupBox13.Controls)
                    {
                        i--;
                        if (i < 64 && i < mc_Leadshine.SlaveNum)
                        {
                            ;
                        }
                        else
                        {
                            item.Visible = false;
                        }
                    }
                }
                else if (this.tabControl1.SelectedTab == tabPage11)
                {
                    WindowsPage = 11;
                }
                else if (this.tabControl1.SelectedTab == tabPage12)
                {
                    WindowsPage = 12;
                    //textBox52.Text = mc_Leadshine.shStartFlag.ToString();
                    textBox52.Text = mc_Leadshine.shStartFlag == 0 ? "0:未启动" : "1:已启动";
                    textBox52.BackColor = mc_Leadshine.shStartFlag != 0 ? Color.GreenYellow : Control.DefaultBackColor;
                    textBox53.Text = mc_Leadshine.shTriggeredFlag.ToString();
                    textBox54.Text = mc_Leadshine.shLostflag.ToString();
                    textBox55.Text = mc_Leadshine.iValidNum.ToString();
                    textBox56.Text = mc_Leadshine.iFreeNum.ToString();
                    textBox57.Text = mc_Leadshine.iObjectTotalBytes.ToString();
                    textBox58.Text = mc_Leadshine.iValidObjectTotalNum.ToString();
                    textBox61.Text = mc_Leadshine.EthercatTime.ToString();
                }
                #endregion
            }
            #region 输出日志信息
            if (ListInfo.Count > 0)
            {
                richTextBox1.AppendText(ListInfo[0]);
                ListInfo.RemoveAt(0);
                string LastRichText = richTextBox1.Lines[richTextBox1.Lines.Length - 2];//获取最后一行字符串
                if (LastRichText.Contains("报警："))
                {
                    int a = richTextBox1.GetFirstCharIndexFromLine(richTextBox1.Lines.Length - 2);//获取当前行的第一个字符在整个txt的的索引号
                    int LastRichTextLength = LastRichText.Length;//获取最后一行字符串的长度
                    richTextBox1.Select(a, LastRichTextLength);//选中最后一行
                    richTextBox1.SelectionColor = Color.Red;//给选中的行改变字体颜色
                }
                richTextBox1.ScrollToCaret();
            }
            #endregion
        }

        /// <summary>
        /// 添加日志信息
        /// </summary>
        /// <param name="var1"></param>
        /// <param name="info"></param>
        static void AddListInfo(List<string> var1, string info)
        {
            string strInfo = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff") + "\t" + ":" + info + "\n";
            var1.Add(strInfo);
        }

        /// <summary>
        /// 触发控制卡连接命令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_Connect_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteConnect)
            {
                Commands.ExcuteConnect = true;
            }
        }

        /// <summary>
        /// 触发控制卡断开连接命令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_DisConnect_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteDisConnect)
            {
                Commands.ExcuteDisConnect = true;
            }
        }

        /// <summary>
        /// 触发控制卡热复位命令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_SoftReset_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteSoftReset && !mc_Leadshine.IsSoftReset && mc_Leadshine.IsConnectDone)
            {
                DialogResult res = MessageBox.Show("是否启动热复位控制卡", "询问", MessageBoxButtons.OKCancel, MessageBoxIcon.Question);
                if (res == DialogResult.OK)
                {
                    Commands.ExcuteSoftReset = true;
                    form2 = new Form2();
                    form2.ShowDialog();
                }
            }
            //if (!mc_Leadshine.IsSoftReset && mc_Leadshine.IsConnectDone)
            //{
            //    DialogResult res = MessageBox.Show("是否启动热复位控制卡", "询问", MessageBoxButtons.OKCancel, MessageBoxIcon.Question);
            //    if (res == DialogResult.OK)
            //    {
            //        if (!Commands.ExcuteSoftReset)
            //        {
            //            Commands.ExcuteSoftReset = true;

            //        }
            //    }
            //}
        }

        /// <summary>
        /// 触发控制卡急停命令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_EMG_Click(object sender, EventArgs e)
        {
            mc_Leadshine.IsEMG = !mc_Leadshine.IsEMG;
        }

        /// <summary>
        /// 清除辅助编码器的值
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_ClearEncoder_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteClearExtraEncoder)
            {
                Commands.ExcuteClearExtraEncoder = true;
            }
        }

        /// <summary>
        /// 总线配置文件.eni+.ini路径确定
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox1_MouseClick(object sender, MouseEventArgs e)
        {
            if (checkBox1.Checked == true)
            {
                openFileDialog2.Filter = "eni|*.eni";
                openFileDialog2.FileName = string.Empty;
                DialogResult res = openFileDialog2.ShowDialog(this);
                if (res == DialogResult.OK)
                {
                    textBox9.Text = openFileDialog2.FileName;
                    textBox10.Text = openFileDialog2.FileName.Replace(".eni", ".ini");
                }
            }
        }

        /// <summary>
        /// 轴参数文件.ini路径确定
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void checkBox2_MouseClick(object sender, MouseEventArgs e)
        {
            if (checkBox2.Checked == true)
            {
                openFileDialog1.Filter = "ini|*.ini";
                openFileDialog1.FileName = string.Empty;
                DialogResult res = openFileDialog1.ShowDialog(this);
                if (res == DialogResult.OK)
                {
                    textBox11.Text = openFileDialog1.FileName;
                }
            }
        }

        /// <summary>
        /// 启动高速位置比较功能
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button8_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteHcmpEnable)
            {
                Commands.ExcuteHcmpEnable = true;
            }

        }

        /// <summary>
        /// 关闭高速位置比较功能
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteHcmpDisable)
            {
                Commands.ExcuteHcmpDisable = true;
            }

        }

        /// <summary>
        /// 启动单轴点位运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_MovePos_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteMovePos)
            {
                Commands.ExcuteMovePos = true;
            }
        }

        /// <summary>
        /// 停止单轴运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_StopMove_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteStopMove)
            {
                Commands.ExcuteStopMove = true;
            }
        }

        /// <summary>
        /// 清除单轴驱动报警
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_ClearError_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteClearAxisError)
            {
                Commands.ExcuteClearAxisError = true;
            }
        }

        /// <summary>
        /// 单轴开关使能
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void label81_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteAxisPowerOn)
            {
                Commands.ExcuteAxisPowerOn = true;
            }
        }

        /// <summary>
        /// 启动插补运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button2_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteVectorMove && dataGridView1.Rows.Count > 1)
            {
                Commands.ExcuteVectorMove = true;
            }
        }

        /// <summary>
        /// 停止插补运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button3_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteStopVectorMove)
            {
                Commands.ExcuteStopVectorMove = true;
            }
        }

        /// <summary>
        /// 获取插补运动轴坐标数据
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button4_Click(object sender, EventArgs e)
        {
            string strData = string.Empty;
            #region 获取雷赛日志文件“Debuglog”里面的坐标数据
            if (!File.Exists("DebugLog.txt"))
            {
                MessageBox.Show("文件不存在！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information, MessageBoxDefaultButton.Button1);
            }
            else
            {
                FileStream fs = new FileStream("DebugLog.txt", FileMode.Open, FileAccess.Read);
                StreamReader sr = new StreamReader(fs);
                uint VectorNum = 0;//参与插补的轴数量
                uiRowNum = 0;
                uiColumnNum = 0;
                while (true)
                {
                    strData = sr.ReadLine();
                    if (strData == null)
                    {
                        break;
                    }
                    if (strData.Contains("dmc_conti_open_list:CardNo") == true && strData.Contains(",return:0") == true)//获取open_list的轴数量
                    {
                        string strNewData = strData.Remove(0, strData.LastIndexOf("AxisNum:") + "AxisNum:".Length);
                        uint.TryParse(strNewData.Substring(0, strNewData.IndexOf(",")), out VectorNum);
                    }
                    if (strData.Contains(",return:0") == false && strData.Contains("pPosList[i]:") == true)//获取line_unit的轴位置坐标
                    {
                        string strNewData = strData.Remove(0, strData.LastIndexOf(":") + 1).Trim();
                        double.TryParse(strNewData, out arrayTargetPos[uiRowNum, uiColumnNum]);
                        uiColumnNum++;
                        if (uiColumnNum >= VectorNum)
                        {
                            uiColumnNum = 0;
                            uiRowNum++;
                        }
                    }
                }
                uiColumnNum = VectorNum;
                sr.Close();
                fs.Close();

                #region UI界面的表格填充数据
                DataTable table = new DataTable();
                table.Columns.Add("序号", typeof(string));
                for (int i = 0; i < uiColumnNum; i++)
                {
                    table.Columns.Add($"轴{i}", typeof(double));
                }
                for (int i = 0; i < uiRowNum; i++)
                {
                    DataRow newRow = table.NewRow();
                    newRow[0] = i;
                    for (int j = 0; j < uiColumnNum; j++)
                    {
                        newRow[j + 1] = arrayTargetPos[i, j];
                    }
                    table.Rows.Add(newRow);
                }
                dataGridView1.DataSource = table;
                //dataGridView1.AutoResizeColumns();
                dataGridView1.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
                #endregion
            }
            #endregion
        }

        /// <summary>
        /// 设置模拟量值
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_DaOut_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteSetDaValue)
            {
                Commands.ExcuteSetDaValue = true;
            }
        }

        #region C#结构体与指针互相转换
        public static T IntPtrToStruct<T>(IntPtr ptr) where T : struct
        {
            object t = Marshal.PtrToStructure(ptr, typeof(T));
            Marshal.FreeHGlobal(ptr);
            if (t is T t1)
            {
                return t1;
            }
            else
            {
                return default(T);
            }
        }

        public static IntPtr StructToIntPtr<T>(T info)      //C#结构体与指针互相转换
        {
            int size = Marshal.SizeOf(info);
            IntPtr intPtr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(info, intPtr, true);
            return intPtr;
        }
        #endregion

        /// <summary>
        /// 启动指令缓存运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button9_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteGroupMove)
            {
                Commands.ExcuteGroupMove = true;
            }
        }

        /// <summary>
        /// 暂停指令缓存运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button10_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteGroupPauseMove)
            {
                Commands.ExcuteGroupPauseMove = true;
            }
        }

        /// <summary>
        /// 停止指令缓存运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button11_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteGroupStopMove)
            {
                Commands.ExcuteGroupStopMove = true;
            }
        }

        /// <summary>
        /// 清零轴位置
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button12_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteClearAxisPos)
            {
                Commands.ExcuteClearAxisPos = true;
            }
        }

        /// <summary>
        /// 启动自动运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button5_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteAutoRun)
            {
                Commands.ExcuteAutoRun = true;
            }
        }

        /// <summary>
        /// 停止自动运行
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button6_Click(object sender, EventArgs e)
        {
            mc_Leadshine.IsAutoRunning = false;
        }

        private void button7_Click(object sender, EventArgs e)
        {

        }

        private void radioButton7_CheckedChanged(object sender, EventArgs e)
        {
            if (radioButton7.Checked)
            {
                #region UI界面的表格填充数据
                //dataGridView2.Columns.Clear();
                //dataGridView2.Rows.Clear();
                DataTable table = new DataTable();
                table.Columns.Add("序号", typeof(string));
                table.Columns.Add($"X轴正向", typeof(double));
                table.Columns.Add($"X轴负向", typeof(double));

                DataRow newRow = table.NewRow();
                newRow[0] = 0;
                newRow[1] = 0;
                newRow[2] = 0;
                table.Rows.Add(newRow);

                dataGridView2.DataSource = table;
                //dataGridView1.AutoResizeColumns();
                dataGridView2.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
                #endregion
            }
        }

        private void radioButton8_CheckedChanged(object sender, EventArgs e)
        {
            #region UI界面的表格填充数据
            if (radioButton8.Checked)
            {
                if (dataGridView2.DataSource == null)
                {
                    DataTable table = new DataTable();
                    table.Columns.Add("序号", typeof(string));
                    table.Columns.Add($"X轴正向", typeof(double));
                    table.Columns.Add($"X轴负向", typeof(double));
                    table.Columns.Add($"Y轴正向", typeof(double));
                    table.Columns.Add($"Y轴负向", typeof(double));

                    DataRow newRow = table.NewRow();
                    newRow[0] = 0;
                    newRow[1] = 0;
                    newRow[2] = 0;
                    newRow[3] = 0;
                    newRow[4] = 0;
                    table.Rows.Add(newRow);

                    dataGridView2.DataSource = table;
                    //dataGridView1.AutoResizeColumns();
                    dataGridView2.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
                }
            }
            #endregion
        }

        private void button14_Click(object sender, EventArgs e)
        {
            while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
            {
                ushort usAxisID = decimal.ToUInt16(numericUpDown36.Value);
                double fStartPos = decimal.ToDouble(numericUpDown38.Value);
                double fLength = decimal.ToDouble(numericUpDown39.Value);
                ushort nPos = decimal.ToUInt16(numericUpDown40.Value);
                double[] pCompPos = new double[10000];
                double[] pCompNeg = new double[10000];
                for (int i = 0; i < pCompPos.Length; i++)
                {
                    pCompPos[i] = 3;
                    pCompNeg[i] = -2;
                }
                short res = LTDMC.dmc_enable_leadscrew_comp(mc_Leadshine.CardID, usAxisID, 0); //关闭轴的螺距补偿
                res = LTDMC.dmc_set_leadscrew_comp_config_unit(mc_Leadshine.CardID, usAxisID, nPos, fStartPos, fLength, pCompPos, pCompNeg);
                res = LTDMC.dmc_enable_leadscrew_comp(mc_Leadshine.CardID, usAxisID, 1); //使能轴的螺距补偿
                break;
            }

        }


        private void button15_Click(object sender, EventArgs e)
        {
            ushort usAxisID = decimal.ToUInt16(numericUpDown36.Value);
            short res = LTDMC.dmc_enable_leadscrew_comp(mc_Leadshine.CardID, usAxisID, 0); //关闭轴的螺距补偿
        }

        /// <summary>
        /// 启动龙门跟随运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button16_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteGearMove && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
            {
                DialogResult res = MessageBox.Show("是否启动龙门跟随", "询问", MessageBoxButtons.OKCancel, MessageBoxIcon.Question);
                if (res == DialogResult.Cancel)
                {
                    ;
                }
                else
                {
                    Commands.ExcuteGearMove = true;
                }
            }
        }

        /// <summary>
        /// 关闭龙门跟随运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button17_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteStopGearMove && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.AxisNum >= 1)
            {
                Commands.ExcuteStopGearMove = true;
            }
        }
        /// <summary>
        /// 临时测试
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button18_Click(object sender, EventArgs e)
        {
            #region PDO直接读写功能测试
            //uint PdoValue = 0;
            //byte[] PdoValueByte=new byte[4];
            //LTDMC.nmc_read_txpdo(mc_Leadshine.CardID,2,1001,0x6041,0,32, PdoValueByte);
            //PdoValue = BitConverter.ToUInt32(PdoValueByte,0);
            #endregion
            /*
            #region 区域碰撞功能说明
            ushort[] arrayAxisList = new ushort[4] { 0, 1, 2, 3 };              //参与运动的轴号列表
            double[] rect1_list = new double[4] { 0, 0, 0, 0 };                 //平面1原点起点列表
            double[] rect2_list = new double[4] { 4000, 4000, 4000, 4000 };     //平面2原点起点列表
            short[] axisDir = new short[2] { 1, 1 };   //方向列表(1,1)，依次填入轴2相对轴0的方向、轴3相对轴1的方向; 1代表相对同向，-1代表相对反向;
            double fRect2Angle = 0;     //平面2绕原点逆时钟旋转的角度，范围0 <=angle < 360
            ushort usEnable = 1;        //0：失能，非0：使能
            ushort usStopMode = 1;      //1：所有轴急停，0：所有轴减速停

            //short res = LTDMC.dmc_dual_2dmove_collisiondetection_set_param(mc_Leadshine.CardID, arrayAxisList, rect1_list, rect2_list, 0, usStopMode);
            //res = LTDMC.dmc_dual_2dmove_collisiondetection_set_param(mc_Leadshine.CardID, arrayAxisList, rect1_list, rect2_list, usEnable, usStopMode);
            //res = LTDMC.dmc_dual_2dmove_collisiondetection_get_param(mc_Leadshine.CardID, arrayAxisList, rect1_list, rect2_list, ref usEnable, ref usStopMode);
            short res = LTDMC.dmc_dual_2dmove_collisiondetection_set_param(mc_Leadshine.CardID, arrayAxisList, rect1_list, rect2_list, axisDir, 0, 0, usStopMode);
            res = LTDMC.dmc_dual_2dmove_collisiondetection_set_param(mc_Leadshine.CardID, arrayAxisList, rect1_list, rect2_list, axisDir, fRect2Angle, usEnable, usStopMode);
            res = LTDMC.dmc_dual_2dmove_collisiondetection_get_param(mc_Leadshine.CardID, arrayAxisList, rect1_list, rect2_list, axisDir, ref fRect2Angle, ref usEnable, ref usStopMode);
            //res = LTDMC.dmc_get_vector_s_profile(mc_Leadshine.CardID, 0, 0, ref fRect2Angle);
            //res = LTDMC.dmc_check_done_multicoor(mc_Leadshine.CardID, 0);
            #endregion
            */
            double[] temDouble = new double[255];
            short res = LTDMC.dmc_get_position_unit(mc_Leadshine.CardID, 255, ref temDouble[0]);//获取所有轴的指令位置
            res = LTDMC.dmc_get_encoder_unit(mc_Leadshine.CardID, 255, ref temDouble[0]);//获取所有轴的指令位置
        }

        /// <summary>
        /// 读取扩展PDO值
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button19_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteReadExtraPDO && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
            {
                Commands.ExcuteReadExtraPDO = true;
            }
        }

        /// <summary>
        /// 设置扩展PDO值
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button20_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteWriteExtraPDO && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
            {
                Commands.ExcuteWriteExtraPDO = true;
            }
        }

        /// <summary>
        /// 读取PDO值
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button21_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteReadPDO && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
            {
                Commands.ExcuteReadPDO = true;
            }
        }

        /// <summary>
        /// 设置PDO值
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button22_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteWritePDO && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
            {
                Commands.ExcuteWritePDO = true;
            }
        }

        /// <summary>
        /// 清除总线通讯报警，必须从站在线才有效
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button23_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteClearEtherCATError && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState != 0)
            {
                Commands.ExcuteClearEtherCATError = true;
            }
        }

        /// <summary>
        /// 开关单个输出口状态
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void label51_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteTurnOnOffOutbit && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
            {
                if (sender is System.Windows.Forms.Label label)
                {
                    if (label.Text.StartsWith("OUT"))
                    {
                        ushort.TryParse(label.Text.Replace("OUT", null).Trim(), out usCurrentOutID);
                        if (usCurrentOutID < mc_Leadshine.OutputNum)
                        {
                            Commands.ExcuteTurnOnOffOutbit = true;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 启动CST转矩运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button24_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteMoveTorque && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState == 0)
            {
                DialogResult res = MessageBox.Show("是否启动转矩运动", "询问", MessageBoxButtons.OKCancel, MessageBoxIcon.Question);
                if (res == DialogResult.Cancel)
                {
                    ;
                }
                else
                {
                    Commands.ExcuteMoveTorque = true;
                }

            }
        }

        /// <summary>
        /// 启动采样跟踪
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button25_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteTraceData && !mc_Leadshine.IsTracing && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState == 0)
            {
                Commands.ExcuteTraceData = true;
            }
        }

        /// <summary>
        /// 停止采样跟踪
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button26_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteStopTraceData && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState == 0)
            {
                Commands.ExcuteStopTraceData = true;
            }
        }

        /// <summary>
        /// 启动单轴正余弦运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button27_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteMoveOscillate && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState == 0)
            {
                DialogResult res = MessageBox.Show("是否启动正余弦运动", "询问", MessageBoxButtons.OKCancel, MessageBoxIcon.Question);
                if (res == DialogResult.Cancel)
                {
                    ;
                }
                else
                {
                    Commands.ExcuteMoveOscillate = true;
                }
            }
        }

        /// <summary>
        /// 停止单轴正余弦运动
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button28_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteStopOscillate && mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset && mc_Leadshine.EtherCATState == 0)
            {
                Commands.ExcuteStopOscillate = true;
            }
        }
    }
}
