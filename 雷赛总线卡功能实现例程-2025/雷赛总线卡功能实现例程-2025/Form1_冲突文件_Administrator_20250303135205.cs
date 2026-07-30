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
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Button;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using System.Diagnostics;
using System.Reflection.Emit;

namespace 雷赛总线卡功能实现例程_2025
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }
        uint WindowsPage = 0;
        MC_Leadshine mc_Leadshine = new MC_Leadshine();
        structCommands Commands = new structCommands();
        private static List<string> ListInfo = new List<string>();      //日志信息的集合

        /// <summary>
        /// 打开软件窗体
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Load(object sender, EventArgs e)
        {
            Thread MainThread = new Thread(() =>
            {
                while (true)
                {
                    #region 启动控制卡连接
                    if (Commands.ExcuteConnect)
                    {
                        while (!mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            if (radioButton2.Checked)//连接EMC控制卡
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
                            else//连接DMC控制卡
                            {
                                short res = LTDMC.dmc_board_init();
                                if (res <= 0 || res > 8)
                                {
                                    AddListInfo(ListInfo, $"报警：连接DMC控制器失败，dmc_board_init()={res}");
                                    break;
                                }
                                AddListInfo(ListInfo, $"连接DMC控制器成功，dmc_board_init()={res}");
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
                            mc_Leadshine.IsConnectDone = true;

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
                        while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                        {
                            mc_Leadshine.IsConnectDone = false;
                            short res = LTDMC.dmc_board_close();
                            if (res == 0)
                            {
                                AddListInfo(ListInfo, $"关闭控制卡成功,dmc_board_close()={res}");
                            }
                            else
                            {
                                AddListInfo(ListInfo, $"报警：关闭控制卡失败,dmc_board_close()={res}");
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
                            #region 获取：控制器发布版本号
                            byte[] byReleaseVersion = new byte[50];
                            short res = LTDMC.dmc_get_release_version(mc_Leadshine.CardID, byReleaseVersion);
                            mc_Leadshine.ReleaseVersion = Encoding.UTF8.GetString(byReleaseVersion).TrimEnd('\0');
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_get_release_version({mc_Leadshine.CardID})={res}");
                                break;
                            }
                            #endregion

                            #region 获取：控制器硬件版本号
                            uint uiCardVersion = 0;
                            res = LTDMC.dmc_get_card_version(mc_Leadshine.CardID, ref uiCardVersion);
                            mc_Leadshine.CardVersion = uiCardVersion;
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_get_card_version({mc_Leadshine.CardID},{uiCardVersion})={res}");
                                break;
                            }
                            #endregion

                            #region 获取：控制器固件版本号
                            uint uiFirmID = 0;
                            uint uiSubFirmID = 0;
                            res = LTDMC.dmc_get_card_soft_version(mc_Leadshine.CardID, ref uiFirmID, ref uiSubFirmID);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_get_card_soft_version({mc_Leadshine.CardID},{uiFirmID},{uiSubFirmID})={res}");
                                break;
                            }
                            mc_Leadshine.FirmID = uiFirmID;
                            mc_Leadshine.SubFirmID = uiSubFirmID;
                            #endregion

                            #region 获取：当前控制卡本体的IO总数
                            ushort usTotalIn = 0;
                            ushort usTotalOut = 0;
                            res = LTDMC.dmc_get_total_ionum(mc_Leadshine.CardID, ref usTotalIn, ref usTotalOut);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_get_total_ionum({mc_Leadshine.CardID},{usTotalIn},{usTotalOut})={res}");
                                break;
                            }
                            #endregion

                            #region 获取：EtherCAT总线扩展IO输入输出口总数
                            ushort usTotalIn_EtherCat = 0;
                            ushort usTotalOut_EtherCat = 0;
                            res = LTDMC.nmc_get_total_ionum(mc_Leadshine.CardID, ref usTotalIn_EtherCat, ref usTotalOut_EtherCat);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_get_total_ionum({mc_Leadshine.CardID},{usTotalIn_EtherCat},{usTotalOut_EtherCat})={res}");
                                break;
                            }
                            mc_Leadshine.InputNum = (uint)usTotalIn + usTotalIn_EtherCat;
                            mc_Leadshine.OutputNum = (uint)usTotalOut + usTotalOut_EtherCat;
                            #endregion

                            #region 获取：当前控制卡本体的轴总数
                            uint uTotalAxisNum = 0;
                            res = LTDMC.dmc_get_total_axes(mc_Leadshine.CardID, ref uTotalAxisNum);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_get_total_axes({mc_Leadshine.CardID},{uTotalAxisNum})={res}");
                                break;
                            }
                            #endregion

                            #region 获取：当前控制卡总线的轴总数
                            uint uiTotalAxisNum_EtherCat = 0;
                            res = LTDMC.nmc_get_total_axes(mc_Leadshine.CardID, ref uiTotalAxisNum_EtherCat);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_get_total_axes({mc_Leadshine.CardID},{uiTotalAxisNum_EtherCat})={res}");
                                break;
                            }
                            mc_Leadshine.AxisNum = uTotalAxisNum + uiTotalAxisNum_EtherCat;
                            #endregion

                            #region 获取：当前控制卡总线的模拟量通道数量
                            ushort usTotalAD_EtherCat = 0;
                            ushort usTotalDA_EtherCat = 0;
                            res = LTDMC.nmc_get_total_adcnum(mc_Leadshine.CardID, ref usTotalAD_EtherCat, ref usTotalDA_EtherCat);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_get_total_adcnum({mc_Leadshine.CardID},{usTotalAD_EtherCat},{usTotalDA_EtherCat})={res}");
                                break;
                            }
                            mc_Leadshine.AdNum = usTotalAD_EtherCat;
                            mc_Leadshine.DaNum = usTotalAD_EtherCat;
                            #endregion

                            #region 设置：总线复位时输出口状态保持（此功能需扩展IO模块支持才行）
                            res = LTDMC.nmc_set_slave_output_retain(mc_Leadshine.CardID, 1);
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_set_slave_output_retain({mc_Leadshine.CardID},{1})={res}");
                                break;
                            }
                            #endregion

                            #region 设置：一些特殊设置（需和雷赛技术沟通后再使用）
                            //res = LTDMC.nmc_set_dc_mode(CardInfo.CardID,2,1);//0:master shift，1:bus shift
                            //if (res != 0)
                            //{
                            //    AddListInfo(ListInfo, $"报警：nmc_set_dc_mode({CardInfo.CardID},{2},{1})={res}");
                            //    break;
                            //}
                            //res = LTDMC.nmc_set_data_offset_time(CardInfo.CardID, 200);//设置控制卡的总线同步偏移，单位us
                            //以上指令执行后，还需要热复位控制卡后才会生效
                            #endregion

                            #region 获取：EtherCAT总线状态
                            ushort usEtherCATState = 1;
                            res = LTDMC.nmc_get_total_slaves(mc_Leadshine.CardID, 6, ref usEtherCATState);
                            res = LTDMC.nmc_get_errcode(mc_Leadshine.CardID, 2, ref usEtherCATState);
                            if (res != 0 || usEtherCATState != 0)
                            {
                                AddListInfo(ListInfo, $"报警：总线通讯异常nmc_get_errcode({mc_Leadshine.CardID},{2},{usEtherCATState})={res}");
                                break;
                            }
                            #endregion

                            #region 设置：下载轴参数文件,文件格式(.ini)。注意：需要在总线通讯状态正常时执行
                            if (checkBox2.Checked == true)
                            {
                                //string strFileName = "AxisParameters.ini";
                                string strFileName = textBox11.Text.Trim();
                                if (File.Exists(strFileName))
                                {
                                    res = LTDMC.dmc_download_configfile(mc_Leadshine.CardID, strFileName);//下载轴参数文件.ini
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：nmc_set_offset_pos({mc_Leadshine.CardID},{strFileName})={res}");
                                        break;
                                    }
                                }
                                else
                                {
                                    AddListInfo(ListInfo, $"报警：轴参数文件{strFileName}不存在");
                                    break;
                                }
                            }
                            #endregion

                            #region 设置：控制卡轴位置和驱动器位置0x6064同步，一般用于绝对值编码器
                            for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                            {
                                res = LTDMC.nmc_set_offset_pos(mc_Leadshine.CardID, i, 0);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：nmc_set_offset_pos({mc_Leadshine.CardID},{i},{0})={res}");
                                    break;
                                }
                            }
                            if (res != 0)
                            {
                                break;
                            }
                            #endregion

                            #region 设置：总线轴遇到限位减速停止
                            //for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                            //{
                            //    res = LTDMC.dmc_set_el_mode(CardInfo.CardID, i, 1, 1, 1);//开启轴遇限位：减速停止功能（冷复位控制卡才能关闭）
                            //    if (res != 0)
                            //    {
                            //        AddListInfo(ListInfo, $"dmc_set_el_mode({CardInfo.CardID}, {i},{1},{1},{1})={res}");
                            //        break;
                            //    }
                            //    res = LTDMC.dmc_set_dec_stop_time(CardInfo.CardID, i, 0.02);//设置轴遇限位时减速停止时间为20ms
                            //    if (res != 0)
                            //    {
                            //        AddListInfo(ListInfo, $"dmc_set_dec_stop_time({CardInfo.CardID}, {i},{0.02})={res}");
                            //        break;
                            //    }
                            //    res = LTDMC.nmc_set_etc_el_stop_mode(CardInfo.CardID, i, 3, 0, 0);//设置轴遇到限位的停止模式3,即按照设置的减速停止时间来规划停止0x607A
                            //    if (res != 0)
                            //    {
                            //        AddListInfo(ListInfo, $"nmc_set_etc_el_stop_mode({CardInfo.CardID}, {i},{3},{0},{0})={res}");
                            //        break;
                            //    }
                            //}
                            //if (res != 0)
                            //{
                            //    break;
                            //}
                            #endregion

                            #region 设置：轴速度前馈设置（需要专用版本才支持）
                            //for (ushort i = 0; i < uiTotalAxisNum_EtherCat; i++)
                            //{
                            //    res = LTDMC.dmc_set_feedforward_profile(CardInfo.CardID, i, 1, 1);
                            //    if (res != 0)
                            //    {
                            //        AddListInfo(ListInfo, $"报警：dmc_set_feedforward_profile({CardInfo.CardID},{i},{1},{1})={res}");
                            //        break;
                            //    }
                            //}
                            //if (res != 0)
                            //{
                            //    break;
                            //}
                            #endregion
                            break;
                        }
                        Commands.ExcuteInit = false;
                    }
                    #endregion

                    #region 获取：轮询卡的各种状态
                    while (mc_Leadshine.IsConnectDone && !mc_Leadshine.IsSoftReset)
                    {
                        short res = 0;
                        #region 获取：总线通讯状态
                        ushort state = 0;
                        res = LTDMC.nmc_get_errcode(mc_Leadshine.CardID, 2, ref state);//读取EtherCAT总线状态
                        mc_Leadshine.EtherCATState = state;
                        if (res != 0)
                        {
                            AddListInfo(ListInfo, $"报警：总线通讯异常nmc_get_errcode({mc_Leadshine.CardID},{2},{mc_Leadshine.EtherCATState})={res}");
                           Commands.ExcuteDisConnect = true;
                            break;
                        }


                        res = LTDMC.nmc_get_errcode(mc_Leadshine.CardID, 6, ref state);//读取控制器背板总线状态
                        mc_Leadshine.LocalBusState = state;
                        if (res != 0)
                        {
                            AddListInfo(ListInfo, $"报警：背板通讯异常nmc_get_errcode({mc_Leadshine.CardID},{6},{mc_Leadshine.LocalBusState})={res}");
                            Commands.ExcuteDisConnect = true;
                            break;
                        }
                        #endregion

                        #region 获取：通用IO状态
                        if (Page == 2)
                        {
                            int i = groupBox3.Controls.Count;
                            uint InportState = 0;
                            stopWatch[9].Restart();
                            res = LTDMC.dmc_read_inport_ex(MC_Leadshine.ConnectNo, 0, ref InportState);
                            stopWatchTime[9] = stopWatch[9].ElapsedTicks / (Stopwatch.Frequency / 1000000);
                            stopWatch[9].Stop();
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"dmc_read_inport_ex({MC_Leadshine.ConnectNo},{0}, {InportState})={res}");
                                ControllerCommand.DisConnect = true;
                                break;
                            }
                            foreach (Control item in groupBox3.Controls)
                            {
                                i--;
                                if (i < 32 && i < MC_Leadshine.InputNum)
                                {
                                    long BitState = InportState & (1 << i);
                                    if (BitState == 0)//输入口低电平，有效了
                                    {
                                        item.BackColor = Color.GreenYellow;
                                    }
                                    else//输入口高电平，无效了
                                    {
                                        item.BackColor = Control.DefaultBackColor;
                                    }
                                }
                            }

                            int j = groupBox4.Controls.Count;
                            uint OutportState = 0;
                            stopWatch[10].Restart();
                            res = LTDMC.dmc_read_outport_ex(MC_Leadshine.ConnectNo, 0, ref OutportState);
                            stopWatchTime[10] = stopWatch[10].ElapsedTicks / (Stopwatch.Frequency / 1000000);
                            stopWatch[10].Stop();
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"dmc_read_outport_ex({MC_Leadshine.ConnectNo},{0}, {OutportState})={res}");
                                ControllerCommand.DisConnect = true;
                                break;
                            }
                            foreach (Control item in groupBox4.Controls)
                            {
                                j--;
                                if (j < 32 && j < MC_Leadshine.OutputNum)
                                {
                                    long BitState = OutportState & (1 << j);
                                    if (BitState == 0)//输出口低电平，打开了
                                    {
                                        item.BackColor = Color.GreenYellow;
                                    }
                                    else//输出口高电平，关闭了
                                    {
                                        item.BackColor = Control.DefaultBackColor;
                                    }
                                }
                            }
                        }
                        #endregion

                        #region 获取：批量读取轴的属性，轴专用IO，轴使能，轴运行状态，轴指令位置，轴反馈位置，轴当前速度
                        if (MC_Leadshine.AxisNum >= 1)
                        {
                            uint[] uiAxisState = new uint[MC_Leadshine.AxisNum];
                            stopWatch[3].Restart();
                            res = LTDMC.dmc_axis_io_status_ex(MC_Leadshine.ConnectNo, 255, ref uiAxisState[0]);//读取所有轴有关运动信号的状态,包括限位，原点，报警等
                            stopWatchTime[3] = stopWatch[3].ElapsedTicks / (Stopwatch.Frequency / 1000000);
                            stopWatch[3].Stop();
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_axis_io_status_ex({MC_Leadshine.ConnectNo},{255},{uiAxisState[0]})={res}");
                                ControllerCommand.DisConnect = true;
                                break;
                            }
                            for (uint i = 0; i < uiAxisState.Length; i++)
                            {
                                MC_Leadshine.AxisAlarm[i] = ((uiAxisState[i] & (1 << 0)) == 0) ? false : true;//伺服报警
                                MC_Leadshine.AxisELP[i] = ((uiAxisState[i] & (1 << 1)) == 0) ? false : true;//正限位ELP
                                MC_Leadshine.AxisELN[i] = ((uiAxisState[i] & (1 << 2)) == 0) ? false : true;//正限位ELN
                                MC_Leadshine.AxisEMG[i] = ((uiAxisState[i] & (1 << 3)) == 0) ? false : true;//急停EMG
                                MC_Leadshine.AxisORG[i] = ((uiAxisState[i] & (1 << 4)) == 0) ? false : true;//原点ORG
                            }
                            ushort[] usStateMachine = new ushort[MC_Leadshine.AxisNum];
                            stopWatch[4].Restart();
                            res = LTDMC.nmc_get_axis_state_machine(MC_Leadshine.ConnectNo, 255, ref usStateMachine[0]);//获取所有轴的使能状态，4为使能，非4未使能
                            stopWatchTime[4] = stopWatch[4].ElapsedTicks / (Stopwatch.Frequency / 1000000);
                            stopWatch[4].Stop();
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：nmc_get_axis_state_machine({MC_Leadshine.ConnectNo},{255},{usStateMachine[0]})={res}");
                                ControllerCommand.DisConnect = true;
                                break;
                            }
                            for (uint i = 0; i < usStateMachine.Length; i++)
                            {
                                MC_Leadshine.AxisPowerOn[i] = (usStateMachine[i] == 4) ? true : false;
                            }
                            stopWatch[5].Restart();
                            res = LTDMC.dmc_check_done_ex(MC_Leadshine.ConnectNo, 255, ref usStateMachine[0]);//获取所有轴运行状态，0运行，1停止
                            stopWatchTime[5] = stopWatch[5].ElapsedTicks / (Stopwatch.Frequency / 1000000);
                            stopWatch[5].Stop();
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_check_done_ex({MC_Leadshine.ConnectNo},{255},{usStateMachine[0]})={res}");
                                ControllerCommand.DisConnect = true;
                                break;
                            }
                            for (uint i = 0; i < usStateMachine.Length; i++)
                            {
                                MC_Leadshine.AxisBusy[i] = (usStateMachine[i] == 0) ? true : false;
                            }
                            double[] temDouble = new double[MC_Leadshine.AxisNum];//临时变量
                            stopWatch[6].Restart();
                            res = LTDMC.dmc_get_position_unit(MC_Leadshine.ConnectNo, 255, ref temDouble[0]);//获取所有轴的指令位置
                            stopWatchTime[6] = stopWatch[6].ElapsedTicks / (Stopwatch.Frequency / 1000000);
                            stopWatch[6].Stop();
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_get_position_unit({MC_Leadshine.ConnectNo},{255},{temDouble[0]})={res}");
                                ControllerCommand.DisConnect = true;
                                break;
                            }
                            Array.Copy(temDouble, MC_Leadshine.AxisCommandPos, temDouble.Length);
                            stopWatch[7].Restart();
                            res = LTDMC.dmc_get_encoder_unit(MC_Leadshine.ConnectNo, 255, ref temDouble[0]);// 获取所有轴的反馈位置
                            stopWatchTime[7] = stopWatch[7].ElapsedTicks / (Stopwatch.Frequency / 1000000);
                            stopWatch[7].Stop();
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_get_encoder_unit({MC_Leadshine.ConnectNo},{255},{temDouble[0]})={res}");
                                ControllerCommand.DisConnect = true;
                                break;
                            }
                            Array.Copy(temDouble, MC_Leadshine.AxisActualPos, temDouble.Length);
                            stopWatch[8].Restart();
                            res = LTDMC.dmc_read_current_speed_unit(MC_Leadshine.ConnectNo, 255, ref temDouble[0]);//获取所有轴的运行速度
                            stopWatchTime[8] = stopWatch[8].ElapsedTicks / (Stopwatch.Frequency / 1000000);
                            stopWatch[8].Stop();
                            if (res != 0)
                            {
                                AddListInfo(ListInfo, $"报警：dmc_read_current_speed_unit({MC_Leadshine.ConnectNo},{255},{temDouble[0]})={res}");
                                ControllerCommand.DisConnect = true;
                                break;
                            }
                            Array.Copy(temDouble, MC_Leadshine.AxisCommandSpeed, temDouble.Length);
                        }
                        #endregion

                        #region  获取：单轴的属性，包含回零结果，停止原因
                        //if (Page == 1)
                        //{
                        //    ushort usAxisID = decimal.ToUInt16(numericUpDown1.Value);
                        //    if (MC_Leadshine.AxisNum > usAxisID)
                        //    {
                        //        ushort usAxisHomeRes = 2;
                        //        res = LTDMC.dmc_get_home_result(MC_Leadshine.ConnectNo, usAxisID, ref usAxisHomeRes);//获取回零结果
                        //        if (res != 0)
                        //        {
                        //            AddListInfo(ListInfo, $"dmc_get_home_result({MC_Leadshine.ConnectNo},{usAxisID},{usAxisHomeRes})={res}");
                        //            ControllerCommand.DisConnect = true;
                        //            break;
                        //        }
                        //        MC_Leadshine.AxisHomeResult[usAxisID] = usAxisHomeRes;
                        //        int iAxisStopReason = 2;
                        //        res = LTDMC.dmc_get_stop_reason(MC_Leadshine.ConnectNo, usAxisID, ref iAxisStopReason);//获取轴的停止原因
                        //        if (res != 0)
                        //        {
                        //            AddListInfo(ListInfo, $"dmc_get_stop_reason({MC_Leadshine.ConnectNo},{usAxisID},{iAxisStopReason})={res}");
                        //            ControllerCommand.DisConnect = true;
                        //            break;
                        //        }
                        //        MC_Leadshine.AxisStopReason[usAxisID] = iAxisStopReason;
                        //    }
                        //}
                        #endregion

                        #region 获取：模拟量输入值
                        if (WindowsPage == 3 && mc_Leadshine.AdNum > 0)
                        {
                            if (radioButton1.Checked)
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
                                    res = LTDMC.dmc_get_ad_input(mc_Leadshine.CardID, channelId, ref AdValue);
                                    if (res != 0)
                                    {
                                        AddListInfo(ListInfo, $"报警：dmc_get_ad_input({mc_Leadshine.CardID}, {channelId}, {AdValue})={res}");
                                        break;
                                    }
                                    mc_Leadshine.ADValue[channelId] = AdValue;
                                }
                                if (res != 0)
                                {
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                            }
                            else
                            {
                                double[] temDouble = new double[255];//临时变量
                                res = LTDMC.dmc_get_ad_input(mc_Leadshine.CardID, 255, ref temDouble[0]);
                                if (res != 0)
                                {
                                    AddListInfo(ListInfo, $"报警：dmc_get_ad_input({mc_Leadshine.CardID}, {255}, {temDouble[0]})={res}");
                                    Commands.ExcuteDisConnect = true;
                                    break;
                                }
                                Array.Copy(temDouble, mc_Leadshine.ADValue, temDouble.Length);
                            }
                        }
                        #endregion

                        #region 获取：辅助编码器的值
                        if (WindowsPage == 4)
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
                        #endregion
                        break;
                    }
                    #endregion
                }
            });
            MainThread.IsBackground = true;
            MainThread.Start();
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
                btn_DisConnect.PerformClick();
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            this.btn_Connect.Enabled = radioButton1.Enabled = radioButton2.Enabled = !mc_Leadshine.IsConnectDone;

            #region 功能页面选择
            if (this.tabControl1.SelectedTab == tabPage1)
            {
                WindowsPage = 1;
                //ushort usAxisID = decimal.ToUInt16(numericUpDown1.Value);
                //label67.BackColor = MC_Leadshine.AxisAlarm[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//伺服报警
                //label68.BackColor = MC_Leadshine.AxisELP[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//正限位ELP
                //label69.BackColor = MC_Leadshine.AxisELN[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//正限位ELN
                //label71.BackColor = MC_Leadshine.AxisEMG[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//急停EMG
                //label70.BackColor = MC_Leadshine.AxisORG[usAxisID] ? Color.OrangeRed : Control.DefaultBackColor;//原点ORG
                //label72.BackColor = MC_Leadshine.AxisPowerOn[usAxisID] ? Color.GreenYellow : Control.DefaultBackColor;//轴使能
                //textBox2.Text = MC_Leadshine.AxisCommandPos[usAxisID].ToString();
                //textBox3.Text = MC_Leadshine.AxisActualPos[usAxisID].ToString();
                //textBox4.Text = MC_Leadshine.AxisCommandSpeed[usAxisID].ToString();
                //textBox5.Text = (MC_Leadshine.AxisBusy[usAxisID] == false) ? "停止" : "运行";
                //textBox20.Text = (MC_Leadshine.AxisStopReason[usAxisID] == 0) ? "正常停止" : $"非正常停止:{MC_Leadshine.AxisStopReason[usAxisID]}";
                //textBox21.Text = (MC_Leadshine.AxisHomeResult[usAxisID] == 1 && MC_Leadshine.AxisStopReason[usAxisID] == 0) ? "完成" : "未完成";
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
            }
            else if (this.tabControl1.SelectedTab == tabPage4)
            {
                WindowsPage = 4;
                textBox18.Text = mc_Leadshine.ExtraEncoder[decimal.ToUInt16(numericUpDown19.Value)].ToString();
                textBox19.Text = mc_Leadshine.ExtraEncoder[decimal.ToUInt16(numericUpDown20.Value)].ToString();
            }
            else if (this.tabControl1.SelectedTab == tabPage5)
            {
                WindowsPage = 5;
            }
            else if (this.tabControl1.SelectedTab == tabPage6)
            {
                WindowsPage = 6;
            }
            else if (this.tabControl1.SelectedTab == tabPage7)
            {
                WindowsPage = 7;
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
            }
            #endregion

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
            if (!Commands.ExcuteSoftReset)
            {
                Commands.ExcuteSoftReset = true;
            }
        }

        /// <summary>
        /// 触发控制卡急停命令
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btn_EMG_Click(object sender, EventArgs e)
        {
            if (!Commands.ExcuteEmgStop)
            {
                Commands.ExcuteEmgStop = true;
            }
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
    }
}
