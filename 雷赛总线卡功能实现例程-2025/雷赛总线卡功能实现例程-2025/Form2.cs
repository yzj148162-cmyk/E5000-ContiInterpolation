using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace 雷赛总线卡功能实现例程_2025
{
    public partial class Form2 : Form
    {
        public Form2()
        {
            InitializeComponent();
        }

        private void Form2_Load(object sender, EventArgs e)
        {
            this.Tag = "1";
            this.ControlBox = false;
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (progressBar1.Value< progressBar1.Maximum)
            {
                progressBar1.Value = progressBar1.Value + 1;
            }
        }
    }
}
