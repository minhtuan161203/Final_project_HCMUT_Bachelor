using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;

namespace Final_Project
{
    public partial class Form1 : Form
    {
        SerialPort mySerial = new SerialPort();
        public Form1()
        {
            InitializeComponent();
            Control.CheckForIllegalCrossThreadCalls = false; // Update UI from Interupt Session

            // Config for Serial Port
            mySerial.PortName = "COM6";
            mySerial.BaudRate = 115200;
            mySerial.DataReceived += MySerial_DataReceived;

            try { mySerial.Open(); }
            catch { MessageBox.Show("Không mở được cổng COM!"); }
        }
        private void MySerial_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int result = mySerial.ReadByte();
            Console.WriteLine(result);
        }

        private void btnSend_Click_1(object sender, EventArgs e)
        {
            if (mySerial.IsOpen)
            {
                byte[] data = new byte[1];
                data[0] = 92; // Thử gửi số 90
                mySerial.Write(data, 0, 1);
            }
        }
    }

}
