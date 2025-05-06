using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
//using System.Threading.Tasks;
using System.Windows.Forms;
using Uranus.Data;
using Uranus.Utilities;
using WisdomConnection;

namespace WindowsFormsAppForRobot
{
    public partial class Form1 : Form
    {
        KbootPacketDecoder KbootDecoder = new KbootPacketDecoder();
        //IMUData UranusData = new IMUData();
        IMUData imuData = new IMUData();
        private SampleCounter counter = new SampleCounter();
        #region data receive
        public void OnKbootDecoderDataReceived(object sender, byte[] buf, int len)
        {
            imuData = IMUData.Decode(buf, len);
            counter.Increment(1);
        }
        #endregion
        public Form1()
        {
            InitializeComponent();
            
            KbootDecoder.OnPacketRecieved += new KbootPacketDecoder.KBootDecoderDataReceivedEventHandler(OnKbootDecoderDataReceived);
        }

        private void comboBoxPortName_DropDown(object sender, EventArgs e)
        {
            
            string[] PortName = SerialPort.GetPortNames();
            Array.Sort(PortName);//给端口名称排序
            comboBoxPortName.Items.Clear();
            for (int i = 0; i < PortName.Length; i++)
            {
                comboBoxPortName.Items.Add(PortName[i]);//给comboBoxPortName添加选项
            }
            spSerialPort.BaudRate = 115200;
            
        }

        private void buttonPortSwitch_Click(object sender, EventArgs e)
        {
            
            try
            {
                if (spSerialPort.IsOpen)
                {
                    spSerialPort.Close();
                    buttonPortSwitch.Text = "打开";
                    timerDataProcessing.Enabled = false;
                }
                else if (spSerialPort.IsOpen == false)
                {
                    spSerialPort.PortName = comboBoxPortName.Text;
                    spSerialPort.Open();
                    buttonPortSwitch.Text = "关闭";
                    OutputQuaternion();
                    timerDataProcessing.Enabled = true;
                }
            }
            catch
            {
                MessageBox.Show("端口错误，请检查端口", "错误");
            }
            
        }
        /// <summary>
        /// 把动作捕捉模块配置成四元数输出模式
        /// </summary>
        public void OutputQuaternion()
        {
            spSerialPort.Write("AT+SETPTL=71\r\n");
        }
        public void Reset()
        {
            spSerialPort.Write("AT+RFCMD=0,AT+RST\r\n");
        }
        private void buttonReset_Click(object sender, EventArgs e)
        {
            Reset();
        }
        private void spSerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            
            if (spSerialPort.IsOpen)
            {
                int bytesToRead = spSerialPort.BytesToRead;
                byte[] readBuffer = new byte[bytesToRead];
                if (spSerialPort.IsOpen)
                {
                    spSerialPort.Read(readBuffer, 0, bytesToRead);
                    KbootDecoder.Input(readBuffer);
                }
            }
            
        }
        private void timerDisplayRefresh_Tick(object sender, EventArgs e)
        {
            if (imuData != null)
            {
                String attitudeDataStr = imuData.ToString();
                labelReceiveRate.Text = "接收速率: " + counter.SampleRate.ToString() + "Hz";
                labelAttitudeData.Text = attitudeDataStr;
                
                labelServoPosition.Text = "ID1:" + servo1Position.ToString() + "\r\n"
                    + "ID2:" + servo2Position.ToString() + "\r\n"
                    + "ID3:" + servo3Position.ToString() + "\r\n"
                    + "ID4:" + servo4Position.ToString() + "\r\n"
                    + "ID31:" + servo31Position.ToString() + "\r\n"
                    + "ID32:" + servo32Position.ToString() + "\r\n"
                    + "ID33:" + servo33Position.ToString() + "\r\n"
                    + "ID34:" + servo34Position.ToString() + "\r\n";
                labelChassisControl.Text = "角度:" + _chassisAngle.ToString() + "\r\n"
                    + "速度:" + _chassisSpeed.ToString() + "\r\n"
                    + "旋转速度:" + _chassisTurnRate.ToString();
            }
        }
        /// <summary>
        /// 获取相对四元数
        /// </summary>
        /// <param name="Qm">Qm</param>
        /// <param name="Qn">Qn</param>
        /// <returns>相对四元数</returns>
        private float[] getQmn(float[] Qm, float[] Qn)
        {
            float[] Qmn = new float[4];
            float[] Qm_1 = new float[4];//Qm的逆
            Qm_1[0] = Qm[0];
            Qm_1[1] = -Qm[1];
            Qm_1[2] = -Qm[2];
            Qm_1[3] = -Qm[3];

            Qmn[0] = Qm_1[0] * Qn[0] - Qm_1[1] * Qn[1] - Qm_1[2] * Qn[2] - Qm_1[3] * Qn[3];
            Qmn[1] = Qm_1[0] * Qn[1] + Qm_1[1] * Qn[0] + Qm_1[2] * Qn[3] - Qm_1[3] * Qn[2];
            Qmn[2] = Qm_1[0] * Qn[2] - Qm_1[1] * Qn[3] + Qm_1[2] * Qn[0] + Qm_1[3] * Qn[1];
            Qmn[3] = Qm_1[0] * Qn[3] + Qm_1[1] * Qn[2] - Qm_1[2] * Qn[1] + Qm_1[3] * Qn[0];
            return Qmn;
        }

        //private float lastAngle,lastLastAngle,currentAngle;
        //private UInt16 count = 3;
        /// <summary>
        /// 将四元数转换为欧拉角
        /// </summary>
        /// <param name="Q">四元数</param>
        /// <returns>欧拉角</returns>
        private float[] quaternionToEulerAngle(float[] Q)
        {
            float[] EulerAngle = new float[3];
            //double TEST = Q[0] * Q[2] - Q[1] * Q[3];
            EulerAngle[0] = (float)(Math.Atan2(2 * (Q[0] * Q[1] + Q[2] * Q[3]), 1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2])) / Math.PI * 180);
            EulerAngle[2] = (float)(Math.Atan2(2 * (Q[0] * Q[3] + Q[1] * Q[2]), (1 - 2 * (Q[2] * Q[2] + Q[3] * Q[3]))) / Math.PI * 180);
            EulerAngle[1] = (float)(Math.Asin(2 * (Q[0] * Q[2] - Q[1] * Q[3])) / Math.PI * 180);
            return EulerAngle;
        }

        /// <summary>
        /// 将欧拉角转换为四元数   
        /// </summary>
        /// <param name="EulerAngle">欧拉角</param>
        /// <returns>四元数</returns>
        private float[] EulerAngleToQuaternion(float[] EulerAngle)
        {
            float[] Q = new float[4];

            float cos_e0 = (float)(Math.Cos(EulerAngle[0] / 180 * Math.PI / 2));
            float sin_e0 = (float)(Math.Sin(EulerAngle[0] / 180 * Math.PI / 2));

            float cos_e1 = (float)(Math.Cos(EulerAngle[1] / 180 * Math.PI / 2));
            float sin_e1 = (float)(Math.Sin(EulerAngle[1] / 180 * Math.PI / 2));

            float cos_e2 = (float)(Math.Cos(EulerAngle[2] / 180 * Math.PI / 2));
            float sin_e2 = (float)(Math.Sin(EulerAngle[2] / 180 * Math.PI / 2));

            Q[0] = cos_e0 * cos_e1 * cos_e2 + sin_e0 * sin_e1 * sin_e2;
            Q[1] = sin_e0 * cos_e1 * cos_e2 - cos_e0 * sin_e1 * sin_e2;
            Q[2] = cos_e0 * sin_e1 * cos_e2 + sin_e0 * cos_e1 * sin_e2;
            Q[3] = cos_e0 * cos_e1 * sin_e2 - sin_e0 * sin_e1 * cos_e2;

            return Q;
        }
        /// <summary>
        /// 获取舵机位置
        /// </summary>
        /// <param name="min">限位最小值</param>
        /// <param name="max">限位最大值</param>
        /// <param name="currentValue">当前数值</param>
        /// <returns>舵机位置数值</returns>
        private UInt16 GetServoPositonValue(UInt16 min, UInt16 max, UInt16 currentValue)
        {
            if (currentValue >= min && currentValue <= max)
            {
                return currentValue;
            }
            else if (currentValue < min)
            {
                return min;
            }
            else if (currentValue > max)
            {
                return max;
            }
            else
            {
                return 2000;
            }
        }
        /// <summary>
        /// 获取底盘速度
        /// </summary>
        /// <param name="min">速度最小值</param>
        /// <param name="max">速度最大值</param>
        /// <param name="invalidSpeed">无效速度值，其值必须小于速度最大值</param>
        /// <param name="currentSpeed">当前速度值</param>
        /// <returns>底盘速度</returns>
        private Int16 GetChassisSpeed(Int16 min, Int16 max, Int16 invalidSpeed, Int16 currentSpeed)
        {
            Int16 chassisSpeed = 0;
            if (currentSpeed > invalidSpeed)
            {
                chassisSpeed = (Int16)(currentSpeed - invalidSpeed);
            }
            else if (currentSpeed <= invalidSpeed)
            {
                chassisSpeed = min;
            }
            if (chassisSpeed >= max)
            {
                chassisSpeed = max;
            }
            return chassisSpeed;
        }
        /// <summary>
        /// 获取底盘转弯速度
        /// </summary>
        /// <param name="min">转弯速度最小值</param>
        /// <param name="max">转弯速度最大值</param>
        /// <param name="invalidTurnRate">无效转弯速度</param>
        /// <param name="currentTurnRate">当前转弯速度</param>
        /// <returns>转弯速度</returns>
        private Int16 GetChassisTurnRate(Int16 min, Int16 max, Int16 invalidTurnRate, Int16 currentTurnRate)
        {
            Int16 chassidTurnRate = 0;
            if (currentTurnRate <= invalidTurnRate && currentTurnRate >= -invalidTurnRate)
            {
                chassidTurnRate = 0;
            }
            else if (currentTurnRate > invalidTurnRate)
            {
                chassidTurnRate = (Int16)(currentTurnRate - invalidTurnRate);
            }
            else if (currentTurnRate < -invalidTurnRate)
            {
                chassidTurnRate = (Int16)(currentTurnRate - (-invalidTurnRate));
            }
            if (currentTurnRate >= max)
            {
                chassidTurnRate = max;
            }
            if (currentTurnRate <= -max)
            {
                chassidTurnRate = (Int16)(-max);
            }
            return chassidTurnRate;
        }
        /// <summary>
        /// 获取底盘角度
        /// </summary>
        /// <param name="invalidAngle">无效角度</param>
        /// <param name="currentAngle">当前角度</param>
        /// <returns>底盘角度值</returns>
        private Int16 GetChassisAngle(Int16 invalidAngle, Int16 currentAngle)
        {
            Int16 chassisAngle = 0;
            if (currentAngle < -invalidAngle)//&& correctedEulerAngle_m4[1] > angleThresholdMin && correctedEulerAngle_m4[1] < angleThresholdMax)
            {
                chassisAngle = 0;

            }
            else if (currentAngle > invalidAngle) //&& correctedEulerAngle_m4[1] > angleThresholdMin && correctedEulerAngle_m4[1] < angleThresholdMax)
            {
                chassisAngle = 180;
            }
            else
            {
                chassisAngle = 0;
            }
            return chassisAngle;
        }

        private float[] correctedEulerAngle_r0 = new float[3];
        private float[] correctedEulerAngle_r1 = new float[3];
        private float[] correctedEulerAngle_r01 = new float[3];
        private float[] correctedEulerAngle_l2 = new float[3];
        private float[] correctedEulerAngle_l3 = new float[3];
        private float[] correctedEulerAngle_l23 = new float[3];
        private float[] correctedEulerAngle_m4 = new float[3];
        private float[] correctedEulerAngle_m4r0 = new float[3];
        private float[] correctedEulerAngle_m4l2 = new float[3];
        private float[] correctedEularAngle_chassis = new float[3];
        private UInt16 servo34Position = 2048;
        private UInt16 servo33Position = 2048;
        private UInt16 servo32Position = 2048;
        private UInt16 servo31Position = 2048;
        private UInt16 servo4Position = 2048;
        private UInt16 servo3Position = 2048;
        private UInt16 servo2Position = 2048;
        private UInt16 servo1Position = 2048;

        private UInt16 servo34PositionMin = 1;
        private UInt16 servo33PositionMin = 600;
        private UInt16 servo32PositionMin = 1;
        private UInt16 servo31PositionMin = 600;
        private UInt16 servo4PositionMin = 1;
        private UInt16 servo3PositionMin = 600;
        private UInt16 servo2PositionMin = 1;
        private UInt16 servo1PositionMin = 600;

        private UInt16 servo34PositionMax = 4000;
        private UInt16 servo33PositionMax = 4000;
        private UInt16 servo32PositionMax = 4000;
        private UInt16 servo31PositionMax = 4000;
        private UInt16 servo4PositionMax = 4000;
        private UInt16 servo3PositionMax = 4000;
        private UInt16 servo2PositionMax = 4000;
        private UInt16 servo1PositionMax = 4000;

        private Int16 _chassisAngle = 0;
        private Int16 _chassisSpeed = 0;
        private Int16 _chassisTurnRate = 0;

        private float[] initQr0 = new float[4];
        private float[] initQr1 = new float[4];
        private float[] initQl2 = new float[4];
        private float[] initQl3 = new float[4];
        private float[] initQm4 = new float[4];
        private float[] initQ5 = new float[4];
        private float[] initQr01 = new float[4];
        private float[] initQl23 = new float[4];
        private float[] initQm4r0 = new float[4];
        private float[] initQm4l2 = new float[4];

        private float[] currentQr0 = new float[4];
        private float[] currentQr1 = new float[4];
        private float[] currentQl2 = new float[4];
        private float[] currentQl3 = new float[4];
        private float[] currentQm4 = new float[4];
        private float[] currentQ5 = new float[4];
        private float[] currentQr01 = new float[4];
        private float[] currentQl23 = new float[4];
        private float[] currentQm4r0 = new float[4];
        private float[] currentQm4l2 = new float[4];

        private Int16[] currentAcc0 = new Int16[3];
        private Int16[] currentAcc1 = new Int16[3];

        private float[] Qr0 = new float[4];
        private float[] Qr1 = new float[4];
        private float[] Ql2 = new float[4];
        private float[] Ql3 = new float[4];
        private float[] Qm4 = new float[4];
        private float[] Q5 = new float[4];
        private float[] Qr01 = new float[4];
        private float[] Ql23 = new float[4];
        private float[] Qm4r0 = new float[4];
        private float[] Qm4l2 = new float[4];
        private float[] Qm4m4init = new float[4];
        private float[] Q5_5init = new float[4];
        /// <summary> 
        /// 角度校正
        /// </summary>
        private void angle_Correct()
        {

            if (imuData.RFQuat != null)
            {
                for (int i = 0; i < 4; i++)
                {
                    initQr0[i] = imuData.RFQuat[0, i];
                    initQr1[i] = imuData.RFQuat[1, i];
                    initQl2[i] = imuData.RFQuat[2, i];
                    initQl3[i] = imuData.RFQuat[3, i];
                    initQm4[i] = imuData.RFQuat[4, i];
                    initQ5[i] = imuData.RFQuat[5, i];
                }
                initQr01 = getQmn(initQr0, initQr1);
                initQl23 = getQmn(initQl2, initQl3);
                initQm4r0 = getQmn(initQm4, initQr0);
                initQm4l2 = getQmn(initQm4, initQl2);
            }
        }

        private UInt16 motionNum = 0;
        private UInt16 motion1 = 0;
        private UInt16 motion2 = 0;
        // private UInt16 motion3 = 0;
        // private UInt16 motion4 = 0;

        private UInt16 getMinValue(UInt16 value, UInt16 range)
        {
            Int16 result = (Int16)(value - range);
            if (result < 0)
            {
                return 0;
            }
            else
            {
                return (UInt16)result;
            }
        }
        private UInt16 getMaxValue(UInt16 value, UInt16 range)
        {
            UInt16 result = (UInt16)(value + range);
            if (result > 4096)
            {
                return 4096;
            }
            else
            {
                return result;
            }
        }

        /// <summary>
        /// 判断手臂是否到达指定空间位置。
        /// </summary>
        /// <param name="pos1">舵机1实时目标位置</param>
        /// <param name="pos2">舵机1实时目标位置</param>
        /// <param name="pos3">舵机1实时目标位置</param>
        /// <param name="pos4">舵机4实时目标位置</param>
        /// <param name="tPos1">舵机1记录位置</param>
        /// <param name="tPos2">舵机2记录位置</param>
        /// <param name="tPos3">舵机3记录位置</param>
        /// <param name="tPos4">舵机4记录位置</param>
        /// <param name="range">舵机位置范围</param>
        /// <returns></returns>
        /// 

        private UInt16 getSkillNum(UInt16 pos1, UInt16 pos2, UInt16 pos3, UInt16 pos4, UInt16 tPos1, UInt16 tPos2, UInt16 tPos3, UInt16 tPos4, UInt16 range)
        {

            if (pos1 > getMinValue(tPos1, range) && pos1 < getMaxValue(tPos1, range)
                && pos2 > getMinValue(tPos2, range) && pos2 < getMaxValue(tPos2, range)
                && pos3 > getMinValue(tPos3, range) && pos3 < getMaxValue(tPos3, range)
                && pos4 > getMinValue(tPos4, range) && pos4 < getMaxValue(tPos4, range)
                )
            {
                return 1;
            }
            else
            {
                return 0;
            }

        }
        private float K = 4.0f;
        private float M = 2.0f;
        private float N = 1.0f;
        private Int16 speedMax = 50;
        private Int16 turnRateMax = 200;
        private Int16 angleDeadRange = 5;
        private Int16 speedDeadRange = 10;
        private Int16 turnRateDeadRange = 20;
        private Boolean isStart = false;

        private void timerDataProcessing_Tick(object sender, EventArgs e)
        {
            
            float[] Qm4_corrected = new float[4];

            if (imuData.RFQuat != null)
            {
                for (int i = 0; i < 4; i++)
                {
                    currentQr0[i] = imuData.RFQuat[0, i];
                    currentQr1[i] = imuData.RFQuat[1, i];
                    currentQl2[i] = imuData.RFQuat[2, i];
                    currentQl3[i] = imuData.RFQuat[3, i];
                    currentQm4[i] = imuData.RFQuat[4, i];
                    currentQ5[i] = imuData.RFQuat[5, i];
                }


                currentQr01 = getQmn(currentQr0, currentQr1);
                currentQl23 = getQmn(currentQl2, currentQl3);
                currentQm4r0 = getQmn(currentQm4, currentQr0);
                currentQm4l2 = getQmn(currentQm4, currentQl2);

                Qr01 = getQmn(currentQr01, initQr01);
                Ql23 = getQmn(currentQl23, initQl23);
                Qm4r0 = getQmn(currentQm4r0, initQm4r0);
                Qm4l2 = getQmn(currentQm4l2, initQm4l2);
                Qm4m4init = getQmn(currentQm4, initQm4);
                Q5_5init = getQmn(currentQ5, initQ5);

                correctedEulerAngle_r01 = quaternionToEulerAngle(Qr01);
                correctedEulerAngle_l23 = quaternionToEulerAngle(Ql23);
                correctedEulerAngle_m4r0 = quaternionToEulerAngle(Qm4r0);
                correctedEulerAngle_m4l2 = quaternionToEulerAngle(Qm4l2);
                //correctedEularAngle_chassis = quaternionToEulerAngle(Qm4m4init);
                correctedEularAngle_chassis = quaternionToEulerAngle(Q5_5init);

                N = (float)numericUpDownAngleN.Value;
                M = (float)numericUpDownSpeedM.Value;
                K = (float)numericUpDownTurnRateK.Value;

                speedMax = (Int16)numericUpDownSpeedMax.Value;
                turnRateMax = (Int16)numericUpDownTurnRateMax.Value;
                angleDeadRange = (Int16)numericUpDownAngleDeadRange.Value;
                speedDeadRange = (Int16)numericUpDownSpeedDeadRange.Value;
                turnRateDeadRange = (Int16)numericUpDownTurnRateDeadRange.Value;



                Int16 angleTemp = (Int16)(correctedEularAngle_chassis[1] * N);
                Int16 speedTemp = (Int16)((Math.Sqrt(correctedEularAngle_chassis[1] * correctedEularAngle_chassis[1])) * M);// + correctedEulerAngle_m4[1] * correctedEulerAngle_m4[1]
                Int16 turnRateTemp = (Int16)((correctedEularAngle_chassis[2]) * K * (-1));

                if (isStart)
                {

                    //底盘运行参数输入范围：角度0-360、速度0-100、和旋转速度-1000-1000
                    _chassisAngle = GetChassisAngle(angleDeadRange, angleTemp);
                    _chassisSpeed = GetChassisSpeed(0, speedMax, speedDeadRange, speedTemp);
                    _chassisTurnRate = GetChassisTurnRate(0, turnRateMax, turnRateDeadRange, turnRateTemp);

                    servo4Position = (UInt16)(correctedEulerAngle_m4l2[0] * 4096 / 280 * (-1) + 2048);
                    servo3Position = (UInt16)(correctedEulerAngle_m4l2[2] * 4096 / 280 * (-1) + 2048);
                    servo2Position = (UInt16)(correctedEulerAngle_l23[0] * 4096 / 250 * (-1) + 2048);
                    servo1Position = (UInt16)(correctedEulerAngle_l23[1] * 4096 / 250 + 2048);

                    servo34Position = (UInt16)(correctedEulerAngle_m4r0[0] * 4096 / 280 * (-1) + 2048);//14.628
                    servo33Position = (UInt16)(correctedEulerAngle_m4r0[2] * 4096 / 280 * (-1) + 2048);//
                    servo32Position = (UInt16)(correctedEulerAngle_r01[0] * 4096 / 250 * (-1) + 2048);
                    servo31Position = (UInt16)(correctedEulerAngle_r01[1] * 4096 / 250 * (-1) + 2048);//16.384

                    motion1 = getSkillNum(servo31Position, servo32Position, servo33Position, servo34Position, 1829, 850, 2121, 1475, 500);
                    motion2 = getSkillNum(servo31Position, servo32Position, servo33Position, servo34Position, 2090, 2120, 3405, 1567, 500);

                    //防止同时触发多个技能
                    if (motion1 == 1)
                    {
                        motionNum = 1;
                    }
                    else if (motion2 == 1)
                    {
                        motionNum = 2;
                    } 
                    else
                    {
                        motionNum = 0;
                    }
                    labelSkill.Text = motionNum.ToString();

                    servo4Position = GetServoPositonValue(servo4PositionMin, servo4PositionMax, servo4Position);
                    servo3Position = GetServoPositonValue(servo3PositionMin, servo3PositionMax, servo3Position);
                    servo2Position = GetServoPositonValue(servo2PositionMin, servo2PositionMax, servo2Position);
                    servo1Position = GetServoPositonValue(servo1PositionMin, servo1PositionMax, servo1Position);

                    servo34Position = GetServoPositonValue(servo34PositionMin, servo34PositionMax, servo34Position);
                    servo33Position = GetServoPositonValue(servo33PositionMin, servo33PositionMax, servo33Position);
                    servo32Position = GetServoPositonValue(servo32PositionMin, servo32PositionMax, servo32Position);
                    servo31Position = GetServoPositonValue(servo31PositionMin, servo31PositionMax, servo31Position);

                    //此处判断是否投影模式
                    if (radioButton1.Checked)
                    {
                        //底盘移动+舵机移动
                        WisdomService.getInstance().sendServoPosition(servo1Position,
                            servo2Position, servo3Position, servo4Position,
                            0, 0, servo31Position, servo32Position, servo33Position, servo34Position,
                            20, _chassisAngle, _chassisSpeed, _chassisTurnRate);
                    }
                    else {
                        WisdomService.getInstance().sendChassisMove(_chassisAngle, _chassisSpeed, _chassisTurnRate);
                        if (motionNum != 0) {
                            WisdomService.getInstance().sendSkillNum((short)motionNum);
                        }
                    }

                }
            }
        }
        UInt16 countDown = 3;
        private Boolean buttonStartStopIsStart = false;
        private void buttonStartStop_Click(object sender, EventArgs e)
        {
            
            if (spSerialPort.IsOpen)
            {
                if (imuData.RFQuat != null)
                {
                    if (buttonStartStopIsStart == false)
                    {
                        timerCountDown.Enabled = true;
                        buttonStartStopIsStart = true;
                        buttonStartStop.Text = "停止";
                        countDown = 3;
                    }
                    else
                    {
                        buttonStartStopIsStart = false;
                        isStart = false;
                        timerCountDown.Enabled = false;
                        buttonStartStop.Text = "开始";
                        labelCountDown.Text = "停止";
                        _chassisAngle = 0;
                        _chassisSpeed = 0;
                        _chassisTurnRate = 0;
                    }
                }
                else
                {
                    MessageBox.Show("无数据，请检查设备是否正确！", "错误");
                }

            }
            else
            {
                MessageBox.Show("端口未打开，请检查端口！", "错误");
            }
            
        }

        private void timerCountDown_Tick(object sender, EventArgs e)
        {
            
            if (countDown != 0)
            {
                labelCountDown.Text = "倒计时：" + countDown.ToString() + "s";
                countDown--;
                isStart = false;
            }
            else
            {
                angle_Correct();
                timerCountDown.Enabled = false;
                labelCountDown.Text = "开始";
                isStart = true; 
            }
            if (countDown == 1)
            {
                Reset();
            }
            
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            System.Environment.Exit(0);
        }

        private void btn_connect_Click(object sender, EventArgs e)
        {
            if (btn_connect.Text.Equals("连接"))
            {
                string ip = tb_ip.Text.ToString().Trim();
                string team = tb_team_name.Text.ToString().Trim();
                if (String.IsNullOrEmpty(ip))
                {
                    MessageBox.Show("请输入ip地址");
                    return;
                }
                if (String.IsNullOrEmpty(team))
                {
                    MessageBox.Show("请输入队伍名称");
                    return;
                }
                WisdomService.getInstance().CreateService(ip, team);
                WisdomService.getInstance().setConnectionCallback(connectionCallback);
            }
            else {
                WisdomService.getInstance().CloseService();
            }
            

        }

        /// <summary>
        /// 控制系统连接状态回调
        /// </summary>
        /// <param name="state"></param>
        private void connectionCallback(int state)
        {
            if (state == 0)
            {
                this.Invoke(new EventHandler(delegate
                {
                    btn_connect.Text = "连接";

                }));
            }
            else
            {
                this.Invoke(new EventHandler(delegate
                {
                    btn_connect.Text = "断开连接";

                }));
            }
        }

        private void btn_single_Click(object sender, EventArgs e)
        {
            WisdomService.getInstance().setChassicMode(0);
        }

        private void btn_double_Click(object sender, EventArgs e)
        {
            WisdomService.getInstance().setChassicMode(2);
        }
    }
}
