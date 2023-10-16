using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using System.IO.Ports;
using System;
using System.Text;
using System.Threading;
using UIWidgets;

public class SerialService : MonoBehaviour
{
    public Dropdown dropDown;
    public string[] serial_portList;
    SerialPort sp = null;
    Thread rx_thread;
    public int serial_baudRate = 115200;//波特率
    public GameObject robot;
    public int armPitch;

    // Start is called before the first frame update
    void Start()
    {
        ScanPorts_API();
        // 设置下拉列表监听
        SetDropDownAddListener(OnValueChange);
    }

    // Update is called once per frame
    void Update()
    {
        var rc = robot.GetComponent<RobotController>();
        rc.targetAngleArmPitch = -armPitch;
        rc.sliderAngleArmPitch.Value = armPitch;
    }

    void SetDropDownAddListener(UnityAction<int> OnValueChangeListener)
    {
        //当点击后值改变是触发 (切换下拉选项)
        dropDown.onValueChanged.AddListener((value) =>
        {
            OnValueChangeListener(value);
        });
    }

    void OnValueChange(int index)
    {
        Debug.Log("select serial port:" + serial_portList[index - 1]);
        OpenSerialPort(serial_portList[index - 1], serial_baudRate, Parity.None, 8, StopBits.One);
    }

    // <summary>
    // 添加一个列表下拉数据
    // <param name="listOptions"></param>
    void AddDropDownOptionsData(string itemText)
    {
        //添加一个下拉选项
        Dropdown.OptionData data = new Dropdown.OptionData();
        data.text = itemText;
        //data.image = "指定一个图片做背景不指定则使用默认"；
        dropDown.options.Add(data);
    }

    //扫描串口
    private void ScanPorts_API()
    {
        serial_portList = SerialPort.GetPortNames();

        for (int i = 0; i < serial_portList.Length; i++)
        {
            Debug.Log("COM:" + serial_portList[i]);
            AddDropDownOptionsData(serial_portList[i]);
        }
    }

    // 打开串口
    // <param name="_portName">端口号</param>
    // <param name="_baudRate">波特率</param>
    // <param name="_parity">校验位</param>
    // <param name="dataBits">数据位</param>
    // <param name="_stopbits">停止位</param>
    private void OpenSerialPort(string _portName, int _baudRate, Parity _parity, int dataBits, StopBits _stopbits)
    {
        try
        {
            Debug.Log("serial port:" + _portName);
            sp = new SerialPort(_portName, _baudRate, _parity, dataBits, _stopbits);//绑定端口
            sp.Open();
            rx_thread = new Thread(DataReceiveFunction);
            rx_thread.Start();
        }
        catch (Exception ex)
        {
            sp = new SerialPort();
            Debug.Log(ex);
        }
    }

    //接收数据
    void DataReceiveFunction()
    {
        while (sp != null && sp.IsOpen)
        {
            try
            {
                RecAndProcessingFunction();
            }
            catch (Exception) { }

            Thread.Sleep(5);
        }
    }

    private void RecAndProcessingFunction()
    {
        int retLen = sp.BytesToRead;
        byte[] buffer = new byte[retLen];

        sp.Read(buffer, 0, retLen);

        if (buffer[0] == 0xAA && buffer[1] == 0xFF)
        {
            int sum = 0;
            int size = buffer[3];
            for (int i = 0; i < size + 4; i++)
            {
                sum += buffer[i];
            }
            int sum_l = sum & 0xFF;

            if (sum_l == buffer[size + 4])
            {
                armPitch = BitConverter.ToInt32(buffer, 8);
                Debug.Log(buffer[4] + " " + armPitch);
            }
        }
    }

    // 关闭串口
    private void CloseSerialPort()
    {
        if (sp.IsOpen)
        {
            Debug.Log("关闭串口");
            sp.Close();
        }
    }

    private void OnDestroy()
    {
        CloseSerialPort();
        rx_thread.Abort();
    }
}
