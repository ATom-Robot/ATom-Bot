using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using System.IO.Ports;
using System;
using System.Text;
using System.Threading;

public class SerialService : MonoBehaviour
{
    public Dropdown dropDown;

    public string[] serial_portList;

    SerialPort sp = null;

    public int serial_baudRate = 115000;//波特率

    string str;

    char[] strchar = new char[128];

    public List<byte> listReceive = new List<byte>();

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
        
    }

    void SetDropDownAddListener(UnityAction<int> OnValueChangeListener)
    {
        //当点击后值改变是触发 (切换下拉选项)
        dropDown.onValueChanged.AddListener((value) => {
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
            Thread thread = new Thread(new ThreadStart(DataReceiveFunction));
            thread.Start();
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
        byte[] buffer = new byte[1024];
        int bytes = 0;
        while (true)
        {
            if (sp != null && sp.IsOpen)
            {
                try
                {
                    bytes = sp.Read(buffer, 0, buffer.Length);
                    if (bytes == 0)
                    {
                        continue;
                    }
                    else
                    {
                        string strbytes = Encoding.Default.GetString(buffer);
                        //接受的数据
                        Debug.Log(strbytes);
                    }
                }
                catch (Exception ex)
                {
                    if (ex.GetType() != typeof(ThreadAbortException))
                    {
                    }
                }
            }
            Thread.Sleep(10);
        }
    }

    //打印数据
    void PrintData()
    {
        for (int i = 0; i < listReceive.Count; i++)
        {
            strchar[i] = (char)(listReceive[i]);
            str = new string(strchar);
        }
        Debug.Log(str);
    }

    // 关闭串口
    private void CloseSerialPort()
    {
        sp.Close();
    }
}
