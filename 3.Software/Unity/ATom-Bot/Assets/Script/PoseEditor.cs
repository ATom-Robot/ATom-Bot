using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

public class PoseEditor : MonoBehaviour
{
    public GameObject framePrefab;
    public GameObject robot;
    [FormerlySerializedAs("timeline")] public List<GameObject> timelineFrames;

    // Start is called before the first frame update
    void Start()
    {
        GameObject frame0 = Instantiate(framePrefab, transform, true);
        SetupFrame(frame0, 0);
        frame0.transform.Find("FrameAdd").Find("Text (TMP)").GetComponent<TMP_Text>().text = "+";
        timelineFrames.Add(frame0);
        SetFrameCapture(0);
    }

    // Update is called once per frame
    void Update()
    {

    }
    public void ModifyFrameCallback(int _id)
    {
        SetFrameCapture(_id);
        Debug.Log("ModifyFrameCallback:" + _id);
    }

    private void SetFrameCapture(int _id)
    {
        GameObject frame = timelineFrames[_id];

        // frame.transform.Find("FrameView").GetComponent<RawImage>().texture =
        //     CaptureCamera(frame.GetComponent<FrameMeta>().renderTexture);

        frame.GetComponent<FrameMeta>().targetAngleBody =
            robot.GetComponent<RobotController>().targetAngleBody;
        frame.GetComponent<FrameMeta>().targetAngleHead =
            robot.GetComponent<RobotController>().targetAngleHead;
        frame.GetComponent<FrameMeta>().targetAngleArmPitch =
            robot.GetComponent<RobotController>().targetAngleArmPitch;
        frame.GetComponent<FrameMeta>().targetAngleArmPitch =
            robot.GetComponent<RobotController>().targetAngleArmPitch;

        Debug.Log("SetFrameCapture:" + robot.GetComponent<RobotController>().targetAngleHead);
    }
    public void AddFrameCallback(int _id)
    {
        GameObject frame = Instantiate(framePrefab, transform, true);
        _id += 1;
        SetupFrame(frame, _id);
        timelineFrames.Insert(frame.GetComponent<FrameMeta>().id, frame);

        if (frame.GetComponent<FrameMeta>().id < timelineFrames.Count - 1)
            for (int i = frame.GetComponent<FrameMeta>().id + 1; i < timelineFrames.Count; i++)
            {
                GameObject f = timelineFrames[i];
                SetupFrame(f, i);
            }

        GetComponent<RectTransform>().sizeDelta = new Vector2(820 + 160 * timelineFrames.Count, 140.0f);
    }

    public void SelectFrameCallback(int _id)
    {
        GameObject frame = timelineFrames[_id];
        var rc = robot.GetComponent<RobotController>();

        rc.targetAngleBody = frame.GetComponent<FrameMeta>().targetAngleBody;
        rc.targetAngleHead = frame.GetComponent<FrameMeta>().targetAngleHead;
        rc.targetAngleArmPitch = frame.GetComponent<FrameMeta>().targetAngleArmPitch;
        rc.sliderAngleBody.Value = (int)rc.targetAngleBody;
        rc.sliderAngleHead.Value = (int)rc.targetAngleHead;
        rc.sliderAngleArmPitch.Value = (int)rc.targetAngleArmPitch;
        Debug.Log("rc.targetAngleHead:" + rc.targetAngleHead);
    }
    public void DeleteFrameCallback(int _id)
    {
        if (_id > 0)
        {
            Destroy(timelineFrames[_id]);
            timelineFrames.RemoveAt(_id);

            for (int i = _id; i < timelineFrames.Count; i++)
            {
                GameObject f = timelineFrames[i];
                SetupFrame(f, i);
            }

            GetComponent<RectTransform>().sizeDelta = new Vector2(820 + 160 * timelineFrames.Count, 140.0f);
        }
    }

    public void FilePathCallback(int _id)
    {
        var openFileName = new OpenFileName();
        openFileName.structSize = Marshal.SizeOf(openFileName);
        openFileName.filter = "文件(*.jpg;*.png;*.bmp;*.mp4;)\0*.jpg;*.png;*.bmp;*.mp4";
        openFileName.file = new string(new char[256]);
        openFileName.maxFile = openFileName.file.Length;
        openFileName.fileTitle = new string(new char[64]);
        openFileName.maxFileTitle = openFileName.fileTitle.Length;
        openFileName.initialDir = Application.streamingAssetsPath.Replace('/', '\\');
        openFileName.title = "选择文件";
        openFileName.flags = 0x00080000 | 0x00001000 | 0x00000800 | 0x00000008;

        if (LocalDialog.GetSaveFileName(openFileName))
        {
            Debug.Log(openFileName.file);
            GameObject frame = timelineFrames[_id];
            frame.GetComponent<FrameMeta>().filePath = openFileName.file;
            var splitFilePath = openFileName.file.Split('\\');
            frame.transform.Find("FilePath").Find("Text").GetComponent<Text>().text =
                splitFilePath[splitFilePath.Length - 1];
        }
    }
    private void SetupFrame(GameObject frame, int _id)
    {
        frame.GetComponent<FrameMeta>().id = _id;
        frame.name = "Frame_" + _id;
        frame.transform.GetComponent<RectTransform>().anchoredPosition3D =
            new Vector3(91 + 160 * _id, 0, 0);
        frame.transform.Find("FrameAdd").GetComponent<Button>().onClick.RemoveAllListeners();
        frame.transform.Find("FrameAdd").GetComponent<Button>().onClick
            .AddListener(() => AddFrameCallback(frame.GetComponent<FrameMeta>().id));
        frame.transform.Find("FrameView").Find("Capture").GetComponent<Button>().onClick.RemoveAllListeners();
        frame.transform.Find("FrameView").Find("Capture").GetComponent<Button>().onClick
            .AddListener(() => ModifyFrameCallback(frame.GetComponent<FrameMeta>().id));
        frame.transform.Find("FrameView").GetComponent<Button>().onClick.RemoveAllListeners();
        frame.transform.Find("FrameView").GetComponent<Button>().onClick
            .AddListener(() => SelectFrameCallback(frame.GetComponent<FrameMeta>().id));
        frame.transform.Find("FrameDelete").GetComponent<Button>().onClick.RemoveAllListeners();
        frame.transform.Find("FrameDelete").GetComponent<Button>().onClick
            .AddListener(() => DeleteFrameCallback(frame.GetComponent<FrameMeta>().id));
        frame.transform.Find("FilePath").GetComponent<Button>().onClick.RemoveAllListeners();
        frame.transform.Find("FilePath").GetComponent<Button>().onClick
            .AddListener(() => FilePathCallback(frame.GetComponent<FrameMeta>().id));

        frame.transform.Find("FrameAdd").Find("Text (TMP)").GetComponent<TMP_Text>().text = "" + _id;
    }
}

[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto)]
public class OpenFileName
{
    public int structSize = 0;
    public IntPtr dlgOwner = IntPtr.Zero;
    public IntPtr instance = IntPtr.Zero;
    public String filter = null;
    public String customFilter = null;
    public int maxCustFilter = 0;
    public int filterIndex = 0;
    public String file = null;
    public int maxFile = 0;
    public String fileTitle = null;
    public int maxFileTitle = 0;
    public String initialDir = null;
    public String title = null;
    public int flags = 0;
    public short fileOffset = 0;
    public short fileExtension = 0;
    public String defExt = null;
    public IntPtr custData = IntPtr.Zero;
    public IntPtr hook = IntPtr.Zero;
    public String templateName = null;
    public IntPtr reservedPtr = IntPtr.Zero;
    public int reservedInt = 0;
    public int flagsEx = 0;
}
public class LocalDialog
{
    [DllImport("Comdlg32.dll", SetLastError = true, ThrowOnUnmappableChar = true, CharSet = CharSet.Auto)]
    public static extern bool GetOpenFileName([In, Out] OpenFileName ofn);

    public static bool GetOFN([In, Out] OpenFileName ofn)
    {
        return GetOpenFileName(ofn); //执行打开文件的操作
    }

    [DllImport("Comdlg32.dll", SetLastError = true, ThrowOnUnmappableChar = true, CharSet = CharSet.Auto)]
    public static extern bool GetSaveFileName([In, Out] OpenFileName ofn);

    public static bool GetSFN([In, Out] OpenFileName ofn)
    {
        return GetSaveFileName(ofn); //执行保存选中文件的操作
    }
}