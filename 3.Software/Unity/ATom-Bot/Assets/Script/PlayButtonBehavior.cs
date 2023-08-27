using UnityEngine;
using UnityEngine.UI;
using System;
using UnityEngine.UI.ProceduralImage;
using UnityEngine.UIElements;
using Image = UnityEngine.UI.Image;

public class PlayButtonBehavior : MonoBehaviour
{
    public Sprite imgPlay;
    public Sprite imgPause;
    public Scrollbar timelineSb;
    public GameObject timelineFrameManager;
    public GameObject robot;
    public int deltaTime = 0;
    public DateTime StartTime;

    public bool isRecording = false;


    // Start is called before the first frame update
    void Start()
    {
        StartTime = DateTime.Now;
    }

    // Update is called once per frame
    void Update()
    {
       
    }

    // public void OnClick()
    // {
    //     isPlaying = !isPlaying;
    //     RobotController rc = robot.GetComponent<RobotController>();
    //     rc.isPlaying = isPlaying;

    //     if (isPlaying)
    //     {
    //         transform.Find("Icon").GetComponent<Image>().sprite = imgPause;
    //         timelineSb.GetComponent<Scrollbar>().value = 0;
    //     }
    //     else
    //     {
    //         transform.Find("Icon").GetComponent<Image>().sprite = imgPlay;
    //     }
    // }
    private void recordloop()
    {
        if(isRecording)
        {
            Debug.Log(Time.time + "  " +isRecording);
        }
    }
    private void recordfinish()
    {
        Debug.Log("录制结束");
        CancelInvoke();
    }
    public void Recorded()
    {
        Debug.Log("开始录制动作,时长:" + deltaTime + "s");
        isRecording = true;

        InvokeRepeating(nameof(recordloop), 0, 0.1f);
        InvokeRepeating(nameof(recordfinish), deltaTime, 0.0f);
    }
    public void RecordedStop()
    {
        Debug.Log("暂停录制动作");
        isRecording = false;
        CancelInvoke();
    }

    public void OnDeltaTimeChanged(float _val)
    {
        Debug.Log(_val);
    }
}