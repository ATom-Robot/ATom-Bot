using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UIWidgets;

public class RobotController : MonoBehaviour
{
    public float slerpRatio = 0.5f;

    public Transform armPitch;
    public Transform head;
    public Transform body;

    public float targetAngleArmPitch;
    public float targetAngleHead;
    public float targetAngleBody;

    // 滑条
    public CenteredSlider sliderAngleArmPitch;
    public CenteredSlider sliderAngleHead;
    public CenteredSlider sliderAngleBody;

    public bool isPlaying = false;

    // Start is called before the first frame update
    void Start()
    {
        targetAngleArmPitch = 0;
        targetAngleHead = 0;
        targetAngleBody = 0;
    }

    // Update is called once per frame
    void Update()
    {
        // armPitch.localRotation = Quaternion.Slerp(armPitch.localRotation,
        //     Quaternion.Euler(targetAngleArmPitch, 0, 0), slerpRatio);
        body.localRotation = Quaternion.Slerp(body.localRotation,
            Quaternion.Euler(0, targetAngleBody, 0), slerpRatio);
        head.localRotation = Quaternion.Slerp(head.localRotation,
            Quaternion.Euler(0, targetAngleHead, 0), slerpRatio);
    }

    public void SetAngleArmPitch(int _val)
    {
        targetAngleArmPitch = _val;
    }

    public void SetAngleBody(int _val)
    {
        targetAngleBody = _val;
    }

    public void SetAngleHead(int _val)
    {
        targetAngleHead = _val;
    }

}
