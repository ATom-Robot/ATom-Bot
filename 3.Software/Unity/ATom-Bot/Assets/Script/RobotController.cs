using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UIWidgets;

public class RobotController : MonoBehaviour
{
    public float slerpRatio = 0.5f;

    public Transform armPitch;
    public Transform armPitch_d;
    public Transform Hanger;
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
        targetAngleHead = 90;
        targetAngleBody = 0;
    }

    // Update is called once per frame
    void Update()
    {
        armPitch.localRotation = Quaternion.Slerp(armPitch.localRotation,
            Quaternion.Euler(0, targetAngleArmPitch, 0), slerpRatio);

        armPitch_d.localRotation = Quaternion.Slerp(armPitch_d.localRotation,
            Quaternion.Euler(0, targetAngleArmPitch, 0), slerpRatio);

        Hanger.localRotation = Quaternion.Slerp(Hanger.localRotation,
            Quaternion.Euler(0, Mathf.Abs(targetAngleArmPitch), 0), slerpRatio);

        head.parent.localRotation = Quaternion.Slerp(head.parent.localRotation,
            Quaternion.Euler(0, targetAngleHead, -90), slerpRatio);
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
