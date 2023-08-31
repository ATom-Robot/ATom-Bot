using System.IO;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
class Action
{
    public float arm_angle;
    public float head_angle;
}

[System.Serializable]
class Action_data
{
    public List<Action> action_list;
}
public class ActionEditor : MonoBehaviour
{
    public GameObject robot;

    Action action = new Action();
    Action_data action_Data = new Action_data();

    void Start()
    {
        action_Data.action_list = new List<Action>();
    }

    void update()
    {
        RobotController rc = robot.GetComponent<RobotController>();

        action.head_angle = rc.targetAngleHead;
        action.arm_angle = rc.targetAngleArmPitch;
    }

    public void write_json_data()
    {
        for(int i = 0; i < 5; i++)
        {
            Action action = new Action();
            action.arm_angle = i;
            action.head_angle = i + 1;
            action_Data.action_list.Add(action);
        }

        string js = JsonUtility.ToJson(action_Data);
        // Debug.Log(js);
        string fileUrl = Application.streamingAssetsPath + "\\pose_action.json";

        using (StreamWriter sw =new StreamWriter(fileUrl))
        {
            //保存数据
            sw.WriteLine(js);
            // 关闭文档
            sw.Close();
            sw.Dispose();
        }
    }
}