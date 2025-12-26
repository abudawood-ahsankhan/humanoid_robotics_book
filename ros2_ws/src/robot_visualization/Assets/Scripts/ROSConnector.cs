using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Ros2UnityEx;

public class ROSConnector : MonoBehaviour
{
    public string rosIp = "127.0.0.1";
    public int rosPort = 10000;
    public string rosTopic = "/unity_visualization/robot_state";

    private ROS2UnityComponent ros2U;
    private bool rosConnected = false;

    // Start is called before the first frame update
    void Start()
    {
        ros2U = GetComponent<ROS2UnityComponent>();
        if (ros2U != null)
        {
            ros2U.ROS2UnitySettings.ROS2ServerURL = rosIp;
            ros2U.ROS2UnitySettings.ROS2ServerPort = rosPort;
            ros2U.Connect();
            rosConnected = true;
        }
    }

    // Update is called once per frame
    void Update()
    {

    }

    public bool IsConnected()
    {
        return rosConnected && ros2U != null && ros2U.Ok();
    }
}