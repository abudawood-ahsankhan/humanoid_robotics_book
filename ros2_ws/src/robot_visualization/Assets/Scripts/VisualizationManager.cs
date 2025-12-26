using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VisualizationManager : MonoBehaviour
{
    public RobotController robotController;
    public ROSConnector rosConnector;

    // Start is called before the first frame update
    void Start()
    {
        if (robotController == null)
        {
            robotController = FindObjectOfType<RobotController>();
        }

        if (rosConnector == null)
        {
            rosConnector = FindObjectOfType<ROSConnector>();
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (rosConnector != null && rosConnector.IsConnected())
        {
            // Handle incoming ROS messages and update robot visualization
            UpdateRobotVisualization();
        }
    }

    void UpdateRobotVisualization()
    {
        // This would typically receive joint states from ROS and update the robot model
        // For now, we'll simulate with some test values
        Dictionary<string, float> testJointPositions = new Dictionary<string, float>
        {
            {"left_shoulder_pitch", 15f},
            {"left_elbow", 30f},
            {"right_shoulder_pitch", -15f},
            {"right_elbow", 25f},
            {"left_hip_pitch", 5f},
            {"left_knee", 10f},
            {"right_hip_pitch", -5f},
            {"right_knee", 12f}
        };

        if (robotController != null)
        {
            robotController.SetJointPositions(testJointPositions);
        }
    }

    public void SetRobotJointStates(Dictionary<string, float> jointStates)
    {
        if (robotController != null)
        {
            robotController.SetJointPositions(jointStates);
        }
    }
}