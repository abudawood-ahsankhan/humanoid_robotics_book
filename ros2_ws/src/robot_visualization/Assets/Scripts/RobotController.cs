using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [System.Serializable]
    public class JointConfig
    {
        public string jointName;
        public Transform jointTransform;
        public float minAngle = -90f;
        public float maxAngle = 90f;
    }

    public List<JointConfig> jointConfigs = new List<JointConfig>();

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    public void SetJointPositions(Dictionary<string, float> jointPositions)
    {
        foreach (var jointConfig in jointConfigs)
        {
            if (jointPositions.ContainsKey(jointConfig.jointName))
            {
                float angle = Mathf.Clamp(jointPositions[jointConfig.jointName], jointConfig.minAngle, jointConfig.maxAngle);
                jointConfig.jointTransform.localRotation = Quaternion.Euler(0, angle, 0);
            }
        }
    }
}