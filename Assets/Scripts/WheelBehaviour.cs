using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelBehaviour : MonoBehaviour
{
    public WheelCollider wheelCol;

    // Update is called once per frame
    void Update()
    {
        wheelCol.GetWorldPose(out Vector3 position, out Quaternion rotation);
        var myTransform = transform;
        myTransform.position = position;
        myTransform.rotation = rotation;
    }
}
