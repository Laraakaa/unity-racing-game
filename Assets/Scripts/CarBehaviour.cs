using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarBehaviour : MonoBehaviour
{
    public WheelCollider wheelColliderFL;
    public WheelCollider wheelColliderFR;
    public WheelCollider wheelColliderBL;
    public WheelCollider wheelColliderBR;
    public Transform centerOfMass;
    public float maxTorque = 500;
    public float brakeTorque = 5000;
    public float maxSteerAngle = 45;
    public float maxSpeedForwardKMH = 150;
    public float maxSpeedBackwardKMH = 30;
    public float sidewaysStiffness = 1.5f;
    public float forewardStiffness = 1.5f;
    public float antiRoll = 10000f;

    private Rigidbody _rigidBody;

    private float _currentSpeedKMH;
    private bool _isForward;

    void Start()
    {
        _rigidBody = GetComponent<Rigidbody>();
        _rigidBody.centerOfMass = new Vector3(centerOfMass.localPosition.x, centerOfMass.localPosition.y, centerOfMass.localPosition.z);
        SetWheelFrictionStiffness(forewardStiffness, sidewaysStiffness);
    }

    void FixedUpdate()
    {
        // Capture inputs
        float verticalInput = Input.GetAxis("Vertical");
        float horizontalInput = Input.GetAxis("Horizontal");

        // Calculate speed
        _currentSpeedKMH = _rigidBody.velocity.magnitude * 3.6f;
        _isForward = Vector3.Angle(transform.forward, _rigidBody.velocity) < 50f;

        // Limit speed forward
        if (_isForward && _currentSpeedKMH >= maxSpeedForwardKMH)
        {
            verticalInput = 0;
        }
        // Limit speed backward
        if (!_isForward && _currentSpeedKMH >= maxSpeedBackwardKMH) {
            verticalInput = 0;
        }

        // Detect if we're braking
        bool doBraking = _currentSpeedKMH > 0.5f && (Input.GetAxis("Vertical") < 0 && _isForward || Input.GetAxis("Vertical") > 0 && !_isForward);
        if (doBraking)
        {
            wheelColliderFL.brakeTorque = brakeTorque;
            wheelColliderFR.brakeTorque = brakeTorque;
            wheelColliderBL.brakeTorque = brakeTorque;
            wheelColliderBR.brakeTorque = brakeTorque;
            wheelColliderFL.motorTorque = 0;
            wheelColliderFR.motorTorque = 0;
        }
        else
        {
            wheelColliderFL.brakeTorque = 0;
            wheelColliderFR.brakeTorque = 0;
            wheelColliderBL.brakeTorque = 0;
            wheelColliderBR.brakeTorque = 0;
            wheelColliderFL.motorTorque = maxTorque * verticalInput;
            wheelColliderFR.motorTorque = wheelColliderFL.motorTorque;
        }

        // Steer
        SetSteerAngle(GetSteerAngle() * horizontalInput);

        // Anti-Roll
        var travelL = 1f;
        var travelR = 1f;

        var groundedL = wheelColliderFL.GetGroundHit(out WheelHit hit);
        if (groundedL) travelL = (-wheelColliderFL.transform.InverseTransformPoint(hit.point).y - wheelColliderFL.radius) / wheelColliderFL.suspensionDistance;
        var groundedR = wheelColliderFR.GetGroundHit(out hit);
        if (groundedR) travelR = (-wheelColliderFR.transform.InverseTransformPoint(hit.point).y - wheelColliderFR.radius) / wheelColliderFR.suspensionDistance;

        float antiRollForce = (travelL - travelR) * antiRoll;

        if (groundedL)
        {
            _rigidBody.AddForceAtPosition(wheelColliderFL.transform.up * -antiRollForce, wheelColliderFL.transform.position);
        }
        if (groundedR)
        {
            _rigidBody.AddForceAtPosition(wheelColliderFR.transform.up * antiRollForce, wheelColliderFR.transform.position);
        }
    }

    float GetSteerAngle()
    {
        var speedFactor = 15 / _currentSpeedKMH;
        if (speedFactor > 1)
        {
            speedFactor = 1;
        }
        return maxSteerAngle * speedFactor;
    }

    void SetSteerAngle(float angle)
    {
        wheelColliderFL.steerAngle = angle;
        wheelColliderFR.steerAngle = angle;
    }

    void SetWheelFrictionStiffness(float newForwardStiffness, float newSidewaysStiffness)
    {
        WheelFrictionCurve fwWFC = wheelColliderFL.forwardFriction;
        WheelFrictionCurve swWFC = wheelColliderFL.sidewaysFriction;
        fwWFC.stiffness = newForwardStiffness;
        swWFC.stiffness = newSidewaysStiffness;

        wheelColliderFL.forwardFriction = fwWFC;
        wheelColliderFL.sidewaysFriction = swWFC;
        wheelColliderFR.forwardFriction = fwWFC;
        wheelColliderFR.sidewaysFriction = swWFC;

        wheelColliderBL.forwardFriction = fwWFC;
        wheelColliderBL.sidewaysFriction = swWFC;
        wheelColliderBR.forwardFriction = fwWFC;
        wheelColliderBR.sidewaysFriction = swWFC;
    }
}
