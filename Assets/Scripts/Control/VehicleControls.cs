using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Rear wheel drive, front steering vehicle control.
/// </summary>

public class VehicleControls
{

    private WheelCollider FLWC; // Front left wheel collider.
    private WheelCollider FRWC; // Front right wheel collider.
    private WheelCollider RLWC; // Rear left wheel collider.
    private WheelCollider RRWC; // Rear right wheel collider.
    private float MotorTorque;
    private float BrakeTorque;

    public VehicleControls(List<WheelCollider> WheelColliders, float motorTorque, float brakeTorque)
    {
        FLWC = WheelColliders[0];
        FRWC = WheelColliders[1];
        RLWC = WheelColliders[2];
        RRWC = WheelColliders[3];
        this.MotorTorque = motorTorque;
        this.BrakeTorque = brakeTorque;
    }

    public void ApplySpeed(float pid_output)
    {
        RLWC.motorTorque = RRWC.motorTorque = MotorTorque * pid_output; // * RWD
    }

    public void ApplySteer(double state_delta, int dir)
    {
        float steer = dir == 1 ? (float)-state_delta : (float)state_delta;
        steer = Mathf.Clamp(steer, (float)-Params.MaxSteer, (float)Params.MaxSteer);
        FLWC.steerAngle = FRWC.steerAngle = steer; // * Steering will always be via the front wheels in all drive modes (FWD / RWD / AWD).
    }


    public void ApplyBrake()
    {
        RLWC.brakeTorque = RRWC.brakeTorque = BrakeTorque;
    }

    public void ReleaseBrake()
    {
        RLWC.brakeTorque = RRWC.brakeTorque = 0;
    }

    public void ReleaseThrottle()
    {
        RLWC.motorTorque = RRWC.motorTorque = 0;
    }

    public void ReleaseSteering()
    {
        FLWC.steerAngle = FRWC.steerAngle = 0;
    }

}
