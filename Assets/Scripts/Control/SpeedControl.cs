using MathNet.Numerics.RootFinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UtilityFunctions;

public class SpeedControl
{

    public (double pidOp, float currSpd) Update(double targetSpeed_ms, Transform agent, Rigidbody rigidbody)
    {
        float CurrentSpeed_ms = Utils.GetCurrentSpeed(agent, rigidbody);
        float CurrentSpeed_kmh = CurrentSpeed_ms * 3.6f;
        double speed_pid_output = PID.proportional_control(targetSpeed_ms, CurrentSpeed_ms);
        return (speed_pid_output, CurrentSpeed_kmh);
    }

}
