using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// PID Speed Controller.
/// Uses P-term only, so a P-Controller. 
/// </summary>

public class PID
{

    public static double proportional_control(double target, double current)
    {
        double error = target - current;
        double p = Params.Kp * error;
        return p;
    }

}
