using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Parameters
/// </summary>

public class Params
{
    public static double Lfc = 1.2f;   // [m] look-ahead distance
    public static double Lbc = 1.4f;   // [m] look-ahead distance for reversing
    public static double Kp = 0.09f;    // speed proportional gain
    public static double dt = 0.02f;    // [s] time tick
    public static double WB = 2.25f;    // [m] wheel base of vehicle
    public static double MaxSteer = 40;    // [degrees] max steering angle
    public static float CarWidth = 1.8f;    // [degrees] max steering angle
    public static float CarLength = 3.5f;    // [degrees] max steering angle
}
