using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleControlCommandFlags
{
    public bool ApplyBrake;
    public bool ReleaseBrake;
    public bool ReleaseThrottle;
    public bool ReleaseSteering;

    public void Reset()
    {
        this.ApplyBrake = false;
        this.ReleaseBrake = false;
        this.ReleaseThrottle = false;
        this.ReleaseSteering = false;
    }
}
