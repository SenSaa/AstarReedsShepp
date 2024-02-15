using System.Collections;
using System.Collections.Generic;
using System;

public class Path
{

    // Path data container

    public List<double> lengths;
    public List<string> ctypes;
    public double L;
    public List<double> x;
    public List<double> y;
    public List<double> yaw;
    public List<int> directions;

    public Path()
    {
        // course segment length  (negative value is backward segment)
        this.lengths = new List<double>();
        // course segment type char ("S": straight, "L": left, "R": right)
        this.ctypes = new List<string>();
        this.L = 0.0;  //  Total lengths of the path
        this.x = new List<double>(); // x positions
        this.y = new List<double>(); // y positions
        this.yaw = new List<double>(); // orientations [rad]
        this.directions = new List<int>(); // directions (1:forward, -1:backward)
    }

}
