using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Newtonsoft.Json;

public class GradientDescent
{

    private Path PathPoints;

    public GradientDescent(Path pathPoints)
    {
        this.PathPoints = pathPoints;
    }

    public Path SmoothedPath()
    {
        Path smoothedXY = Optimise(PathPoints);
        return smoothedXY;
    }

    /// <summary>
    /// Gradient Descent optimisation.
    /// </summary>
    /// <param name="path">Path points (x,y)</param>
    /// <param name="weight_data">Data update weight</param>
    /// <param name="weight_smooth">Smoothing weight</param>
    /// <param name="tolerance">Change per iteration</param>
    /// <returns></returns>    
    private Path Optimise(Path path, float dataUpdateWeight = 0.5f, float smoothingWeight = 0.4f, float tolerance = 0.00001f)
    {
        Path newPath = new Path();
        List<double> xPath = path.x.Clone<List<double>>();
        List<double> zPath = path.y.Clone<List<double>>();
        List<double> newPathX = xPath.Clone<List<double>>();
        List<double> newPathZ = zPath.Clone<List<double>>();
        List<double> newPathYaw = path.yaw.Clone<List<double>>();
        List<int> newPathDir = path.directions.Clone<List<int>>();
        double change = tolerance;

        int iteration = 0;
        while (change >= tolerance)
        {
            if (iteration >= 500) { break; }
            change = 0.0f;
            for (int i = 1; i < path.x.Count - 1; i++)
            {
                double xtemp = newPathX[i];
                double ytemp = newPathZ[i];

                newPathX[i] += dataUpdateWeight * (xPath[i] - newPathX[i]);
                newPathZ[i] += dataUpdateWeight * (zPath[i] - newPathZ[i]);

                newPathX[i] += smoothingWeight * (newPathX[i - 1] + newPathX[i + 1] - (2.0f * newPathX[i]));
                newPathZ[i] += smoothingWeight * (newPathZ[i - 1] + newPathZ[i + 1] - (2.0f * newPathZ[i]));

                change += Math.Abs(xtemp - newPathX[i]);
                change += Math.Abs(ytemp - newPathZ[i]);
            }
            iteration += 1;
        }

        newPath.x.AddRange(newPathX);
        newPath.y.AddRange(newPathZ);
        newPath.yaw.AddRange(newPathYaw);
        newPath.directions.AddRange(newPathDir);

        return newPath;
    }

}
