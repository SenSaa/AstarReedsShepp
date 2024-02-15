using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using astar;
using UnityEngine.UIElements;
using System.Linq;

public class PathHelpers
{

    public static List<Vector3> NodeListToV3List(List<Node> nodes)
    {
        var path = new List<Vector3>();
        var pos = Vector3.zero;
        for (int i = 0; i < nodes.Count; i++)
        {
            pos.Set(nodes[i].GetNodePosition().Item1, 0.1f, nodes[i].GetNodePosition().Item2);
            path.Add(pos);
        }
        return path;
    }
    public static List<Vector3> NodeListToV3List(Path rsPath)
    {
        var path = new List<Vector3>();
        var pos = Vector3.zero;
        for (int i = 0; i < rsPath.x.Count; i++)
        {
            pos.Set((float)rsPath.x[i], 0.1f, (float)rsPath.y[i]);
            path.Add(pos);
        }
        return path;
    }

    public static Path PopulatePath(List<Vector3> astarPath, Path RSpath)
    {
        var path = new Path();
        var pos = Vector3.zero;
        for (int i = 0; i < astarPath.Count; i++)
        {
            try
            {
                pos.Set(astarPath[i].x, 0.1f, astarPath[i].z);
                path.x.Add(pos.x);
                path.y.Add(pos.z);
                path.yaw.Add(0);
                path.directions.Add(1);
            }
            catch (Exception e) { Debug.LogError(e); }
        }
        pos = Vector3.zero;
        if (RSpath != null)
        {
            for (int i = 0; i < RSpath.x.Count; i++)
            {
                try
                {
                    pos.Set((float)RSpath.x[i], 0.1f, (float)RSpath.y[i]);
                    path.x.Add(pos.x);
                    path.y.Add(pos.z);
                    path.yaw.Add(RSpath.yaw[i]);
                    if (i <= RSpath.directions.Count - 1)
                    {
                        path.directions.Add(RSpath.directions[i]);
                    }
                    else { path.directions.Add(1); }
                }
                catch (Exception e) { Debug.LogError(e); }
            }
        }
        return path;
    }


    public static List<Vector3> SpaceWPs(List<Vector3> rawPoints, float wpGap)
    {
        var points = new List<Vector3>();

        Vector3 prevWP = Vector3.zero;
        float dist;
        Vector3 p;
        float fraction = 0.1f;

        for (int i = 0; i < rawPoints.Count; i++)
        {
            p = rawPoints[i];

            if (prevWP.Equals(Vector3.zero)) { prevWP = p; }
            dist = Vector3.Distance(prevWP, p);

            // *
            // If distance between points exceeds waypoint gap value passed as arg,
            // or if at first waypoint,
            // or if at the last fraction of waypoints,
            // Add point to list.
            ///if (dist >= wpGap || i == 0 || i > rawPoints.Count * (1-fraction))
            if (dist >= wpGap || i == 0 || i == rawPoints.Count - 1)
            {
                points.Add(p);
                prevWP = p;
            }
        }

        return points;
    }

    public static List<Vector3> CreateEvenlySpacedIntermediatePts(List<Vector3> dataPoints, float maxSpacing)
    {
        List<Vector3> points = new List<Vector3>();
        for (int i = 1; i < dataPoints.Count; i++)
        {
            // Get every 2 segments of point set (current and previous).
            Vector3[] pointsArray = new Vector3[2];
            pointsArray[0] = dataPoints[i - 1];
            pointsArray[1] = dataPoints[i];

            Vector3 pt1 = pointsArray[0];
            Vector3 pt2 = pointsArray[1];

            Vector3 dir = pt2 - pt1;
            float dist = dir.magnitude;
            dir.Normalize();

            int count = Mathf.FloorToInt(dist / maxSpacing);
            if (count == 0) count = 1;
            float n = dist / (count + 1);

            for (int j = 0; j < count; j++)
            {
                Vector3 p = pt1 + dir * n * (j + 1);
                points.Add(p);
            }
        }
        return points;
    }

    public static Path Lerp(Path path, float step_size)
    {
        Path interpolatedPath = new Path();
        Vector3 pos = Vector3.zero;
        for (int j = 1; j < path.x.Count(); j++)
        {
            double start_value_x = path.x[j - 1];
            double end_value_x = path.x[j];
            double start_value_y = path.y[j - 1];
            double end_value_y = path.y[j];
            for (float i = 0; i < 1; i += step_size)
            {
                double pos_x = Lerp(start_value_x, end_value_x, i);
                double pos_y = Lerp(start_value_y, end_value_y, i);
                pos.Set((float)pos_x, 0, (float)pos_y);
                interpolatedPath.x.Add(pos.x);
                interpolatedPath.y.Add(pos.z);
                interpolatedPath.yaw.Add(path.yaw[j]);
                interpolatedPath.directions.Add(path.directions[j]);
            }
        }
        return interpolatedPath;
    }
    private static double Lerp(double start_value, double end_value, double pct)
    {
        return (start_value + (end_value - start_value) * pct);
    }

}
