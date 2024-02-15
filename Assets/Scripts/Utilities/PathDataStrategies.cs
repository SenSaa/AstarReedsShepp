using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathDataStrategies
{
    public class ListVector3PathData : IPathData
    {
        private readonly List<Vector3> points;

        public ListVector3PathData(List<Vector3> points)
        {
            this.points = points;
        }

        public List<Vector3> GetPoints()
        {
            return points;
        }
    }

    public class PathData : IPathData
    {
        private readonly Path path;

        public PathData(Path path)
        {
            this.path = path;
        }

        public List<Vector3> GetPoints()
        {
            List<Vector3> points = new List<Vector3>();
            for (int i = 0; i < path.x.Count; i++)
            {
                points.Add(new Vector3((float)path.x[i], 0.1f, (float)path.y[i]));
            }
            return points;
        }
    }
}
