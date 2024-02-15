using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using astar;
using System.Runtime.CompilerServices;
using static UnityEngine.GraphicsBuffer;
using System.IO;

public class Visualisations
{

    public void DrawPath(IPathData pathData, IPathRenderDesign drawPath, string name)
    {
        GameObject lineRenderGO = new GameObject(name);
        var lineRenderer = lineRenderGO.AddComponent<LineRenderer>();
        List<Vector3> points = pathData.GetPoints();
        lineRenderer.positionCount = points.Count;
        for (int i = 0; i < points.Count; i++)
        {
            lineRenderer.SetPosition(i, points[i]);
        }
        lineRenderer.widthMultiplier = 0.15f;
        lineRenderer = drawPath.ApplyRenderDesign(lineRenderer);
    }

    public void DrawPaths(List<Path> paths, Color color, string name)
    {
        foreach (var path in paths)
        {
            GameObject lineRenderGO = new GameObject(name);
            var lineRenderer = lineRenderGO.AddComponent<LineRenderer>();
            lineRenderer.positionCount = path.x.Count;
            Vector3 pos = Vector3.zero;
            for (int i = 0; i < path.x.Count; i++)
            {
                pos.Set((float)path.x[i], 0.1f, (float)path.y[i]);
                lineRenderer.SetPosition(i, pos);
            }
            lineRenderer.widthMultiplier = 0.15f;
            lineRenderer.material.color = color;
        }
    }

    public void DrawPoint(Node node, string parentName, float cellSize, Material material)
    {
        Vector3 pos = new Vector3(node.GetNodePosition().Item1, 0.01f, node.GetNodePosition().Item2);
        Vector3 scale = new Vector3(cellSize - (cellSize / 5), 0.01f, cellSize - (cellSize / 5));
        GameObject pointObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        pointObj.transform.position = pos;
        pointObj.transform.localScale = scale;
        pointObj.transform.GetComponent<MeshRenderer>().material = material;
        pointObj.transform.GetComponent<Collider>().isTrigger = true;
        pointObj.name = "Node_Point";
        pointObj.transform.parent = GetObjParent(parentName);
    }

    public void DrawPoints(IPathData pathData, string parentName, float cellSize, Material material)
    {
        Vector3 pos = Vector3.zero;
        Vector3 scale = new Vector3(cellSize - (cellSize / 5), 0.01f, cellSize - (cellSize / 5));
        List<Vector3> points = pathData.GetPoints();
        for (int i = 0; i < points.Count; i++)
        {
            pos.Set(points[i].x, 0.01f, points[i].z);
            GameObject pointObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            pointObj.transform.position = pos;
            pointObj.transform.localScale = scale;
            pointObj.transform.GetComponent<MeshRenderer>().material = material;
            pointObj.transform.parent = GetObjParent(parentName);
        }
    }

    public void DrawHeadings(List<Node> path, GameObject prefab)
    {
        Vector3 pos = Vector3.zero;
        Vector3 rot = Vector3.zero;
        string pathStr = "";
        string headingsStr = "";
        for (int i = 0; i < path.Count; i++)
        {
            pos.Set(path.ElementAt(i).GetNodePosition().Item1, 0.1f, path.ElementAt(i).GetNodePosition().Item2);
            rot.Set(0, path.ElementAt(i).GetHeading(), 0);
            GameObject obj = Object.Instantiate(prefab, pos, Quaternion.Euler(rot));
            pathStr += pos + ", ";
            headingsStr += rot + ", ";
        }
    }
    
    public void DrawObstacles(HashSet<Obstacle> obstacles, string parentName, float cellSize)
    {
        Vector3 pos = Vector3.zero;
        Vector3 scale = Vector3.one;
        Color transparentRed = new Color(255, 0, 0, 0);
        transparentRed.a = 0;
        for (int i = 0; i < obstacles.Count; i++)
        {
            pos.Set(obstacles.ElementAt(i).x, 0.01f, obstacles.ElementAt(i).y);
            scale.Set(cellSize * 2, 0.1f, cellSize * 2);
            GameObject pointObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            pointObj.transform.position = pos;
            pointObj.transform.localScale = scale;
            pointObj.GetComponent<MeshRenderer>().material.color = transparentRed;
            pointObj.GetComponent<Collider>().enabled = false;
            pointObj.name = "ObstacleWithMargin";
            pointObj.transform.parent = GetObjParent(parentName);
        }
    }

    public Transform renderObject(string name)
    {
        GameObject go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        Transform tf = go.transform;
        tf.localScale = new UnityEngine.Vector3(0.5f, 0.1f, 0.5f);
        tf.GetComponent<Collider>().isTrigger = true;
        tf.name = name;
        return tf;
    }
    public Transform renderObject(string name, Material mat)
    {
        GameObject go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        Transform tf = go.transform;
        tf.localScale = new UnityEngine.Vector3(0.1f, 6.5f, 0.1f);
        tf.GetComponent<Collider>().isTrigger = true;
        tf.name = name;
        tf.GetComponent<MeshRenderer>().material = mat;
        return tf;
    }

    public void updateObject(Transform tf, double x, double y)
    {
        tf.position = new UnityEngine.Vector3((float)x, 0, (float)y);
    }

    public Transform GetObjParent(string name)
    {
        var obj = GameObject.Find(name);
        if (obj == null)
        {
            obj = new GameObject(name);
        }
        return obj.transform;
    }

}
