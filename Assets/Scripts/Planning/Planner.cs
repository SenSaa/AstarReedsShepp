using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using astar;
using System.Linq;
using System;
using System.Threading;
using UnityEngine.Diagnostics;

public class Planner : MonoBehaviour
{

    [SerializeField] private Transform StartTransform;
    [SerializeField] private Transform GoalTransform;
    [SerializeField] private GameObject HeadingPrefab;
    [SerializeField] private Material GridCellMaterial;
    [SerializeField] private Transform ObstaclesParent;
    private Vector3 GoalPos;
    private AStarRS astarRS;
    private (float, float, float) StartNodePos;
    private (float, float, float) GoalNodePos;
    private HashSet<Obstacle> Obstacles;
    private float GridCellSize;
    private int GridWidth;
    private int GridHeight;
    private float ObstacleSafetyMargin;
    private List<Node> SearchPathPoints;
    private bool PathFound;
    private Path Current_RS_Path;
    private List<Vector3> SearchPath;
    private Path CombinedPath;
    private Path GradientDescentPath;
    Thread PathSearchThread;
    [SerializeField] private Material TransparentPathMat;
    Visualisations visualisation;
    BroadcastData BroadcastData;
    private Queue<Node> NodeQueue;


    private void Awake()
    {
        MessageBroker.SearchResults += HandleSearchResults;
        MessageBroker.SearchUpdate += HandleSearchUpdate;
        MessageBroker.CurrentRSpath += HandleCurrentRSpath;
    }

    private void HandleSearchResults(object o, MessageBroker.SearchResultsEventArgs e)
    {
        PathFound = e.Results.success;
        SearchPathPoints = e.Results.Path;
    }

    private void HandleSearchUpdate(object o, MessageBroker.SearchUpdateEventArgs e)
    {
        NodeQueue = e.Update.NodeQueue;
    }

    private void HandleCurrentRSpath(object o, MessageBroker.CurrentRSpathEventArgs e)
    {
        Current_RS_Path = e.CurrentRSpath;
    }


    void Start()
    {
        Vector3 StartPos = new Vector3(StartTransform.position.x, StartTransform.position.y, StartTransform.position.z);
        GoalPos = new Vector3(GoalTransform.position.x, GoalTransform.position.y, GoalTransform.position.z);
        float StartRotation = StartTransform.rotation.eulerAngles.y;
        float GoalRotation = GoalTransform.rotation.eulerAngles.y;
        StartNodePos = (StartPos.x, StartPos.z, StartRotation);
        GoalNodePos = (GoalPos.x, GoalPos.z, GoalRotation);
        GridCellSize = 1.0f;
        GridWidth = Mathf.CeilToInt(Mathf.Abs(GoalNodePos.Item1) + Mathf.Abs(StartNodePos.Item1)) * 2;
        GridHeight = Mathf.CeilToInt(Mathf.Abs(GoalNodePos.Item2) + Mathf.Abs(StartNodePos.Item2)) * 2;
        ObstacleSafetyMargin = Params.CarWidth;
        Obstacles = new HashSet<Obstacle>();
        GetEnviroObstacles();

        PathSearchThread = new Thread(RunAStarSearch);
        PathSearchThread.Start();

        visualisation = new Visualisations();
        visualisation.DrawObstacles(Obstacles, "ObstaclesParent", GridCellSize);

        this.BroadcastData = new BroadcastData();
        this.BroadcastData.BroadcastConfiguration(StartPos, StartRotation, GoalPos, GoalRotation);

        StartCoroutine(CurrentSearchUpdateVisual());
    }

    private void RunAStarSearch()
    {
        astarRS = new AStarRS(StartNodePos, GoalNodePos, Obstacles, ObstacleSafetyMargin, GridCellSize, GridWidth, GridHeight);
        astarRS.Search();
        
        Debug.Log("Time in ms = " + astarRS.GetElapsedTime());
        Debug.Log("Time in s = " + astarRS.GetElapsedTime() / 1000);
    }


    void Update()
    {
        if (PathFound)
        {
            PathFound = !PathFound; // Run visualisation per path search!
            CombinePaths();
            SmoothPath();
            VisualisePath();
            this.BroadcastData.BroadcastSmoothPath(GradientDescentPath);
        }
    }


    // * Note:
    // Drawing current node can occur in update by broadcasting current node event (and checking iteration not to draw duplicates).
    // However, search iterations execute faster than drawing steps, so we would need to put the path search thread to sleep (for ~50ms) per iteration.
    // Instead it's better to push the current node (lowest cost node) to a queue data structure, then send it for rendering, and dequeuing it.
    private IEnumerator CurrentSearchUpdateVisual()
    {
        while (NodeQueue.Count > 0)
        {
            Node head = NodeQueue.Dequeue(); // front
            visualisation.DrawPoint(head, "SearchNode", GridCellSize, GridCellMaterial);
            if (Current_RS_Path != null) { visualisation.DrawPath(new PathDataStrategies.PathData(Current_RS_Path), new PathRenderStrategies.PathRenderWihMaterial(TransparentPathMat), "CurrentRSpath"); }
            yield return new WaitForSeconds(0f);
        }
    }


    // Combine Search path and Reeds Shepp path segments.
    private void CombinePaths()
    {
        SearchPathPoints.Remove(SearchPathPoints[SearchPathPoints.Count - 1]); // Remove goal node from search path!
        SearchPath = PathHelpers.NodeListToV3List(SearchPathPoints);
        CombinedPath = PathHelpers.PopulatePath(SearchPath, Current_RS_Path);
    }

    private void SmoothPath()
    {
        var interpolatedPoints = PathHelpers.Lerp(CombinedPath, 1.0f);
        GradientDescent gradientDescent = new GradientDescent(interpolatedPoints);
        GradientDescentPath = gradientDescent.SmoothedPath();
    }

    private void VisualisePath()
    {
        visualisation.DrawPath(new PathDataStrategies.PathData(GradientDescentPath), new PathRenderStrategies.PathRenderWihColor(Color.magenta), "OptimisedPath");
    }


    private void GetEnviroObstacles()
    {
        var x = float.MaxValue;
        var y = float.MaxValue;

        foreach(Transform obst in ObstaclesParent)
        {
            if (obst.tag == "Obstacle")
            {
                x = obst.transform.position.x;
                y = obst.transform.position.z;
                Obstacles.Add(new Obstacle(x, y, ObstacleSafetyMargin, ObstacleSafetyMargin));
            }
        }

        // If obstacles are at start or goal positions, remove them!
        foreach (var obst in Obstacles.ToList())
        {
            if ((obst.x == StartNodePos.Item1 && obst.y == StartNodePos.Item2) || (obst.x == GoalNodePos.Item1 && obst.y == GoalNodePos.Item2))
            {
                Obstacles.Remove(obst);
            }
        }
    }


    private void OnApplicationQuit()
    {
        PathSearchThread.Abort();
    }

}
