using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System.Linq;
using System;
using ReedsShepp;
using UtilityFunctions;
using System.Threading;
using System.IO;
using UnityEngine.UIElements;
using PurePursuit;

namespace astar
{
    public class AStarRS
    {

        private (float x, float z, float yaw) StartNodePose;
        private (float x, float z, float yaw) GoalNodePose;
        private (float x, float z) StartNodePos;
        private (float x, float z) GoalNodePos;
        Node StartNode;
        Node GoalNode;
        private int MaxIteration = 500;
        private Graph Graph;
        private HashSet<Node> NodeSet;
        private List<Node> ClosedSet;
        HeapPriorityQueue<Node> OpenSet;
        Dictionary<Node, float> GcostDict;
        private List<Node> Path;
        private HashSet<Obstacle> Obstacles;
        private float ObstacleSafetyMargin;
        private Stopwatch Stopwatch;
        private float ElapsedTime;
        BroadcastData BroadcastData;
        private ReedsSheppPath ReedsShepp;
        private float MaxRSpathDist;
        private Queue<Node> NodeQueue; // * For visualisation.

        public AStarRS((float x, float z, float yaw) StartNodePose, (float x, float z, float yaw) GoalNodePose, HashSet<Obstacle> obs, 
            float obstacleSafetyMargin, float cellSize = 1, int gridWidth = 201, int gridHeight = 201)
        {
            Stopwatch = new Stopwatch();
            Stopwatch.Start();

            this.StartNodePose = StartNodePose;
            this.StartNodePos = (StartNodePose.x, StartNodePose.z);
            this.GoalNodePose = GoalNodePose;
            this.GoalNodePos = (GoalNodePose.x, GoalNodePose.z);
            Graph = new Graph(cellSize);

            ObstacleSafetyMargin = obstacleSafetyMargin;
            Obstacles = obs;

            Graph.SetGridWidth(gridWidth);
            Graph.SetGridHeight(gridHeight);
            NodeSet = new HashSet<Node>();

            // Instead of starting from 1, start from the lowest value of x & y between the start and goal.
            // ^ For when start or goal have negative position values!
            // And for cell increments, instead of 1 unit increments, increment cells by cellsize.
            // ^ So that cellsize can be variable, and not just 1 unit-sized!
            float startvalX = Mathf.Floor(Math.Min(StartNodePos.x, GoalNodePos.x));
            float startvalY = Mathf.Floor(Math.Min(StartNodePos.z, GoalNodePos.z));

            for (float j = startvalY; j <= Graph.GetGridHeight(); j += cellSize)
            {
                for (float i = startvalX; i <= Graph.GetGridWidth(); i += cellSize)
                {
                    Node node = new Node((i, j));
                    node.SetGcost(float.PositiveInfinity); // If g-cost is not infinite...
                    node.SetHcost(float.PositiveInfinity);
                    node.SetFcost(float.PositiveInfinity);
                    NodeSet.Add(node);
                }
            }

            ReedsShepp = new ReedsSheppPath();
            this.BroadcastData = new BroadcastData();
            MaxRSpathDist = 12;
            NodeQueue = new Queue<Node>();
        }

        public void Search()
        {
            OpenSet = new HeapPriorityQueue<Node>((a, b) => a.GetFcost().CompareTo(b.GetFcost()));
            ClosedSet = new List<Node>();

            // Start Node
            // put the starting node on the open
            float startHeading = StartNodePose.yaw;
            StartNode = FindNearestNode(StartNodePos);
            StartNode.SetHeading(startHeading);
            OpenSet.Insert(StartNode);

            // Initialise costs as 0
            StartNode.SetGcost(0);
            StartNode.SetHcost(0);
            StartNode.SetFcost(0);

            // Goal node
            float goalHeading = GoalNodePose.yaw;
            GoalNode = FindNearestNode(GoalNodePos);
            GoalNode.SetHeading(goalHeading);

            GcostDict = new Dictionary<Node, float>();

            // while the open list is not empty
            // To avoid being stuck in the loop, set an iteration bound
            int iteration = 0;
            while (OpenSet.GetSize() > 0 && iteration < MaxIteration)
            {
                iteration++;
                UnityEngine.Debug.Log("i -> " + iteration);

                // Find the node with the least f cost in the open list.
                // And Deque it.
                Node currentNode = OpenSet.Delete();

                ClosedSet.Add(currentNode);

                NodeQueue.Enqueue(currentNode);
                BroadcastData.BroadcastSearchUpdate(currentNode, NodeQueue, iteration);

                Path rsPath = GenerateRSPath(currentNode, MaxRSpathDist);

                if (rsPath != null)
                {
                    BroadcastData.BroadcastCurrentRSpath(rsPath);
                    // Check if RS path to goal is collision free.
                    // If yes, extract path.
                    bool pathIsFeasible = PathCheck(rsPath);
                    if (pathIsFeasible)
                    {
                        UnityEngine.Debug.Log("RS Path Found!");

                        this.Path = ExtractPath(currentNode);

                        Stopwatch.Stop();
                        ElapsedTime = Stopwatch.ElapsedMilliseconds;

                        BroadcastData.BroadcastSearchResults(true, Path);

                        return;
                    }
                }

                ExpandNode(currentNode);
            }
            UnityEngine.Debug.Log("Path not found!");

            BroadcastData.BroadcastSearchResults(false, Path);
        }

        private Path GenerateRSPath(Node currentNode, float RS_Path_Dist)
        {
            if (Math.Abs(GoalNodePos.x - currentNode.GetNodePosition().x) < RS_Path_Dist 
                && Math.Abs(GoalNodePos.z - currentNode.GetNodePosition().z) < RS_Path_Dist)
            {
                var start_x = currentNode.GetNodePosition().x;
                var start_y = currentNode.GetNodePosition().z;
                var start_yaw = currentNode.GetHeading();
                start_yaw = Utils.TransformYaw(start_yaw) * Mathf.Deg2Rad;

                var goal_x = GoalNodePos.x;
                var goal_y = GoalNodePos.z;
                var goal_yaw = GoalNodePose.yaw;
                goal_yaw = Utils.TransformYaw(goal_yaw) * Mathf.Deg2Rad;

                var max_curvature = 0.14;
                var step_size = 0.1;

                var xs_ys_yaws_modes_lengths_dirs = ReedsShepp.reeds_shepp_path_planning(
                    start_x, start_y,
                    start_yaw, goal_x,
                    goal_y, goal_yaw,
                    max_curvature,
                    step_size);
                List<double> xs = xs_ys_yaws_modes_lengths_dirs.Item1;
                List<double> ys = xs_ys_yaws_modes_lengths_dirs.Item2;
                List<double> yaws = xs_ys_yaws_modes_lengths_dirs.Item3;
                List<string> modes = xs_ys_yaws_modes_lengths_dirs.Item4;
                List<double> lengths = xs_ys_yaws_modes_lengths_dirs.Item5;
                List<int> dirs = xs_ys_yaws_modes_lengths_dirs.Item6;
                var path = Utils.ListsToPath(xs, ys, yaws, modes, lengths, dirs);

                return path;
            }
            return null;
        }

        private bool PathCheck(Path path)
        {
            // Collision check
            bool collisionDetected = LineSquareIntersection.IsCollisionWithSquares(path, Obstacles, ObstacleSafetyMargin);
            if(collisionDetected) 
            {
                return false;
            }
            // Check if path is too close to the goal.
            double dx = GoalNodePos.x - path.x[0];
            double dy = GoalNodePos.z - path.y[0];
            var goalTooClose = 2;
            var distThreshold = Helpers.Hypotenuse(dx, dy) < goalTooClose;
            if (distThreshold)
            {
                return false;
            }
            return true;
        }

        private void ExpandNode(Node currentNode)
        {
            // generate current node's successors/children and set their parents/predecessors to the current node
            List<Node> neighbours = FindNodeNeighbours(currentNode);
            currentNode.SetNeighbours(neighbours);

            // for each successor/child
            foreach (var neighbour in neighbours)
            {
                if (ClosedSet.Contains(neighbour))
                {
                    continue;
                }

                // successor/child (g) cost = current node (g) cost + distance between successor/child and current node
                float gcost = currentNode.GetGcost() + Heurisrics.EuclideanDistance(currentNode.GetNodePosition(), neighbour.GetNodePosition());

                // if successor is not already in open set set,
                if (!GcostDict.ContainsKey(neighbour) && gcost < neighbour.GetGcost())
                {
                    // Set current node as parent of neighbour.
                    neighbour.SetParent(currentNode);

                    // successor/child (g) cost = current node (g) cost + distance between successor/child and current node
                    neighbour.SetGcost(gcost);
                    // successor/child heuristic (h) cost = distance from goal to successor/child node  (e.g. Manhattan, Diagonal and Euclidean distance).
                    neighbour.SetHcost(Heurisrics.DiagonalDistance(GoalNodePos, neighbour.GetNodePosition(), 1.1f));
                    // successor/child (f) cost = successor/child (g) + successor/child (h)
                    neighbour.SetFcost(neighbour.GetGcost() + neighbour.GetHcost());

                    // Add neighbour to open set.
                    OpenSet.Insert(neighbour);
                    GcostDict.Add(neighbour, neighbour.GetGcost());
                }
            }
        }


        private Node FindNearestNode((float x, float z) nodePos)
        {
            foreach (var node in NodeSet)
            {
                if (Mathf.Abs(nodePos.x - node.GetNodePosition().x) < Graph.GetGridCellSize()
                    && Mathf.Abs(nodePos.z - node.GetNodePosition().z) < Graph.GetGridCellSize())
                {
                    return node;
                }
            }
            return null;
        }

        private List<Node> FindNodeNeighbours(Node node)
        {
            List<Node> neighbours = new List<Node>();
            var dirs = new List<(float x, float z)>
                                                        { (Graph.GetGridCellSize(), 0), // right
                                                        (0, Graph.GetGridCellSize()), // up
                                                        (-Graph.GetGridCellSize(), 0), // left
                                                        (0, -Graph.GetGridCellSize()), // down
                                                        (Graph.GetGridCellSize(), Graph.GetGridCellSize()), // up-right
                                                        (Graph.GetGridCellSize(), -Graph.GetGridCellSize()), // down-right
                                                        (-Graph.GetGridCellSize(), Graph.GetGridCellSize()), // up-left
                                                        (-Graph.GetGridCellSize(), -Graph.GetGridCellSize()) }; // down-left
            
            foreach (var dir in dirs)
            {
                var neighborPos = (node.GetNodePosition().x + dir.x, node.GetNodePosition().z + dir.z);
                var neighbor = FindNearestNode(neighborPos);
                if (neighbor == null) { continue; }

                // Instead of limiting the neighbours bounds to (0,0), limit it to the smallest value between the start and goal (whichever is smaller).
                // In terms of max bounds, the max bounds should the grid width & height.
                if (Math.Min(StartNodePos.x, GoalNodePos.x) <= neighbor.GetNodePosition().x && neighbor.GetNodePosition().x < Graph.GetGridWidth()
                    && Math.Min(StartNodePos.z, GoalNodePos.z) <= neighbor.GetNodePosition().z && neighbor.GetNodePosition().z < Graph.GetGridHeight())
                {
                    if (NodeObstalceFree(neighbor.GetNodePosition()))
                    {
                        neighbours.Add(neighbor);
                    }
                }
            }
            return neighbours;
        }

        // In addition to obstacle pos, also consider size (width/height).
        private bool NodeObstalceFree((float x, float z) nodePos)
        {
            foreach (var obst in Obstacles)
            {
                // Check for overlap in x-axis
                bool xOverlap = Math.Abs(nodePos.x - obst.x) < (obst.width);
                // Check for overlap in y-axis
                bool yOverlap = Math.Abs(nodePos.z - obst.y) < (obst.height);
                // If there is overlap in both x and y axes, then there is a collision
                if (xOverlap && yOverlap)
                {
                    return false;
                }
            }
            return true;
        }


        public List<Node> ExtractPath(Node currentNode)
        {
            List<Node> path = new List<Node>();
            Node goalNode = new Node(GoalNodePos);
            Node node = currentNode;
            path.Add(goalNode);
            path.Add(node);
            int iteration = 0;
            while (!node.GetNodePosition().Equals(StartNode.GetNodePosition())
                && iteration < NodeSet.Count)
            {
                if (node.GetParent() != null)
                {
                    // Compute headings/yaw/orientation while extracting path points.
                    var delta_x = node.GetNodePosition().x - node.GetParent().GetNodePosition().x;
                    var delta_y = node.GetNodePosition().z - node.GetParent().GetNodePosition().z;
                    var delta = Mathf.Atan2(delta_x, delta_y) * Mathf.Rad2Deg;
                    node.SetHeading(delta);

                    path.Add(node.GetParent());
                }
                node = node.GetParent();
            }
            path.Reverse();
            //Path = path;
            return path;
        }

        public string PrintClosedSetString()
        {
            string closedSetString = "";
            for (int i = 0; i < ClosedSet.Count; i++)
            {
                closedSetString += ClosedSet.ElementAt(i).GetNodePosition().x + " , " + ClosedSet.ElementAt(i).GetNodePosition().z + "   |   ";
            }
            return closedSetString;
        }

        public float GetElapsedTime()
        {
            return ElapsedTime;
        }

    }

}
