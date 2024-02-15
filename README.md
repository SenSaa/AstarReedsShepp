# AstarReedsShepp
A* + Smoothing + Reeds Sheep in Unity (C#)
<br></br>

Videos:

...


# Architecture

![image](https://github.com/SenSaa/AstarReedsShepp/assets/19212519/90450ae0-7b34-40d9-8922-5beacef4c37e)

The approach to solve the unstructured planning (parking) problem is as follows:
* Planner: 
The planner consists of A* search and Reeds Shepp path final segment.
This path is then smoothed.
In addition to planning the path, we need to plan for the behaviours and set target speeds to the controller based on the different manoeuvres.
* Controller:
The controller consists of longitudinal control (speed) and lateral control (steering), which produce a speed command and a steering angle respectively.

A good approach from a software design pattern prespective to decouple the planning and control modules, whilst also communicating the required data (path and target speed), is to utilise the observer (publisher-subscriber) design pattern.


# Unstructured Planner

### A*
The A* implementation is similar to the implementation done in the earlier A* project:

https://github.com/SenSaa/AStar

The graph of square grid cell nodes with uniform edge weights is constructed as standard.
Most of the steps are also the standard A* steps as before.
This time we are using a PriorityQueue implemented with a MinHeap, as it should allow us to maintain the best (lowest f-cost) node at the root of the tree, saving us the additional time for searching for the lowest cost node as an extra step had we gone with say an arraylist.
The main difference between a standard A* and this implementation, is that instead of searching until the goal node-by-node, and terminating the search once we reach the goal, instead here we check if a Reeds Shepp path to the goal is feasible once we reach a certain distance!

### Reeds-Shepp
For every iteration in the A* search, once we retrieve the lowest cost node from the priority queue, we check if the current node is within a certain distance from the goal.
If that's the case, we construct a reeds shepp path from the current node to the goal, and if this path is feasible, we end the search.
This is where the similarity between this approach and Hybrid A* (approach used by Stanford in the DARPA urban challenege) becomes apparent, except with simplifications in approach and reduced optimality. :)

### Path Smoothing
The approach used is gradient descent optimisation to produce a smooth path.
In this approach we minimise the distance between the points.
In the first step we minimise the difference between the original and smooth point, then in the second step we minimise the distance between the smooth points.
Details about the algorithm can be found in Sebastian Thrun's Udacity courses (AI for robotics and possibly in SDC):
https://www.youtube.com/watch?v=pjgfffzvbUg


Path before smoothing:

<img width="450" alt="Path_Before_Smoothing" src="https://github.com/SenSaa/AstarReedsShepp/assets/19212519/513db146-ab4d-4db9-9f4c-fd712576aba6">


Path after smoothing:

<img width="449" alt="Path_After_Smoothing" src="https://github.com/SenSaa/AstarReedsShepp/assets/19212519/0f96e091-4e62-4b38-87ee-6001a28713d3">


Alternatively, another option that can work well is to use curve interpolation such as cubic splines (B-spline, hermite spline, etc.)


#### Obstacles
Due to the non-holonomic nature of vehicles, special considerations must be taken to generate paths that are within safe margin of obstacles. To prevent the vehicle from colliding with or turning into an obstacle.

Generating paths that do not intersect collisions are in two steps:

1. When expanding nodes and finding neighbours to current A* node, while considering a neighbour, instead of checking if the node cell is at a obstacle position, we do an AABB overlap check, with the size accounting for a safety margin (vehicle width).
2. To ensure the Reeds Shepp path segment is also safe, we do a line-rectangle intersection check, and similarly to the step above, with account for the same safety margin (vehicle width).

The screenshot below visualises the safety margin in red, extending from the physical obstacles in white:

<img width="516" alt="Screenshot" src="https://github.com/SenSaa/AstarReedsShepp/assets/19212519/522dcc16-467c-4e38-a3a1-1999f6d75630">


The search steps slowed down to show how the path searching takes place:

...





# Behaviour Planning
![image](https://github.com/SenSaa/AstarReedsShepp/assets/19212519/5d28354b-7307-46c5-85b2-2d98bbfaa9ce)

The behaviour planning component is kept simple. There are three states:
- Idle (Initial / stopped state)
- Cruise (When driving)
- Stopping (Near destination)

# Path Tracking (Control)
Speed Control:

For speed control a PID, more precisely proportional (p) controller is used.

Steer Control:

For steer control Pure Pursuit algorithm is used.
