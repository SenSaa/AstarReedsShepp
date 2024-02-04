# AstarReedsShepp
A* + Smoothing + Reeds Sheep in Unity (C#)
<br></br>

# Architecture

# Architecture

# A*
The A* implementation is similar to the implementation done in the earlier A* project:

https://github.com/SenSaa/AStar

A graph of square grid cell nodes with uniform edge weights.

MinHeap...PriorityQueue...

Obstacles... Margin

Smoothing

# Reeds-Shepp
...
Path collision ... LineIntersection ...


# Path Smoothing
The approach used is gradient descent optimisation to produce a smooth path.
In this approach we minimise the distance between the points.
In the first step we minimise the difference between the original and smooth point, then in the second step we minimise the distance between the smooth points.
Details about the algorithm can be found in Sebastian Thrun's Udacity courses (AI for robotics and possibly in SDC):
https://www.youtube.com/watch?v=pjgfffzvbUg

Alternatively, another option that can work well is to use curve interpolation such as cubic splines (B-spline, hermite spline, etc.)

# Behaviour Planning
FSM...

# Path Tracking (Control)
Speed... PID...
Steer... Pure Pursuit
