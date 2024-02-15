using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// Obstacles are treated as rectangles and paths as lines.
// So we perform Line & Rectangle intersection (collision check).
// Based on this implementations:
// https://www.jeffreythompson.org/collision-detection/line-rect.php

public class LineSquareIntersection
{

    public static bool IsCollisionWithSquares(Path path, HashSet<Obstacle> obstacles, float obstacleSafetyMargin)
    {
        for (int i = 0; i < path.x.Count - 1; i++)
        {
            for (int j = 0; j < obstacles.Count; j++)
            {
                float x1 = (float)path.x[i];
                float y1 = (float)path.y[i];
                float x2 = (float)path.x[i + 1];
                float y2 = (float)path.y[i + 1];
                var rx = obstacles.ElementAt(j).x;
                var ry = obstacles.ElementAt(j).y;
                var rw = obstacles.ElementAt(j).width;
                var rh = obstacles.ElementAt(j).height;

                // Inflate obstacle on each side (for an additional padding)!
                var expandedRx = rx - obstacleSafetyMargin;
                var expandedRy = ry - obstacleSafetyMargin;
                var expandedRw = rw + obstacleSafetyMargin;
                var expandedRh = rh + obstacleSafetyMargin;

                if (LineRect(x1, y1, x2, y2, expandedRx, expandedRy, expandedRw, expandedRh))
                {
                    return true;
                }
            }
        }
        return false;
    }

    // Line-Rectangle intersection check.
    public static bool LineRect(float x1, float y1, float x2, float y2, float rx, float ry, float rw, float rh)
    {

        // Check if the line has hit any of the rectangle's sides.
        bool left = LineLine(x1, y1, x2, y2, rx, ry, rx, ry + rh);
        bool right = LineLine(x1, y1, x2, y2, rx + rw, ry, rx + rw, ry + rh);
        bool top = LineLine(x1, y1, x2, y2, rx, ry, rx + rw, ry);
        bool bottom = LineLine(x1, y1, x2, y2, rx, ry + rh, rx + rw, ry + rh);

        // If any of the above are true, the line intersects the rectangle.
        if (left || right || top || bottom)
        {
            return true;
        }
        return false;
    }

    // Line-Line intersection check.
    private static bool LineLine(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
    {

        // Calculate the direction of the lines.
        float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

        // If uA and uB are between 0-1, lines are intersecting.
        if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1)
        {

            // Optionally, draw a circle where the lines meet
            //float intersectionX = x1 + (uA * (x2 - x1));
            //float intersectionY = y1 + (uA * (y2 - y1));

            return true;
        }
        return false;
    }

}
