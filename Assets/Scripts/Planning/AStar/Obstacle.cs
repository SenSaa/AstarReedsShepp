using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Obstacle
{
    public float x, y, width, height;
    public Obstacle(float x, float y, float width = 1, float height = 1)
    {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }
}
