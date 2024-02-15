using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace astar
{
    public class Graph
    {
        private float GridCellSize;
        private float GridWidth;
        private float GridHeight;

        public Graph(float GridCellSize)
        {
            this.GridCellSize = GridCellSize;
        }

        public void SetGridCellSize(float GridCellSize)
        {
            this.GridCellSize = GridCellSize;
        }
        public float GetGridCellSize()
        {
            return this.GridCellSize;
        }

        public void SetGridWidth(float GridWidth)
        {
            this.GridWidth = GridWidth;
        }
        public float GetGridWidth()
        {
            return this.GridWidth;
        }

        public void SetGridHeight(float GridHeight)
        {
            this.GridHeight = GridHeight;
        }
        public float GetGridHeight()
        {
            return this.GridHeight;
        }


    }
}