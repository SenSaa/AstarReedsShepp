using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace astar
{
    //public class Node
    public class Node : IComparable<Node>
    {

        private (float x, float z) Position;
        private float Heading;
        private float gCost;
        private float hCost;
        private float fCost;
        private List<Node> Neighbours;
        private Node Parent;


        public Node((float x, float z) Position)
        {
            this.Position = Position;
        }


        public (float x, float z) GetNodePosition()
        {
            return Position;
        }

        public void SetHeading(float Heading)
        {
            this.Heading = Heading;
        }
        public float GetHeading()
        {
            return Heading;
        }

        public void SetGcost(float cost)
        {
            this.gCost = cost;
        }
        public float GetGcost()
        {
            return this.gCost;
        }

        public void SetHcost(float cost)
        {
            this.hCost = cost;
        }
        public float GetHcost()
        {
            return this.hCost;
        }

        public void SetFcost(float cost)
        {
            this.fCost = cost;
        }
        public float GetFcost()
        {
            return this.fCost;
        }


        public void SetNeighbours(List<Node> Neighbours)
        {
            this.Neighbours = Neighbours;
        }
        public List<Node> GetNeighbours()
        {
            return this.Neighbours;
        }


        public void SetParent(Node Parent)
        {
            this.Parent = Parent;
        }

        public Node GetParent()
        {
            return this.Parent;
        }

        public int CompareTo(Node other)
        {
            return this.GetFcost().CompareTo(other.GetFcost());
        }

    }
}
