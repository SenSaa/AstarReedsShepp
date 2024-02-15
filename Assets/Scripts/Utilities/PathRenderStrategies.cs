using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathRenderStrategies
{
    public class PathRenderWihColor : IPathRenderDesign
    {
        private readonly Color color;

        public PathRenderWihColor(Color color)
        {
            this.color = color;
        }

        public LineRenderer ApplyRenderDesign(LineRenderer lineRenderer)
        {
            lineRenderer.material.color = color;
            return lineRenderer;
        }
    }

    public class PathRenderWihMaterial : IPathRenderDesign
    {
        private readonly Material material;

        public PathRenderWihMaterial(Material material)
        {
            this.material = material;
        }
        public LineRenderer ApplyRenderDesign(LineRenderer lineRenderer)
        {
            lineRenderer.material = material;
            return lineRenderer;
        }
    }
}
