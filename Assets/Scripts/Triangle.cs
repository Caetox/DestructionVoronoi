using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Triangle
{
    public Point[] Vertices { get; } = new Point[3];
    public Point Circumcenter;
    public float RadiusSquared;

    public IEnumerable<Triangle> TrianglesWithSharedEdge {
        get {
            var neighbors = new HashSet<Triangle>();
            foreach (var vertex in Vertices)
            {
                var trianglesWithSharedEdge = vertex.AdjacentTriangles.Where(o =>
                {
                    return o != this && SharesEdgeWith(o);
                });
                neighbors.UnionWith(trianglesWithSharedEdge);
            }

            return neighbors;
        }
    }

    public Triangle(Point point1, Point point2, Point point3)
    {
        if (!IsCounterClockwise(point1, point2, point3))
        {
            Vertices[0] = point1;
            Vertices[1] = point3;
            Vertices[2] = point2;
        }
        else
        {
            Vertices[0] = point1;
            Vertices[1] = point2;
            Vertices[2] = point3;
        }

        Vertices[0].AdjacentTriangles.Add(this);
        Vertices[1].AdjacentTriangles.Add(this);
        Vertices[2].AdjacentTriangles.Add(this);

		var p0 = Vertices[0].Loc;
		var p1 = Vertices[1].Loc;
		var p2 = Vertices[2].Loc;
		var dA = p0.x * p0.x + p0.z * p0.z;
		var dB = p1.x * p1.x + p1.z * p1.z;
		var dC = p2.x * p2.x + p2.z * p2.z;

		var aux1 = (dA * (p2.z - p1.z) + dB * (p0.z - p2.z) + dC * (p1.z - p0.z));
		var aux2 = -(dA * (p2.x - p1.x) + dB * (p0.x - p2.x) + dC * (p1.x - p0.x));
		var div = (2 * (p0.x * (p2.z - p1.z) + p1.x * (p0.z - p2.z) + p2.x * (p1.z - p0.z)));

		if (Mathf.Abs(div) > 0)
		{
			var center = new Vector3(aux1 / div, 0, aux2 / div);
			Circumcenter = new Point(center.x, center.z);
			RadiusSquared = (center.x - p0.x) * (center.x - p0.x) + (center.z - p0.z) * (center.z - p0.z);
		}
	}

	private Vector3 GetNormal(Point point1, Point point2, Point point3)
	{
		Vector3 side1 = point2.Loc - point1.Loc;
		Vector3 side2 = point3.Loc - point1.Loc;
		Vector3 x = Vector3.Cross(side1, side2).normalized;
		return x;
	}
	private bool IsCounterClockwise(Point point1, Point point2, Point point3)
    {
        return GetNormal(point1, point2, point3).y > 0;
    }

    public bool SharesEdgeWith(Triangle triangle)
    {
        var sharedVertices = Vertices.Where(o => triangle.Vertices.Contains(o)).Count();
        return sharedVertices == 2;
    }

    public bool IsPointInsideCircumcircle(Point point)
    {
        var d_squared = (point.Loc.x - Circumcenter.Loc.x) * (point.Loc.x - Circumcenter.Loc.x) +
            (point.Loc.z - Circumcenter.Loc.z) * (point.Loc.z - Circumcenter.Loc.z);
        return d_squared < RadiusSquared;
    }
}
