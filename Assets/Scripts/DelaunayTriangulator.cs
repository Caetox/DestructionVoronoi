using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class DelaunayTriangulator
{
	private float MinX { get; set; }
	private float MinY { get; set; }
	private float MaxX { get; set; }
    private float MaxY { get; set; }

    static float NormalizedRandom(float mean, float stddev) {
		var u1 = UnityEngine.Random.value;
		var u2 = UnityEngine.Random.value;
		var randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
		    Mathf.Sin(2.0f * Mathf.PI * u2);

		return mean + stddev * randStdNormal;
}

    public IEnumerable<Point> GenerateClusteredPoints(Vector2 contactPoint, int amount, Vector3 objectSize, float radius, Vector2 Stretching)
    {
		MinX = -0.5f * objectSize.x;
		MinY = -0.5f * objectSize.z;
		MaxX =  0.5f * objectSize.x;
        MaxY =  0.5f * objectSize.z;
        UnityEngine.Random.InitState(2);

        var points = new List<Point>() {};

        int pointIndex = 0;
		for (int i = 0; i < amount; i++) {
		    var dist = Mathf.Abs(NormalizedRandom(0.2f, 1.0f/2.0f));
		    var angle = 2.0f * Mathf.PI * UnityEngine.Random.value;

		    Vector2 seed = contactPoint + (new Vector2(
				dist * Mathf.Cos(angle) * radius,
				dist * Mathf.Sin(angle) * radius)* Stretching);

            //seed *= Stretching;

            if ((seed.x >= MinX) && (seed.x <= MaxX) && (seed.y >= MinY) && (seed.y <= MaxY)) {
                points.Add(new Point(seed.x, seed.y, pointIndex++));
            }
		}

        return points;
            
    }

    public IEnumerable<Triangle> BowyerWatson(IEnumerable<Point> points)
    {
        var point0 = new Point(MinX, MinY);
        var point1 = new Point(MinX, MaxY);
        var point2 = new Point(MaxX, MaxY);
        var point3 = new Point(MaxX, MinY);
        var tri1 = new Triangle(point0, point1, point2);
        var tri2 = new Triangle(point0, point2, point3);
        point0.Corner = true;
        point1.Corner = true;
        point2.Corner = true;
		point3.Corner = true;

        IEnumerable<Triangle> border = new List<Triangle>() { tri1, tri2 };
        List<Point> bounds = new List<Point>() { point0, point1, point2, point3 };

        var triangulation = new HashSet<Triangle>(border);

        foreach (var point in points)
        {
            var badTriangles = FindBadTriangles(point, triangulation);
            var polygon = FindHoleBoundaries(badTriangles);

            foreach (var triangle in badTriangles)
            {
                foreach (var vertex in triangle.Vertices)
                {
                    vertex.AdjacentTriangles.Remove(triangle);
                }
            }
            triangulation.RemoveWhere(o => badTriangles.Contains(o));

            foreach (var edge in polygon.Where(possibleEdge => possibleEdge.Point1 != point && possibleEdge.Point2 != point))
            {
                var triangle = new Triangle(point, edge.Point1, edge.Point2);
                triangulation.Add(triangle);
            }
        }

		//remove border triangles
		HashSet<Triangle> borderTriangles = new HashSet<Triangle>();
		foreach (var triangle in triangulation)
		{
			foreach (var point in bounds)
			{
				if (((triangle.Vertices[0] == point) || (triangle.Vertices[1] == point) || (triangle.Vertices[2] == point)))
				{
					//borderTriangles.Add(triangle);
                    triangle.Vertices[0].anchored = true;
                    triangle.Vertices[1].anchored = true;
                    triangle.Vertices[2].anchored = true;
				}
			}
		}

		/* triangulation.RemoveWhere(o => borderTriangles.Contains(o));

		foreach (var triangle in borderTriangles)
		{
			foreach (var vertex in triangle.Vertices)
			{
				vertex.AdjacentTriangles.Remove(triangle);
			}
		} */

		return triangulation;
    }

    private List<Edge> FindHoleBoundaries(ISet<Triangle> badTriangles)
    {
        var edges = new List<Edge>();
        foreach (var triangle in badTriangles)
        {
            edges.Add(new Edge(triangle.Vertices[0], triangle.Vertices[1]));
            edges.Add(new Edge(triangle.Vertices[1], triangle.Vertices[2]));
            edges.Add(new Edge(triangle.Vertices[2], triangle.Vertices[0]));
        }
        var grouped = edges.GroupBy(o => o);
        var boundaryEdges = edges.GroupBy(o => o).Where(o => o.Count() == 1).Select(o => o.First());
        return boundaryEdges.ToList();
    }

    private ISet<Triangle> FindBadTriangles(Point point, HashSet<Triangle> triangles)
    {
        var badTriangles = triangles.Where(o => o.IsPointInsideCircumcircle(point));
        return new HashSet<Triangle>(badTriangles);
    }
}
