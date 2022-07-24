using System.Collections.Generic;
using UnityEngine;

public class Voronoi
{
    public Polygon[] GenerateEdgesFromDelaunay(IEnumerable<Point> points, IEnumerable<Triangle> triangulation, int numSeeds)
    {
        Polygon[] polys = new Polygon[numSeeds];

        int c = 0;
        foreach (var point in points)
        {
            polys[c++] = new Polygon(point);
        }

        //int[,] edges = new int[polygons.Count, 16];
		//int[] edgesIndex = new int[polygons.Count];

		foreach (var triangle in triangulation)
        {
			foreach (var neighbor in triangle.TrianglesWithSharedEdge)
            {
                Edge edge = null;
                // Find the vertices that both triangles share
                for (int i = 0; i < 3; ++i)
                {
                    if (triangle.Vertices[i].AdjacentTriangles.Contains(neighbor))
                    {
                        if (edge == null)
                            edge = new Edge(triangle.Circumcenter, neighbor.Circumcenter);
                        // Add edges to the related polygons
                        polys[triangle.Vertices[i].Index].Edges.Add(edge);
                    }
                }
            }
        }

        for (int i = 0; i < c; ++i)
		{
            polys[i].Sort();
		}

		return polys;
    }
}
