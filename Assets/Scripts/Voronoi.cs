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
                bool cornerA = neighbor.Vertices[0].Corner;
                bool cornerB = neighbor.Vertices[1].Corner;
				bool cornerC = neighbor.Vertices[2].Corner;
				for (int i = 0; i < 3; ++i)
                {
                    if (triangle.Vertices[i].AdjacentTriangles.Contains(neighbor))
                    {
                        if (edge == null)
                        {
                            if (cornerA || cornerB || cornerC)
                            {
                                if (cornerA)
                                    edge = new Edge(triangle.Circumcenter, neighbor.Vertices[0]);
								if (cornerB)
									edge = new Edge(triangle.Circumcenter, neighbor.Vertices[1]);
								if (cornerC)
									edge = new Edge(triangle.Circumcenter, neighbor.Vertices[2]);
							}
                            else
                            {
                                edge = new Edge(triangle.Circumcenter, neighbor.Circumcenter);
                            }
                        }
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
