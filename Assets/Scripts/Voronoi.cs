using System.Collections.Generic;
using UnityEngine;

public class Voronoi
{
    public IEnumerable<Polygon> GenerateEdgesFromDelaunay(IEnumerable<Point> points, IEnumerable<Triangle> triangulation)
    {
		List<Polygon> polygons = new List<Polygon>();

        foreach (var point in points)
        {
            Polygon poly = new Polygon(point);
			polygons.Add(poly);

            //Debug.Log("polygon: " + point.ToString());
        }

        foreach (var triangle in triangulation)
        {
			foreach (var neighbor in triangle.TrianglesWithSharedEdge)
            {
                var edge = new Edge(triangle.Circumcenter, neighbor.Circumcenter);

                // Find the vertices that both triangles share
                for (int i = 0; i < 3; ++i)
                {
                    if (triangle.Vertices[i].AdjacentTriangles.Contains(neighbor))
                    {
                        //Debug.Log("index " + i + ": " + triangle.Vertices[i].ToString());

                        // Add edges to the related polygons
                        foreach (var polygon in polygons)
                        {
                            if (polygon.Centroid.Equals(triangle.Vertices[i]))
							{
                                polygon.Edges.Add(edge);
							}
                        }
                    }
                }
            }
        }

        // Sort edges in polygons
        foreach (var poly in polygons)
        {
            poly.Sort();
        }

		return polygons;
    }
}
