using System.Collections.Generic;


public class Voronoi
{
    public IEnumerable<Polygon> GenerateEdgesFromDelaunay(IEnumerable<Point> points, IEnumerable<Triangle> triangulation)
    {
		List<Polygon> polygons = new List<Polygon>();
        foreach (var point in points)
        {
            Polygon poly = new Polygon(point.Loc);
			polygons.Add(poly);
		}

		int index = 0;
        foreach (var triangle in triangulation)
        {
            polygons.Add(new Polygon());
            polygons[index].Edges = new List<Edge>();
			foreach (var neighbor in triangle.TrianglesWithSharedEdge)
            {
                var edge = new Edge(triangle.Circumcenter, neighbor.Circumcenter);
                polygons[index].Edges.Add(edge);
            }
            index++;
        }


        return polygons;
    }
}
