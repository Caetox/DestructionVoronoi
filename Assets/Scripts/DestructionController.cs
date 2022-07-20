using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class DestructionController : MonoBehaviour
{
    public int number_of_seeds = 50;
    public int clustering_Factor = 5; // higher value -> seeds are closer to contact point

    public GameObject FragmentPrefab;

    private Vector3 objectSize;
	private Vector2 shift;
    private DelaunayTriangulator delaunay = new DelaunayTriangulator();
    private Voronoi voronoi = new Voronoi();


    void Start() {
        Collider collider = gameObject.GetComponent<Collider>();
        objectSize = collider.bounds.size;
        shift = new Vector2(collider.bounds.min.x, collider.bounds.min.z);
    }


    void OnCollisionEnter(Collision collision)
    {
        // location of collision
        var contactPoint = new Vector2(collision.contacts[0].point.x - shift.x, collision.contacts[0].point.z - shift.y);

        // generate seeds
        var seeds = delaunay.GenerateClusteredPoints(contactPoint, number_of_seeds, objectSize, clustering_Factor, shift);

        // run delaunay triangulation
        var triangulation = delaunay.BowyerWatson(seeds, shift);

        // construct voronoi diagram
        var polygons = voronoi.GenerateEdgesFromDelaunay(seeds, triangulation);

        GenerateFragments(polygons, shift, objectSize);

		// visualization of delaunay triangulation
		//var shiftedTriangulation = new HashSet<Triangle>();
		//foreach (var triangle in triangulation) {
		//    var p1 = new Point(triangle.Vertices[0].X + shift.x, triangle.Vertices[0].Y + shift.y);
		//    var p2 = new Point(triangle.Vertices[1].X + shift.x, triangle.Vertices[1].Y + shift.y);
		//    var p3 = new Point(triangle.Vertices[2].X + shift.x, triangle.Vertices[2].Y + shift.y);
		//    var shiftedTriangle = new Triangle(p1, p2, p3);
		//    shiftedTriangulation.Add(shiftedTriangle);
		//}
		var shiftedEdges = new List<Edge>();
		foreach (var shiftedTriangle in triangulation)
		{
			shiftedEdges.Add(new Edge(shiftedTriangle.Vertices[0], shiftedTriangle.Vertices[1]));
			shiftedEdges.Add(new Edge(shiftedTriangle.Vertices[1], shiftedTriangle.Vertices[2]));
			shiftedEdges.Add(new Edge(shiftedTriangle.Vertices[2], shiftedTriangle.Vertices[0]));
		}
		foreach (var shiftedEdge in shiftedEdges) {
            Debug.DrawLine(new Vector3((float)shiftedEdge.Point1.Loc.x, 0.1f, (float)shiftedEdge.Point1.Loc.z),new Vector3((float)shiftedEdge.Point2.Loc.x, 0.1f, (float)shiftedEdge.Point2.Loc.z), Color.grey, 100f);
        }

        // visualization of voronoi edges
        //foreach (var polygon in polygons)
        //{
        //    foreach (var edge in polygon.Edges)
        //    {
        //        Debug.DrawLine(new Vector3((float)edge.Point1.X, 0.1f, (float)edge.Point1.Y), new Vector3((float)edge.Point2.X, 0.1f, (float)edge.Point2.Y), Color.white, 100f);
        //    }
        //}

        // Update own mesh

        Destroy(gameObject);
    }

    void GenerateFragments(IEnumerable<Polygon> polygons, Vector2 shift, Vector3 objectSize)
	{
        Vector3 projectedShift = new Vector3(shift.x, 0, shift.y);
		foreach (var polygon in polygons)
		{
            if (polygon.IsValid)
            {
                GameObject Child = Instantiate<GameObject>(FragmentPrefab, polygon.Centroid.Loc + projectedShift, Quaternion.identity);
                Fragment Frag = Child.GetComponent<Fragment>();
                Frag.Generate(polygon, objectSize.y);

                Vector3 RandomForce = new Vector3(UnityEngine.Random.Range(-1, 1), UnityEngine.Random.Range(-1, 1), UnityEngine.Random.Range(-1, 1));
                Child.GetComponent<Rigidbody>().AddForce(RandomForce * 100.0f);
            }
		}
	}
}
