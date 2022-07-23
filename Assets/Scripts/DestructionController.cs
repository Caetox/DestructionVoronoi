using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Particle
{
    public Vector3 Pos;
    public Quaternion Rot;
    public int NumVertices;
    public int Offset;
    public Vector3 Vel;
	public Vector3 Acc;

	public Particle(Vector3 pos, Quaternion rot, Vector3 acc, int numVertices, int offset)
	{
        Pos = pos;
        Rot = rot;
        NumVertices = numVertices;
        Offset = offset;
		Vel = new Vector3();
		Acc = acc;
	}
}


public class DestructionController : MonoBehaviour
{
    public int number_of_seeds = 50;
	public int number_of_gameObjects = 10;
	public int clustering_Factor = 5; // higher value -> seeds are closer to contact point

    public GameObject FragmentPrefab;

	public bool instantiate_game_objects;

	private Vector3 objectSize;
	private Vector2 shift;
    private DelaunayTriangulator delaunay = new DelaunayTriangulator();
    private Voronoi voronoi = new Voronoi();

	private List<GameObject> ObjectPool;

    private List<Particle> Particles;
	private List<Polygon> Polygons;

	public GameObject WallObject;

	void Start() {
		objectSize = WallObject.transform.localScale;
		shift = new Vector2(objectSize.x * 0.5f, objectSize.z * 0.5f);

		if (instantiate_game_objects && FragmentPrefab != null)
		{
			ObjectPool = new List<GameObject>();
			for (int i = 0; i < number_of_seeds; ++i)
			{
				GameObject obj = Instantiate<GameObject>(FragmentPrefab, transform.position, transform.rotation);
				ObjectPool.Add(obj);
			}
		}
    }


    void OnCollisionEnter(Collision collision)
    {

        // Only care about collisions with objects that have a projectile-component.
        Projectile projectile = collision.gameObject.GetComponent<Projectile>();
        if (projectile == null)
		{
            return;
		}

        Vector3 collisionPosition = WallObject.transform.InverseTransformPoint(collision.contacts[0].point);

		// location of collision
		var contactPoint = new Vector2(collisionPosition.x * objectSize.x, collisionPosition.z * objectSize.z);

		// generate seeds
		var seeds = delaunay.GenerateClusteredPoints(contactPoint, number_of_seeds, objectSize, clustering_Factor, -shift);

        // run delaunay triangulation
        var triangulation = delaunay.BowyerWatson(seeds, -shift);

		// construct voronoi diagram
		Polygons = voronoi.GenerateEdgesFromDelaunay(seeds, triangulation);

		// Generates particles using game objects
		if (instantiate_game_objects)
		{
			GenerateFragments(Polygons, -shift, objectSize);
		}
		else
		{
			// Generates own particle structure
			GenerateFragmentParticles(Polygons, collision.contacts[0].otherCollider.transform.position, collision.impulse);
			GenerateMesh(objectSize);
		}

		Destroy(WallObject);

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

        
    }

    void GenerateFragments(IEnumerable<Polygon> polygons, Vector2 shift, Vector3 objectSize)
	{
        int poolIndex = 0;
        Vector3 projectedShift = new Vector3(shift.x, 0, shift.y);
		foreach (var polygon in polygons)
		{
            if (polygon.IsValid)
            {

				Quaternion WallRotation = WallObject.transform.rotation;
				Vector3 localPosition = polygon.Centroid.Loc;
				Vector3 worldPosition = WallRotation * localPosition;
				worldPosition += WallObject.transform.position;
				GameObject Child = ObjectPool[poolIndex++];
                if (Child != null)
                {
                    Child.transform.position = worldPosition;
                    Child.transform.rotation = WallObject.transform.rotation;
                    Fragment Frag = Child.GetComponent<Fragment>();
                    Frag.Generate(polygon, objectSize.y);

                    Vector3 RandomForce = new Vector3(UnityEngine.Random.Range(-1, 1), UnityEngine.Random.Range(-1, 1), UnityEngine.Random.Range(-1, 1));
                    Child.GetComponent<Rigidbody>().AddForce(RandomForce * 100.0f);
                }
            }
		}
        Destroy(gameObject);
    }

    void GenerateMesh(Vector3 objectSize)
    {
        float depth = objectSize.y;

		MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
		MeshFilter meshFilter = GetComponent<MeshFilter>();
		MeshCollider meshCollider = GetComponent<MeshCollider>();
		Mesh mesh = new Mesh();

        // Count edges
        int OverallEdgeCount = 0;
        foreach (var polygon in Polygons)
        {
            if (polygon.IsValid)
            {
                OverallEdgeCount += polygon.Edges.Count;
            }
        }

		if (OverallEdgeCount == 0)
		{
			return;
		}

        // Create buffers
		Vector3[] vertices = new Vector3[OverallEdgeCount * 4];
		Vector3[] normals = new Vector3[OverallEdgeCount * 4];
		Vector2[] uv = new Vector2[OverallEdgeCount * 4];
		int[] triangles = new int[(OverallEdgeCount * 2 + (2 * (OverallEdgeCount - 2))) * 3];

		int vertexIndex = 0;
		int normalIndex = 0;
		int uvIndex = 0;
		int triangleIndex = 0;

		// Generate vertices
		int OverallIndexOffset = 0;
		for (int k = 0; k < Particles.Count; ++k)
        {
			Particle particle = Particles[k];
			Polygon polygon = Polygons[k];
            if (polygon.IsValid)
            {


                int EdgeCount = polygon.Edges.Count;

				// Front

				for (int i = 0; i < EdgeCount; ++i)
				{
					Vector3 vecPos = polygon.Edges[i].Point2.Loc - polygon.Centroid.Loc;
					vecPos = particle.Rot * vecPos;
					vecPos += particle.Pos;
					vertices[vertexIndex++] = vecPos;
					normals[normalIndex++] = particle.Rot * -Vector3.forward;
					uv[uvIndex++] = polygon.Edges[i].Point2.Uv;
				}

				for (int i = 0; i < EdgeCount - 2; ++i)
				{
					triangles[triangleIndex++] = OverallIndexOffset + (2 + i);
					triangles[triangleIndex++] = OverallIndexOffset + (1 + i);
					triangles[triangleIndex++] = OverallIndexOffset + 0;
				}

				// Back
				Vector3 depthVec = new Vector3(0, -depth, 0);
				for (int i = 0; i < EdgeCount; ++i)
				{
					Vector3 vecPos = polygon.Edges[i].Point2.Loc + depthVec - polygon.Centroid.Loc;
					vecPos = particle.Rot * vecPos;
					vecPos += particle.Pos;
					vertices[vertexIndex++] = vecPos;
					normals[normalIndex++] = particle.Rot * Vector3.forward;
					uv[uvIndex++] = polygon.Edges[i].Point2.Uv;
				}

				for (int i = 0; i < EdgeCount - 2; ++i)
				{
					triangles[triangleIndex++] = OverallIndexOffset + EdgeCount + 0;
					triangles[triangleIndex++] = OverallIndexOffset + EdgeCount + (1 + i);
					triangles[triangleIndex++] = OverallIndexOffset + EdgeCount + (2 + i);
				}

				// Sides

				for (int i = 0; i < EdgeCount; ++i)
				{
					Vector3 vecPosA = polygon.Edges[i].Point2.Loc - polygon.Centroid.Loc;
					vecPosA = particle.Rot * vecPosA;
					vecPosA += particle.Pos;
					Vector3 vecPosB = polygon.Edges[i].Point2.Loc + depthVec - polygon.Centroid.Loc;
					vecPosB = particle.Rot * vecPosB;
					vecPosB += particle.Pos;
					vertices[vertexIndex++] = vecPosA;
					vertices[vertexIndex++] = vecPosB;
					normals[normalIndex++] = Vector3.forward;
					normals[normalIndex++] = Vector3.forward;

					float u = (float)i / (float)EdgeCount;
					uv[uvIndex++] = new Vector2(u, 1.0f);
					uv[uvIndex++] = new Vector2(u, 0.0f);
				}

				int IndexOffset = EdgeCount * 2;
				for (int i = 0; i < EdgeCount - 2; ++i)
				{
					triangles[triangleIndex++] = OverallIndexOffset + IndexOffset + 2 + (i * 2);
					triangles[triangleIndex++] = OverallIndexOffset + IndexOffset + 1 + (i * 2);
					triangles[triangleIndex++] = OverallIndexOffset + IndexOffset + 0 + (i * 2);
					triangles[triangleIndex++] = OverallIndexOffset + IndexOffset + 2 + (i * 2);
					triangles[triangleIndex++] = OverallIndexOffset + IndexOffset + 3 + (i * 2);
					triangles[triangleIndex++] = OverallIndexOffset + IndexOffset + 1 + (i * 2);

					Vector3 normal = particle.Rot * Polygon.CalcNormal(vertices[IndexOffset + 2 + (i * 2)], vertices[IndexOffset + 1 + (i * 2)], vertices[IndexOffset + 0 + (i * 2)]);

					normals[IndexOffset + 2 + (i * 2)] = normal;
					normals[IndexOffset + 1 + (i * 2)] = normal;
					normals[IndexOffset + 0 + (i * 2)] = normal;
					normals[IndexOffset + 2 + (i * 2)] = normal;
					normals[IndexOffset + 3 + (i * 2)] = normal;
					normals[IndexOffset + 1 + (i * 2)] = normal;
				}

				// Debug
				//Vector3 testoffset = new Vector3(0, Random.Range(-0.01f, 0.01f), 0);
				//Color randomColor = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
				//for (int i = 1; i < EdgeCount; ++i)
				//{
				//	Debug.DrawLine(vertices[OverallIndexOffset + i] + testoffset, vertices[OverallIndexOffset + i - 1] + testoffset, randomColor, 100f);
				//}

				OverallIndexOffset = vertexIndex;
			}
        }

		// Apply

		mesh.vertices = vertices;
		mesh.uv = uv;
		mesh.normals = normals;
		mesh.triangles = triangles;
		meshFilter.mesh = mesh;
	}

	void GenerateFragmentParticles(IEnumerable<Polygon> polygons, Vector3 impactPoint, Vector3 impulse)
    {
		Particles = new List<Particle>();

		foreach (var polygon in polygons)
        {
			Quaternion WallRotation = WallObject.transform.rotation;
			Vector3 localPosition = polygon.Centroid.Loc;
			Vector3 worldPosition = WallRotation * localPosition;
			Vector3 dir = (polygon.Centroid.Loc - impactPoint);
			float distance = 1.0f / dir.magnitude * dir.magnitude * dir.magnitude;
			Vector3 acc = (new Vector3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z) * 10.0f - (impulse * distance)) * 0.20f;
			acc += new Vector3(UnityEngine.Random.Range(-1.0f, 1.0f), UnityEngine.Random.Range(-1.0f, 1.0f), UnityEngine.Random.Range(-1.0f, 1.0f)) * 100.0f;
			Particle p = new Particle(worldPosition, WallObject.transform.rotation, acc, polygon.Edges.Count, 0);
			Particles.Add(p);
        }
    }

    void UpdateParticles()
	{
		if (Particles != null)
		{
			Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);
			float linearDamping = 0.0001f;
			float delta = Time.deltaTime;
			int numParticles = Particles.Count;
			for (int i = 0; i < numParticles; ++i)
			{
				Particle p = Particles[i];
				p.Pos += Particles[i].Vel * delta;
				p.Vel += (Particles[i].Acc + gravity) * delta;
				p.Acc *= linearDamping;
			}
			GenerateMesh(objectSize);
		}
	}

	private void Update()
	{
		UpdateParticles();
	}
}
