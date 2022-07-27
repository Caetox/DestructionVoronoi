using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using UnityEngine.Profiling;
public class Particle
{
    public Vector3 Pos;
	public Quaternion Rot;
	public Vector3 AngularVel;
	public Vector3 AngularMomentum;
	public int NumVertices;
    public int Offset;
    public Vector3 Vel;
	public Vector3 Acc;
	public float Mass;
	public float InvMass;
	public bool IsStatic;
	public float Lifetime;
	public Vector3 Ix;
	public Vector3 Iy;
	public Vector3 Iz;
	public Matrix4x4 Ibody;
	public Matrix4x4 IbodyInv;
	public Matrix4x4 I;

	public Particle(Vector3 pos, Quaternion rot, float mass, int numVertices, int offset, bool isStatic, float lifetime, Vector3 sidelength)
	{
        Pos = pos;
        Rot = rot;
        NumVertices = numVertices;
        Offset = offset;
		Vel = new Vector3();
		Acc = new Vector3();
		AngularVel = new Vector3();
		AngularMomentum = new Vector3();
		Mass = mass;
		InvMass = 1.0f / Mass;
		IsStatic = isStatic;
		Lifetime = lifetime;
		CalculateInertiaTensor(sidelength);
	}

	public void ApplyForce(Vector3 Location, Vector3 Force)
	{
		Acc += Force * InvMass;

		// Torque
		Vector3 Arm = Location - Pos;
		Vector3 Torque = Vector3.Cross(Arm, Force);

		AngularMomentum += I.MultiplyPoint(Torque);
	}

	public void CalculateInertiaTensor(Vector3 Sidelength)
	{
		float m = Mass / 12.0f;
		Vector3 bounds = Sidelength * 0.5f;
		Vector3 bounds2 = new Vector3(bounds.x * bounds.x, bounds.y * bounds.y, bounds.z * bounds.z);
		Ix = m * new Vector3(bounds2.y + bounds2.z, 0.0f, 0.0f);
		Iy = m * new Vector3(0.0f, bounds2.x + bounds2.z, 0.0f);
		Iz = m * new Vector3(0.0f, 0.0f, bounds2.x + bounds2.y);
		Ibody = new Matrix4x4(new Vector4(Ix.x, Ix.y, Ix.z, 0.0f), new Vector4(Iy.x, Iy.y, Iy.z, 0.0f), new Vector4(Iz.x, Iz.y, Iz.z, 0.0f), new Vector4(0,0,0,1));
		IbodyInv = Ibody.inverse;

		// I_inverse = R * Ibody_inverse * R_transposed
		Matrix4x4 rotationMatrix = Matrix4x4.TRS(new Vector3(), Rot, new Vector3(1, 1, 1));
		Matrix4x4 Il = IbodyInv * rotationMatrix.transpose;
		I = rotationMatrix * Il;
	}
}


public class DestructionController : MonoBehaviour
{
    public int number_of_seeds = 50;
	public int number_of_gameObjects = 10;
	//public int clustering_Factor = 5; // higher value -> seeds are closer to contact point

    public GameObject FragmentPrefab;

	public bool instantiateGameObjects;
	public bool instantiateParticles;

	private Vector3 objectSize;
    private DelaunayTriangulator delaunay = new DelaunayTriangulator();
    private Voronoi voronoi = new Voronoi();
	private EnclosePolygon enclose = new EnclosePolygon();

	private List<GameObject> ObjectPool;

    private List<Particle> Particles;
	private Polygon[] Polygons;
	private float WallMass;

	public GameObject WallObject;

	public Material WallMaterial;
	public Material SideMaterial;

	public Vector2 Stretching = new Vector2(1,1);

	public float ParticleThreshold = 1.0f;

	public bool IsAgainstWall = false;

	public bool debugging;

	void Start() {
		objectSize = WallObject.transform.localScale;

		if (instantiateGameObjects && FragmentPrefab != null)
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
        var impulse = Mathf.Abs((collision.impulse.x + collision.impulse.y + collision.impulse.z) / Time.fixedDeltaTime);
        WallMass = (int)collision.rigidbody.mass;
        number_of_seeds = (int)impulse/10;
        var seeds = delaunay.GenerateClusteredPoints(contactPoint, number_of_seeds, objectSize, WallMass, Stretching);
		Debug.Log("Impulse: " + impulse + "    mass: " + WallMass + "     number of seeds: " + number_of_seeds);

        // run delaunay triangulation
        var triangulation = delaunay.BowyerWatson(seeds);

		// construct voronoi diagram
		Profiler.BeginSample("Voronoi");
		Polygons = voronoi.GenerateEdgesFromDelaunay(seeds, triangulation, number_of_seeds, objectSize);
		Profiler.EndSample();
		// cut off polygons at the boundaries
		Debug.Log("object dimensions: " +(float)-0.5 * objectSize.x + " " + (float)0.5 * objectSize.x + " " + (float)-0.5 * objectSize.z + " " + (float)0.5 * objectSize.z);

		//GenerateFragments(Polygons, objectSize);

		for (int i = 0; i < Polygons.Length; i++)
		{
			if (Polygons[i] != null && Polygons[i].IsValid)
			{
				//Polygons[i] = enclose.enclosePoly(Polygons[i], -0.5f * objectSize.x, 0.5f * objectSize.x, -0.5f * objectSize.z, 0.5f * objectSize.z);
				//Polygons[i].Sort(objectSize);
			}
		}
		// Generates particles using game objects
		if (instantiateGameObjects)
		{
			GenerateFragments(Polygons, objectSize);
		}
		
		if (instantiateParticles)
		{
			Profiler.BeginSample("Meshes");
			// Generates own particle structure
			GenerateFragmentParticles(Polygons, collisionPosition, collision.impulse);
			GenerateMesh(objectSize);
			Profiler.EndSample();
		}

		Destroy(WallObject);

		if (debugging)
		{
			//visualization of delaunay triangulation
			var edges = new List<Edge>();
			foreach (var triangle in triangulation)
			{
				edges.Add(new Edge(triangle.Vertices[0], triangle.Vertices[1]));
				edges.Add(new Edge(triangle.Vertices[1], triangle.Vertices[2]));
				edges.Add(new Edge(triangle.Vertices[2], triangle.Vertices[0]));
			}
			foreach (var shiftedEdge in edges)
			{
				Debug.DrawLine(new Vector3((float)shiftedEdge.Point1.Loc.x - 15.0f, 0.1f, (float)shiftedEdge.Point1.Loc.z), new Vector3((float)shiftedEdge.Point2.Loc.x - 15.0f, 0.1f, (float)shiftedEdge.Point2.Loc.z), Color.white, 100f);
			}
		}
    }

    void GenerateFragments(IEnumerable<Polygon> polygons, Vector3 objectSize)
	{
        int poolIndex = 0;
		foreach (var polygon in polygons)
		{
            if (polygon != null && polygon.IsValid && polygon.Surface > ParticleThreshold)
            {
				//Debug.Log("P" + polygon.Edges.Count);
				//Debug.Log(polygon.Centroid.Index);
				//Debug.Log(polygon.Centroid.Corner);
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
                    Frag.Generate(polygon, objectSize.y, WallMaterial, SideMaterial, objectSize);

				if (!polygon.anchored)
				{
					Vector3 RandomForce = new Vector3(UnityEngine.Random.Range(-1, 1), UnityEngine.Random.Range(-1, 1), UnityEngine.Random.Range(-1, 1));
					Child.GetComponent<Rigidbody>().mass = polygon.Surface * objectSize.y;
					Child.GetComponent<Rigidbody>().AddForce(RandomForce * 100.0f);
					//Destroy(Child.GetComponent<Rigidbody>());
				}
				else 
				{
					Destroy(Child.GetComponent<Rigidbody>());
				}
                    
				Child.GetComponent<MeshRenderer>().material = WallMaterial;
                }
            }
		}
        //Destroy(gameObject);
    }

    void GenerateMesh(Vector3 objectSize)
    {
		List<Particle> CollisionResolveList = new List<Particle>();
		List<Vector3> CollisionResolvePointList = new List<Vector3>();

		Vector2 uvScale = new Vector2(1.0f / objectSize.x, -1.0f / objectSize.z);
		float depth = objectSize.y;
		Vector3 depthVec = new Vector3(0, -depth, 0);
		Vector3 halfDepthVec = new Vector3(0, depth * 0.5f, 0);

		MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
		MeshFilter meshFilter = GetComponent<MeshFilter>();
		MeshCollider meshCollider = GetComponent<MeshCollider>();
		Mesh mesh = new Mesh();

        // Count edges
        int OverallEdgeCount = 0;
        foreach (var polygon in Polygons)
        {
			if (polygon != null && polygon.IsValid && polygon.ParticleIndex != -1)
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
		//for (int k = 0; k < Particles.Count; ++k)
		foreach (var polygon in Polygons)
		{
			//Particle particle = Particles[k];
			//Polygon polygon = Polygons[k];
            if (polygon != null && polygon.IsValid && polygon.ParticleIndex != -1)
            {
				bool doCollisionCheck = false;
				Particle particle = Particles[polygon.ParticleIndex];

				int EdgeCount = polygon.Edges.Count;

				// Front

				for (int i = 0; i < EdgeCount; ++i)
				{
					Vector3 vecPos = polygon.Edges[i].Point2.Loc - polygon.Centroid.Loc + halfDepthVec;
					vecPos = particle.Rot * vecPos;
					vecPos += particle.Pos;
					vertices[vertexIndex++] = vecPos;
					normals[normalIndex++] = particle.Rot * Vector3.up;
					uv[uvIndex++] = (polygon.Edges[i].Point2.Uv * uvScale) + new Vector2(0.5f, 0.5f);
				}

				for (int i = 0; i < EdgeCount - 2; ++i)
				{
					triangles[triangleIndex++] = OverallIndexOffset + (2 + i);
					triangles[triangleIndex++] = OverallIndexOffset + (1 + i);
					triangles[triangleIndex++] = OverallIndexOffset + 0;
				}

				// Back
				for (int i = 0; i < EdgeCount; ++i)
				{
					Vector3 vecPos = polygon.Edges[i].Point2.Loc - polygon.Centroid.Loc - halfDepthVec;
					vecPos = particle.Rot * vecPos;
					vecPos += particle.Pos;
					vertices[vertexIndex++] = vecPos;
					normals[normalIndex++] = particle.Rot * -Vector3.up;
					uv[uvIndex++] = (polygon.Edges[i].Point2.Uv * uvScale) + new Vector2(0.5f, 0.5f);
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
					Vector3 vecPosA = polygon.Edges[i].Point2.Loc - polygon.Centroid.Loc + halfDepthVec;
					vecPosA = particle.Rot * vecPosA;
					vecPosA += particle.Pos;
					Vector3 vecPosB = polygon.Edges[i].Point2.Loc - polygon.Centroid.Loc - halfDepthVec;
					vecPosB = particle.Rot * vecPosB;
					vecPosB += particle.Pos;
					vertices[vertexIndex++] = vecPosA;
					vertices[vertexIndex++] = vecPosB;
					normals[normalIndex++] = Vector3.forward;
					normals[normalIndex++] = Vector3.forward;

					float u = (float)i / (float)EdgeCount;
					uv[uvIndex++] = new Vector2(u, 0.05f);
					uv[uvIndex++] = new Vector2(u, 0.0f);

					// Collision
					if (!doCollisionCheck && HasGroundCollision(vecPosA, particle))
					{
						doCollisionCheck = true;
						CollisionResolveList.Add(particle);
						CollisionResolvePointList.Add(vecPosA);
					}
					if (!doCollisionCheck && HasGroundCollision(vecPosB, particle))
					{
						doCollisionCheck = true;
						CollisionResolveList.Add(particle);
						CollisionResolvePointList.Add(vecPosB);
					}
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

					Vector3 normal = Polygon.CalcNormal(vertices[IndexOffset + 2 + (i * 2)], vertices[IndexOffset + 1 + (i * 2)], vertices[IndexOffset + 0 + (i * 2)]);

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
		meshRenderer.material = WallMaterial;

		for (int i = 0; i < CollisionResolveList.Count; ++i)
		{
			GroundCollision(CollisionResolvePointList[i], CollisionResolveList[i]);
		}
	}

	void GenerateFragmentParticles(IEnumerable<Polygon> polygons, Vector3 impactPoint, Vector3 impulse)
    {
		Particles = new List<Particle>();

		int index = 0;
		foreach (var polygon in polygons)
		{
			if (polygon != null && polygon.IsValid && polygon.Surface < ParticleThreshold)
			{
				Quaternion WallRotation = WallObject.transform.rotation;
				Vector3 localPosition = polygon.Centroid.Loc;
				Vector3 worldPosition = WallRotation * localPosition;
				Vector3 dir = (polygon.Centroid.Loc - impactPoint);
				Vector3 sidelength = new Vector3(polygon.Sidelength.x, objectSize.y, polygon.Sidelength.z);
				Particle p = new Particle(worldPosition, WallObject.transform.rotation, polygon.Surface * objectSize.y * WallMass, polygon.Edges.Count, 0, polygon.anchored, 35.0f, sidelength);
				Particles.Add(p);

				if (IsAgainstWall)
				{
					p.ApplyForce(impactPoint, impulse);
				}
				else
				{
					p.ApplyForce(impactPoint, -impulse);
				}
				polygon.ParticleIndex = index++;
			}
        }
    }

    void UpdateParticles()
	{
		if (Particles != null)
		{
			Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);
			float linearDamping = 0.99f;
			float angularDamping = 0.99f;
			float delta = Time.deltaTime;
			int numParticles = Particles.Count;
			for (int i = 0; i < numParticles; ++i)
			{
				Particle p = Particles[i];
				p.Lifetime -= delta;

				if (p.IsStatic)
				{
					continue;
				}

				if (p.Lifetime < 0.0f)
				{
					p.Pos = new Vector3(-10000.0f, -1000.0f, -10000.0f);
				}

				// Translation
				{
					p.Vel += Particles[i].Acc * delta;
					p.Pos += Particles[i].Vel * delta;
					p.Acc = new Vector3(0,0,0);
					p.Vel *= linearDamping;
				}

				// Rotation
				{
					// I_inverse = R * Ibody_inverse * R_transposed
					Matrix4x4 rotationMatrix = Matrix4x4.TRS(new Vector3(), p.Rot, new Vector3(1, 1, 1));
					Matrix4x4 Il = p.IbodyInv * rotationMatrix.transpose;
					p.I = rotationMatrix * Il;


					Quaternion q = new Quaternion(p.AngularVel.x * delta, p.AngularVel.y * delta, p.AngularVel.z * delta, 0.0f);
					Quaternion spin = q * p.Rot;
					Quaternion spin2 = new Quaternion(spin.x * 0.5f, spin.y * 0.5f, spin.z * 0.5f, spin.w * 0.5f);
					p.Rot = new Quaternion(p.Rot.x + spin2.x, p.Rot.y + spin2.y, p.Rot.z + spin2.z, p.Rot.w + spin2.w);
					p.Rot.Normalize();

					p.AngularVel += p.AngularMomentum * delta;
					p.AngularVel *= angularDamping;
					p.AngularMomentum = new Vector3();
				}

				p.Acc += gravity;
			}
			GenerateMesh(objectSize);
		}
	}

	private void Update()
	{
		UpdateParticles();
	}

	public bool HasGroundCollision(Vector3 vertex, Particle particle)
	{
		Vector3 Floor = new Vector3(0, -4.1f, 0);
		Vector3 FloorNormal = new Vector3(0, 1.0f, 0);
		float SoftMargin = 0.001f;
		return (vertex.y < Floor.y);
	}

	public void GroundCollision(Vector3 vertex, Particle particle)
	{
		Vector3 Floor = new Vector3(0, -4.1f, 0);
		Vector3 FloorNormal = new Vector3(0, 1.0f, 0);
		float SoftMargin = 0.001f;

		// Resolve collision
		float depth = (Floor.y - vertex.y);
		particle.Pos += (FloorNormal * depth);// + (FloorNormal * SoftMargin * 0.5f);

		Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);
		particle.Acc -= gravity;
		particle.Acc = new Vector3(0,0,0);

		// Apply force
		//particle.ApplyForce(vertex, -particle.Vel * particle.Mass);
		if (depth > SoftMargin)
		{
			Vector3 dir = vertex;
			particle.ApplyForce(vertex, FloorNormal * particle.Mass * 1.0f);
		}
		
	}
}
