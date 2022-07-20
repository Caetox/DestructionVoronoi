using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Fragment : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void Generate(Polygon Data, float depth)
	{
		MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
		MeshFilter meshFilter = GetComponent<MeshFilter>();
		MeshCollider meshCollider = GetComponent<MeshCollider>();
		Mesh mesh = new Mesh();

		int EdgeCount = Data.Edges.Count;

		Vector3[] vertices = new Vector3[EdgeCount * 4];
		Vector3[] normals = new Vector3[EdgeCount * 4];
		Vector2[] uv = new Vector2[EdgeCount * 4];
		int[] triangles = new int[(EdgeCount * 2 + (2 * (EdgeCount - 2))) * 3];

		int vertexIndex = 0;
		int normalIndex = 0;
		int uvIndex = 0;
		int triangleIndex = 0;

		// Front

		for (int i = 0; i < EdgeCount; ++i)
		{	
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc - Data.Centroid.Loc;
			normals[normalIndex++] = -Vector3.forward;
			uv[uvIndex++] = Data.Edges[i].Point2.Uv / 10.0f;
		}

		for (int i = 0; i < EdgeCount - 2; ++i)
		{
			triangles[triangleIndex++] = (2 + i);
			triangles[triangleIndex++] = (1 + i);
			triangles[triangleIndex++] = 0;
		}

		// Back
		Vector3 depthVec = new Vector3(0, -depth, 0);
		for (int i = 0; i < EdgeCount; ++i)
		{
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc - Data.Centroid.Loc + depthVec;
			normals[normalIndex++] = Vector3.forward;
			uv[uvIndex++] = Data.Edges[i].Point2.Uv / 10.0f;
		}

		for (int i = 0; i < EdgeCount - 2; ++i)
		{
			triangles[triangleIndex++] = EdgeCount + 0;
			triangles[triangleIndex++] = EdgeCount + (1 + i);
			triangles[triangleIndex++] = EdgeCount + (2 + i);
		}

		// Sides

		for (int i = 0; i < EdgeCount; ++i)
		{
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc - Data.Centroid.Loc;
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc - Data.Centroid.Loc + depthVec;
			normals[normalIndex++] = Vector3.forward;
			normals[normalIndex++] = Vector3.forward;

			float u = (float)i / (float)EdgeCount;
			uv[uvIndex++] = new Vector2(u, 1.0f);
			uv[uvIndex++] = new Vector2(u, 0.0f);
		}

		int IndexOffset = EdgeCount * 2;
		for (int i = 0; i < EdgeCount - 2; ++i)
		{
			triangles[triangleIndex++] = IndexOffset + 2 + (i * 2);
			triangles[triangleIndex++] = IndexOffset + 1 + (i * 2);
			triangles[triangleIndex++] = IndexOffset + 0 + (i * 2);
			triangles[triangleIndex++] = IndexOffset + 2 + (i * 2);
			triangles[triangleIndex++] = IndexOffset + 3 + (i * 2);
			triangles[triangleIndex++] = IndexOffset + 1 + (i * 2);

			Vector3 normal = Polygon.CalcNormal(vertices[IndexOffset + 2 + (i * 2)], vertices[IndexOffset + 1 + (i * 2)], vertices[IndexOffset + 0 + (i * 2)]);

			normals[IndexOffset + 2 + (i * 2)] = normal;
			normals[IndexOffset + 1 + (i * 2)] = normal;
			normals[IndexOffset + 0 + (i * 2)] = normal;
			normals[IndexOffset + 2 + (i * 2)] = normal;
			normals[IndexOffset + 3 + (i * 2)] = normal;
			normals[IndexOffset + 1 + (i * 2)] = normal;
		}


		// Debug
		Vector3 testoffset = new Vector3(0, Random.Range(-0.05f, 0.05f), 0);
		Color randomColor = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
		for (int i = 1; i < EdgeCount; ++i)
		{
			Debug.DrawLine(vertices[i] + testoffset + Data.Centroid.Loc, vertices[i - 1] + testoffset + Data.Centroid.Loc, randomColor, 100f);
		}

		

		// Apply

		mesh.vertices = vertices;
		mesh.uv = uv;
		mesh.normals = normals;
		mesh.triangles = triangles;

		meshFilter.mesh = mesh;

		meshCollider.sharedMesh = mesh;
	}
}
