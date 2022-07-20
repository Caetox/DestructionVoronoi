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

    public void Generate(Polygon Data)
	{
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
		MeshFilter meshFilter = GetComponent<MeshFilter>();
		MeshCollider meshCollider = GetComponent<MeshCollider>();
		Mesh mesh = new Mesh();

		int EdgeCount = Data.Edges.Count;

		Vector3[] vertices = new Vector3[EdgeCount * 2];
		Vector3[] normals = new Vector3[EdgeCount * 2];
		Vector2[] uv = new Vector2[EdgeCount * 2];
		int[] triangles = new int[(EdgeCount * 2 + (2 * (EdgeCount - 2))) * 3];

		int vertexIndex = 0;
		int normalIndex = 0;
		int uvIndex = 0;
		int triangleIndex = 0;

		float depth = 0.3f;

		// Front
		Vector3 MainPoint = Data.Edges[0].Point1.Loc;

		Vector3 testoffset = new Vector3(0, Random.Range(-0.01f, 0.01f), 0);
		Color randomColor = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);

		for (int i = 0; i < EdgeCount; ++i)
		{
			vertices[vertexIndex++] = Data.Edges[i].Point1.Loc;
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc;

			normals[normalIndex++] = Vector3.forward;
			normals[normalIndex++] = Vector3.forward;

			uv[uvIndex++] = Data.Edges[i].Point1.Uv;
			uv[uvIndex++] = Data.Edges[i].Point2.Uv;

			
			Debug.DrawLine(vertices[vertexIndex - 2] + testoffset, vertices[vertexIndex - 1] + testoffset, randomColor, 100f);
		}

		for (int i = 0; i < EdgeCount - 2; ++i)
		{
			triangles[triangleIndex++] = (2 + i);
			triangles[triangleIndex++] = (1 + i);
			triangles[triangleIndex++] = 0;
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
