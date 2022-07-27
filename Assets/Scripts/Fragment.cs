using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

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

    public void Generate(Polygon Data, float depth, Material FrontMat, Material SideMat, Vector3 objectSize)
	{
		Vector2 uvScale = new Vector2(1.0f / objectSize.x, -1.0f / objectSize.z);
		Vector3 depthVec = new Vector3(0, -depth, 0);
		Vector3 halfDepthVec = new Vector3(0, depth * 0.5f, 0);

		MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
		MeshFilter meshFilter = GetComponent<MeshFilter>();
		MeshCollider meshCollider = GetComponent<MeshCollider>();
		Mesh mesh = new Mesh();

		int EdgeCount = Data.Edges.Count;
		if (EdgeCount == 0)
		{
			return;
		}

		Vector3[] vertices = new Vector3[EdgeCount * 4];
		Vector3[] normals = new Vector3[EdgeCount * 4];
		Vector2[] uv = new Vector2[EdgeCount * 4];
		int[] triangles = new int[(EdgeCount * 2 + (2 * (EdgeCount))) * 3];

		int vertexIndex = 0;
		int normalIndex = 0;
		int uvIndex = 0;
		int triangleIndex = 0;

		// Front

		for (int i = 0; i < EdgeCount; ++i)
		{	
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc - Data.Centroid.Loc + halfDepthVec;
			normals[normalIndex++] = Vector3.up;
			uv[uvIndex++] = (Data.Edges[i].Point2.Uv * uvScale) + new Vector2(0.5f,0.5f);
		}

		for (int i = 0; i < EdgeCount - 2; ++i)
		{
			triangles[triangleIndex++] = (2 + i);
			triangles[triangleIndex++] = (1 + i);
			triangles[triangleIndex++] = 0;
		}

		// Back
		
		for (int i = 0; i < EdgeCount; ++i)
		{
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc - Data.Centroid.Loc - halfDepthVec;
			normals[normalIndex++] = -Vector3.up;
			uv[uvIndex++] = (Data.Edges[i].Point2.Uv * uvScale) + new Vector2(0.5f, 0.5f);
		}

		for (int i = 0; i < EdgeCount - 2; ++i)
		{
			triangles[triangleIndex++] = EdgeCount + 0;
			triangles[triangleIndex++] = EdgeCount + (1 + i);
			triangles[triangleIndex++] = EdgeCount + (2 + i);
		}

		int FirstSubmeshLength = triangleIndex;

		// Sides
		for (int i = 0; i < EdgeCount; ++i)
		{
			if (EdgeCount == 3)
			{
				Debug.Log("T");
			}
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc - Data.Centroid.Loc + halfDepthVec;
			vertices[vertexIndex++] = Data.Edges[i].Point2.Loc - Data.Centroid.Loc - halfDepthVec;
			normals[normalIndex++] = Vector3.forward;
			normals[normalIndex++] = Vector3.forward;

			float u = ((float)i / (float)EdgeCount) * 5.0f;
			uv[uvIndex++] = new Vector2(u, depth);
			uv[uvIndex++] = new Vector2(u, 0.0f);
		}

		int IndexOffset = EdgeCount * 2;
		for (int i = 0; i < EdgeCount /*- 2*/; ++i)
		{
			triangles[triangleIndex++] = IndexOffset + 2 + (i);// * 2);
			triangles[triangleIndex++] = IndexOffset + 1 + (i);// * 2);
			triangles[triangleIndex++] = IndexOffset + 0 + (i);// * 2);
			triangles[triangleIndex++] = IndexOffset + 2 + (i);// * 2);
			triangles[triangleIndex++] = IndexOffset + 3 + (i);// * 2);
			triangles[triangleIndex++] = IndexOffset + 1 + (i);// * 2);

			Vector3 normal = Polygon.CalcNormal(vertices[IndexOffset + 2 + (i/* * 2*/)], vertices[IndexOffset + 1 + (i /** 2*/)], vertices[IndexOffset + 0 + (i /** 2*/)]);

			normals[IndexOffset + 2 + (i /** 2*/)] = normal;
			normals[IndexOffset + 1 + (i /** 2*/)] = normal;
			normals[IndexOffset + 0 + (i /** 2*/)] = normal;
			normals[IndexOffset + 2 + (i /** 2*/)] = normal;
			normals[IndexOffset + 3 + (i /** 2*/)] = normal;
			normals[IndexOffset + 1 + (i /** 2*/)] = normal;
		}


		// Debug
		if (!Data.anchored)
		{
			Vector3 testoffset = new Vector3(0, Random.Range(-0.22f, 0.22f), 0);
			Color randomColor = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
			for (int i = 1; i < EdgeCount; ++i)
			{
				Debug.DrawLine(vertices[i] + testoffset + Data.Centroid.Loc, vertices[i - 1] + testoffset + Data.Centroid.Loc, randomColor, 100f);
			}
		}


		// Apply
		mesh.vertices = vertices;
		mesh.uv = uv;
		mesh.normals = normals;
		mesh.triangles = triangles;
		
		meshFilter.mesh = mesh;
		
		mesh.subMeshCount = 2;
		mesh.SetSubMesh(0, new SubMeshDescriptor(0, FirstSubmeshLength));
		mesh.SetSubMesh(1, new SubMeshDescriptor(FirstSubmeshLength, triangleIndex - FirstSubmeshLength));
		
		Material[] Mats = { FrontMat, SideMat };
		meshRenderer.materials = Mats;
		
		meshCollider.sharedMesh = mesh;
	}
}
