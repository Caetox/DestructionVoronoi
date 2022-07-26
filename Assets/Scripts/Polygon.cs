using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Polygon
{
	public Polygon(Point Center)
	{
		Centroid = Center;
		Edges = new List<Edge>();
		IsValid = false;
		anchored = Centroid.anchored;
		ParticleIndex = -1;
	}

	public List<Edge> Edges;
	public Point Centroid;
	public bool IsValid;
	public float Circumference;
	public float Surface;
	public bool anchored;
	public int ParticleIndex;

	public void Sort(Vector3 Scale)
	{
		Vector2 uvScale = new Vector2(1.0f / Scale.x, 1.0f / Scale.z);
		List<Edge> SortedEdges = new List<Edge>();

		if (Edges.Count < 3)
		{
			return;
		}

		// Clamp to edge
		Scale = Scale * 0.5f;
		for (int i = 1; i < Edges.Count; ++i)
		{
			{
				float x = Edges[i].Point1.Loc.x;
				float y = Edges[i].Point1.Loc.y;
				float z = Edges[i].Point1.Loc.z;
				Edges[i].Point1.Loc = new Vector3(Mathf.Clamp(x, -Scale.x, Scale.x), Mathf.Clamp(y, -Scale.y, Scale.y), Mathf.Clamp(z, -Scale.z, Scale.z));
				Edges[i].Point1.Uv = new Vector2(Mathf.Clamp(Edges[i].Point1.Uv.x, -Scale.x, Scale.x), Mathf.Clamp(Edges[i].Point1.Uv.y, -Scale.z, Scale.z));
			}
			{
				float x = Edges[i].Point2.Loc.x;
				float y = Edges[i].Point2.Loc.y;
				float z = Edges[i].Point2.Loc.z;
				Edges[i].Point2.Loc = new Vector3(Mathf.Clamp(x, -Scale.x, Scale.x), Mathf.Clamp(y, -Scale.y, Scale.y), Mathf.Clamp(z, -Scale.z, Scale.z));
				Edges[i].Point2.Uv = new Vector2(Mathf.Clamp(Edges[i].Point2.Uv.x, -Scale.x, Scale.x), Mathf.Clamp(Edges[i].Point2.Uv.y, -Scale.z, Scale.z));
			}
		}

		for (int i = 1; i < Edges.Count; ++i)
		{
			Vector3 a = Edges[0].Point1.Loc - Edges[0].Point2.Loc;
			Vector3 b = Edges[0].Point1.Loc - Edges[1].Point2.Loc;

			float d = Vector3.Dot(a, b);
			if (d < 0.001f)
			{
				Edges[0].Point2 = Edges[1].Point2;
				Edges[1] = Edges[0];
			}


		}

		// Find errors in winding order
		SortedEdges.Add(Edges[0]);

		for (int i = 0; i < Edges.Count; ++i)
		{
			Point last = SortedEdges[SortedEdges.Count - 1].Point2;
			Point lastOpposite = SortedEdges[SortedEdges.Count - 1].Point1;

			for (int j = 0; j < Edges.Count; ++j)
			{
				if (last.Equals(Edges[j].Point1) && !lastOpposite.Equals(Edges[j].Point2))
				{
					if ((Edges[j].Point1.Loc - Edges[j].Point2.Loc).sqrMagnitude > 0.0001f)
					{
						SortedEdges.Add(Edges[j]);
					}
					break;
				}
				// Check for flipped edges
				// Make sure it does not match with itself
				else if (last.Equals(Edges[j].Point2) && !lastOpposite.Equals(Edges[j].Point1))
				{
					// Flip edge
					if ((Edges[j].Point1.Loc - Edges[j].Point2.Loc).sqrMagnitude > 0.0001f)
					{
						Edge FlippedEdge = new Edge(Edges[j].Point2, Edges[j].Point1);
						SortedEdges.Add(FlippedEdge);
					}
					break;
				}
			}
		}

		// Check if enough vertices are left
		if (SortedEdges.Count < 3)
		{
			return;
		}
		//Debug.Log("Using:");
		//Debug.Log(SortedEdges[0].Point1.Loc.ToString());
		//Debug.Log(SortedEdges[0].Point2.Loc.ToString());
		//Debug.Log(SortedEdges[1].Point2.Loc.ToString());

		//Debug.Log(CalcNormal(SortedEdges[0].Point1.Loc, SortedEdges[0].Point2.Loc, SortedEdges[1].Point2.Loc).ToString());
		if (CalcNormal(SortedEdges[0].Point1.Loc, SortedEdges[0].Point2.Loc, SortedEdges[1].Point2.Loc).y > 0)
		{
			SortedEdges.Reverse();
		}
		IsValid = true;

		Edges = SortedEdges;

		Circumference = 0;
		Surface = 0;
		for (int i = 1; i < Edges.Count; ++i)
		{
			Vector3 a = Edges[i].Point1.Loc;
			Vector3 b = Edges[i].Point2.Loc;
			//Circumference += Vector3.Distance(a, b);
			Surface += Mathf.Abs(a.x * b.z - b.x * a.z);
		}
		Surface *= 0.5f;
	}

	static public Vector3 CalcNormal(Vector3 edgeA, Vector3 edgeB, Vector3 edgeC)
	{
		Vector3 side1 = edgeB - edgeA;
		Vector3 side2 = edgeC - edgeA;
		Vector3 x = Vector3.Cross(side1, side2).normalized;
		return x;
	}
}
