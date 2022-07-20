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
	}

	public List<Edge> Edges;
	public Point Centroid;
	public bool IsValid;

	public void Sort()
	{
		List<Edge> SortedEdges = new List<Edge>();

		if (Edges.Count < 3)
		{
			return;
		}

		SortedEdges.Add(Edges[0]);

		for (int i = 0; i < Edges.Count; ++i)
		{
			Point last = SortedEdges[SortedEdges.Count - 1].Point2;
			Point lastOpposite = SortedEdges[SortedEdges.Count - 1].Point1;

			for (int j = 0; j < Edges.Count; ++j)
			{
				if (last.Equals(Edges[j].Point1) && !lastOpposite.Equals(Edges[j].Point2))
				{
					SortedEdges.Add(Edges[j]);
					break;
				}
				// Check for flipped edges
				// Make sure it does not match with itself
				else if (last.Equals(Edges[j].Point2) && !lastOpposite.Equals(Edges[j].Point1))
				{
					// Flip edge
					Edge FlippedEdge = new Edge(Edges[j].Point2, Edges[j].Point1);
					SortedEdges.Add(FlippedEdge);
					break;
				}
			}
		}

		for (int i = SortedEdges.Count - 1; i > 0; --i)
		{
			if ((SortedEdges[i].Point1.Loc - SortedEdges[i].Point2.Loc).magnitude < 0.0001f)
			{
				SortedEdges.RemoveAt(i);
			}
		}

		// Check for winding order
		if (SortedEdges.Count >= 3)
		{
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
		}

		Edges = SortedEdges;
	}

	static public Vector3 CalcNormal(Vector3 edgeA, Vector3 edgeB, Vector3 edgeC)
	{
		Vector3 side1 = edgeB - edgeA;
		Vector3 side2 = edgeC - edgeA;
		Vector3 x = Vector3.Cross(side1, side2).normalized;
		return x;
	}
}
