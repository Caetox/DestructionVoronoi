using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Polygon
{
	public Polygon(Vector3 Center)
	{
		Centroid = Center;
	}

	public List<Edge> Edges;
	public Vector3 Centroid;
}
