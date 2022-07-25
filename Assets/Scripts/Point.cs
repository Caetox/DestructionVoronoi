using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Point
{
    private static int IndexCounter;
    public readonly int ID = IndexCounter++;

    public Vector3 Loc;
	public Vector2 Uv;
    public int Index;
    public bool Corner = false;
    public bool anchored;


    public HashSet<Triangle> AdjacentTriangles { get; } = new HashSet<Triangle>();

    public Point(float x, float z)
    {
        Loc = new Vector3((float)x, 0, (float)z);
		Uv = new Vector2((float)x, (float)z);
        Index = 0;
	}

	public Point(float x, float z, int index)
	{
		Loc = new Vector3((float)x, 0, (float)z);
		Uv = new Vector2((float)x, (float)z);
        Index = index;
	}

	public bool Equals(Point other)
    {
        return other.ID == ID;
    }
}
