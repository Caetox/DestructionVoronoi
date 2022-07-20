using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Point
{
    private static int IndexCounter;
    private readonly int ID = IndexCounter++;

    public Vector3 Loc;
	public Vector2 Uv;

	//public float X { get; }
    //public float Y { get; }
    public HashSet<Triangle> AdjacentTriangles { get; } = new HashSet<Triangle>();

    public Point(float x, float z)
    {
        //X = x;
        //Y = z;

        Loc = new Vector3((float)x, 0, (float)z);
		Uv = new Vector2((float)x, (float)z);
	}

    public bool Equals(Point other)
    {
        return other.ID == ID;
    }
}
