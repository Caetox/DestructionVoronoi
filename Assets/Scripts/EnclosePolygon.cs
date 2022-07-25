using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnclosePolygon //: MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public EnclosePolygon()
    {

    }

    public Polygon enclosePoly(Polygon input, float minX, float maxX, float minZ, float maxZ){
        // prepare new result polygon
        List<Point> enclosedPoints = new List<Point>();
        Polygon result = new Polygon(input.Centroid);
        int countIntersections = 0;

        // create a new list of enclosed points, and then create new edge list afterwards
        // corner handling comes at the end

        foreach (Edge e in input.Edges){

            // 1) add intersections of edges and boundaries
            // minX handling
            if ((e.Point1.Loc.x < minX) ^ (e.Point2.Loc.x < minX)){
                float m = (e.Point1.Loc.z - e.Point2.Loc.z) / (e.Point1.Loc.x - e.Point2.Loc.x);
                float z = e.Point1.Loc.z + ((minX - e.Point1.Loc.x) * m);
                if (z < maxZ && z > minZ){
                    Point intersection = new Point(minX, z);
                    enclosedPoints.Add(intersection);
                    countIntersections++;
                }
            }
            // maxZ handling
            if ((e.Point1.Loc.z > maxZ) ^ (e.Point2.Loc.z > maxZ)){
                float m = (e.Point1.Loc.x - e.Point2.Loc.x) / (e.Point1.Loc.z - e.Point2.Loc.z);
                float x = e.Point2.Loc.x + ((maxZ - e.Point2.Loc.z) * m);
                if (x < maxX && x > minX){
                    Point intersection = new Point(x, maxZ);
                    enclosedPoints.Add(intersection);
                    countIntersections++;
                }
            }
            // maxX handling
            if ((e.Point1.Loc.x > maxX) ^ (e.Point2.Loc.x > maxX)){
                float m = (e.Point1.Loc.z - e.Point2.Loc.z) / (e.Point1.Loc.x - e.Point2.Loc.x);
                float z = e.Point2.Loc.z + ((maxX - e.Point2.Loc.x) * m);
                if (z < maxZ && z > minZ){
                    Point intersection = new Point(maxX, z);
                    enclosedPoints.Add(intersection);
                    countIntersections++;
                }
            }
            // minZ handling
            if  ((e.Point1.Loc.z < minZ) ^ (e.Point2.Loc.z < minZ)){
                float m = (e.Point2.Loc.x - e.Point1.Loc.x) / (e.Point2.Loc.z - e.Point1.Loc.z);
                float x = e.Point1.Loc.x + ((minZ - e.Point1.Loc.z) * m);
                if (x < maxX && x > minX){
                    Point intersection = new Point(x, maxZ);
                    enclosedPoints.Add(intersection);
                    countIntersections++;
                }
            }

            // 2) add old points inside the boundaries to new list of points
            if (e.Point2.Loc.x < maxX && e.Point2.Loc.z < maxZ && e.Point2.Loc.x > minX && e.Point2.Loc.z > minZ){
                enclosedPoints.Add(e.Point2);
            }
        }

        // if less than two points are inside the boundaries, return empty polygon
        if (enclosedPoints.Count < 2){
            return result;
        } 

        // 3) add corner points 
        // assume that the area is fragmented enough, so that no polygon covers more than one corner
        // also assume that polygons are defined clockwise
        // if a point is on the edge and the next point is on the next clockwise edge, add a corner point
        if (countIntersections == 2){
            // connection between last and first point needs extra handling
            if (enclosedPoints[enclosedPoints.Count].Loc.x == minX){
                if (enclosedPoints[0].Loc.z == maxZ){
                    enclosedPoints.Add(new Point(minX, maxZ));
                }
            }
            if (enclosedPoints[enclosedPoints.Count].Loc.z == maxZ){
                if (enclosedPoints[0].Loc.x == maxX){
                    enclosedPoints.Add(new Point(maxX, maxZ));
                }
            }
            if (enclosedPoints[enclosedPoints.Count].Loc.x == maxX){
                if (enclosedPoints[0].Loc.z == minZ){
                    enclosedPoints.Add(new Point(maxX, minZ));
                }
            }
            if (enclosedPoints[enclosedPoints.Count].Loc.z == minZ){
                if (enclosedPoints[0].Loc.x == minX){
                    enclosedPoints.Add(new Point(minX, minZ));
                }
            }

            // handling for normal points
            for (int i = 0; i < enclosedPoints.Count - 1; i++){
                if (enclosedPoints[i].Loc.x == minX){
                    if (enclosedPoints[i + 1].Loc.z == maxZ){
                        enclosedPoints.Add(new Point(minX, maxZ));
                    }
                }
                if (enclosedPoints[i].Loc.z == maxZ){
                    if (enclosedPoints[i + 1].Loc.x == maxX){
                        enclosedPoints.Add(new Point(maxX, maxZ));
                    }
                }
                if (enclosedPoints[i].Loc.x == maxX){
                    if (enclosedPoints[i + 1].Loc.z == minZ){
                        enclosedPoints.Add(new Point(maxX, minZ));
                    }
                }
                if (enclosedPoints[i].Loc.z == minZ){
                    if (enclosedPoints[i + 1].Loc.x == minX){
                        enclosedPoints.Add(new Point(minX, minZ));
                    }
                }
            }
        }

        // 4) create new list of edges
        // start with the loop around to make the order of the edges line up with the input
        result.Edges.Add(new Edge(enclosedPoints[enclosedPoints.Count], enclosedPoints[0]));
        for(int i = 0; i < enclosedPoints.Count - 1; i++){
            result.Edges.Add(new Edge(enclosedPoints[i], enclosedPoints[i+1]));
        }
        return result;
    }
}