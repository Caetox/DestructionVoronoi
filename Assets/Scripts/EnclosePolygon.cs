using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnclosePolygon : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // call this method in line 50
    public Polygon enclosePolygon(Polygon input,float minX,float maxX,float minZ,float maxZ){
        // prepare new result polygon
        List<Point> enclosedPoints = new List<Point>();
        Polygon result = new Polygon(input.Centroid);

        // create a new list of enclosed points, and then create new edge list afterwards
        // every combination of Point1 or Point2 inside or outside needs different handling 
        // corner handling comes at the end

        foreach (Edge e in input.Edges){

            // order of operation matters for edges that start and end outside the boundaries
            // 1) add intersections of edges and boundaries
            // 1.1) edges that start outside and end inside
            // minX handling
            if (e.Point1.x < minX){
                if (e.Point2.x < minX){
                    // both points outisde, ignore edge
                }
                else {
                    // intersection exits
                    float m = (e.Point1.y - e.Point2.y) / (e.Point1.x - e.Point2.x);
                    float y = e.Point1.y + ((minX - e.Point1.x) * m);
                    Point intersection = new Point(minX, y);
                    enclosedPoints.Add(intersection);
                }
            }
            // maxY handling
            if (e.Point1.y > maxY){
                if (e.Point2.y > maxY){
                    // both points outisde, ignore edge
                }
                else {
                    // intersection exits
                    float m = (e.Point1.x - e.Point2.x) / (e.Point1.y - e.Point2.y);
                    float x = e.Point2.x + ((maxY - e.Point2.y) * m);
                    Point intersection = new Point(x, maxY);
                    enclosedPoints.Add(intersection);
                }
            }
            // maxX handling
            if (e.Point1.x > maxX){
                if (e.Point2.x > maxX){
                    // both points outisde, ignore edge
                }
                else {
                    // intersection exits
                    float m = (e.Point1.y - e.Point2.y) / (e.Point1.x - e.Point2.x);
                    float y = e.Point2.y + ((maxX - e.Point2.x) * m);
                    Point intersection = new Point(maxX, y);
                    enclosedPoints.Add(intersection);
                }
            }
            // minY handling
            if (e.Point1.y < minY){
                if (e.Point2.y < minY){
                    // both points outisde, ignore edge
                }
                else {
                    // intersection exits
                    float m = (e.Point2.x - e.Point1.x) / (e.Point2.y - e.Point1.y);
                    float x = e.Point1.x + ((minY - e.Point1.y) * m);
                    Point intersection = new Point(x, maxY);
                    enclosedPoints.Add(intersection);
                }
            }

            
            // 1.2) edges that start inside and end outisde
            if (e.Point2.x < minX){
                if (e.Point1.x < minX){
                    // both points outisde, ignore edge
                }
                else {
                    // intersection exits
                    float m = (e.Point2.y - e.Point1.y) / (e.Point2.x - e.Point1.x);
                    float y = e.Point2.y + ((minX - e.Point2.x) * m);
                    Point intersection = new Point(minX, y);
                    enclosedPoints.Add(intersection);
                }
            }
            // maxY handling
            if (e.Point2.y > maxY){
                if (e.Point1.y > maxY){
                    // both points outisde, ignore edge
                }
                else {
                    // intersection exits
                    float m = (e.Point2.x - e.Point1.x) / (e.Point2.y - e.Point1.y);
                    float x = e.Point1.x + ((maxY - e.Point1.y) * m);
                    Point intersection = new Point(x, maxY);
                    enclosedPoints.Add(intersection);
                }
            }
            // maxX handling
            if (e.Point2.x > maxX){
                if (e.Point1.x > maxX){
                    // both points outisde, ignore edge
                }
                else {
                    // intersection exits
                    float m = (e.Point2.y - e.Point1.y) / (e.Point2.x - e.Point1.x);
                    float y = e.Point1.y + ((maxX - e.Point1.x) * m);
                    Point intersection = new Point(maxX, y);
                    enclosedPoints.Add(intersection);
                }
            }
            // minY handling
            if (e.Point2.y < minY){
                if (e.Point1.y < minY){
                    // both points outisde, ignore edge
                }
                else {
                    // intersection exits
                    float m = (e.Point1.x - e.Point2.x) / (e.Point1.y - e.Point2.y);
                    float x = e.Point2.x + ((minY - e.Point2.y) * m);
                    Point intersection = new Point(x, maxY);
                    enclosedPoints.Add(intersection);
                }
            }


            if (e.Point2.x < maxX && e.Point2.y < maxY && e.Point2.x > minX && e.Point2.y > minY){
                enclosedPoints.add(e.Point2);
            }
        }
        // 2) create new list of edges
        /*
        for(int i = 0; i < enclosedPoints.){

        }
        result.add(new Edge())
        */
        return enclosePolygon;
    }
}
