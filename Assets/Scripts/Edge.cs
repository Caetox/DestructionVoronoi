
public class Edge
{
	public Point Point1;
	public Point Point2;

	public Edge(Point point1, Point point2)
	{
		Point1 = point1;
		Point2 = point2;
	}

	public override bool Equals(object other)
	{
		Edge edge = other as Edge;
		if (edge == null)
		{
			return false;
		}
		
		var samePoints = Point1 == edge.Point1 && Point2 == edge.Point2;
		var samePointsReversed = Point1 == edge.Point2 && Point2 == edge.Point1;
		return samePoints || samePointsReversed;
	}

	public override int GetHashCode()
	{
		int hCode = (int)Point1.Loc.x ^ (int)Point1.Loc.z ^ (int)Point2.Loc.x ^ (int)Point2.Loc.z;
		return hCode.GetHashCode();
	}
}
