

using System;

public class Obstacle
{
    public Pose[] points;

    public Obstacle(Pose[] poses)
    {
        points = poses;
    }

    public Obstacle()
    {

    }

    public bool collide(Pose pt1, Pose pt2)
    {
        for (int i = 0; i < points.Length; i++)
        {
            int next = i + 1;
            if (next == points.Length)
                next = 0;

            try
            {
                double m1, m2;

                // Check if the edge of the polygon is vertical
                if (points[next].x == points[i].x)
                {
                    m1 = double.PositiveInfinity; // Vertical line
                }
                else
                {
                    m1 = (points[next].y - points[i].y) / (points[next].x - points[i].x);
                }

                // Check if the input line (pt1 to pt2) is vertical
                if (pt1.x == pt2.x)
                {
                    m2 = double.PositiveInfinity; // Vertical line
                }
                else
                {
                    m2 = (pt1.y - pt2.y) / (pt1.x - pt2.x);
                }

                double x, y;

                if (double.IsInfinity(m1) && double.IsInfinity(m2))
                {
                    // Both lines are vertical (parallel or overlapping)
                    continue; // Skip this iteration, as no intersection occurs
                }
                else if (double.IsInfinity(m1))
                {
                    // Polygon edge is vertical
                    x = points[i].x;
                    y = m2 * (x - pt1.x) + pt1.y;
                }
                else if (double.IsInfinity(m2))
                {
                    // Input line is vertical
                    x = pt1.x;
                    y = m1 * (x - points[i].x) + points[i].y;
                }
                else
                {
                    // General case: neither line is vertical
                    x = (m1 * points[i].x - points[i].y - m2 * pt1.x + pt1.y) / (m1 - m2);
                    y = m1 * (x - points[i].x) + points[i].y;
                }

                // Check if the intersection point lies between the endpoints of both line segments
                if (Pose.between(pt1, pt2, new Pose(x, y)) && Pose.between(points[i], points[next], new Pose(x, y)))
                {
                    return true;
                }
            }
            catch
            {
                continue;
            }
        }
        return false;
    }

    
}