using System;
using System.Linq;

namespace BezierNavigator
{
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
            if (points == null || points.Length < 3)
            {
                // A valid polygon must have at least 3 points
                return false;
            }

            for (int i = 0; i < points.Length; i++)
            {
                int next = (i + 1) % points.Length;

                // Check if the line intersects with any edge of the polygon
                if (LineIntersectsLine(pt1, pt2, points[i], points[next]))
                {
                    return true;
                }
            }

            // Check if the line is entirely inside the polygon
            if (IsPointInsidePolygon(pt1) && IsPointInsidePolygon(pt2))
            {
                return true;
            }

            return false;
        }

        private bool LineIntersectsLine(Pose a1, Pose a2, Pose b1, Pose b2)
        {
            double det = (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);

            // If determinant is zero, the lines are parallel or coincident
            if (Math.Abs(det) < 1e-10)
            {
                return false;
            }

            double t = ((b1.x - a1.x) * (b2.y - b1.y) - (b1.y - a1.y) * (b2.x - b1.x)) / det;
            double u = ((b1.x - a1.x) * (a2.y - a1.y) - (b1.y - a1.y) * (a2.x - a1.x)) / det;

            // Check if the intersection point is on both line segments
            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        }

        private bool IsPointInsidePolygon(Pose point)
        {
            bool inside = false;

            for (int i = 0, j = points.Length - 1; i < points.Length; j = i++)
            {
                // Ray-casting algorithm to determine if point is inside polygon
                if ((points[i].y > point.y) != (points[j].y > point.y) &&
                    point.x < (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
                {
                    inside = !inside;
                }
            }

            return inside;
        }
    }
}
