using System;
using System.Linq;

namespace BezierNavigator
{
    /// <summary>
    /// <c>Obstacle</c> represents the obstacles in the environment
    /// By Kennedy Brundidge
    /// </summary>
    public class Obstacle
    {
        public Pose[] points;

        /// <summary>
        /// Instantiates the obstacle with its vertices
        /// </summary>
        /// <param name="poses">vertices that define the shape of the obstacle</param>
        public Obstacle(Pose[] poses)
        {
            points = poses;
        }

        public Obstacle()
        {
        }

        /// <summary>
        /// <c>collide</c> takes two end points on a line and checks for intersections
        /// </summary>
        /// <param name="pt1"></param> 
        /// <param name="pt2"></param>
        /// <returns>true if there is a collision, and false if there is not</returns>
        public bool collide(Pose pt1, Pose pt2)
        {
            if (points == null || points.Length < 3)
            {
                // A valid polygon must have at least 3 points
                return false;
            }

            // Step 1: Check if the line intersects any edge of the polygon
            for (int i = 0; i < points.Length; i++)
            {
                int next = (i + 1) % points.Length; // Wrap around to the first vertex
                if (LineIntersectsLine(pt1, pt2, points[i], points[next]))
                {
                    return true; // Collision detected
                }
            }

            // Step 2: Check if both points lie on the boundary of the polygon
            if (IsPointOnPolygonEdge(pt1) || IsPointOnPolygonEdge(pt2))
            {
                return true;
            }

            // Step 3: Check if the line segment is completely inside the polygon
            if (IsPointInsidePolygon(pt1) && IsPointInsidePolygon(pt2))
            {
                return true;
            }

            return false; // No collision detected
        }

        private bool LineIntersectsLine(Pose a1, Pose a2, Pose b1, Pose b2)
        {
            // Determine if two line segments intersect
            double det = (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);

            if (Math.Abs(det) < 1e-10)
            {
                // Lines are parallel
                return false;
            }

            double t = ((b1.x - a1.x) * (b2.y - b1.y) - (b1.y - a1.y) * (b2.x - b1.x)) / det;
            double u = ((b1.x - a1.x) * (a2.y - a1.y) - (b1.y - a1.y) * (a2.x - a1.x)) / det;

            // Check for valid t and u in the range [0, 1] to confirm intersection
            return (t >= 0 && t <= 1) && (u >= 0 && u <= 1);
        }

        private bool IsPointInsidePolygon(Pose point)
        {
            // Ray-casting algorithm to check if a point is inside the polygon
            bool inside = false;

            for (int i = 0, j = points.Length - 1; i < points.Length; j = i++)
            {
                if ((points[i].y > point.y) != (points[j].y > point.y) &&
                    point.x < (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        private bool IsPointOnPolygonEdge(Pose point)
        {
            // Check if the point lies on any of the edges of the polygon
            for (int i = 0; i < points.Length; i++)
            {
                int next = (i + 1) % points.Length;

                if (IsPointOnLineSegment(point, points[i], points[next]))
                {
                    return true;
                }
            }

            return false;
        }

        private bool IsPointOnLineSegment(Pose p, Pose a, Pose b)
        {
            // Check if point p lies exactly on the line segment [a, b]
            double crossProduct = (p.y - a.y) * (b.x - a.x) - (p.x - a.x) * (b.y - a.y);

            if (Math.Abs(crossProduct) > 1e-10)
                return false; // Point is not on the line

            double dotProduct = (p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y);
            if (dotProduct < 0)
                return false; // Point is before a

            double squaredLengthBA = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            if (dotProduct > squaredLengthBA)
                return false; // Point is after b

            return true; // Point is on the segment
        }
    }
}
