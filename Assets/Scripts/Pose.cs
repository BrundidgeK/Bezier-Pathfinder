using System;

namespace BezierNavigator
{
    /// <summary>
    /// <c>Pose</c> a point in space
    /// By Kennedy Brundidge
    /// </summary>
    public class Pose
    {
        public double x, y;
        /// <summary>
        /// Instantiates the Pose
        /// </summary>
        /// <param name="x">x coordinate</param>
        /// <param name="y">y coordinate</param>
        public Pose(double x, double y)
        {
            this.x = x;
            this.y = y;
        }

        public bool equals(Pose other)
        {
            return (other.x == x && other.y == y);
        }

        public string toString()
        {
            return x + ", " + y;
        }

        /// <summary>
        /// <c>between</c> checks if a point is within the bounding box created by two other points
        /// </summary>
        /// <param name="pt1">bounding box point 1</param>
        /// <param name="pt2">bounding box point 2</param>
        /// <param name="between">the point within the bounding box</param>
        /// <returns>True if the point lies within the bounding box, and false if otherwise</returns>
        public static bool between(Pose pt1, Pose pt2, Pose between)
        {
            // Check bounding box constraints
            double ymin = Math.Min(pt1.y, pt2.y);
            double ymax = Math.Max(pt1.y, pt2.y);
            double xmin = Math.Min(pt1.x, pt2.x);
            double xmax = Math.Max(pt1.x, pt2.x);

            if (between.y < ymin || between.y > ymax || between.x < xmin || between.x > xmax)
            {
                return false;
            }

            double crossProduct = (pt2.y - pt1.y) * (between.x - pt1.x) - (pt2.x - pt1.x) * (between.y - pt1.y);
            return Math.Abs(crossProduct) < 1e-5;
        }

        /// <summary>
        /// <c>distance</c> finds the distance between two poses
        /// </summary>
        /// <returns>The length of the distance between</returns>
        public static double distance(Pose pt1, Pose pt2)
        {
            return Math.Sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
        }

        /// <summary>
        /// <c>midPoint</c> finds the point in the middle of two other points
        /// </summary>
        /// <param name="pt1"></param>
        /// <param name="pt2"></param>
        /// <returns>The midpoint</returns>
        public static Pose midPoint(Pose pt1, Pose pt2)
        {
            return new Pose((pt1.x + pt2.x) / 2.0, (pt2.y + pt1.y) / 2.0);
        }
    }
}