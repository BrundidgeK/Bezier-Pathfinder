using System;

namespace BezierNavigator
{
    public class Pose
    {
        public double x, y;
        public float he;
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

        public static bool between(Pose pt1, Pose pt2, Pose between)
        {
            double ymin = Math.Min(pt1.y, pt2.y);
            double ymax = Math.Max(pt1.y, pt2.y);
            double xmin = Math.Min(pt1.x, pt2.x);
            double xmax = Math.Max(pt1.x, pt2.x);

            return between.y >= ymin && between.y <= ymax && between.x >= xmin && between.x <= xmax;
        }

        public static double distance(Pose pt1, Pose pt2)
        {
            return Math.Sqrt((pt1.x - pt1.x) * (pt1.x - pt1.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
        }
    }
}