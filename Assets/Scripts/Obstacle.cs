

using System;

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
            for (int i = 0; i < points.Length; i++)
            {
                int next = (i + 1) % points.Length;

                try
                {
                    double det = (pt2.x - pt1.x) * (points[next].y - points[i].y) - (pt2.y - pt1.y) * (points[next].x - points[i].x);
                    if (Math.Abs(det) < 1e-10) continue; 

                    double t = ((points[i].x - pt1.x) * (points[next].y - points[i].y) - (points[i].y - pt1.y) * (points[next].x - points[i].x)) / det;
                    double u = ((points[i].x - pt1.x) * (pt2.y - pt1.y) - (points[i].y - pt1.y) * (pt2.x - pt1.x)) / det;

                    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
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
}