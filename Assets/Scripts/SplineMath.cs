using System;
using System.Collections;
using System.Collections.Generic;
using Unity.PlasticSCM.Editor.WebApi;
using UnityEngine;

namespace BezierNavigator
{
    public class SplineMath
    {
        public static Pose calculate(Pose p1, Pose p2, Pose p3, Pose p4, double t)
        {
            Pose a = lerp(p1, p2, t),
                    b = lerp(p2, p3, t),
                    c = lerp(p3, p4, t),
                    d = lerp(a, b, t),
                    e = lerp(b, c, t);

            return lerp(d, e, t);

        }

        public static Pose calculate(Pose[] poses, double t)
        {
            return calculate(poses[0], poses[1], poses[2], poses[3], t);
        }

        private static double lerp(double c1, double c2, double t)
        {
            return (1 - t) * c1 + t * c2;
        }

        private static Pose lerp(Pose c1, Pose c2, double t)
        {
            return new Pose(
                lerp(c1.x, c2.x, t),
                lerp(c1.y, c2.y, t)
                );
        }

        private static Pose derivative(Pose[] pts, double t)
        {
            Pose a = new Pose(pts[0].x * (-3 * (t * t) + 6 * t - 3), pts[0].y * (-3 * (t * t) + 6 * t - 3)),
                b = new Pose(pts[1].x * (9 * (t * t) - 12 * t + 3), pts[1].y * (9 * (t * t) - 12 * t + 3)),
                c = new Pose(pts[2].x * (-9 * (t * t) + 6 * t), pts[2].y * (-9 * (t * t) + 6 * t)),
                d = new Pose(pts[3].x * 3 * t * t, pts[3].y * 3 * t * t);

            return new Pose(a.x + b.x + c.x + d.x, a.y + b.y + c.y + d.y);
        }

        public static int mostInfluenceControl(double t)
        {
            double b = 3 * (t * t * t) - 6 * (t * t) + 3 * t,
                c = -3 * (t * t * t) + 3 * (t * t);

            if (b > c) return 1;
            else if (c > b) return 2;
            else return -1;
        }

        public static List<double> insideObstacles(Pose[] spline, double t_res, Obstacle[] obs, double radius)
        {
            List<double> intersectTs = new List<double>();

            bool active = false;
            int loops = ((int)(1 / t_res))+1;

            double start = 0;

            for (int i = 0; i < loops; i++)
            {
                Pose p = calculate(spline, i * t_res);
                Pose d = derivative(spline, i * t_res);

                double n1 = Math.Atan2(d.y, d.x) + Math.PI / 2.0, n2 = Math.Atan2(d.y, d.x) - Math.PI / 2.0;
                Pose p1 = new Pose(radius * Math.Cos(n1) + p.x, radius + Math.Sin(n1) + p.y);
                Pose p2 = new Pose(radius * Math.Cos(n2) + p.x, radius + Math.Sin(n2) + p.y);

                foreach (Obstacle obstacle in obs)
                {
                    if (obstacle.collide(p1, p2))
                    {
                        if (!active)
                        {
                            start = t_res * i;
                            active = true;
                        }
                    }
                    else
                    {
                        if (active)
                        {
                            intersectTs.Add((start + (t_res * (i - 1))) / 2);
                            active = false;
                        }
                    }
                }
            }

            return intersectTs;
        }
    }
}