using System;
using System.Collections;
using System.Collections.Generic;


namespace BezierNavigator
{
    /// <summary>
    /// Class <c>Spline Math</c> contains the equations and formulas related to the spline
    /// By Kennedy Brundidge
    /// </summary>
    public class SplineMath
    {
        /// <summary>
        /// <c>calculate</c> finds a pose at a given t along the spline
        /// </summary>
        /// <param name="p1"></param> First point on the spline (1st main point)
        /// <param name="p2"></param> Second point (1st control point)
        /// <param name="p3"></param> Third point (2nd control point)
        /// <param name="p4"></param> Last point (2nd main point)
        /// <param name="t"></param> a decimal between 0 and 1 used for linear interpolation
        /// <returns>The pose on the spline at the given t</returns>
        public static Pose calculate(Pose p1, Pose p2, Pose p3, Pose p4, double t)
        {
            Pose a = lerp(p1, p2, t),
                    b = lerp(p2, p3, t),
                    c = lerp(p3, p4, t),
                    d = lerp(a, b, t),
                    e = lerp(b, c, t);

            return lerp(d, e, t);

        }

        /// <summary>
        /// <c>calculate</c> finds a pose at a given t along the spline
        /// </summary>
        /// <param name="poses"></param> an array of the main and control points
        /// <param name="t"></param> a decimal between 0 and 1 used for linear interpolation
        /// <returns>The pose on the spline at the given t</returns>
        public static Pose calculate(Pose[] poses, double t)
        {
            return calculate(poses[0], poses[1], poses[2], poses[3], t);
        }

        /// <summary>
        /// <c>lerp</c> linear interpolation
        /// </summary>
        /// <param name="c1"></param> start value
        /// <param name="c2"></param> end value
        /// <param name="t"></param> decimal between 0 and 1
        /// <returns>The number interpolated at t</returns>
        private static double lerp(double c1, double c2, double t)
        {
            return (1 - t) * c1 + t * c2;
        }

        /// <summary>
        /// <c>lerp</c> linear interpolation for Poses
        /// </summary>
        /// <param name="c1"></param> start Pose
        /// <param name="c2"></param> end Pose
        /// <param name="t"></param> decimal between 0 and 1
        /// <returns>The number interpolated at t</returns>
        private static Pose lerp(Pose c1, Pose c2, double t)
        {
            return new Pose(
                lerp(c1.x, c2.x, t),
                lerp(c1.y, c2.y, t)
                );
        }

        /// <summary>
        /// <c>curveLength</c> estimates the length of a curve
        /// Smaller t resolution allows for more accurate and longer calculations
        /// </summary>
        /// <param name="poses"></param> The poses that create the spline
        /// <param name="t_res"></param> the spacing between points (1/t_res = amount of calculated points on spline)
        /// <returns>The length of the curve</returns>
        public static double curveLength(Pose[] poses, double t_res)
        {
            double distance = 0;
            int loops = (int)(1 / t_res);

            Pose prev = calculate(poses, 0);
            for (int i = 1; i <= loops; i++)
            {
                Pose current = calculate(poses, t_res*i);
                distance += Pose.distance(prev, current);
                prev = current;
            }

            return distance;
        }

        /// <summary>
        /// <c>derivative</c> finds the derivative/tangent line of the curve at a given t value
        /// </summary>
        /// <param name="pts"></param> The poses that create the spline
        /// <param name="t"></param> decimal between 0 and 1
        /// <returns>A Pose with the slope of the tangent line</returns>
        public static Pose derivative(Pose[] pts, double t)
        {
            Pose a = new Pose(pts[0].x * (-3 * (t * t) + 6 * t - 3), pts[0].y * (-3 * (t * t) + 6 * t - 3)),
                b = new Pose(pts[1].x * (9 * (t * t) - 12 * t + 3), pts[1].y * (9 * (t * t) - 12 * t + 3)),
                c = new Pose(pts[2].x * (-9 * (t * t) + 6 * t), pts[2].y * (-9 * (t * t) + 6 * t)),
                d = new Pose(pts[3].x * 3 * t * t, pts[3].y * 3 * t * t);

            return new Pose(a.x + b.x + c.x + d.x, a.y + b.y + c.y + d.y);
        }

        /// <summary>
        /// <c>secondDerivative</c> finds the second derivative of the curve at a given t value
        /// </summary>
        /// <param name="pts"></param> The poses that create the spline
        /// <param name="t"></param> decimal between 0 and 1
        /// <returns>A Pose with the slope of the second derivative</returns>
        public static Pose secondDerivative(Pose[] pts, double t)
        {
            Pose a = new Pose(pts[0].x * (-6 * t + 6), pts[0].y * (-6 * t + 6)),
                b = new Pose(pts[1].x * (18 * t - 12), pts[1].y * (18 * t - 12)),
                c = new Pose(pts[2].x * (-18 * t + 6), pts[2].y * (-18 * t + 6)),
                d = new Pose(pts[3].x * 6 * t, pts[3].y * 6 * t);

            return new Pose(a.x + b.x + c.x + d.x, a.y + b.y + c.y + d.y);
        }

        /// <summary>
        /// <c>mostInfluenceControl</c> finds, between the control points, which has the most influence/impact
        /// of the spline at a given t
        /// </summary>
        /// <param name="t"></param> decimal between 0 and 1
        /// <returns>index of the most influential point (1 - 1st control point, 2 - 2nd control point, -1 - both)</returns>
        public static int mostInfluenceControl(double t)
        {
            double b = 3 * (t * t * t) - 6 * (t * t) + 3 * t,
                c = -3 * (t * t * t) + 3 * (t * t);

            if (b > c) return 1;
            else if (c > b) return 2;
            else return -1;
        }

        /// <summary>
        /// <c>insideObstacles</c> finds the t values where the spline intersects/is inside of the obstacles 
        /// </summary>
        /// <param name="spline">The points of the spline</param>
        /// <param name="t_res"></param> The spacing between points (1/t_res = amount of calculated points on spline)
        /// <param name="obs"></param> The array of the obstacles the spline must avoid
        /// <param name="radius"></param> The thickness/radius of the spline, dictates the minimum distance from obstacles
        /// <returns>A list of t values where the line collides with obstacles</returns>
        public static List<double> insideObstacles(Pose[] spline, double t_res, Obstacle[] obs, double radius)
        {
            List<double> intersectTs = new List<double>();

            // To reduce the amount of t values that can be added to the list,
            // average the start and end values of a collision
            bool active = false;
            int loops = ((int)(1 / t_res))+1;

            double start = 0;

            for (int i = 0; i < loops; i++)
            {
                Pose p = calculate(spline, i * t_res);
                Pose d = derivative(spline, i * t_res);

                // Finds the normal/perpendicular Poses to simulate the line thickness
                double n1 = Math.Atan2(d.y, d.x) + Math.PI / 2.0, n2 = Math.Atan2(d.y, d.x) - Math.PI / 2.0;
                Pose p1 = new Pose(radius * Math.Cos(n1) + p.x, radius * Math.Sin(n1) + p.y);
                Pose p2 = new Pose(radius * Math.Cos(n2) + p.x, radius * Math.Sin(n2) + p.y);

                foreach (Obstacle obstacle in obs)
                {
                    // Checks if the line at the current t is colliding with obstacles
                    if (obstacle.collide(p1, p2))
                    {
                        // If not already colliding, start tracking how long spline is colliding for
                        if (!active)
                        {
                            intersectTs.Add(t_res * i);
                            start = t_res * i;
                            active = true;
                        }
                    }
                    else
                    {
                        if (active)
                        {
                            // When there is no collision, average the start and end values of a collision
                            intersectTs.Add((start + (t_res * (i - 1))) / 2);
                            active = false;
                        }
                    }
                }
            }
            
            //If the line ends and it is still colliding, then average the end (1) with start
            if (active)
            {
                intersectTs.Add((start + 1) / 2);
            }
            return intersectTs;
        }
    }
}