using System;
using System.Collections;
using System.Collections.Generic;

namespace BezierNavigator
{
    namespace PathFollower
    {
        public class PursuitMath
        {
            /// <summary>
            /// Approximates the intersection of a circle and spline
            /// </summary>
            /// <param name="obj">The center of the circle</param>
            /// <param name="pts">The main/control points of the spline</param>
            /// <param name="look">The radius of the circle</param>
            /// <param name="t_res">The resolution of the t</param>
            /// <returns></returns>
            public static double waypointCalc(Pose obj, Pose[] pts, double look, double t_res)
            {
                double t = 0.5; // Initial guess
                double prevT = 0;

                //Only checks a max of 1/t_res times
                for (int i = 0; i <= (int)(1/t_res); i++)
                {
                    // Check for convergence
                    if (Math.Abs(t - prevT) < t_res)
                        break;

                    // Compute x(t) and y(t) for the bezier curve
                    double x = (1 - t) * (1 - t) * (1 - t) * pts[0].x
                             + 3 * (1 - t) * (1 - t) * t * pts[1].x
                             + 3 * (1 - t) * t * t * pts[2].x
                             + t * t * t * pts[3].x;

                    double y = (1 - t) * (1 - t) * (1 - t) * pts[0].y
                             + 3 * (1 - t) * (1 - t) * t * pts[1].y
                             + 3 * (1 - t) * t * t * pts[2].y
                             + t * t * t * pts[3].y;

                    // Compute the derivative of F(t)
                    double dx_dt = -3 * (1 - t) * (1 - t) * pts[0].x
                                 + 3 * (1 - t) * (1 - t) * pts[1].x
                                 - 6 * (1 - t) * t * pts[1].x
                                 + 6 * (1 - t) * t * pts[2].x
                                 - 3 * t * t * pts[2].x
                                 + 3 * t * t * pts[3].x;

                    double dy_dt = -3 * (1 - t) * (1 - t) * pts[0].y
                                 + 3 * (1 - t) * (1 - t) * pts[1].y
                                 - 6 * (1 - t) * t * pts[1].y
                                 + 6 * (1 - t) * t * pts[2].y
                                 - 3 * t * t * pts[2].y
                                 + 3 * t * t * pts[3].y;

                    double derivative = 2 * (x - obj.x) * dx_dt + 2 * (y - obj.y) * dy_dt;

                    // Compute the function value F(t)
                    double func = Math.Pow(x - obj.x, 2) + Math.Pow(y - obj.y, 2) - look * look;

                    // Stop if the function value is close to zero (convergence)
                    if (Math.Abs(func) < 1e-6)
                        break;

                    // Update t using Newton's method
                    prevT = t;
                    if (Math.Abs(derivative) > 1e-6)
                        t = t - (func / derivative);
                    else
                        break; // Avoid division by zero

                    // Clamp t to the valid range [0, 1]
                    t = Math.Max(0, Math.Min(1, t));
                }

                return t;
            }

            /// <summary>
            /// Gets the vector for how an object should move based on the PurePursuit algorithm
            /// </summary>
            /// <param name="obj">The position of the object/center of circle</param>
            /// <param name="pts">The main/control points of the spline</param>
            /// <param name="look">The radius of the circle</param>
            /// <param name="t_res">The resolution of the t</param>
            /// <returns>The normalized sum of the derivative and movement vectors</returns>
            public static Pose getMovementVector1(Pose obj, Pose[] pts, double look, double t_res)
            {
                double length = SplineMath.curveLength(pts, t_res);
                double tPerLength = t_res / length;
                double t = waypointCalc(obj, pts, look, t_res);

                // Finds estimated t value for the target position
                double tEst = t - (tPerLength * look);
                Pose target = SplineMath.calculate(pts, tEst);

                // Vectors
                Pose movement = Pose.normalize(new Pose(target.x - obj.x, target.y - obj.y)); //Moves straight to target
                Pose pathDirection = Pose.normalize(SplineMath.derivative(pts, t)); // Moves based on the derivative/direction of curve

                return Pose.normalize(pathDirection*2 + movement);
            }
        }
    }
}