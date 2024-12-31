using System;
using System.Collections.Generic;
using Unity.VisualScripting.Antlr3.Runtime.Tree;
using UnityEngine;
using UnityEngine.UI;

namespace BezierNavigator
{
    /// <summary>
    /// <c>PathFinder</c> contains the algorithm to find the optimal bezier curve path
    /// By Kennedy Brundidge
    /// </summary>
    public class PathFinder
    {
        private Pose[] poses;
        private double robotRadius;
        private Obstacle[] obstacles;


        public PathFinder()
        {
        }

        /// <summary>
        /// <c>setSplinePoints</c> sets the points on the current spline
        /// </summary>
        /// <param name="splinePoints"></param>
        public void setSplinePoints(Pose[] splinePoints)
        {
            if (splinePoints.Length != 4)
                return;
            poses = splinePoints;
        }

        /// <returns>The splines points</returns>
        public Pose[] getSplinePoints()
        {
            return poses;
        }

        /// <summary>
        /// <c>setRadius</c> sets the radius/thickness of the spline curve
        /// </summary>
        /// <param name="radius"></param>
        public void setRadius(double radius)
        {
            robotRadius = radius;
        }

        /// <summary>
        /// <c>setObstacles</c> sets the obstacles
        /// </summary>
        /// <param name="obstacles">an array of the obstacles for the spline to avoid</param>
        public void setObstacles(Obstacle[] obstacles)
        {
            this.obstacles = obstacles;
        }

        /// <summary>
        /// <c>findPath</c> runs the algorithm to find the path in a set amount of loops
        /// </summary>
        /// <param name="t_res">Controls collision-check intervals (1/t resolution = number of points on curve)</param>
        /// <param name="initDistance">The starting search distance around control points for optimal placement</param>
        /// <param name="decayRate">Linearly decreases the search distance (as a percentage) during optimization</param>
        /// <param name="pointsOnCircle">Number of points on the search circle evaluated during control point optimization</param>
        /// <param name="maxLoops">Maximum number of loops for the algorithm to optimize the control points</param>
        public void findPath(double t_res, double initDistance, double decayRate, int pointsOnCircle, int maxLoops)
        {
            // Resets the control points to fall on a line created by the main points
            // Checks if a simple straight line is the best path
            Pose origin = Pose.midPoint(poses[0], poses[3]);
            int distanceSpline = (int)Pose.distance(poses[0], poses[3]);

            poses[1] = Pose.midPoint(poses[0], origin);
            poses[2] = Pose.midPoint(poses[3], origin);

            List<double> intersections = SplineMath.insideObstacles(poses, t_res, obstacles, robotRadius);
            int pathingLoops = 0;

            int con1 = 0, con2 = 0;
            // Finds which control point causes the most intersections
            foreach (double intersection in intersections)
            {
                int index = SplineMath.mostInfluenceControl(intersection);
                switch (index)
                {
                    case 1:
                        con1++;
                        break;
                    case 2:
                        con2++;
                        break;
                }
            }
            // Focuses on the most influential point
            int con_focus = con1 > con2 ? 1 : 2;
            int init = con_focus;

            //Stops the algorithm when it has looped too many times
            while (maxLoops > pathingLoops)
            {
                // Stops the algorithm if the search distance becomes too small
                if (initDistance < 1e-2) break;
                pathingLoops++;

                Pose bestPose = null;
                Heuristic best_H = calculate(poses[con_focus], con_focus, t_res);

                // Finds which point on the circle has the best heuristic value
                for (int i = 0; i < pointsOnCircle; i++)
                {
                    double angle = i * (Math.PI * 2) / pointsOnCircle;
                    Pose current = new Pose(poses[con_focus].x + initDistance * Math.Cos(angle), poses[con_focus].y + initDistance * Math.Sin(angle));
                    Heuristic h = calculate(current, con_focus, t_res);
                    if (compareHeuristics(h, best_H))
                    {
                        best_H = h;
                        bestPose = current;
                    }
                }

                // Only decays the distance when both control points have iterated through
                if (con_focus != init)
                    initDistance *= decayRate;
                if (bestPose == null)
                    continue;

                // Sets the current control point to the calculated best
                poses[con_focus] = bestPose;

                intersections = SplineMath.insideObstacles(poses, t_res, obstacles, robotRadius);
                con_focus = con_focus == 1 ? 2 : 1; // Switches the control point of focus
            }
        }


        /// <summary>
        /// Finds the heuristic value of a point 
        /// </summary>
        /// <param name="gridSpace">The Pose to be calculated</param>
        /// <param name="con_focus">The current control point of focus</param>
        /// <param name="t_res">Controls collision-check intervals (1/t resolution = number of points on curve)</param>
        /// <returns>A Heuristic of the the pose</returns>
        private Heuristic calculate(Pose gridSpace, int con_focus, double t_res)
        {
            Heuristic heuristic = new();
            // Bases the heuristic on the rest of the spline
            Pose[] splinePts = poses;
            splinePts[con_focus] = gridSpace;

            List<double> inter = SplineMath.insideObstacles(splinePts, t_res, obstacles, robotRadius);
            heuristic.intersectionCount = inter.Count;

            // Finds if the calculated control point causes collisions
            foreach (var interObj in inter)
            {
                double fault = SplineMath.mostInfluenceControl(interObj);
                if (fault == con_focus || fault == -1)
                    heuristic.influencedIntersection++;
            }

            // Finds the length of the curve with the
            heuristic.curveLength = SplineMath.curveLength(splinePts, t_res);

            heuristic.distanceFromMain = con_focus == 1 ? Pose.distance(splinePts[0], gridSpace) : Pose.distance(splinePts[1], gridSpace);
            // Influenced Intersections has a big impact of grid space's value
            // If there are none, do not count the total intersections
            // If there are, then cube the number to emphasize importance
            // Square curve length to emphasize importance of a short and optimal curve
            heuristic.value = heuristic.curveLength * heuristic.influencedIntersection * heuristic.intersectionCount +
                heuristic.distanceFromMain + heuristic.curveLength;

            heuristic.valuesOrdered = new double[]
            {
                heuristic.curveLength,
                heuristic.distanceFromMain,
                heuristic.intersectionCount,
                heuristic.value,
            };

            return heuristic;
        }

        /// <summary>
        /// <c>compareHeuristics</c> compares two heuristic based on which is higher priority in path finding
        /// </summary>
        /// <param name="cur"></param>
        /// <param name="refer"></param>
        /// <returns>True if <c>cur</c> has lower values, false if <c>cur</c> has higher values</returns>
        private bool compareHeuristics(Heuristic cur, Heuristic refer)
        {
            if (cur.influencedIntersection < refer.influencedIntersection)
                return true;

            if (cur.influencedIntersection == refer.influencedIntersection)
            {
                for (int i = 0; i < cur.valuesOrdered.Length; i++)
                {
                    if (cur.valuesOrdered[i] < refer.valuesOrdered[i])
                        return true;
                    else if (cur.valuesOrdered[i] > refer.valuesOrdered[i])
                        return false;
                }
            }

            Debug.Log("WHY");
            
            return false;
        }
        private bool compareSplineHeuristics(Heuristic cur, Heuristic refer)
        {
            for (int i = 0; i < cur.valuesOrdered.Length; i++)
            {
                if (cur.valuesOrdered[i] < refer.valuesOrdered[i])
                    return true;
                else if (cur.valuesOrdered[i] > refer.valuesOrdered[i])
                    return false;
            }
            
            return false;
        }

        
    }
}