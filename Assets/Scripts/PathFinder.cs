using System;
using System.Collections;
using System.Collections.Generic;

namespace BezierNavigator
{
    public class PathFinder
    {
        private Pose[] poses;
        private double robotRadius;
        private Obstacle[] obstacles;

        public PathFinder()
        {
        }

        public void setSplinePoints(Pose[] splinePoints)
        {
            if (splinePoints.Length != 4)
                return;
            poses = splinePoints;
        }
        public Pose[] getSplinePoints()
        {
            return poses;
        }

        public void setRadius(double radius)
        {
            robotRadius = radius;
        }

        public void setObstacles(Obstacle[] obstacles)
        {
            this.obstacles = obstacles;
        }

        public void findPath(double t_res, double initDistance, double decayRate, int pointsOnCircle, int maxLoops)
        {
            Pose origin = Pose.midPoint(poses[0], poses[3]);
            int distanceSpline = (int)Pose.distance(poses[0], poses[3]);

            poses[1] = Pose.midPoint(poses[0], origin);
            poses[2] = Pose.midPoint(poses[3], origin);

            List<double> intersections = SplineMath.insideObstacles(poses, t_res, obstacles, robotRadius);
            int pathingLoops = 0;
            int same = -1;
            int count = intersections.Count;

            int con1 = 0, con2 = 0;
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
            int con_focus = con1 > con2 ? 1 : 2;
            int init = con_focus;

            while (maxLoops > pathingLoops)
            {
                if (initDistance < 1e-2) break;
                if (count == intersections.Count)
                    same++;
                if (same >= maxLoops / 4)
                    break;
                pathingLoops++;

                Pose bestPose = null;
                double[] best_H = calculate(poses[con_focus], con_focus, t_res);

                for (int i = 0; i < pointsOnCircle; i++)
                {
                    double angle = i * (Math.PI * 2) / pointsOnCircle;
                    Pose current = new Pose(poses[con_focus].x + initDistance * Math.Cos(angle), poses[con_focus].y + initDistance * Math.Sin(angle));
                    double[] h = calculate(current, con_focus, t_res);
                    if (h[1] < best_H[1] ||
                        (h[1] == best_H[1] && (h[3] < best_H[3])))
                    {
                        best_H = h;
                        bestPose = current;
                    }
                }

                if (con_focus != init)
                    initDistance *= decayRate;
                if (bestPose == null)
                    continue;
                poses[con_focus] = bestPose;

                intersections = SplineMath.insideObstacles(poses, t_res, obstacles, robotRadius);
                con_focus = con_focus == 1 ? 2 : 1;
            }
        }

        private double[] calculate(Pose gridSpace, int con_focus, double t_res)
        {
            double[] values = new double[4] { 999, 999, 999, 999 };
            Pose[] splinePts = poses;
            splinePts[con_focus] = gridSpace;

            List<double> inter = SplineMath.insideObstacles(splinePts, t_res, obstacles, robotRadius);
            //if (inter.Count > 0)
            //    return values;
            foreach (var interObj in inter)
            {
                if (SplineMath.mostInfluenceControl(interObj) == con_focus)
                    values[1]++;
            }

            Pose start = SplineMath.secondDerivative(splinePts, 0);
            Pose end = SplineMath.secondDerivative(splinePts, 1);
            double continuity = Math.Sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));

            values[2] = con_focus == 1 ? Pose.distance(splinePts[0], gridSpace) : Pose.distance(splinePts[1], gridSpace);
            values[0] = inter.Count * t_res * 100;
            values[3] = values[0] * MathF.Pow((float)values[1], 2) * values[2] * continuity;

            return values;
        }
    }
}