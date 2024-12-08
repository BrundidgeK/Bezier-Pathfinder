using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BezierNavigator
{
    public class PathFinder
    {
        private int gridX, gridY;
        private Pose[] poses;
        private double robotRadius;
        private Obstacle[] obstacles;

        public PathFinder(int gridX, int gridY)
        {
            this.gridX = gridX;
            this.gridY = gridY;
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

        public void findPath(double t_res)
        {
            List<double> intersections = SplineMath.insideObstacles(poses, t_res, obstacles, robotRadius);

            while (intersections.Count > 0)
            {
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

                Pose bestPose = null;
                double best_H = Mathf.Infinity;

                for (int x = 0; x < gridX; x++)
                {
                    for (int y = 0; y < gridY; y++)
                    {
                        Pose current = new Pose(x, y);
                        double h = calculate(current, con_focus, t_res);
                        if (h < best_H)
                        {
                            best_H = h;
                            bestPose = current;
                        }
                    }
                }

                poses[con_focus] = bestPose;

                intersections = SplineMath.insideObstacles(poses, t_res, obstacles, robotRadius);
            }
        }

        public void findPath(double t_res, int maxLoops)
        {
            List<double> intersections = SplineMath.insideObstacles(poses, t_res, obstacles, robotRadius);
            int pathingLoops = 0;

            while (intersections.Count > 0 && maxLoops > pathingLoops)
            {
                pathingLoops++;

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

                Pose bestPose = null;
                double best_H = Mathf.Infinity;

                for (int x = 0; x < gridX; x++)
                {
                    for (int y = 0; y < gridY; y++)
                    {
                        Pose current = new Pose(x, y);
                        double h = calculate(current, con_focus, t_res);
                        if (h < best_H)
                        {
                            best_H = h;
                            bestPose = current;
                        }
                    }
                }

                poses[con_focus] = bestPose;

                intersections = SplineMath.insideObstacles(poses, t_res, obstacles, robotRadius);
            }
        }

        private double calculate(Pose gridSpace, int con_focus, double t_res)
        {
            double value = 0;
            Pose[] splinePts = poses;
            splinePts[con_focus] = gridSpace;

            List<double> inter = SplineMath.insideObstacles(splinePts, t_res, obstacles, robotRadius);
            foreach (var interObj in inter)
            {
                if (interObj == con_focus)
                    value++;
            }

            value += Pose.distance(splinePts[0], gridSpace);
            value += Pose.distance(splinePts[1], gridSpace);

            return value;
        }
    }
}