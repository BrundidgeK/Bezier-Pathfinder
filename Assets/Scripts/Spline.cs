using System;
using System.Collections.Generic;
using UnityEngine.UIElements;

namespace BezierNavigator
{
    public class Spline
    {
        private Pose[] spline;
        private Heuristic heuristic;

        public Spline(Pose[] pts)
        {
            this.spline = pts;
        }

        public Pose[] getPoints()
        {
            return spline; 
        }

        public void setPoint(int index, Pose pt)
        {
            spline[index] = pt;
        }
        
        public Pose getPoint(int index)
        {
            return spline[index];
        }

        public void calculate(Obstacle[] obs, int con_focus, double radius, double t_res)
        {
            List<double> inter = SplineMath.insideObstacles(spline, t_res, obs, radius);
            heuristic.intersectionCount = inter.Count;

            // Finds if the calculated control point causes collisions
            foreach (var interObj in inter)
            {
                double fault = SplineMath.mostInfluenceControl(interObj);
                if (fault == con_focus || fault == -1)
                    heuristic.influencedIntersection++;
            }

            // Finds the length of the curve with the
            heuristic.curveLength = SplineMath.curveLength(spline, t_res);

            heuristic.distanceFromMain = con_focus == 1 ? Pose.distance(spline[0], spline[1]) : Pose.distance(spline[2], spline[3]);
            // Influenced Intersections has a big impact of grid space's value
            // If there are none, do not count the total intersections
            // If there are, then cube the number to emphasize importance
            // Square curve length to emphasize importance of a short and optimal curve
            heuristic.evaluate();

            heuristic.valuesOrdered = new double[]
            {
                heuristic.curveLength,
                heuristic.distanceFromMain,
                heuristic.intersectionCount,
                heuristic.value,
            };
        }

        public void calculateSpline(Obstacle[] obs, double radius, double t_res)
        {
            List<double> inter = SplineMath.insideObstacles(spline, t_res, obs, radius);
            heuristic.intersectionCount = inter.Count;
            heuristic.influencedIntersection = 0;

            UnityEngine.Debug.Log("Interections: " + inter.Count);

            // Finds the length of the curve with the
            heuristic.curveLength = SplineMath.curveLength(spline, t_res);

            heuristic.distanceFromMain =Pose.distance(spline[0], spline[1]) + Pose.distance(spline[2], spline[3]);
            heuristic.evaluate();

            heuristic.valuesOrdered = new double[]
            {
                heuristic.intersectionCount,
                heuristic.value,
                heuristic.curveLength,
                heuristic.distanceFromMain,
            };
        }

        public Heuristic GetHeuristic()
        {
            return heuristic;
        }

        public String toString()
        {
            return spline[0].toString() + ", " + spline[1].toString() + ", " + spline[2].toString() + ", " + spline[3].toString();
        }
    }

	public struct Heuristic
	{
		public double curveLength,
				distanceFromMain,
				value;
		public int intersectionCount,
            influencedIntersection;

		public double[] valuesOrdered;

        public void evaluate()
        {
            value = 1000 * intersectionCount +
                  500 * influencedIntersection +
                  curveLength +
                  0.1 * distanceFromMain;
        }
	}
}
