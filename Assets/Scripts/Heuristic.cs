using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Heuristic 
{
    private Pose pose;
    public double value;

    public Heuristic(Pose pose)
    {
        this.pose = pose;
    }

    public void calculate(Obstacle[] obstacles, Pose[] splinePts, int con_focus, double radius, double t_res)
    {
        value = 0;
        splinePts[con_focus] = pose;

        List<double> inter = SplineMath.insideObstacles(splinePts, t_res, obstacles, radius);
        foreach(var interObj in inter)
        {
            if (interObj == con_focus)
                value++;
        }

        value += Pose.distance(splinePts[0], pose);
        value += Pose.distance(splinePts[1], pose);
    }
}
