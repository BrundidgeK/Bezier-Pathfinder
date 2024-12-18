using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Constants : MonoBehaviour
{
    public double t_Res;
    public double robotRadius;
    public int maxIterations = 25;
    public double decayRate = .8;
    public double initDistance = 20;
    public int pointsOnCircle = 32;

    public enum constant { res, radius, decay, iteration, distance, points};

    public void setValues()
    {
        UnityPath.robotRadius = robotRadius;
        UnityPath.t_Res = t_Res;
        UnityPath.maxIterations = maxIterations;
        UnityPath.decayRate = decayRate;
        UnityPath.initDistance = initDistance;
        UnityPath.pointsOnCircle = pointsOnCircle;

        ObstacleHolder[] holders = FindObjectsOfType<ObstacleHolder>();
        BezierNavigator.Obstacle[] obstacles = new BezierNavigator.Obstacle[holders.Length];
        for (int i = 0; i < holders.Length; i++)
        {
            obstacles[i] = holders[i].GetObstacle();
        }
        UnityPath.obstacles = obstacles;

        SplineHolder[] splines = FindObjectsOfType<SplineHolder>();
        foreach(SplineHolder spline in splines)
        {
            spline.rend.endWidth = (float)(robotRadius * 2);
            spline.rend.startWidth = (float)(robotRadius * 2);
        }

        foreach (UnityPath p in FindObjectsOfType<UnityPath>())
        {
            p.findPath();
            Debug.Log("Complete!");
        }
    }

    public void setTRes(string text)
    {
        try
        {
            t_Res = double.Parse(text);
        }
        catch
        {
            Console.WriteLine("Invalid input for t_Res");
        }
    }

    public void setRadius(string text)
    {
        try
        {
            robotRadius = double.Parse(text);
        }
        catch
        {
            Console.WriteLine("Invalid input for robotRadius");
        }
    }

    public void setPointsOnCircle(string text)
    {
        try
        {
            pointsOnCircle = int.Parse(text);
        }
        catch
        {
            Console.WriteLine("Invalid input for pointsOnCircle");
        }
    }

    public void setDecayRate(string text)
    {
        try
        {
            decayRate = double.Parse(text);
        }
        catch
        {
            Console.WriteLine("Invalid input for decayRate");
        }
    }

    public void setInitDistance(string text)
    {
        try
        {
            initDistance = double.Parse(text);
        }
        catch
        {
            Console.WriteLine("Invalid input for initDistance");
        }
    }

    public void setIterationLimit(string text)
    {
        try
        {
            maxIterations = int.Parse(text);
        }
        catch
        {
            Console.WriteLine("Invalid input for iteration limit");
        }
    }

}
