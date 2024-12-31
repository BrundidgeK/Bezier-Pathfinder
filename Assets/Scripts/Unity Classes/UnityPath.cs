using System;
using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using TMPro;
using Unity.VisualScripting;
using UnityEngine;

public class UnityPath : MonoBehaviour
{
    private BezierNavigator.PathFinder finder;

    public static double t_Res;
    public static double robotRadius;
    public static BezierNavigator.Obstacle[] obstacles;
    public static int maxIterations = 25;
    public static double decayRate = .8;
    public static double initDistance = 20;
    public static int pointsOnCircle = 32;

    public BezierNavigator.Pose[] poses;
    public TMP_Text timeText;

    [SerializeField]
    public GameObject circl;

    private void Start()
    {
        finder = new BezierNavigator.PathFinder();
    }

    public void findPath()
    {
        long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
        finder.setObstacles(obstacles);
        finder.setRadius(robotRadius);
        finder.setSplinePoints(poses);

        finder.findPath(t_Res, 20, decayRate, pointsOnCircle, maxIterations);

        BezierNavigator.Pose[] newSpline = finder.getSplinePoints();
        GetComponent<SplineHolder>().changePts(
            new Vector3[]
            {
                PoseToVector(newSpline[0]),
                PoseToVector(newSpline[1]),
                PoseToVector(newSpline[2]),
                PoseToVector(newSpline[3])
            });

        double timeElasped = ((DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond) - milliseconds) / 1000.0;
        timeText.text = "Calculated in " + timeElasped + " seconds";
    }

    public static BezierNavigator.Pose TransformToPose(Transform t)
    {
        return new BezierNavigator.Pose(t.position.x, t.position.y);
    }
    public static Vector3 PoseToVector(BezierNavigator.Pose p)
    {
        return new Vector3((float)p.x, (float)p.y, 0);
    }
}
