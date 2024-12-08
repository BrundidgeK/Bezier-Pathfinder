using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using Unity.VisualScripting;
using UnityEngine;

public class UnityPath : MonoBehaviour
{
    private BezierNavigator.PathFinder finder;

    public static int gridX, gridY;
    public static double t_Res;
    public static double robotRadius;
    public static BezierNavigator.Obstacle[] obstacles;

    public BezierNavigator.Pose[] poses;

    public void findPath()
    {
        finder = new BezierNavigator.PathFinder(gridX, gridY);
        finder.setObstacles(obstacles);
        finder.setRadius(robotRadius);
        finder.setSplinePoints(poses);

        finder.findPath(t_Res, 100);
        BezierNavigator.Pose[] newSpline = finder.getSplinePoints();
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
