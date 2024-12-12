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

        finder.findPath(t_Res, 1);
        BezierNavigator.Pose[] newSpline = finder.getSplinePoints();
        GetComponent<SplineHolder>().changePts(
            new Vector3[]
            {
                PoseToVector(newSpline[0]),
                PoseToVector(newSpline[1]),
                PoseToVector(newSpline[2]),
                PoseToVector(newSpline[3])
            });
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
