using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Constants : MonoBehaviour
{
    public int gridX, gridY;
    public double t_Res;
    public double robotRadius;
    public int maxIterations = 25;

    void Start()
    {
        setValues();
    }

    private void setValues()
    {
        UnityPath.gridX = gridX;
        UnityPath.gridY = gridY;
        UnityPath.robotRadius = robotRadius;
        UnityPath.t_Res = t_Res;
        UnityPath.maxIterations = maxIterations;

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
    }

    void Update()
    {
        setValues(); 
        if (Input.GetKeyDown(KeyCode.Space))
        {
            Debug.Log("Processing...");
            foreach (UnityPath p in FindObjectsOfType<UnityPath>())
            {
                p.findPath();
                Debug.Log("Complete!");
            }
        }
    }
}
