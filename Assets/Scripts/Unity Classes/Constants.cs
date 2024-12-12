using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Constants : MonoBehaviour
{
    public int gridX, gridY;
    public double t_Res;
    public double robotRadius;

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

        ObstacleHolder[] holders = FindObjectsOfType<ObstacleHolder>();
        BezierNavigator.Obstacle[] obstacles = new BezierNavigator.Obstacle[holders.Length];
        for (int i = 0; i < holders.Length; i++)
        {
            obstacles[i] = holders[i].GetObstacle();
        }
        UnityPath.obstacles = obstacles;

    }

    void Update()
    {
        setValues(); foreach (UnityPath p in FindObjectsOfType<UnityPath>())
        {
            p.findPath();
            Debug.Log("Complete!");
        }
        if (Input.GetKeyDown(KeyCode.Space))
        {
            Debug.Log("Processing...");
            
        }
    }
}
