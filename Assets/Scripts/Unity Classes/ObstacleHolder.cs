using BezierNavigator;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObstacleHolder : MonoBehaviour
{
    private List<Transform> pts = new List<Transform>();
    private BezierNavigator.Obstacle obstacle;
    private LineRenderer rend;

    // Start is called before the first frame update
    void Awake()
    {
        rend = GetComponent<LineRenderer>();
        Render();
    }

    private void Update()
    {
        Render();
    }

    public void Render()
    {
        pts = GetChildrenInOrder();
        rend.positionCount = pts.Count + 1;
        for (int i = 0; i < pts.Count; i++)
        {
            rend.SetPosition(i, pts[i].position);
            if (i + 1 == pts.Count)
                rend.SetPosition(i+1, pts[0].position);
            else
                rend.SetPosition(i+1, pts[i + 1].position);
        }

        obstacle = new BezierNavigator.Obstacle(convert());
    }

    private List<Transform> GetChildrenInOrder()
    {
        List<Transform> children = new List<Transform>();

        // Iterate through all children of the parent Transform
        for (int i = 0; i < transform.childCount; i++)
        {
            children.Add(transform.GetChild(i));
        }

        return children;
    }

    private BezierNavigator.Pose[] convert()
    {
        BezierNavigator.Pose[] poses = new BezierNavigator.Pose[pts.Count];
        for(int i = 0; i < pts.Count;i++)
        {
            poses[i] = UnityPath.TransformToPose(pts[i]);
        }

        return poses;
    }

    public BezierNavigator.Obstacle GetObstacle()
    {
        return obstacle;
    }
}
