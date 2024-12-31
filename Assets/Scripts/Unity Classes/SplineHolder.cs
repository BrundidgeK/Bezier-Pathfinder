using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SplineHolder : MonoBehaviour
{
    public LineRenderer rend;
    [SerializeField]
    private Transform[] splinePts;

    private void Awake()
    {
        rend = GetComponent<LineRenderer>();
        RenderSpline();
    }

    // Update is called once per frame
    void Update()
    {
        RenderSpline();
    }

    private void RenderSpline()
    {
        UnityPath path = GetComponent<UnityPath>();
        BezierNavigator.Pose[] poses = new BezierNavigator.Pose[splinePts.Length];
        for(int i = 0; i < splinePts.Length; i++)
        {
            poses[i] = UnityPath.TransformToPose(splinePts[i]);
        }
        path.poses = poses;

        double t_res = FindObjectOfType<Constants>().t_Res;
        int loops = (int)(1 / t_res)+1;
        rend.positionCount = loops;
        for(int i = 0; i < loops; i++)
        {
            rend.SetPosition(i, UnityPath.PoseToVector(BezierNavigator.SplineMath.calculate(poses, i * t_res)));
        }
    }

    public void changePts(Vector3[] pts)
    {
        for(int i = 0;i < pts.Length;i++)
        {
            splinePts[i].position = pts[i];
        }
    }

    public Transform[] getPoints()
    {
        return splinePts;
    }
}
