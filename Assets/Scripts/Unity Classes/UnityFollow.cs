using BezierNavigator.PathFollower;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class UnityFollow : MonoBehaviour
{
    private SplineHolder spline;
    private Follower follower;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            spline = FindObjectOfType<SplineHolder>();
            follower = new Follower(convert(spline.getPoints()), 5, UnityPath.t_Res);
        }
        if (Input.GetKey(KeyCode.Space))
        {
            try
            {
                transform.position = Vector2.MoveTowards(transform.position, UnityPath.PoseToVector(follower.getTarget(UnityPath.TransformToPose(transform))), 25 * Time.deltaTime);
            } catch
            {
                transform.position = spline.getPoints()[0].position;
            }
        }
    }

    BezierNavigator.Pose[] convert(Transform[] ts)
    {
        BezierNavigator.Pose[] p = new BezierNavigator.Pose[ts.Length];
        for (int i = 0; i < ts.Length; i++)
        {
            p[i] = UnityPath.TransformToPose(ts[i]);
        }
        return p;
    }
}
