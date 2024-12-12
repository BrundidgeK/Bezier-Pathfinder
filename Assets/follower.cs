using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class follower : MonoBehaviour
{
    public LineRenderer lineRenderer;
    public int index;
    Boolean increasing = false;
    Vector3 points;

    private void Start()
    {
        transform.position = lineRenderer.GetPosition(0);
        points = lineRenderer.GetPosition(lineRenderer.positionCount/2);
    }

    void Update()
    {
        if(points != lineRenderer.GetPosition(lineRenderer.positionCount / 2))
        {
            transform.position = lineRenderer.GetPosition(index);
            points = lineRenderer.GetPosition(lineRenderer.positionCount / 2);
        }

        transform.position = Vector2.MoveTowards(transform.position, lineRenderer.GetPosition(index), 50*Time.deltaTime);
        if(Vector2.Distance(transform.position, lineRenderer.GetPosition(index)) <= .5)
        {
            if (increasing)
            {
                index++;
                if(index == lineRenderer.positionCount)
                {
                    index -= 2;
                    increasing = false;
                }
            }
            else
            {
                index--; 
                if (index == -1)
                {
                    index += 2;
                    increasing = true;
                }
            }
        }
    }
}
