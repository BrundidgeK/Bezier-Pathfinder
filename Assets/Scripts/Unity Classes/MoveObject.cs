using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveObject : MonoBehaviour
{
    private bool dragging;

    private void Update()
    {
        if (dragging)
        {
            Camera cam = Camera.main;
            Vector2 position = cam.ScreenToWorldPoint(Input.mousePosition);
            transform.position = new Vector3(position.x, position.y, transform.position.z);
        }
    }

    private void OnMouseDown()
    {
        dragging = true;
    }

    private void OnMouseUp()
    {
        dragging = false;
    }
}
