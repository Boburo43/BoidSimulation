using UnityEngine;

public class BoidWaypoint : MonoBehaviour
{
    [Tooltip("Width of the door opening.")]
    public float width = 4f;

    [Tooltip("Visual height of the door gizmo (does not affect detection).")]
    public float height = 3f;

    // Detects if a boid passed through the "door" plane between two frames
    public bool HasCrossed(Vector3 prevWorld, Vector3 currWorld)
    {
        // Convert world positions to local space relative to the waypoint's orientation
        Vector3 prev = transform.InverseTransformPoint(prevWorld);
        Vector3 curr = transform.InverseTransformPoint(currWorld);

        // If local Z signs are the same, the boid is still on the same side of the door
        if (Mathf.Sign(prev.z) == Mathf.Sign(curr.z)) return false;

        // Calculate the exact intersection point on the Z=0 plane (the door frame)
        float t = Mathf.Abs(prev.z) / (Mathf.Abs(prev.z) + Mathf.Abs(curr.z));
        float cx = Mathf.Lerp(prev.x, curr.x, t);

        // Verify the boid passed between the door posts (within the width), not through the "wall"
        return Mathf.Abs(cx) <= width * 0.5f;
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        // Identify if this is the first waypoint (the spawn area)
        bool isSpawn = false;
        var mgr = FindObjectOfType<BoidManager>();
        if (mgr != null && mgr.waypoints.Count > 0 && mgr.waypoints[0] == this)
            isSpawn = true;

        Matrix4x4 old = Gizmos.matrix;
        // Match Gizmo drawing to the object's transform (rotation/scale)
        Gizmos.matrix = transform.localToWorldMatrix;

        if (isSpawn)
        {
            Gizmos.color = new Color(0f, 1f, 0f, 0.25f);
            DrawCircleXZ(Vector3.zero, width * 0.5f);
            Gizmos.color = Color.green;
            DrawCircleXZ(Vector3.zero, width * 0.5f);
        }
        else
        {
            float hw = width * 0.5f;
            float hh = height * 0.5f;

            Gizmos.color = new Color(0f, 1f, 1f, 0.15f);
            DrawQuad(hw, hh);

            Gizmos.color = Color.cyan;
            DrawDoorFrame(hw, hh);

            // Yellow arrow indicates the required direction of travel (Z-forward)
            Gizmos.color = new Color(1f, 1f, 0f, 0.8f);
            Gizmos.DrawRay(Vector3.zero, Vector3.forward * (width * 0.4f));
        }

        Gizmos.matrix = old;

        // Draw text label above the waypoint in the Scene View
        UnityEditor.Handles.color = isSpawn ? Color.green : Color.cyan;
        UnityEditor.Handles.Label(transform.position + Vector3.up * (height * 0.5f + 0.3f),
            isSpawn ? gameObject.name + " (SPAWN)" : gameObject.name);
    }

    void DrawDoorFrame(float hw, float hh)
    {
        Gizmos.DrawLine(new Vector3(-hw, -hh, 0), new Vector3(hw, -hh, 0));
        Gizmos.DrawLine(new Vector3(-hw, hh, 0), new Vector3(hw, hh, 0));
        Gizmos.DrawLine(new Vector3(-hw, -hh, 0), new Vector3(-hw, hh, 0));
        Gizmos.DrawLine(new Vector3(hw, -hh, 0), new Vector3(hw, hh, 0));
    }

    void DrawQuad(float hw, float hh)
    {
        // Fills the door area with lines to make it visible as a surface
        int steps = 8;
        for (int i = 0; i <= steps; i++)
        {
            float y = Mathf.Lerp(-hh, hh, i / (float)steps);
            Gizmos.DrawLine(new Vector3(-hw, y, 0), new Vector3(hw, y, 0));
        }
    }

    void DrawCircleXZ(Vector3 centre, float r)
    {
        int seg = 24;
        float step = 360f / seg;
        for (int i = 0; i < seg; i++)
        {
            float a1 = i * step * Mathf.Deg2Rad;
            float a2 = (i + 1) * step * Mathf.Deg2Rad;
            Gizmos.DrawLine(
                centre + new Vector3(Mathf.Cos(a1), 0, Mathf.Sin(a1)) * r,
                centre + new Vector3(Mathf.Cos(a2), 0, Mathf.Sin(a2)) * r);
        }
    }
#endif
}