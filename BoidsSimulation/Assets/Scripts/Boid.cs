using UnityEngine;

public class Boid : MonoBehaviour
{
    // These are modified by the BoidManager simulation loop every frame
    [HideInInspector] public Vector3 velocity;
    [HideInInspector] public Vector3 pos;

    // Local copies of weights so different boid groups can have different behaviors
    [HideInInspector] public float wAlignment;
    [HideInInspector] public float wCohesion;
    [HideInInspector] public float wSeparation;
    [HideInInspector] public float wWaypoint;

    [HideInInspector] public float spawnTime;
    [HideInInspector] public bool done;
    [HideInInspector] public int collisions; // Counter for personal boid-to-boid hits
    [HideInInspector] public bool isSecondary;

    private BoidManager mgr;
    private Rigidbody rb;

    public void Init(BoidManager m, Vector3 startVel,
                     float align, float cohesion, float sep, float wp,
                     bool secondary)
    {
        mgr = m;
        velocity = startVel;
        pos = transform.position;
        wAlignment = align;
        wCohesion = cohesion;
        wSeparation = sep;
        wWaypoint = wp;
        isSecondary = secondary;
        spawnTime = Time.time;
        done = false;
        collisions = 0;

        // Configures physics to be non-simulated (Kinematic) so we can move it manually
        rb = gameObject.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        // Lock to 2D plane
        rb.constraints = RigidbodyConstraints.FreezeRotation
                         | RigidbodyConstraints.FreezePositionY;
    }

    public void ApplyVelocity(float minSpeed, float maxSpeed)
    {
        velocity.y = 0f;
        float spd = velocity.magnitude;

        // Clamp speed within defined limits
        if (spd > maxSpeed) velocity = velocity.normalized * maxSpeed;
        else if (spd < minSpeed && spd > 0.001f) velocity = velocity.normalized * minSpeed;

        // Use MovePosition for smoother movement with Rigidbody interpolation
        Vector3 next = pos + velocity * Time.deltaTime;
        next.y = mgr.floorY;
        rb.MovePosition(next);
        pos = next;

        // Rotate to face the direction of travel
        if (velocity.sqrMagnitude > 0.001f)
            rb.MoveRotation(Quaternion.LookRotation(
                new Vector3(velocity.x, 0f, velocity.z)));
    }
}