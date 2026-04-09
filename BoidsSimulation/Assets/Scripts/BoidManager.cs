using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

public class BoidManager : MonoBehaviour
{
    [Header("Spawn")]
    public GameObject boidPrefab;
    [Range(1, 500)] public int boidCount = 60;
    public float spawnRadius = 3f;
    public float floorY = 0f;

    [Header("Speed")]
    [Range(0.5f, 20f)] public float minSpeed = 2f;
    [Range(1f, 40f)] public float maxSpeed = 5f;

    [Header("Perception")]
    [Range(1f, 30f)] public float perceptionRadius = 5f;
    [Range(0.1f, 10f)] public float separationRadius = 1.5f;

    [Header("Steering Weights (Primary Group)")]
    [Range(0f, 5f)] public float wSeparation = 2f;
    [Range(0f, 5f)] public float wAlignment = 1f;
    [Range(0f, 5f)] public float wCohesion = 1f;
    [Tooltip("Pull toward next waypoint. 2-4 is gentle, 8+ is urgent.")]
    [Range(0f, 10f)] public float wWaypoint = 3f;

    [Header("Heterogeneous Groups")]
    [Tooltip("0 = all primary, 0.5 = 50/50, 1 = all secondary")]
    [Range(0f, 1f)] public float secondaryFraction = 0f;
    public Color secondaryColor = new Color(1f, 0.35f, 0.1f, 1f);

    [Header("Steering Weights (Secondary Group)")]
    [Range(0f, 5f)] public float s_wSeparation = 2f;
    [Range(0f, 5f)] public float s_wAlignment = 1f;
    [Range(0f, 5f)] public float s_wCohesion = 1f;
    [Range(0f, 10f)] public float s_wWaypoint = 3f;

    [Header("Obstacle Avoidance")]
    public LayerMask obstacleLayer = 0;
    [Range(0.5f, 20f)] public float obstacleDistance = 4f;
    [Range(10f, 90f)] public float obstacleFanAngle = 55f;
    [Range(3, 21)] public int obstacleFanRays = 7;
    [Range(0f, 20f)] public float obstacleWeight = 6f;

    [Header("Course")]
    public bool useWaypoints = true;
    public List<BoidWaypoint> waypoints = new List<BoidWaypoint>();
    public bool loop = false;

    [Header("Boundary (when waypoints disabled)")]
    public float boundaryHalfSize = 20f;
    [Range(0f, 10f)] public float boundaryWeight = 3f;

    [Header("Iteration & Speed")]
    [Range(1, 100)] public int iterations = 1;
    public float maxIterationTime = 100f;
    [Range(1f, 10f)] public float simSpeedScale = 1f;

    [Header("Collision Detection")]
    public float collisionRadius = 1f;
    public float collisionGracePeriod = 2f;

    [Header("Metrics")]
    public bool showHUD = true;
    public Vector2 hudPosition = new Vector2(10f, 10f);

    private List<Boid> boids = new List<Boid>();
    private List<int> wpIndex = new List<int>(); // Current target waypoint index for each boid
    private List<float> wpOffset = new List<float>(); // Horizontal offset to prevent boids crowding single point

    private Vector3[] cPos; // Cached positions for thread-safe/performant distance checks
    private Vector3[] cVel; // Cached velocities
    private Vector3[] cPrevPos; // Used for crossing-plane detection

    private float simStart;
    private int spawned;
    private int finished;
    private int colBoidBoid;
    private int colBoidWall;
    private float transitSum;
    private List<float> transitList = new List<float>();
    private float tpWinStart;
    private int tpWinCount;
    private float tpRecent;

    // Stores unique pair IDs to ensure a single collision is only counted once per contact
    private HashSet<long> overlappingPairs = new HashSet<long>();

    private int currentIteration = 0;
    private bool allDone = false;

    private List<IterationResult> results = new List<IterationResult>();

    private struct IterationResult
    {
        public int iteration;
        public float duration;
        public float meanTransit;
        public float minTransit;
        public float maxTransit;
        public float throughput;
        public int colBoidBoid;
        public int colBoidWall;
        public int totalCol;
        public int finished;
        public int spawned;
    }

    void Start()
    {
        currentIteration = 0;
        allDone = false;
        results.Clear();
        StartNextIteration();
    }

    void Update()
    {
        Time.timeScale = allDone ? 1f : simSpeedScale;

        if (!allDone)
        {
            Simulate();
            TickThroughputWindow();

            if (Time.time - simStart >= maxIterationTime)
            {
                Debug.Log("[Boids] Iteration timed out at " + maxIterationTime + " seconds.");
                FinishIteration();
            }

        }
    }

    void OnDestroy() => Time.timeScale = 1f;
    void OnApplicationQuit() => Time.timeScale = 1f;

    void StartNextIteration()
    {
        currentIteration++;
        SpawnBoids();
    }

    void FinishIteration()
    {
        float dur = Time.time - simStart;

        var r = new IterationResult
        {
            iteration = currentIteration,
            duration = dur,
            meanTransit = MeanTransit(),
            minTransit = MinTransit(),
            maxTransit = MaxTransit(),
            throughput = spawned / Mathf.Max(0.001f, dur),
            colBoidBoid = colBoidBoid,
            colBoidWall = colBoidWall,
            totalCol = colBoidBoid + colBoidWall,
            finished = finished,
            spawned = spawned
        };
        results.Add(r);

        Debug.Log($"[Boids] Iteration {currentIteration}/{iterations} done. Duration={dur:F1}s MeanTransit={r.meanTransit:F2}s Collisions={r.totalCol}");

        if (currentIteration < iterations) StartNextIteration();
        else
        {
            allDone = true;
            Time.timeScale = 1f;
            SaveCSV();
        }
    }

    void SpawnBoids()
    {
        foreach (var b in boids) if (b) Destroy(b.gameObject);
        boids.Clear();
        wpIndex.Clear();
        wpOffset.Clear();
        overlappingPairs.Clear();

        simStart = Time.time;
        tpWinStart = Time.time;
        tpWinCount = 0;
        tpRecent = 0f;
        spawned = 0;
        finished = 0;
        colBoidBoid = 0;
        colBoidWall = 0;
        transitSum = 0f;
        transitList.Clear();

        Vector3 spawnCentre = waypoints.Count > 0
            ? new Vector3(waypoints[0].transform.position.x, floorY, waypoints[0].transform.position.z)
            : new Vector3(0f, floorY, 0f);

        // Calculate initial direction based on vector between first two waypoints
        Vector3 heading = Vector3.forward;
        if (waypoints.Count > 1)
        {
            Vector3 d = waypoints[1].transform.position - waypoints[0].transform.position;
            d.y = 0f;
            if (d.sqrMagnitude > 0.001f) heading = d.normalized;
        }

        int secCount = Mathf.RoundToInt(boidCount * secondaryFraction);

        for (int i = 0; i < boidCount; i++)
        {
            Vector2 rc = Random.insideUnitCircle * spawnRadius;
            Vector3 p = spawnCentre + new Vector3(rc.x, 0f, rc.y);
            var go = Instantiate(boidPrefab, p, Quaternion.identity, transform);
            var b = go.GetComponent<Boid>();

            bool sec = i < secCount;
            Vector3 jitter = new Vector3(Random.Range(-0.4f, 0.4f), 0f, Random.Range(-0.4f, 0.4f));
            Vector3 startV = (heading + jitter).normalized * minSpeed;

            b.Init(this, startV,
                   sec ? s_wAlignment : wAlignment,
                   sec ? s_wCohesion : wCohesion,
                   sec ? s_wSeparation : wSeparation,
                   sec ? s_wWaypoint : wWaypoint,
                   sec);

            if (sec && secondaryColor.a > 0f)
            {
                var rend = go.GetComponentInChildren<Renderer>();
                if (rend) rend.material.color = secondaryColor;
            }

            go.name = sec ? $"Boid_B_{i}" : $"Boid_A_{i}";
            boids.Add(b);
            wpIndex.Add(waypoints.Count > 1 ? 1 : 0);
            wpOffset.Add(RandomDoorOffset(waypoints.Count > 1 ? 1 : 0));
            spawned++;
        }

        cPos = new Vector3[boidCount];
        cVel = new Vector3[boidCount];
        cPrevPos = new Vector3[boidCount];
        for (int i = 0; i < boids.Count; i++) cPrevPos[i] = boids[i].pos;
    }

    void Simulate()
    {
        int n = boids.Count;
        if (n == 0) return;

        // Resize cache arrays if boids were added/removed
        if (cPos.Length < n)
        {
            cPos = new Vector3[n];
            cVel = new Vector3[n];
            cPrevPos = new Vector3[n];
        }

        // Fill cache with current frame data
        for (int i = 0; i < n; i++)
        {
            cPrevPos[i] = cPos[i];
            cPos[i] = boids[i].pos;
            cVel[i] = boids[i].velocity;
        }

        float perSq = perceptionRadius * perceptionRadius;
        float sepSq = separationRadius * separationRadius;
        float dt = Time.deltaTime;

        for (int i = n - 1; i >= 0; i--)
        {
            Boid b = boids[i];
            Vector3 pos = cPos[i];
            Vector3 vel = cVel[i];

            Vector3 sumAlign = Vector3.zero;
            Vector3 sumPos = Vector3.zero;
            Vector3 sumSep = Vector3.zero;
            int nbCount = 0;
            int sepCount = 0;

            // Nested loop to calculate relative positions of neighbors
            for (int j = 0; j < n; j++)
            {
                if (j == i) continue;
                Vector3 diff = pos - cPos[j];
                diff.y = 0f;
                float dsq = diff.sqrMagnitude;

                if (dsq < perSq)
                {
                    sumAlign += cVel[j]; // For Alignment
                    sumPos += cPos[j];   // For Cohesion
                    nbCount++;

                    if (dsq < sepSq && dsq > 0.0001f)
                    {
                        // Weighted separation: stronger push when closer
                        sumSep += diff.normalized / Mathf.Sqrt(dsq);
                        sepCount++;
                    }
                }
            }

            Vector3 steering = Vector3.zero;

            if (nbCount > 0)
            {
                // Alignment: Head in same direction as neighbors
                Vector3 avgVel = sumAlign / nbCount; avgVel.y = 0f;
                steering += Seek(vel, avgVel.normalized * maxSpeed) * b.wAlignment;

                // Cohesion: Move toward the center of the local group
                Vector3 com = sumPos / nbCount; com.y = floorY;
                steering += Seek(vel, (com - pos).normalized * maxSpeed) * b.wCohesion;
            }
            if (sepCount > 0)
            {
                // Separation: Avoid crowding neighbors
                Vector3 sep = sumSep / sepCount; sep.y = 0f;
                steering += Seek(vel, sep.normalized * maxSpeed) * b.wSeparation;
            }

            bool reached = false;
            if (useWaypoints)
            {
                steering += WaypointSteering(i, pos, vel, b.wWaypoint, out reached);
                if (reached) { FinishBoid(i); continue; }
            }

            if (obstacleLayer.value != 0)
                steering += ObstacleSteering(pos, vel);

            if (!useWaypoints || waypoints.Count == 0)
                steering += BoundarySteering(pos);

            steering.y = 0f;
            b.velocity += steering * dt;
            b.ApplyVelocity(minSpeed, maxSpeed);
        }

        // Collision logic using unique pair IDs to prevent multi-counting the same hit
        float colSq = collisionRadius * collisionRadius;
        float simElapsed = Time.time - simStart;
        int cn = boids.Count;

        for (int i = 0; i < cn - 1; i++)
        {
            for (int j = i + 1; j < cn; j++)
            {
                Vector3 d = boids[i].pos - boids[j].pos; d.y = 0f;
                float dsq = d.sqrMagnitude;
                long key = (long)i * 100000 + j;
                bool inRange = dsq < colSq;
                bool wasIn = overlappingPairs.Contains(key);

                if (inRange && !wasIn)
                {
                    overlappingPairs.Add(key);
                    // Only count if boids have existed long enough to escape the spawn pile
                    if (simElapsed > collisionGracePeriod
                        && (Time.time - boids[i].spawnTime) > collisionGracePeriod
                        && (Time.time - boids[j].spawnTime) > collisionGracePeriod)
                    {
                        colBoidBoid++;
                        boids[i].collisions++;
                        boids[j].collisions++;
                    }
                }
                else if (!inRange && wasIn) overlappingPairs.Remove(key);
            }
        }
    }

    static Vector3 Seek(Vector3 current, Vector3 desired)
    {
        desired.y = 0f;
        return desired - current;
    }

    float RandomDoorOffset(int wi)
    {
        if (wi < 0 || wi >= waypoints.Count) return 0f;
        float hw = waypoints[wi].width * 0.5f * 0.8f;
        return Random.Range(-hw, hw);
    }

    Vector3 WaypointSteering(int i, Vector3 pos, Vector3 vel, float weight, out bool finished)
    {
        finished = false;
        if (waypoints.Count == 0) return Vector3.zero;

        int wi = wpIndex[i];
        if (wi >= waypoints.Count) return Vector3.zero;

        // If the boid crossed the CURRENT waypoint rectangle/plane
        if (waypoints[wi].HasCrossed(cPrevPos[i], pos))
        {
            // If this was the LAST waypoint, finish immediately
            if (wi == waypoints.Count - 1)
            {
                if (loop)
                {
                    wi = 1; // keep waypoint 0 as spawn/start
                    wpIndex[i] = wi;
                    wpOffset[i] = RandomDoorOffset(wi);
                }
                else
                {
                    finished = true;
                    return Vector3.zero;
                }
            }
            else
            {
                wi++;
                wpIndex[i] = wi;
                wpOffset[i] = RandomDoorOffset(wi);
            }
        }

        BoidWaypoint wp = waypoints[wi];
        Vector3 centre = new Vector3(wp.transform.position.x, floorY, wp.transform.position.z);
        Vector3 goal = centre + wp.transform.right * wpOffset[i];
        goal.y = floorY;

        Vector3 toGoal = goal - pos;
        toGoal.y = 0f;

        if (toGoal.sqrMagnitude < 0.001f) return Vector3.zero;
        return Seek(vel, toGoal.normalized * maxSpeed) * weight;
    }

    Vector3 ObstacleSteering(Vector3 pos, Vector3 vel)
    {
        Vector3 fwd = vel; fwd.y = 0f;
        if (fwd.sqrMagnitude < 0.001f) return Vector3.zero;
        fwd = fwd.normalized;

        Vector3 avoidSum = Vector3.zero;
        int hits = 0;
        float step = obstacleFanRays > 1 ? obstacleFanAngle * 2f / (obstacleFanRays - 1) : 0f;
        Vector3 origin = new Vector3(pos.x, floorY + 0.1f, pos.z);

        // Raycast fan to find obstacles ahead
        for (int r = 0; r < obstacleFanRays; r++)
        {
            float angle = -obstacleFanAngle + step * r;
            Vector3 dir = Quaternion.AngleAxis(angle, Vector3.up) * fwd;
            if (Physics.Raycast(origin, dir, out RaycastHit hit, obstacleDistance, obstacleLayer, QueryTriggerInteraction.Ignore))
            {
                float t = 1f - hit.distance / obstacleDistance; // Higher weight for closer hits
                Vector3 push = hit.normal; push.y = 0f;
                if (push.sqrMagnitude < 0.001f) push = -dir;
                avoidSum += push.normalized * t;
                hits++;
            }
        }

        if (hits == 0) return Vector3.zero;
        avoidSum.y = 0f;
        return avoidSum.normalized * (maxSpeed * obstacleWeight);
    }

    Vector3 BoundarySteering(Vector3 pos)
    {
        Vector3 f = Vector3.zero; float h = boundaryHalfSize;
        if (pos.x > h) f.x -= 1f;
        if (pos.x < -h) f.x += 1f;
        if (pos.z > h) f.z -= 1f;
        if (pos.z < -h) f.z += 1f;
        return f * boundaryWeight * maxSpeed;
    }

    void FinishBoid(int i)
    {
        Boid b = boids[i];
        b.done = true;
        float transit = Time.time - b.spawnTime;
        transitSum += transit;
        transitList.Add(transit);
        finished++;
        tpWinCount++;

        boids.RemoveAt(i);
        wpIndex.RemoveAt(i);
        wpOffset.RemoveAt(i);
        Destroy(b.gameObject);

        if (boids.Count == 0) FinishIteration();
    }

    public void OnCollision(bool boidBoid)
    {
        if (!boidBoid) colBoidWall++;
    }

    void TickThroughputWindow()
    {
        float e = Time.time - tpWinStart;
        if (e >= 5f)
        {
            tpRecent = tpWinCount / e; // Boids per second over the last 5s
            tpWinStart = Time.time;
            tpWinCount = 0;
        }
    }

    float MeanTransit() => transitList.Count == 0 ? 0f : transitSum / transitList.Count;

    float MinTransit()
    {
        if (transitList.Count == 0) return 0f;
        float m = float.MaxValue;
        foreach (float t in transitList) if (t < m) m = t;
        return m;
    }

    float MaxTransit()
    {
        if (transitList.Count == 0) return 0f;
        float m = 0f;
        foreach (float t in transitList) if (t > m) m = t;
        return m;
    }

    void SaveCSV()
    {
        var lines = new List<string>();

        // Header
        lines.Add(
            "Iteration," +
            "wSeparation,wAlignment,wCohesion,wWaypoint," +
            "s_wSeparation,s_wAlignment,s_wCohesion,s_wWaypoint," +
            "SecondaryFraction," +
            "MeanTransit,Throughput,TotalCollisions,BoidBoidCollisions,BoidWallCollisions," +
            "Finished,Spawned"
        );

        foreach (var r in results)
        {
            lines.Add(
                $"{r.iteration}," +
                $"{wSeparation},{wAlignment},{wCohesion},{wWaypoint}," +
                $"{s_wSeparation},{s_wAlignment},{s_wCohesion},{s_wWaypoint}," +
                $"{secondaryFraction}," +
                $"{r.meanTransit:F3},{r.throughput:F3},{r.totalCol},{r.colBoidBoid},{r.colBoidWall}," +
                $"{r.finished},{r.spawned}"
            );
        }

        string folder = Path.Combine(Application.dataPath, "..", "..", "Data", "data");

        if (!Directory.Exists(folder))
            Directory.CreateDirectory(folder);

        string path = Path.Combine(
            folder,
            $"boids_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv"
        );

        File.WriteAllLines(path, lines.ToArray(), Encoding.UTF8);
        Debug.Log($"[Boids] Results saved to: {Path.GetFullPath(path)}");
    }

    float Avg(System.Func<IterationResult, float> selector)
    {
        if (results.Count == 0) return 0f;
        float sum = 0f;
        foreach (var r in results) sum += selector(r);
        return sum / results.Count;
    }

    void OnGUI()
    {
        if (!showHUD) return;

        float dur = Time.time - simStart;
        float tp = spawned > 0 ? finished / Mathf.Max(0.001f, dur) : 0f;

        var sb = new StringBuilder();

        if (allDone)
        {
            sb.AppendLine("=== ALL DONE ===");
            sb.AppendLine($"Iterations: {results.Count}");
            sb.AppendLine($"Avg duration   {Avg(r => r.duration):F2} s");
            sb.AppendLine($"Avg throughput {Avg(r => r.throughput):F2} /s");
            sb.AppendLine($"Avg transit    {Avg(r => r.meanTransit):F2} s");
            sb.AppendLine($"Avg collisions {Avg(r => r.totalCol):F1}");
            sb.AppendLine("[CSV saved]");
        }
        else
        {
            sb.AppendLine($"=== Run {currentIteration} / {iterations} ===");
            sb.AppendLine($"Speed x{simSpeedScale:F1}");
            sb.AppendLine($"Time       {dur:F1} s");
            sb.AppendLine($"Active     {boids.Count}");
            sb.AppendLine($"Finished   {finished} / {spawned}");
            sb.AppendLine($"Throughput {tp:F2} /s");
            sb.AppendLine($"Tp (5s)    {tpRecent:F2} /s");
            sb.AppendLine($"Transit    {MeanTransit():F2} s (mean)");
            sb.AppendLine($"Boid hits  {colBoidBoid}");
            sb.AppendLine($"Wall hits  {colBoidWall}");
            if (results.Count > 0)
            {
                sb.AppendLine($"--- prev avg ---");
                sb.AppendLine($"Transit  {Avg(r => r.meanTransit):F2} s");
                sb.AppendLine($"Col      {Avg(r => r.totalCol):F1}");
            }
        }

        GUIStyle st = new GUIStyle(GUI.skin.box)
        {
            alignment = TextAnchor.UpperLeft,
            fontSize = 13
        };
        st.normal.textColor = Color.white;

        float w = 230f, h = allDone ? 170f : 280f;
        GUI.Box(new Rect(hudPosition.x, hudPosition.y, w, h), sb.ToString(), st);

        if (allDone && GUI.Button(
            new Rect(hudPosition.x, hudPosition.y + h + 4f, w, 24f), "Save CSV Again"))
        {
            SaveCSV();
        }
    }

  

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (waypoints.Count == 0)
        {
            Gizmos.color = new Color(0f, 1f, 1f, 0.4f);
            Gizmos.DrawWireCube(new Vector3(0, floorY, 0),
                new Vector3(boundaryHalfSize * 2, 0.05f, boundaryHalfSize * 2));
        }

        for (int i = 1; i < waypoints.Count; i++)
        {
            if (waypoints[i] == null) continue;
            bool isLast = i == waypoints.Count - 1 && !loop;
            Gizmos.color = isLast ? Color.red : Color.yellow;
            int prev = i == 1 ? 0 : i - 1;
            if (waypoints[prev] != null)
                Gizmos.DrawLine(waypoints[prev].transform.position,
                                waypoints[i].transform.position);
        }

        if (boids != null && boids.Count > 0 && boids[0] != null
            && obstacleLayer.value != 0)
        {
            Vector3 bp = boids[0].pos;
            Vector3 fwd = boids[0].velocity; fwd.y = 0f;
            if (fwd.sqrMagnitude > 0.001f)
            {
                fwd = fwd.normalized;
                float stp = obstacleFanRays > 1
                            ? obstacleFanAngle * 2f / (obstacleFanRays - 1) : 0f;
                for (int r = 0; r < obstacleFanRays; r++)
                {
                    float a = -obstacleFanAngle + stp * r;
                    Vector3 d = Quaternion.AngleAxis(a, Vector3.up) * fwd;
                    Gizmos.color = new Color(1f, 0.5f, 0f, 0.35f);
                    Gizmos.DrawRay(new Vector3(bp.x, floorY + 0.1f, bp.z), d * obstacleDistance);
                }
            }
            Gizmos.color = new Color(0f, 1f, 0f, 0.2f);
            Gizmos.DrawWireSphere(bp, perceptionRadius);
            Gizmos.color = new Color(1f, 0f, 0f, 0.2f);
            Gizmos.DrawWireSphere(bp, separationRadius);
        }
    }
#endif
}