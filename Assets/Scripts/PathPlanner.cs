using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(BotMover))]
public class PathPlanner : MonoBehaviour
{
    private enum NavigationState
    {
        DirectMove,
        MoveToWaypoint,
        Backtrack,
        KnownRoute
    }

    private class CellMemory
    {
        public int visits;
        public int deadEnd;
        public int proven;
        public float y;
    }

    private class RouteNode
    {
        public Vector3 pos;
        public bool alive = true;
        public List<int> links = new List<int>();
    }

    [Header("Target")]
    public Transform target;
    public float stoppingDistance = 0.5f;

    [Tooltip("Higher = switches to the next waypoint earlier = smoother, wider turns.")]
    public float lookAheadTime = 0.25f;

    [Header("Memory")]
    [Tooltip("How strongly the bot avoids revisited cells and prefers proven ones. 0 = no memory influence.")]
    public float memoryStrength = 1.5f;
    public float memoryCellSize = 1f;

    [Tooltip("How close the target (and the bot) must be to a remembered spot to reuse the known route.")]
    public float routeMatchRadius = 2f;

    [Header("Temporary Goals")]
    public float waypointDistance = 3f;
    public float waypointReachedDistance = 0.6f;

    [Header("Fast Decision Making")]
    public float replanInterval = 0.12f;
    public float waypointRecheckInterval = 0.2f;
    public float stuckRecoveryCooldown = 0.5f;

    [Header("Recovery")]
    public float nodeSpacing = 1.5f;
    public float stuckCheckInterval = 0.45f;
    public float minimumProgressDistance = 0.08f;

    [Header("Debug")]
    public bool showDebugLogs = false;
    public bool drawNavigationGizmos = true;

    // ---- Output read by BotMover ----
    public Vector3 Destination { get; private set; }
    public bool HasDestination { get; private set; }
    public bool IsFinalLeg => !hasWaypoint;

    private const float RoutePlanCooldown = 0.5f;
    private const int MaxRouteNodes = 1500;

    private static readonly float[] SearchAngles =
    {
        0f, 30f, -30f, 45f, -45f, 60f, -60f,
        90f, -90f, 120f, -120f, 150f, -150f, 180f
    };

    private readonly float[] searchDistances = new float[3];

    private readonly Dictionary<Vector2Int, CellMemory> memory =
        new Dictionary<Vector2Int, CellMemory>();

    private readonly List<RouteNode> routeNodes = new List<RouteNode>();
    private readonly List<int> knownRouteIds = new List<int>();
    private float[] planDist = new float[0];
    private int[] planPrev = new int[0];
    private bool[] planDone = new bool[0];
    private int lastRouteNode = -1;
    private int knownRouteIndex;
    private Vector3 routeGoalPosition;
    private float nextRoutePlanTime;

    private BotMover mover;

    private NavigationState state = NavigationState.DirectMove;

    private Vector3 currentWaypoint;
    private Vector3 lastNodePosition;
    private Vector3 lastProgressPosition;

    private readonly Stack<Vector3> pathHistory = new Stack<Vector3>();

    private Vector2Int lastCell;

    private bool hasWaypoint;
    private bool arrivalRecorded;

    private float stuckTimer;
    private float nextReplanTime;
    private float nextWaypointCheckTime;
    private float nextRecoveryTime;

    private void Awake()
    {
        mover = GetComponent<BotMover>();
    }

    private void Start()
    {
        if (target == null)
        {
            Log("Assign a target in the Inspector.");
            enabled = false;
            return;
        }

        lastNodePosition = transform.position;
        lastProgressPosition = transform.position;
        pathHistory.Push(transform.position);

        lastCell = new Vector2Int(int.MinValue, int.MinValue);

        SetMovementDestination(target.position);
    }

    private void Update()
    {
        if (target == null)
            return;

        UpdateNavigation();
        CheckForStuck();
        RecordPathHistory();
        RecordRouteNode();
        TrackVisits();
    }

    // =========================================================
    // SENSING (provided by the mover)
    // =========================================================

    private bool HasObstacleOnPath(Vector3 destination)
    {
        return mover.IsPathBlocked(destination);
    }

    private bool TryGetGroundPosition(Vector3 candidate, out Vector3 grounded)
    {
        return mover.TryGetGroundPosition(candidate, out grounded);
    }

    // =========================================================
    // ROUTE MAP (remembered walked paths)
    // =========================================================

    private void RecordRouteNode()
    {
        if (!mover.IsGrounded || mover.IsJumping)
            return;

        Vector3 pos = transform.position;

        if (IsDeadEnd(pos))
            return;

        if (lastRouteNode >= 0 &&
            routeNodes[lastRouteNode].alive &&
            HorizontalDistance(pos, routeNodes[lastRouteNode].pos) < nodeSpacing)
        {
            return;
        }

        int id = NearestRouteNode(pos, nodeSpacing * 0.6f);

        if (id < 0)
        {
            if (routeNodes.Count >= MaxRouteNodes)
                return;

            routeNodes.Add(new RouteNode { pos = pos });
            id = routeNodes.Count - 1;
        }

        if (lastRouteNode >= 0 &&
            lastRouteNode != id &&
            routeNodes[lastRouteNode].alive)
        {
            LinkNodes(lastRouteNode, id);
        }

        lastRouteNode = id;
    }

    private void LinkNodes(int a, int b)
    {
        if (!routeNodes[a].links.Contains(b))
            routeNodes[a].links.Add(b);

        if (!routeNodes[b].links.Contains(a))
            routeNodes[b].links.Add(a);
    }

    private int NearestRouteNode(Vector3 position, float radius)
    {
        int best = -1;
        float bestDistance = radius;

        for (int i = 0; i < routeNodes.Count; i++)
        {
            RouteNode node = routeNodes[i];

            if (!node.alive)
                continue;

            if (Mathf.Abs(node.pos.y - position.y) > mover.heightTolerance)
                continue;

            float d = HorizontalDistance(node.pos, position);

            if (d < bestDistance)
            {
                bestDistance = d;
                best = i;
            }
        }

        return best;
    }

    private int FindVisibleStartNode()
    {
        int best = -1;
        float bestDistance = routeMatchRadius;

        for (int i = 0; i < routeNodes.Count; i++)
        {
            RouteNode node = routeNodes[i];

            if (!node.alive)
                continue;

            if (Mathf.Abs(node.pos.y - transform.position.y) > mover.heightTolerance)
                continue;

            float d = HorizontalDistance(node.pos, transform.position);

            if (d >= bestDistance)
                continue;

            if (d > 0.3f && HasObstacleOnPath(node.pos))
                continue;

            bestDistance = d;
            best = i;
        }

        return best;
    }

    private void RemoveRouteNodesNear(Vector3 position, float radius)
    {
        for (int i = 0; i < routeNodes.Count; i++)
        {
            RouteNode node = routeNodes[i];

            if (!node.alive)
                continue;

            if (HorizontalDistance(node.pos, position) > radius)
                continue;

            node.alive = false;

            foreach (int other in node.links)
                routeNodes[other].links.Remove(i);

            node.links.Clear();

            if (lastRouteNode == i)
                lastRouteNode = -1;
        }
    }

    private bool TryFollowKnownRoute()
    {
        if (Time.time < nextRoutePlanTime)
            return false;

        if (!PlanKnownRoute())
        {
            nextRoutePlanTime = Time.time + RoutePlanCooldown;
            return false;
        }

        knownRouteIndex = 0;
        routeGoalPosition = target.position;

        currentWaypoint = routeNodes[knownRouteIds[0]].pos;
        hasWaypoint = true;
        state = NavigationState.KnownRoute;

        SetMovementDestination(currentWaypoint);
        nextWaypointCheckTime = Time.time + waypointRecheckInterval;

        Log("Following known route (" + knownRouteIds.Count + " nodes).");

        return true;
    }

    private bool PlanKnownRoute()
    {
        int goal = NearestRouteNode(target.position, routeMatchRadius);

        if (goal < 0)
            return false;

        int start = FindVisibleStartNode();

        if (start < 0)
            return false;

        int n = routeNodes.Count;

        if (planDist.Length < n)
        {
            planDist = new float[n];
            planPrev = new int[n];
            planDone = new bool[n];
        }

        for (int i = 0; i < n; i++)
        {
            planDist[i] = float.PositiveInfinity;
            planPrev[i] = -1;
            planDone[i] = false;
        }

        planDist[start] = 0f;

        while (true)
        {
            int u = -1;
            float best = float.PositiveInfinity;

            for (int i = 0; i < n; i++)
            {
                if (!planDone[i] && routeNodes[i].alive && planDist[i] < best)
                {
                    best = planDist[i];
                    u = i;
                }
            }

            if (u < 0 || u == goal)
                break;

            planDone[u] = true;

            foreach (int v in routeNodes[u].links)
            {
                if (!routeNodes[v].alive || planDone[v])
                    continue;

                float nd = planDist[u] +
                           Vector3.Distance(routeNodes[u].pos, routeNodes[v].pos);

                if (nd < planDist[v])
                {
                    planDist[v] = nd;
                    planPrev[v] = u;
                }
            }
        }

        if (float.IsPositiveInfinity(planDist[goal]))
            return false;

        knownRouteIds.Clear();

        for (int c = goal; c != -1; c = planPrev[c])
            knownRouteIds.Add(c);

        knownRouteIds.Reverse();

        return true;
    }

    private bool AdvanceKnownRoute()
    {
        knownRouteIndex++;

        if (knownRouteIndex >= knownRouteIds.Count)
            return false;

        currentWaypoint = routeNodes[knownRouteIds[knownRouteIndex]].pos;
        hasWaypoint = true;

        SetMovementDestination(currentWaypoint);

        return true;
    }

    // =========================================================
    // SPATIAL MEMORY
    // =========================================================

    private Vector2Int ToCell(Vector3 position)
    {
        float size = Mathf.Max(0.1f, memoryCellSize);

        return new Vector2Int(
            Mathf.FloorToInt(position.x / size),
            Mathf.FloorToInt(position.z / size)
        );
    }

    private CellMemory GetCell(Vector2Int cell, float y)
    {
        if (!memory.TryGetValue(cell, out CellMemory data))
        {
            data = new CellMemory { y = y };
            memory[cell] = data;
        }

        return data;
    }

    private void TrackVisits()
    {
        if (!mover.IsGrounded || mover.IsJumping)
            return;

        Vector2Int cell = ToCell(transform.position);

        if (cell == lastCell)
            return;

        lastCell = cell;
        GetCell(cell, transform.position.y).visits++;
    }

    private bool IsDeadEnd(Vector3 position)
    {
        return memory.TryGetValue(ToCell(position), out CellMemory data)
               && data.deadEnd > 0;
    }

    private void MarkDeadEnd(Vector3 position)
    {
        Vector2Int center = ToCell(position);

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                CellMemory data = GetCell(
                    new Vector2Int(center.x + dx, center.y + dz),
                    position.y
                );

                data.deadEnd++;
                data.proven = 0;
            }
        }

        RemoveRouteNodesNear(position, memoryCellSize * 1.5f);

        Log("Dead end remembered at " + position);
    }

    private void CommitProvenPath()
    {
        Vector3 previous = transform.position;

        foreach (Vector3 node in pathHistory)
        {
            MarkSegmentProven(previous, node);
            previous = node;
        }

        MarkSegmentProven(previous, previous);

        foreach (CellMemory data in memory.Values)
            data.visits = 0;

        Log("Route proven and stored.");
    }

    private void MarkSegmentProven(Vector3 from, Vector3 to)
    {
        float step = Mathf.Max(0.1f, memoryCellSize * 0.5f);
        float length = HorizontalDistance(from, to);
        int steps = Mathf.Max(1, Mathf.CeilToInt(length / step));

        for (int i = 0; i <= steps; i++)
        {
            Vector3 point = Vector3.Lerp(from, to, i / (float)steps);
            CellMemory data = GetCell(ToCell(point), point.y);

            data.proven++;
            data.deadEnd = 0;
        }
    }

    // =========================================================
    // NAVIGATION DECISIONS
    // =========================================================

    private void UpdateNavigation()
    {
        float targetDistance =
            HorizontalDistance(transform.position, target.position);

        if (targetDistance <= stoppingDistance)
        {
            if (!mover.IsJumping)
            {
                HasDestination = false;
                hasWaypoint = false;
                state = NavigationState.DirectMove;

                if (!arrivalRecorded)
                {
                    arrivalRecorded = true;
                    CommitProvenPath();
                }
            }

            return;
        }

        if (targetDistance > stoppingDistance * 2f)
            arrivalRecorded = false;

        CheckWaypointArrival();

        if (Time.time < nextReplanTime)
            return;

        nextReplanTime = Time.time + replanInterval;

        switch (state)
        {
            case NavigationState.DirectMove:
                HandleDirectMove();
                break;

            case NavigationState.MoveToWaypoint:
                HandleWaypointMove();
                break;

            case NavigationState.Backtrack:
                HandleBacktracking();
                break;

            case NavigationState.KnownRoute:
                HandleKnownRoute();
                break;
        }
    }

    private void CheckWaypointArrival()
    {
        if (!hasWaypoint)
            return;

        float reach = Mathf.Max(
            waypointReachedDistance,
            mover.HorizontalSpeed * lookAheadTime
        );

        if (HorizontalDistance(transform.position, currentWaypoint) > reach)
            return;

        hasWaypoint = false;

        if (state == NavigationState.KnownRoute && AdvanceKnownRoute())
            return;

        state = NavigationState.DirectMove;
        SetMovementDestination(target.position);

        HandleDirectMove();
        nextReplanTime = Time.time + replanInterval;
    }

    private void HandleDirectMove()
    {
        if (!HasObstacleOnPath(target.position))
        {
            hasWaypoint = false;
            SetMovementDestination(target.position);
            return;
        }

        if (TryFollowKnownRoute())
            return;

        if (TryFindWaypoint())
            return;

        if (StartBacktracking())
            return;

        Log("No available route yet.");
    }

    private void HandleKnownRoute()
    {
        if (!hasWaypoint)
        {
            state = NavigationState.DirectMove;
            return;
        }

        if (Time.time < nextWaypointCheckTime)
            return;

        nextWaypointCheckTime = Time.time + waypointRecheckInterval;

        if (HorizontalDistance(target.position, routeGoalPosition)
            > routeMatchRadius)
        {
            hasWaypoint = false;
            state = NavigationState.DirectMove;
            nextRoutePlanTime = 0f;
            HandleDirectMove();
            return;
        }

        if (!HasObstacleOnPath(target.position))
        {
            hasWaypoint = false;
            state = NavigationState.DirectMove;
            SetMovementDestination(target.position);
        }
    }

    private void HandleWaypointMove()
    {
        if (!hasWaypoint)
        {
            state = NavigationState.DirectMove;
            return;
        }

        if (Time.time < nextWaypointCheckTime)
            return;

        nextWaypointCheckTime = Time.time + waypointRecheckInterval;

        if (!HasObstacleOnPath(target.position))
        {
            hasWaypoint = false;
            state = NavigationState.DirectMove;
            SetMovementDestination(target.position);
            return;
        }

        if (TryFollowKnownRoute())
            return;

        if (!HasObstacleOnPath(currentWaypoint))
            return;

        MarkDeadEnd(currentWaypoint);
        hasWaypoint = false;

        if (TryFindWaypoint())
            return;

        StartBacktracking();
    }

    private void HandleBacktracking()
    {
        if (!hasWaypoint)
        {
            state = NavigationState.DirectMove;
            return;
        }

        if (!HasObstacleOnPath(currentWaypoint))
            return;

        MarkDeadEnd(currentWaypoint);
        hasWaypoint = false;

        if (TryFindWaypoint())
            return;

        StartBacktracking();
    }

    // =========================================================
    // WAYPOINT SEARCH (memory-aware)
    // =========================================================

    private bool TryFindWaypoint()
    {
        return TryFindWaypoint(false) || TryFindWaypoint(true);
    }

    private bool TryFindWaypoint(bool ignoreDeadEnds)
    {
        Vector3 toGoal = target.position - transform.position;
        toGoal.y = 0f;

        if (toGoal.sqrMagnitude < 0.001f)
            return false;

        Vector3 goalDirection = toGoal.normalized;

        searchDistances[0] = waypointDistance;
        searchDistances[1] = waypointDistance * 1.5f;
        searchDistances[2] = waypointDistance * 0.7f;

        bool found = false;
        Vector3 bestWaypoint = Vector3.zero;
        float bestScore = float.PositiveInfinity;

        foreach (float distance in searchDistances)
        {
            foreach (float angle in SearchAngles)
            {
                Vector3 direction =
                    Quaternion.Euler(0f, angle, 0f) * goalDirection;

                Vector3 candidate =
                    transform.position + direction * distance;

                if (!TryGetGroundPosition(candidate, out Vector3 grounded))
                    continue;

                if (Mathf.Abs(grounded.y - transform.position.y)
                    > mover.heightTolerance)
                    continue;

                if (!ignoreDeadEnds && IsDeadEnd(grounded))
                    continue;

                if (HasObstacleOnPath(grounded))
                    continue;

                float remaining = HorizontalDistance(grounded, target.position);
                float travel = HorizontalDistance(transform.position, grounded);

                float headingPenalty =
                    Vector3.Angle(transform.forward, direction) * 0.002f;

                float score =
                    remaining +
                    travel * 0.15f +
                    Mathf.Abs(angle) * 0.003f +
                    headingPenalty;

                if (memory.TryGetValue(ToCell(grounded), out CellMemory data))
                {
                    if (data.proven > 0)
                        score -= memoryStrength * 2f;
                    else
                        score += data.visits * memoryStrength;

                    if (ignoreDeadEnds && data.deadEnd > 0)
                        score += data.deadEnd * memoryStrength * 2f;
                }

                if (score < bestScore)
                {
                    bestScore = score;
                    bestWaypoint = grounded;
                    found = true;
                }
            }

            if (found)
                break;
        }

        if (!found)
            return false;

        currentWaypoint = bestWaypoint;
        hasWaypoint = true;
        state = NavigationState.MoveToWaypoint;

        SetMovementDestination(currentWaypoint);
        nextWaypointCheckTime = Time.time + waypointRecheckInterval;

        Log("New waypoint: " + currentWaypoint);

        return true;
    }

    // =========================================================
    // STUCK RECOVERY
    // =========================================================

    private void CheckForStuck()
    {
        if (!HasDestination || mover.IsJumping)
        {
            stuckTimer = 0f;
            lastProgressPosition = transform.position;
            return;
        }

        stuckTimer += Time.deltaTime;

        if (stuckTimer < stuckCheckInterval)
            return;

        float progress = HorizontalDistance(
            transform.position,
            lastProgressPosition
        );

        lastProgressPosition = transform.position;
        stuckTimer = 0f;

        if (progress >= minimumProgressDistance)
            return;

        if (Time.time < nextRecoveryTime)
            return;

        nextRecoveryTime = Time.time + stuckRecoveryCooldown;

        Log("Stuck detected; recovering.");

        bool wasKnownRoute = state == NavigationState.KnownRoute;

        if (hasWaypoint)
            MarkDeadEnd(currentWaypoint);

        hasWaypoint = false;

        if (wasKnownRoute)
        {
            nextRoutePlanTime = 0f;

            if (TryFollowKnownRoute())
                return;
        }

        if (TryFindWaypoint())
            return;

        if (StartBacktracking())
            return;

        state = NavigationState.DirectMove;
        nextReplanTime = 0f;
    }

    private bool StartBacktracking()
    {
        MarkDeadEnd(transform.position);

        while (pathHistory.Count > 1)
        {
            if (HorizontalDistance(
                pathHistory.Peek(),
                transform.position
            ) < waypointReachedDistance)
            {
                pathHistory.Pop();
                continue;
            }

            Vector3 previousNode = pathHistory.Pop();

            if (HorizontalDistance(
                previousNode,
                transform.position
            ) < waypointReachedDistance)
            {
                continue;
            }

            if (HasObstacleOnPath(previousNode))
            {
                MarkDeadEnd(previousNode);
                continue;
            }

            currentWaypoint = previousNode;
            hasWaypoint = true;
            state = NavigationState.Backtrack;

            SetMovementDestination(currentWaypoint);

            Log("Backtracking.");

            return true;
        }

        return false;
    }

    private void RecordPathHistory()
    {
        if (HorizontalDistance(
            transform.position,
            lastNodePosition
        ) < nodeSpacing)
        {
            return;
        }

        lastNodePosition = transform.position;

        if (pathHistory.Count == 0 ||
            HorizontalDistance(
                pathHistory.Peek(),
                transform.position
            ) > waypointReachedDistance)
        {
            pathHistory.Push(transform.position);
        }

        if (pathHistory.Count > 100)
        {
            Vector3[] recent = pathHistory.ToArray();
            pathHistory.Clear();

            for (int i = recent.Length - 1; i >= 1; i--)
                pathHistory.Push(recent[i]);
        }
    }

    // =========================================================
    // HELPERS
    // =========================================================

    private void SetMovementDestination(Vector3 destination)
    {
        Destination = destination;
        HasDestination = true;
    }

    private float HorizontalDistance(Vector3 a, Vector3 b)
    {
        a.y = 0f;
        b.y = 0f;

        return Vector3.Distance(a, b);
    }

    private void Log(string message)
    {
        if (showDebugLogs)
            Debug.Log("[PathPlanner] " + message, this);
    }

    // =========================================================
    // GIZMOS
    // =========================================================

    private void OnDrawGizmosSelected()
    {
        if (!drawNavigationGizmos)
            return;

        if (target != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(transform.position, target.position);
            Gizmos.DrawSphere(target.position, 0.2f);
        }

        if (Application.isPlaying && hasWaypoint)
        {
            Gizmos.color =
                state == NavigationState.Backtrack ? Color.red :
                state == NavigationState.KnownRoute ? Color.cyan :
                Color.magenta;

            Gizmos.DrawSphere(currentWaypoint, 0.2f);
            Gizmos.DrawLine(transform.position, currentWaypoint);
        }

        if (!Application.isPlaying)
            return;

        Gizmos.color = new Color(0.3f, 0.5f, 1f, 0.8f);

        for (int i = 0; i < routeNodes.Count; i++)
        {
            RouteNode node = routeNodes[i];

            if (!node.alive)
                continue;

            Gizmos.DrawSphere(node.pos, 0.1f);

            foreach (int other in node.links)
            {
                if (other > i)
                    Gizmos.DrawLine(node.pos, routeNodes[other].pos);
            }
        }

        float size = Mathf.Max(0.1f, memoryCellSize);
        Vector3 cube = new Vector3(size * 0.9f, 0.05f, size * 0.9f);

        foreach (KeyValuePair<Vector2Int, CellMemory> pair in memory)
        {
            CellMemory data = pair.Value;

            Vector3 center = new Vector3(
                (pair.Key.x + 0.5f) * size,
                data.y + 0.05f,
                (pair.Key.y + 0.5f) * size
            );

            if (data.deadEnd > 0)
                Gizmos.color = Color.red;
            else if (data.proven > 0)
                Gizmos.color = Color.green;
            else if (data.visits > 0)
                Gizmos.color = new Color(1f, 0.9f, 0.2f, 0.6f);
            else
                continue;

            Gizmos.DrawWireCube(center, cube);
        }
    }
}