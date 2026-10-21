using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody), typeof(CapsuleCollider))]
public class Auto_movement : MonoBehaviour
{
    private enum NavigationState
    {
        DirectMove,
        MoveToWaypoint,
        Backtrack
    }

    private class CellMemory
    {
        public int visits;
        public int deadEnd;
        public int proven;
        public float y;
    }

    [Header("Target")]
    public Transform target;
    public float stoppingDistance = 0.5f;

    [Header("Movement")]
    public float moveSpeed = 5f;
    public float rotateSpeed = 14f;

    [Header("Smooth Movement")]
    public float acceleration = 18f;
    public float deceleration = 14f;
    [Range(0f, 1f)] public float airControl = 0.8f;
    [Range(0f, 1f)] public float minimumTurnSpeed = 0.35f;

    [Tooltip("Higher = switches to the next waypoint earlier = smoother, wider turns.")]
    public float lookAheadTime = 0.25f;

    [Header("Memory")]
    [Tooltip("How strongly the bot avoids revisited cells and prefers proven ones. 0 = no memory influence.")]
    public float memoryStrength = 1.5f;
    public float memoryCellSize = 1f;

    [Header("Obstacle Detection")]
    public float obstacleCheckDistance = 2.5f;
    public float maxJumpableHeight = 1.5f;
    public float capsuleCastPadding = 0.05f;
    public LayerMask obstacleMask = ~0;

    [Header("Ground Check")]
    public LayerMask groundLayer;
    public float groundCheckDistance = 0.25f;
    public float groundProbeHeight = 3f;
    public float maximumGroundProbeDistance = 10f;
    [Range(0f, 1f)] public float minimumGroundNormalY = 0.7f;
    public float groundSampleStep = 0.5f;

    [Header("Jumping")]
    public float jumpForce = 7f;
    public float jumpForwardSpeed = 5f;
    public float jumpForwardDuration = 0.45f;
    public float jumpCooldown = 0.8f;
    public float jumpLandingTimeout = 3f;

    [Header("Temporary Goals")]
    public float waypointDistance = 3f;
    public float waypointReachedDistance = 0.6f;
    public float waypointHeightTolerance = 1.5f;

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

    private static readonly float[] SearchAngles =
    {
        0f, 30f, -30f, 45f, -45f, 60f, -60f,
        90f, -90f, 120f, -120f, 150f, -150f, 180f
    };

    private readonly float[] searchDistances = new float[3];
    private readonly RaycastHit[] castBuffer = new RaycastHit[24];

    private readonly Dictionary<Vector2Int, CellMemory> memory =
        new Dictionary<Vector2Int, CellMemory>();

    private Rigidbody rb;
    private CapsuleCollider col;

    private NavigationState state = NavigationState.DirectMove;

    private Vector3 currentWaypoint;
    private Vector3 movementDestination;
    private Vector3 lastNodePosition;
    private Vector3 lastProgressPosition;
    private Vector3 jumpDirection;

    private readonly Stack<Vector3> pathHistory = new Stack<Vector3>();

    private Vector2Int lastCell;

    private bool hasWaypoint;
    private bool hasMovementDestination;
    private bool isGrounded;
    private bool isJumping;
    private bool arrivalRecorded;

    private float nextJumpAllowedTime;
    private float jumpStartedTime;
    private float stuckTimer;
    private float nextReplanTime;
    private float nextWaypointCheckTime;
    private float nextRecoveryTime;

    private Collider currentJumpObstacle;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        col = GetComponent<CapsuleCollider>();

        if (target == null)
        {
            Log("Assign a target in the Inspector.");
            enabled = false;
            return;
        }

        rb.constraints |= RigidbodyConstraints.FreezeRotationX;
        rb.constraints |= RigidbodyConstraints.FreezeRotationZ;

        lastNodePosition = transform.position;
        lastProgressPosition = transform.position;
        pathHistory.Push(transform.position);

        lastCell = new Vector2Int(int.MinValue, int.MinValue);

        isGrounded = CheckGrounded();

        SetMovementDestination(target.position);
    }

    private void Update()
    {
        if (target == null)
            return;

        isGrounded = CheckGrounded();

        UpdateJumpState();
        UpdateNavigation();

        if (!isJumping)
            CheckForJumpableObstacle();

        CheckForStuck();
        RecordPathHistory();
        TrackVisits();
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
        if (!isGrounded || isJumping)
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

    // Marks the cell and its neighbours as a failed area.
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

        Log("Dead end remembered at " + position);
    }

    // Called once on arrival: the history stack is the working route.
    private void CommitProvenPath()
    {
        Vector3 previous = transform.position;

        foreach (Vector3 node in pathHistory)
        {
            MarkSegmentProven(previous, node);
            previous = node;
        }

        MarkSegmentProven(previous, previous);

        // New trip: forget how often cells were visited, keep the rest.
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
    // JUMP STATE
    // =========================================================

    private void UpdateJumpState()
    {
        if (!isJumping)
            return;

        bool minimumAirTimePassed = Time.time - jumpStartedTime > 0.2f;

        if (isGrounded && minimumAirTimePassed)
        {
            isJumping = false;
            Log("Landed.");
        }
        else if (Time.time - jumpStartedTime > jumpLandingTimeout && isGrounded)
        {
            isJumping = false;
            Log("Jump state recovered.");
        }
    }

    // =========================================================
    // SMOOTH PHYSICS MOVEMENT
    // =========================================================

    private void FixedUpdate()
    {
        if (rb == null || target == null)
            return;

        if (!hasMovementDestination)
        {
            StopHorizontalMovement();
            return;
        }

        Vector3 direction = movementDestination - transform.position;
        direction.y = 0f;

        float distance = direction.magnitude;

        if (distance < 0.01f)
        {
            if (!hasWaypoint)
                StopHorizontalMovement();
            return;
        }

        // Only the final target stops the bot. Waypoints never do.
        if (!hasWaypoint && distance <= stoppingDistance && !isJumping)
        {
            StopHorizontalMovement();
            return;
        }

        direction /= distance;

        Quaternion desiredRotation = Quaternion.LookRotation(direction);

        float alignment = Mathf.Clamp01(
            (Vector3.Dot(transform.forward, direction) + 1f) * 0.5f
        );

        float turnFactor = Mathf.Lerp(minimumTurnSpeed, 1f, alignment);

        rb.MoveRotation(
            Quaternion.Slerp(
                rb.rotation,
                desiredRotation,
                rotateSpeed * turnFactor * Time.fixedDeltaTime
            )
        );

        float angle = Vector3.Angle(transform.forward, direction);

        float turnSpeedFactor = Mathf.Lerp(
            0.4f,
            1f,
            1f - Mathf.Clamp01(angle / 120f)
        );

        Vector3 desiredVelocity;

        if (isJumping)
        {
            desiredVelocity =
                Time.time - jumpStartedTime < jumpForwardDuration
                    ? jumpDirection * jumpForwardSpeed
                    : direction * moveSpeed;
        }
        else
        {
            desiredVelocity = direction * moveSpeed * turnSpeedFactor;
        }

        Vector3 currentHorizontalVelocity = new Vector3(
            rb.linearVelocity.x,
            0f,
            rb.linearVelocity.z
        );

        float rate =
            desiredVelocity.sqrMagnitude > currentHorizontalVelocity.sqrMagnitude
                ? acceleration
                : deceleration;

        float controlFactor = isGrounded ? 1f : airControl;

        Vector3 newVelocity = Vector3.MoveTowards(
            currentHorizontalVelocity,
            desiredVelocity,
            rate * controlFactor * Time.fixedDeltaTime
        );

        rb.linearVelocity = new Vector3(
            newVelocity.x,
            rb.linearVelocity.y,
            newVelocity.z
        );
    }

    // =========================================================
    // NAVIGATION DECISIONS
    // =========================================================

    private void UpdateNavigation()
    {
        if (target == null)
            return;

        float targetDistance =
            HorizontalDistance(transform.position, target.position);

        if (targetDistance <= stoppingDistance)
        {
            if (!isJumping)
            {
                hasMovementDestination = false;
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

        // Target moved away: allow a new arrival to be recorded.
        if (targetDistance > stoppingDistance * 2f)
            arrivalRecorded = false;

        // Runs every frame so the bot never waits at a waypoint.
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
        }
    }

    private void CheckWaypointArrival()
    {
        if (!hasWaypoint)
            return;

        float speed = new Vector2(
            rb.linearVelocity.x,
            rb.linearVelocity.z
        ).magnitude;

        float reach = Mathf.Max(
            waypointReachedDistance,
            speed * lookAheadTime
        );

        if (HorizontalDistance(transform.position, currentWaypoint) > reach)
            return;

        // Arrived: pick the next goal immediately, in this same frame.
        hasWaypoint = false;
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

        if (TryFindWaypoint())
            return;

        if (StartBacktracking())
            return;

        Log("No available route yet.");
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
        // First pass respects remembered dead ends. If everything is
        // a dead end, retry them rather than freezing.
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
                    > waypointHeightTolerance)
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

                // Memory: proven cells are preferred, revisits are penalized.
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

    private bool TryGetGroundPosition(
        Vector3 candidate,
        out Vector3 groundedPosition)
    {
        Vector3 origin = new Vector3(
            candidate.x,
            transform.position.y + groundProbeHeight,
            candidate.z
        );

        if (!Physics.Raycast(
            origin,
            Vector3.down,
            out RaycastHit hit,
            maximumGroundProbeDistance,
            groundLayer,
            QueryTriggerInteraction.Ignore))
        {
            groundedPosition = Vector3.zero;
            return false;
        }

        if (hit.normal.y < minimumGroundNormalY)
        {
            groundedPosition = Vector3.zero;
            return false;
        }

        float verticalOffset = 0f;

        if (Physics.Raycast(
            transform.position + Vector3.up * 0.1f,
            Vector3.down,
            out RaycastHit currentGround,
            groundCheckDistance + 0.2f,
            groundLayer,
            QueryTriggerInteraction.Ignore))
        {
            verticalOffset = transform.position.y - currentGround.point.y;
        }

        groundedPosition = new Vector3(
            candidate.x,
            hit.point.y + verticalOffset,
            candidate.z
        );

        return true;
    }

    // =========================================================
    // PATH BLOCKING
    // =========================================================

    private bool HasObstacleOnPath(Vector3 destination)
    {
        GetCapsulePoints(
            out Vector3 point1,
            out Vector3 point2,
            out float radius
        );

        Vector3 direction = destination - transform.position;
        direction.y = 0f;

        float distance = direction.magnitude;

        if (distance <= 0.1f)
            return false;

        direction /= distance;

        float castDistance = Mathf.Max(0f, distance - stoppingDistance);

        if (castDistance <= 0.05f)
            return false;

        int count = Physics.CapsuleCastNonAlloc(
            point1,
            point2,
            radius,
            direction,
            castBuffer,
            castDistance,
            obstacleMask,
            QueryTriggerInteraction.Ignore
        );

        for (int i = 0; i < count; i++)
        {
            Collider obstacle = castBuffer[i].collider;

            if (obstacle == null)
                continue;

            if (IsOwnCollider(obstacle) || IsTargetCollider(obstacle))
                continue;

            if (IsGroundLayer(obstacle.gameObject.layer) ||
                IsPassableLayer(obstacle.gameObject.layer))
                continue;

            return true;
        }

        return !HasGroundAlongPath(destination);
    }

    private bool HasGroundAlongPath(Vector3 destination)
    {
        Vector3 start = transform.position;

        Vector3 flat = destination - start;
        flat.y = 0f;

        float dist = flat.magnitude;

        if (dist < 0.1f)
            return true;

        Vector3 dir = flat / dist;

        for (float d = groundSampleStep; d <= dist; d += groundSampleStep)
        {
            if (!TryGetGroundPosition(start + dir * d, out Vector3 g))
                return false;

            if (Mathf.Abs(g.y - start.y) > waypointHeightTolerance)
                return false;
        }

        return true;
    }

    private void GetCapsulePoints(
        out Vector3 point1,
        out Vector3 point2,
        out float radius)
    {
        Vector3 center = transform.TransformPoint(col.center);
        Vector3 scale = transform.lossyScale;

        radius = col.radius * Mathf.Max(
            Mathf.Abs(scale.x),
            Mathf.Abs(scale.z)
        );

        float height = Mathf.Max(
            col.height * Mathf.Abs(scale.y),
            radius * 2f
        );

        radius = Mathf.Max(radius - capsuleCastPadding, 0.01f);

        float halfSegment = Mathf.Max(height * 0.5f - radius, 0f);

        point1 = center + transform.up * halfSegment;
        point2 = center - transform.up * halfSegment;
    }

    private bool IsOwnCollider(Collider other)
    {
        return other.transform == transform ||
               other.transform.IsChildOf(transform);
    }

    private bool IsTargetCollider(Collider other)
    {
        return other.transform == target ||
               other.transform.IsChildOf(target);
    }

    private bool IsGroundLayer(int layer)
    {
        return (groundLayer.value & (1 << layer)) != 0;
    }

    private bool IsPassableLayer(int layer)
    {
        int passableLayer = LayerMask.NameToLayer("passable");

        return passableLayer >= 0 && layer == passableLayer;
    }

    // =========================================================
    // JUMP DETECTION AND EXECUTION
    // =========================================================

    private void CheckForJumpableObstacle()
    {
        if (!isGrounded || isJumping || Time.time < nextJumpAllowedTime)
            return;

        Vector3 direction = GetDesiredDirection();

        if (direction.sqrMagnitude < 0.001f)
            return;

        GetCapsulePoints(
            out Vector3 point1,
            out Vector3 point2,
            out float radius
        );

        int count = Physics.CapsuleCastNonAlloc(
            point1,
            point2,
            radius,
            direction,
            castBuffer,
            obstacleCheckDistance,
            obstacleMask,
            QueryTriggerInteraction.Ignore
        );

        bool previousObstacleStillAhead = false;

        for (int i = 0; i < count; i++)
        {
            Collider obstacle = castBuffer[i].collider;

            if (obstacle == null ||
                IsOwnCollider(obstacle) ||
                IsTargetCollider(obstacle) ||
                IsGroundLayer(obstacle.gameObject.layer) ||
                IsPassableLayer(obstacle.gameObject.layer))
            {
                continue;
            }

            if (obstacle == currentJumpObstacle)
            {
                previousObstacleStillAhead = true;
                continue;
            }

            float groundY = transform.position.y;

            if (Physics.Raycast(
                transform.position + Vector3.up * 0.1f,
                Vector3.down,
                out RaycastHit groundHit,
                groundCheckDistance + 0.2f,
                groundLayer,
                QueryTriggerInteraction.Ignore))
            {
                groundY = groundHit.point.y;
            }

            float obstacleHeight = obstacle.bounds.max.y - groundY;

            if (obstacleHeight > 0f && obstacleHeight <= maxJumpableHeight)
            {
                currentJumpObstacle = obstacle;
                Jump();
                return;
            }
        }

        if (!previousObstacleStillAhead)
            currentJumpObstacle = null;
    }

    private void Jump()
    {
        if (!isGrounded || isJumping)
            return;

        jumpDirection = GetDesiredDirection();

        if (jumpDirection.sqrMagnitude < 0.001f)
            jumpDirection = transform.forward;

        jumpDirection.y = 0f;
        jumpDirection.Normalize();

        isJumping = true;
        jumpStartedTime = Time.time;
        nextJumpAllowedTime = Time.time + jumpCooldown;

        Vector3 velocity = rb.linearVelocity;
        velocity.y = 0f;
        rb.linearVelocity = velocity;

        rb.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);

        Log("Jumping forward.");
    }

    // =========================================================
    // GROUND DETECTION
    // =========================================================

    private bool CheckGrounded()
    {
        Vector3 center = transform.TransformPoint(col.center);

        float bottomOffset =
            col.height * Mathf.Abs(transform.lossyScale.y) * 0.5f;

        Vector3 bottom = center - transform.up * bottomOffset;
        Vector3 origin = bottom + Vector3.up * 0.1f;

        bool grounded = Physics.Raycast(
            origin,
            Vector3.down,
            out RaycastHit hit,
            groundCheckDistance + 0.1f,
            groundLayer,
            QueryTriggerInteraction.Ignore
        );

        if (grounded && hit.normal.y < minimumGroundNormalY)
            grounded = false;

        Debug.DrawRay(
            origin,
            Vector3.down * (groundCheckDistance + 0.1f),
            grounded ? Color.green : Color.red
        );

        return grounded;
    }

    private Vector3 GetDesiredDirection()
    {
        Vector3 destination = target.position;

        if ((state == NavigationState.MoveToWaypoint ||
             state == NavigationState.Backtrack) && hasWaypoint)
        {
            destination = currentWaypoint;
        }

        Vector3 direction = destination - transform.position;
        direction.y = 0f;

        return direction.normalized;
    }

    // =========================================================
    // STUCK RECOVERY
    // =========================================================

    private void CheckForStuck()
    {
        if (!hasMovementDestination || isJumping)
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

        if (hasWaypoint)
            MarkDeadEnd(currentWaypoint);

        hasWaypoint = false;

        if (TryFindWaypoint())
            return;

        if (StartBacktracking())
            return;

        state = NavigationState.DirectMove;
        nextReplanTime = 0f;
    }

    private bool StartBacktracking()
    {
        // The area we are retreating from did not work: remember it.
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
                // Unreachable node from here: part of the failed branch.
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
        movementDestination = destination;
        hasMovementDestination = true;
    }

    private void StopHorizontalMovement()
    {
        if (rb == null)
            return;

        Vector3 velocity = rb.linearVelocity;
        velocity.x = 0f;
        velocity.z = 0f;
        rb.linearVelocity = velocity;
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
            Debug.Log("[Auto_movement] " + message, this);
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
                state == NavigationState.Backtrack ? Color.red : Color.magenta;

            Gizmos.DrawSphere(currentWaypoint, 0.2f);
            Gizmos.DrawLine(transform.position, currentWaypoint);
        }

        if (!Application.isPlaying)
            return;

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