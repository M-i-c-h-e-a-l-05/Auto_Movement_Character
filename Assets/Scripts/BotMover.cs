using UnityEngine;

[DefaultExecutionOrder(-10)]
[RequireComponent(typeof(Rigidbody), typeof(CapsuleCollider))]
public class BotMover : MonoBehaviour
{
    [Header("Run")]
    public float moveSpeed = 5f;
    public float rotateSpeed = 14f;
    public float acceleration = 18f;
    public float deceleration = 14f;
    [Range(0f, 1f)] public float airControl = 0.8f;
    [Range(0f, 1f)] public float minimumTurnSpeed = 0.35f;

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

    [Tooltip("Largest height change the bot treats as walkable ground.")]
    public float heightTolerance = 1.5f;

    [Header("Jump")]
    public float jumpForce = 7f;
    public float jumpForwardSpeed = 5f;
    public float jumpForwardDuration = 0.45f;
    public float jumpCooldown = 0.8f;
    public float jumpLandingTimeout = 3f;

    [Header("Debug")]
    public bool showDebugLogs = false;

    // ---- State read by PathPlanner ----
    public bool IsGrounded { get; private set; }
    public bool IsJumping { get; private set; }

    public float HorizontalSpeed =>
        new Vector2(rb.linearVelocity.x, rb.linearVelocity.z).magnitude;

    private readonly RaycastHit[] castBuffer = new RaycastHit[24];

    private Rigidbody rb;
    private CapsuleCollider col;
    private PathPlanner planner;

    private Vector3 jumpDirection;
    private float nextJumpAllowedTime;
    private float jumpStartedTime;
    private Collider currentJumpObstacle;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        col = GetComponent<CapsuleCollider>();
        planner = GetComponent<PathPlanner>();

        if (planner == null)
        {
            Debug.LogWarning("[BotMover] Add a PathPlanner to this object.");
            enabled = false;
            return;
        }

        rb.constraints |= RigidbodyConstraints.FreezeRotationX;
        rb.constraints |= RigidbodyConstraints.FreezeRotationZ;
    }

    private void Update()
    {
        IsGrounded = CheckGrounded();

        UpdateJumpState();

        if (planner.target == null)
            return;

        HandleAbilities();
    }

    // =========================================================
    // ABILITIES (add Slide / Climb here)
    // =========================================================

    private void HandleAbilities()
    {
        if (IsJumping)
            return;

        TryJumpOverObstacle();

        // Future abilities go here, for example:
        // TrySlide();
        // TryClimb();
    }

    // =========================================================
    // RUN
    // =========================================================

    private void FixedUpdate()
    {
        if (planner.target == null)
            return;

        if (!planner.HasDestination)
        {
            StopHorizontalMovement();
            return;
        }

        Vector3 direction = planner.Destination - transform.position;
        direction.y = 0f;

        float distance = direction.magnitude;

        if (distance < 0.01f)
        {
            if (planner.IsFinalLeg)
                StopHorizontalMovement();
            return;
        }

        // Only the final target stops the bot. Waypoints never do.
        if (planner.IsFinalLeg &&
            distance <= planner.stoppingDistance &&
            !IsJumping)
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

        if (IsJumping)
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

        float controlFactor = IsGrounded ? 1f : airControl;

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

    private void StopHorizontalMovement()
    {
        Vector3 velocity = rb.linearVelocity;
        velocity.x = 0f;
        velocity.z = 0f;
        rb.linearVelocity = velocity;
    }

    // =========================================================
    // JUMP
    // =========================================================

    private void UpdateJumpState()
    {
        if (!IsJumping)
            return;

        bool minimumAirTimePassed = Time.time - jumpStartedTime > 0.2f;

        if (IsGrounded && minimumAirTimePassed)
        {
            IsJumping = false;
            Log("Landed.");
        }
        else if (Time.time - jumpStartedTime > jumpLandingTimeout && IsGrounded)
        {
            IsJumping = false;
            Log("Jump state recovered.");
        }
    }

    private void TryJumpOverObstacle()
    {
        if (!IsGrounded || Time.time < nextJumpAllowedTime)
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
        if (!IsGrounded || IsJumping)
            return;

        jumpDirection = GetDesiredDirection();

        if (jumpDirection.sqrMagnitude < 0.001f)
            jumpDirection = transform.forward;

        jumpDirection.y = 0f;
        jumpDirection.Normalize();

        IsJumping = true;
        jumpStartedTime = Time.time;
        nextJumpAllowedTime = Time.time + jumpCooldown;

        Vector3 velocity = rb.linearVelocity;
        velocity.y = 0f;
        rb.linearVelocity = velocity;

        rb.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);

        Log("Jumping forward.");
    }

    private Vector3 GetDesiredDirection()
    {
        Vector3 destination = planner.HasDestination
            ? planner.Destination
            : planner.target.position;

        Vector3 direction = destination - transform.position;
        direction.y = 0f;

        return direction.normalized;
    }

    // =========================================================
    // SENSING (also used by PathPlanner)
    // =========================================================

    public bool IsPathBlocked(Vector3 destination)
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

        float castDistance = Mathf.Max(0f, distance - planner.stoppingDistance);

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

            if (Mathf.Abs(g.y - start.y) > heightTolerance)
                return false;
        }

        return true;
    }

    public bool TryGetGroundPosition(
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
        Transform t = planner.target;

        return t != null &&
               (other.transform == t || other.transform.IsChildOf(t));
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

    private void Log(string message)
    {
        if (showDebugLogs)
            Debug.Log("[BotMover] " + message, this);
    }
}