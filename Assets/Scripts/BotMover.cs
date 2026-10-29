using UnityEngine;



[DefaultExecutionOrder(-10)]

[RequireComponent(typeof(Rigidbody), typeof(CapsuleCollider))]

public class BotMover : MonoBehaviour

{

    [Header("Movement Mode")]

    [Tooltip("Used automatically while the planner is exploring or discovering a route.")]
    public float walkSpeed = 2.5f;

    [Tooltip("Used automatically while following a route the planner has already learned.")]
    public float runSpeed = 5f;

    public float crouchSpeed = 1.5f;

    public float slideSpeed = 6f;

    public float moveSpeed = 5f;

    public float rotateSpeed = 14f;

    public float acceleration = 18f;

    public float deceleration = 14f;

    [Range(0f, 1f)] public float airControl = 0.8f;

    [Range(0f, 1f)] public float minimumTurnSpeed = 0.35f;



    [Header("Obstacle Detection")]

    public float obstacleCheckDistance = 2.5f;

    public float capsuleCastPadding = 0.05f;

    public LayerMask obstacleMask = ~0;



    [Header("Obstacle Types (by tag)")]

    public string jumpTag = "Jump_type";

    public string slideTag = "Slide_type";



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



    [Header("Crouch and Slide")]

    [Range(0.2f, 0.9f)] public float crouchHeightRatio = 0.42f;

    [Range(0.2f, 1f)] public float crouchRadiusRatio = 0.45f;

    public float slideStartDistance = 1.2f;

    [Tooltip("Slide movement lasts this long and follows the direction the bot is facing when it starts.")]
    public float slideDuration = 1.0f;

    [Tooltip("Child transform name used as the alignment point inside a Slide_type obstacle.")]
    public string slideCenterChildName = "center";

    [Tooltip("How close the bot must get to the slide center before it crouches or slides.")]
    public float slideCenterReachDistance = 0.35f;

    [Tooltip("Extra distance beyond the obstacle bounds before a crouching bot may stand up.")]
    public float slideExitPadding = 0.35f;



    [Header("Debug")]

    public bool showDebugLogs = false;



    // ---- State read by PathPlanner ----

    public bool IsGrounded { get; private set; }

    public bool IsJumping { get; private set; }

    public bool IsSliding { get; private set; }

    public bool IsCrouching { get; private set; }

    public bool IsRunning => planner != null && planner.IsFollowingKnownRoute && !IsCrouching && !IsSliding;

    // PathPlanner pauses during slide, but remains active while crouching.
    public bool IsBusy => IsJumping || IsSliding || isApproachingSlide;



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

    [Header("Landing Confirmation")]
    [Tooltip("Minimum time after takeoff before landing can be detected.")]
    public float minimumJumpAirTime = 0.2f;

    [Tooltip("How long grounded detection must remain true to confirm landing.")]
    public float landingConfirmationTime = 0.12f;

    private float groundedSince = -1f;



    private float slideEndTime;

    private float originalHeight;

    private float originalRadius;

    private Vector3 originalCenter;

    private bool colliderReduced;

    private bool isApproachingSlide;
    private Transform slideCenterTarget;
    private Collider slideObstacle;
    private Vector3 slideDirection;
    private bool slideHasPassedObstacle;



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

    // ABILITIES (add Climb here)

    // =========================================================



    private void HandleAbilities()

    {

        UpdateSlide();

        if (IsJumping || IsSliding || isApproachingSlide || IsCrouching)
            return;



        TryUseObstacleAbility();

    }



    private void TryUseObstacleAbility()

    {

        if (!IsGrounded || Time.time < nextJumpAllowedTime)

            return;



        Vector3 direction = GetDesiredDirection();



        if (direction.sqrMagnitude < 0.001f)

            return;



        GetCapsulePoints(out Vector3 point1, out Vector3 point2, out float radius);



        int count = Physics.CapsuleCastNonAlloc(

            point1, point2, radius, direction,

            castBuffer, obstacleCheckDistance,

            obstacleMask, QueryTriggerInteraction.Collide

        );



        bool previousObstacleStillAhead = false;



        for (int i = 0; i < count; i++)

        {

            Collider obstacle = castBuffer[i].collider;



            if (obstacle == null || IsOwnCollider(obstacle) || IsTargetCollider(obstacle))

                continue;



            if (obstacle.CompareTag(jumpTag))

            {

                if (obstacle == currentJumpObstacle)

                {

                    previousObstacleStillAhead = true;

                    continue;

                }



                currentJumpObstacle = obstacle;

                Jump();

                return;

            }



            if (obstacle.CompareTag(slideTag) &&
                castBuffer[i].distance <= slideStartDistance)
            {
                BeginSlideApproach(obstacle);
                return;
            }

        }



        if (!previousObstacleStillAhead)

            currentJumpObstacle = null;

    }



    // =========================================================

    // RUN

    // =========================================================



    private void FixedUpdate()
    {
        if (planner == null || planner.target == null)
            return;

        // During a slide, ignore planner destinations completely. Keep the captured
        // facing direction and travel at slideSpeed for the configured duration.
        if (IsSliding)
        {
            ApplyHorizontalVelocity(slideDirection.normalized * slideSpeed, true);
            return;
        }

        if (!planner.HasDestination)
        {
            StopHorizontalMovement();
            return;
        }

        Vector3 movementDestination = planner.Destination;
        if (isApproachingSlide)
            movementDestination = slideCenterTarget != null
                ? slideCenterTarget.position
                : slideCenterFallback;

        Vector3 direction = movementDestination - transform.position;
        direction.y = 0f;
        float distance = direction.magnitude;

        if (isApproachingSlide && distance <= slideCenterReachDistance)
        {
            if (IsRunning)
                StartSlideAtCenter();
            else
                StartCrouchAtCenter();

            if (IsSliding)
            {
                ApplyHorizontalVelocity(slideDirection.normalized * slideSpeed, true);
                return;
            }

            // Crouching keeps the planner on. Re-evaluate its destination below.
            if (planner.HasDestination)
            {
                direction = planner.Destination - transform.position;
                direction.y = 0f;
                distance = direction.magnitude;
            }
        }

        if (distance < 0.01f)
        {
            if (planner.IsFinalLeg && !IsCrouching && !isApproachingSlide)
                StopHorizontalMovement();
            return;
        }

        if (planner.IsFinalLeg && distance <= planner.stoppingDistance &&
            !IsJumping && !IsCrouching && !isApproachingSlide)
        {
            StopHorizontalMovement();
            return;
        }

        direction.Normalize();
        Quaternion desiredRotation = Quaternion.LookRotation(direction);
        float alignment = Mathf.Clamp01((Vector3.Dot(transform.forward, direction) + 1f) * 0.5f);
        float turnFactor = Mathf.Lerp(minimumTurnSpeed, 1f, alignment);

        rb.MoveRotation(Quaternion.Slerp(
            rb.rotation,
            desiredRotation,
            rotateSpeed * turnFactor * Time.fixedDeltaTime));

        float angle = Vector3.Angle(transform.forward, direction);
        float turnSpeedFactor = Mathf.Lerp(0.4f, 1f, 1f - Mathf.Clamp01(angle / 120f));

        Vector3 desiredVelocity;
        if (IsJumping)
            desiredVelocity = jumpDirection * jumpForwardSpeed;
        else if (IsCrouching)
            desiredVelocity = direction * crouchSpeed * turnSpeedFactor;
        else
            desiredVelocity = direction * (IsRunning ? runSpeed : walkSpeed) * turnSpeedFactor;

        ApplyHorizontalVelocity(desiredVelocity, false);
    }

    private void ApplyHorizontalVelocity(Vector3 desiredVelocity, bool ignoreGroundControl)
    {
        Vector3 currentHorizontalVelocity = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
        float rate = desiredVelocity.sqrMagnitude > currentHorizontalVelocity.sqrMagnitude
            ? acceleration : deceleration;
        float controlFactor = (!ignoreGroundControl && !IsGrounded) ? airControl : 1f;
        Vector3 newVelocity = Vector3.MoveTowards(
            currentHorizontalVelocity, desiredVelocity,
            rate * controlFactor * Time.fixedDeltaTime);
        rb.linearVelocity = new Vector3(newVelocity.x, rb.linearVelocity.y, newVelocity.z);
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

        float elapsed = Time.time - jumpStartedTime;

        // Prevent the ground check from ending the jump on the takeoff frame.
        if (elapsed < minimumJumpAirTime)
        {
            groundedSince = -1f;
            return;
        }

        // Require continuous grounded detection to avoid false landing reports.
        if (!IsGrounded)
        {
            groundedSince = -1f;
            return;
        }

        if (groundedSince < 0f)
            groundedSince = Time.time;

        if (Time.time - groundedSince < landingConfirmationTime)
            return;

        IsJumping = false;
        groundedSince = -1f;

        // Remove downward velocity after landing without affecting horizontal motion.
        Vector3 velocity = rb.linearVelocity;
        if (velocity.y < 0f)
            velocity.y = 0f;
        rb.linearVelocity = velocity;

        Log("Landing confirmed at " + transform.position);
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



    // =========================================================

    // SLIDE

    // =========================================================



    private void BeginSlideApproach(Collider obstacle)
    {
        if (isApproachingSlide || IsSliding || obstacle == null)
            return;

        slideObstacle = obstacle;
        slideCenterTarget = FindSlideCenter(obstacle.transform);

        // Shrink before alignment, not after reaching the center. This prevents
        // the full-height capsule from colliding with the obstacle on the way in.
        SaveAndReduceCollider();

        // If the child is missing, use the obstacle's center as a safe fallback.
        if (slideCenterTarget == null)
        {
            slideDirection = GetDesiredDirection();
            slideCenterFallback = obstacle.bounds.center;
        }
        else
        {
            slideCenterFallback = slideCenterTarget.position;
        }

        Vector3 toCenter = slideCenterFallback - transform.position;
        toCenter.y = 0f;

        if (toCenter.sqrMagnitude < 0.001f)
            toCenter = GetDesiredDirection();

        // Capture the direction through the obstacle, so the bot does not stop
        // or turn back toward the center once it begins sliding.
        slideDirection = GetDesiredDirection();
        if (slideDirection.sqrMagnitude < 0.001f)
            slideDirection = transform.forward;
        slideDirection.y = 0f;
        slideDirection.Normalize();

        isApproachingSlide = true;
        Log("Aligning with slide center: " + (slideCenterTarget != null ? slideCenterTarget.name : "obstacle bounds center"));
    }

    private Vector3 slideCenterFallback;

    private Transform FindSlideCenter(Transform root)
    {
        if (root == null)
            return null;

        string expected = string.IsNullOrWhiteSpace(slideCenterChildName)
            ? "center"
            : slideCenterChildName.Trim();

        Transform[] children = root.GetComponentsInChildren<Transform>(true);
        foreach (Transform child in children)
        {
            if (child == root)
                continue;

            if (string.Equals(child.name, expected, System.StringComparison.OrdinalIgnoreCase))
                return child;
        }

        return null;
    }

    private void SaveAndReduceCollider()
    {
        if (colliderReduced)
            return;

        originalHeight = col.height;
        originalRadius = col.radius;
        originalCenter = col.center;

        float reducedRadius = Mathf.Max(0.05f, originalRadius * crouchRadiusRatio);
        float reducedHeight = Mathf.Max(reducedRadius * 2f, originalHeight * crouchHeightRatio);
        col.radius = reducedRadius;
        col.height = Mathf.Min(originalHeight, reducedHeight);
        col.center = new Vector3(
            originalCenter.x,
            originalCenter.y - (originalHeight - col.height) * 0.5f,
            originalCenter.z);

        colliderReduced = true;
        Physics.SyncTransforms();
        Log("Collider reduced: height " + col.height + ", radius " + col.radius);
    }

    private void StartSlideAtCenter()
    {
        if (!isApproachingSlide || IsSliding)
            return;

        isApproachingSlide = false;
        slideCenterTarget = null;

        // Keep the forward route direction captured before alignment. The bot must
        // not turn toward a waypoint while its timed slide is active.
        slideDirection.y = 0f;
        if (slideDirection.sqrMagnitude < 0.001f)
            slideDirection = transform.forward;
        slideDirection.Normalize();
        SaveAndReduceCollider();
        Physics.SyncTransforms();
        IsCrouching = false;
        IsSliding = true;
        slideEndTime = Time.time + Mathf.Max(0.1f, slideDuration);
        Log("Sliding forward for " + slideDuration + " seconds.");
    }

    private void StartCrouchAtCenter()
    {
        if (!isApproachingSlide || IsSliding)
            return;

        isApproachingSlide = false;
        slideCenterTarget = null;
        SaveAndReduceCollider();
        IsCrouching = true;
        slideHasPassedObstacle = false;
        Log("Crouching; pathfinding remains enabled.");
    }

    private void UpdateSlide()
    {
        if (IsSliding && Time.time >= slideEndTime)
        {
            RestoreCollider();
            IsSliding = false;
            slideObstacle = null;
            Log("Timed slide finished; pathfinding resumes.");
        }

        if (!IsCrouching || slideObstacle == null)
            return;

        // Crouch remains active while the obstacle is overhead or the bot has not
        // passed the far side. Unlike sliding, the planner keeps updating the route.
        Vector3 flatDirection = GetDesiredDirection();
        if (flatDirection.sqrMagnitude < 0.001f)
            flatDirection = transform.forward;
        flatDirection.y = 0f;
        flatDirection.Normalize();

        Bounds bounds = slideObstacle.bounds;
        Vector3 ext = bounds.extents;
        float projectedExtent = Mathf.Abs(flatDirection.x) * ext.x +
                                Mathf.Abs(flatDirection.z) * ext.z;
        float passedDistance = Vector3.Dot(transform.position - bounds.center, flatDirection);
        float botRadius = Mathf.Max(col.radius * Mathf.Abs(transform.lossyScale.x), 0.1f);

        if (passedDistance <= projectedExtent + botRadius + slideExitPadding)
            return;

        if (HasOverheadClearance())
        {
            RestoreCollider();
            IsCrouching = false;
            slideObstacle = null;
            slideHasPassedObstacle = false;
            Log("Passed crouch obstacle; stood up.");
        }
    }

    private bool HasOverheadClearance()
    {
        float fullHeight = colliderReduced ? originalHeight : col.height;
        Vector3 rayOrigin = transform.TransformPoint(col.center);
        float headroom = fullHeight * Mathf.Abs(transform.lossyScale.y);

        return !Physics.Raycast(rayOrigin, Vector3.up, out RaycastHit hit,
            headroom, obstacleMask, QueryTriggerInteraction.Collide) || IsOwnCollider(hit.collider);
    }

    private void RestoreCollider()
    {
        if (!colliderReduced)
            return;

        col.height = originalHeight;
        col.radius = originalRadius;
        col.center = originalCenter;
        colliderReduced = false;
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

            QueryTriggerInteraction.Collide

        );



        for (int i = 0; i < count; i++)

        {

            Collider obstacle = castBuffer[i].collider;



            if (obstacle == null)

                continue;



            if (IsOwnCollider(obstacle) || IsTargetCollider(obstacle))

                continue;



            // Jump/Slide obstacles can be passed, so they don't block the path.

            if (obstacle.CompareTag(jumpTag) || obstacle.CompareTag(slideTag))

            {

                Log("Passable " + obstacle.tag + " obstacle ignored in path check: " + obstacle.name);

                continue;

            }

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



    private void OnDisable()
    {
        if (col != null && colliderReduced)
            RestoreCollider();
    }

    private void Log(string message)

    {

        if (showDebugLogs)

            Debug.Log("[BotMover] " + message, this);

    }

}