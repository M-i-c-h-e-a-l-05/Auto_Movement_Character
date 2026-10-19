using System.Collections.Generic;
using UnityEngine;

public class Auto_movement : MonoBehaviour
{
    [Header("Target")]
    public Transform target;

    [Header("Movement")]
    public float moveSpeed = 5f;
    public float rotateSpeed = 10f;
    public float jumpForce = 7f;
    public float stepDistance = 0.5f;

    [Header("Obstacle Rules")]
    public float obstacleCheckDistance = 2f;
    public float maxJumpableHeight = 1.5f;

    [Header("Ground Check")]
    public float groundCheckDistance = 0.2f;
    public LayerMask groundLayer;

    [Header("Navigation / Backtracking")]
    public float nodeSpacing = 1.5f;        
    public float alternateCheckDistance = 2f;
    public float alternateAngle = 45f;   
    public float angleTestRotateSpeed = 3f;  
    public float angleFacingTolerance = 5f;   


    private bool searchingAlternate = false;
    private Vector3 searchBlockedDir;
    private List<float> pendingAngles = new List<float>();
    private float currentTestAngle;
    private Vector3 currentTestDir;
    private Vector3 activeAlternateDir = Vector3.zero;
    private bool useAlternateDir = false;

    private class PathNode
    {
        public Vector3 position;
        public List<Vector3> triedDirections = new List<Vector3>();
    }

    private Stack<PathNode> pathHistory = new Stack<PathNode>();
    private Vector3 lastNodePos;
    private bool backtracking = false;
    private PathNode backtrackTargetNode;

    private Rigidbody rb;
    private CapsuleCollider col;

    private bool isGrounded = false;
    private bool isJumping = false;
    private bool blocked = false;
    private bool aligning = false;

    private Vector3 currentCenter;


    void Start()
    {
        rb = GetComponent<Rigidbody>();
        col = GetComponent<CapsuleCollider>();
        InitNavigation();
    }

    void Update()
    {
        isGrounded = CheckGrounded();

        CheckCollision();

        if (blocked) return;

        if (!isJumping)
        {
            // Debug.Log("Moving toward target.");
            AnalyzeForwardObstacle();
            HandleAlignment();
            MoveToTarget();
        }

        // Reset jump flag when landed
        if (isGrounded && isJumping)
        {
            isJumping = false;
            Debug.Log("Landed.");
        }
    }

    // =====================================================
    // GROUND CHECK
    // =====================================================

    // bool CheckGrounded()
    // {
    //     Vector3 bottom = transform.position + Vector3.down * (col.height / 2f);

    //     bool grounded = Physics.SphereCast(
    //         bottom + Vector3.up * 0.1f,
    //         col.radius * 0.9f,
    //         Vector3.down,
    //         out RaycastHit hit,
    //         groundCheckDistance,
    //         groundLayer
    //     );
    //     Debug.Log("Grounded: " + grounded);

    //     Debug.DrawRay(bottom, Vector3.down * groundCheckDistance, grounded ? Color.green : Color.red);
    //     return grounded;
    // }

    bool CheckGrounded()
    {
        Vector3 center = transform.TransformPoint(col.center);
        Vector3 bottom = center - transform.up * (col.height / 2f);
        Vector3 origin = bottom + Vector3.up * 0.1f;

        bool grounded = Physics.Raycast(origin, Vector3.down, out RaycastHit hit, groundCheckDistance + 0.1f, groundLayer);

        // if (grounded)
        //     Debug.Log("Hit: " + hit.collider.gameObject.name + " | Layer: " + LayerMask.LayerToName(hit.collider.gameObject.layer));
        // else
        //     Debug.Log("Hit nothing");

        Debug.DrawRay(origin, Vector3.down * (groundCheckDistance + 0.1f), grounded ? Color.green : Color.red);
        return grounded;
    }

    // =====================================================
    // CORE MOVEMENT
    // =====================================================

    void InitNavigation()
    {
        lastNodePos = transform.position;
        pathHistory.Push(new PathNode { position = transform.position });
    }

void MoveToTarget()
{
    if (target == null || aligning) return;

    if (backtracking)
    {
        HandleBacktrack();
        return;
    }

    if (searchingAlternate)
    {
        HandleAlternateSearch();
        return;
    }

    Vector3 desiredDir;

    if (useAlternateDir)
    {
        desiredDir = activeAlternateDir;

        // Once mostly facing/following this direction, hand control back to normal seeking
        if (Vector3.Angle(transform.forward, activeAlternateDir) < 5f)
        {
            useAlternateDir = false;
        }
    }
    else
    {
        Vector3 dir = (target.position - transform.position);
        dir.y = 0f;

        if (dir.magnitude < 0.2f) return;

        desiredDir = dir.normalized;
    }

    if (Vector3.Distance(transform.position, lastNodePos) >= nodeSpacing)
    {
        lastNodePos = transform.position;
        pathHistory.Push(new PathNode { position = transform.position });
    }

    if (isGrounded && !IsWalkable(transform.position + desiredDir * stepDistance))
    {
        Debug.Log("Path blocked — beginning angle search.");
        StartAlternateSearch(desiredDir);
        rb.linearVelocity = new Vector3(0f, rb.linearVelocity.y, 0f);
        return;
    }

    Quaternion lookRot = Quaternion.LookRotation(desiredDir);
    transform.rotation = Quaternion.Slerp(transform.rotation, lookRot, rotateSpeed * Time.deltaTime);

    float angleToTarget = Vector3.Angle(transform.forward, desiredDir);

    if (angleToTarget < 15f)
    {
        Vector3 velocity = transform.forward * moveSpeed;
        velocity.y = rb.linearVelocity.y;
        rb.linearVelocity = velocity;
    }
    else
    {
        rb.linearVelocity = new Vector3(0f, rb.linearVelocity.y, 0f);
    }
}

// =====================================================
// ALTERNATE DIRECTION SEARCH (stateful, spans frames)
// =====================================================

void StartAlternateSearch(Vector3 blockedDir)
    {
        searchBlockedDir = blockedDir;
        pendingAngles.Clear();

        // Build ordered list of angles to try: +45, -45, +90, -90, +135, -135
        for (float angle = alternateAngle; angle <= 135f; angle += alternateAngle)
        {
            pendingAngles.Add(angle);
            pendingAngles.Add(-angle);
        }

        searchingAlternate = true;
        PickNextTestAngle();
    }

void PickNextTestAngle()
    {
        if (pendingAngles.Count == 0)
        {
            Debug.Log("No alternate direction found — dead end.");
            searchingAlternate = false;
            StartBacktrack();
            return;
        }

        currentTestAngle = pendingAngles[0];
        pendingAngles.RemoveAt(0);
        currentTestDir = Quaternion.Euler(0, currentTestAngle, 0) * searchBlockedDir;

        Debug.Log("Testing angle: " + currentTestAngle);
    }

void HandleAlternateSearch()
    {
        Quaternion lookRot = Quaternion.LookRotation(currentTestDir);
        transform.rotation = Quaternion.Slerp(transform.rotation, lookRot, angleTestRotateSpeed * Time.deltaTime);

        rb.linearVelocity = new Vector3(0f, rb.linearVelocity.y, 0f);

        float angleRemaining = Vector3.Angle(transform.forward, currentTestDir);

        if (angleRemaining < angleFacingTolerance)
        {
            PathNode current = pathHistory.Peek();

            if (IsDirectionUsable(currentTestDir, current))
            {
                Debug.Log("Found walkable path at angle: " + currentTestAngle);
                current.triedDirections.Add(currentTestDir);

                activeAlternateDir = currentTestDir;
                useAlternateDir = true;
                searchingAlternate = false;
            }
            else
            {
                Debug.Log("Angle " + currentTestAngle + " not walkable, trying next.");
                PickNextTestAngle();
            }
        }
    }
bool IsDirectionUsable(Vector3 dir, PathNode node)
    {
        // Skip directions already tried and failed from this node
        foreach (Vector3 tried in node.triedDirections)
        {
            if (Vector3.Angle(tried, dir) < 10f) return false;
        }

        return IsWalkable(transform.position + dir * alternateCheckDistance);
    }

void StartBacktrack()
    {
        if (pathHistory.Count <= 1)
        {
            Debug.Log("No path left to backtrack to.");
            return;
        }

        pathHistory.Pop(); // discard current dead-end node
        backtrackTargetNode = pathHistory.Peek();
        backtracking = true;
    }

void HandleBacktrack()
    {
        Vector3 dir = backtrackTargetNode.position - transform.position;
        dir.y = 0f;

        if (dir.magnitude < 0.2f)
        {
            backtracking = false;
            Debug.Log("Reached backtrack node — resuming search.");
            return;
        }

        Vector3 moveDir = dir.normalized;
        Quaternion lookRot = Quaternion.LookRotation(moveDir);
        transform.rotation = Quaternion.Slerp(transform.rotation, lookRot, rotateSpeed * Time.deltaTime);

        Vector3 velocity = moveDir * moveSpeed;
        velocity.y = rb.linearVelocity.y;
        rb.linearVelocity = velocity;
    }
    // =====================================================
    // WALKABLE CHECKER
    // =====================================================

    bool IsWalkable(Vector3 checkPos)
    {
        Vector3 origin = new Vector3(checkPos.x, transform.position.y + 1f, checkPos.z);

        if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, 5f, groundLayer))
        {
            if (hit.normal.y > 0.7f)
            {
                Debug.DrawRay(origin, Vector3.down * 5f, Color.green);
                return true;
            }
        }

        Debug.DrawRay(origin, Vector3.down * 5f, Color.red);
        return false;
    }

    // =====================================================
    // SIMULATED COLLISION
    // =====================================================

    void CheckCollision()
    {
        Vector3 center = transform.TransformPoint(col.center);
        float radius = col.radius * transform.localScale.x;
        float halfHeight = Mathf.Max(col.height * 0.5f - col.radius, 0f);

        Vector3 point1 = center + transform.up * halfHeight;
        Vector3 point2 = center - transform.up * halfHeight;

        Vector3 moveDir = target != null
            ? (target.position - transform.position).normalized
            : transform.forward;

        moveDir.y = 0f;

        if (Physics.CapsuleCast(point1, point2, radius, moveDir, out RaycastHit hit, 0.15f))
        {
            if (hit.collider.gameObject != gameObject &&
                hit.collider.gameObject.layer != LayerMask.NameToLayer("passable"))
            {
                if (!blocked)
                {
                    blocked = true;
                    rb.linearVelocity = new Vector3(0, rb.linearVelocity.y, 0); // stop horizontal, keep gravity
                    Debug.Log("Blocked by: " + hit.collider.name);
                }
            }
        }
        else
        {
            if (blocked)
            {
                blocked = false;
                Debug.Log("Unblocked.");
            }
        }
    }

    // =====================================================
    // FORWARD SENSOR
    // =====================================================

    void AnalyzeForwardObstacle()
    {
        Vector3 center = transform.TransformPoint(col.center);
        float radius = col.radius * transform.localScale.x;
        float halfHeight = Mathf.Max(col.height * 0.5f - col.radius, 0f);

        Vector3 point1 = center + transform.up * halfHeight;
        Vector3 point2 = center - transform.up * halfHeight;

        Debug.DrawRay(center, transform.forward * obstacleCheckDistance, Color.cyan);

        if (Physics.CapsuleCast(point1, point2, radius, transform.forward,
            out RaycastHit hit, obstacleCheckDistance))
        {
            if (hit.collider.gameObject != gameObject) 
            {
                // Debug.Log("Obstacle detected: " + hit.collider.name);
                AnalyzeObstacle(hit);
            }
        }
    }

    // =====================================================
    // DECISION MAKING
    // =====================================================

    void AnalyzeObstacle(RaycastHit hit)
    {
        Debug.Log("Analyzing obstacle: " + hit.collider.name);
        Collider obstacle = hit.collider;
        float height = obstacle.bounds.size.y;

        Transform center = obstacle.transform.Find("Center");
        if (center != null && !aligning)
        {
            currentCenter = center.position;
            aligning = true;
        }

        // if (obstacle.CompareTag("Jump_type"))
        // {
        //     Jump();
        //     return;
        // }

        // if (obstacle.CompareTag("Slide_type"))
        // {
        //     // Slide logic here
        //     return;
        // }

        if (height <= maxJumpableHeight)
        {
            Debug.Log("Jumpable obstacle detected: " + obstacle.name);
            Jump();
        }
        else
        {
            Debug.Log("Obstacle too large: " + obstacle.name);
        }
    }

    // =====================================================
    // ALIGNMENT
    // =====================================================

    void HandleAlignment()
    {
        if (!aligning) return;

        Vector3 targetPos = new Vector3(currentCenter.x, transform.position.y, currentCenter.z);

        transform.position = Vector3.MoveTowards(
            transform.position,
            targetPos,
            moveSpeed * Time.deltaTime
        );

        float dist = Vector2.Distance(
            new Vector2(transform.position.x, transform.position.z),
            new Vector2(currentCenter.x, currentCenter.z)
        );

        if (dist < 0.05f)
        {
            aligning = false;
            Debug.Log("Aligned.");
        }
    }

    // =====================================================
    // JUMP
    // =====================================================

    void Jump()
    {
        if (isJumping || !isGrounded) return;

        Debug.Log("Preparing to jump.");
        Debug.Log("Preparing to jump.");
        isJumping = true;
        rb.linearVelocity = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z); // reset Y first
        rb.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);
        Debug.Log("Jump!");
    }

    // =====================================================
    // DEBUG
    // =====================================================

    void OnDrawGizmosSelected()
    {
        if (target == null) return;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, target.position);

        if (searchingAlternate)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawSphere(transform.position + currentTestDir * alternateCheckDistance, 0.15f);
        }
    }
}