using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Goal-free frontier exploration controller for Bot2.
/// Attach alongside BotMover, PathPlanner, and WorldMapBuilder.
/// It creates its own temporary target for PathPlanner, then repeatedly selects
/// observed walkable cells next to unobserved cells. It never uses Bot1's goal.
/// </summary>
[DisallowMultipleComponent]
[DefaultExecutionOrder(-100)]
public class WorldMapExplorer : MonoBehaviour
{
    [Header("Required components")]
    public WorldMapBuilder mapBuilder;
    public BotMover mover;
    public PathPlanner planner;

    [Header("Frontier exploration")]
    [Min(0.25f)] public float chooseTargetEvery = 1.0f;
    [Min(0.1f)] public float targetReachedDistance = 1.0f;
    [Min(0.1f)] public float targetStuckDistance = 0.25f;
    [Min(1f)] public float targetTimeout = 12f;
    [Min(0.1f)] public float minimumTargetSeparation = 1.0f;
    [Min(0f)] public float distanceWeight = 1f;
    [Min(0f)] public float frontierWeight = 2f;
    [Min(0f)] public float revisitPenalty = 4f;
    public bool saveMapWhenExplorationStops = true;
    public bool drawDebugGizmos = true;
    public bool logExploration = true;

    [Header("Exploration state")]
    [SerializeField] private Vector3 currentExplorationTarget;
    [SerializeField] private int targetsVisited;
    [SerializeField] private int targetsAbandoned;
    [SerializeField] private bool explorationComplete;

    private Transform targetTransform;
    private readonly Dictionary<Vector3Int, float> failedFrontiers = new Dictionary<Vector3Int, float>();
    private readonly HashSet<Vector3Int> visitedFrontiers = new HashSet<Vector3Int>();
    private float nextTargetSelectionTime;
    private float targetStartedTime;
    private float bestDistanceToTarget = float.PositiveInfinity;
    private Vector3 lastProgressPosition;
    private float noProgressTimer;
    private bool hasTarget;
    private bool stopped;

    public bool HasExplorationTarget => hasTarget;
    public Vector3 ExplorationTarget => currentExplorationTarget;
    public int TargetsVisited => targetsVisited;
    public int TargetsAbandoned => targetsAbandoned;
    public bool ExplorationComplete => explorationComplete;

    private void Awake()
    {
        if (mover == null) mover = GetComponent<BotMover>();
        if (planner == null) planner = GetComponent<PathPlanner>();
        if (mapBuilder == null) mapBuilder = GetComponent<WorldMapBuilder>();

        // PathPlanner disables itself in Start if target is null, so create and assign
        // this target in Awake before any Start methods run.
        GameObject targetObject = new GameObject("Bot2_ExplorationTarget");
        targetObject.hideFlags = HideFlags.HideInHierarchy;
        targetTransform = targetObject.transform;
        targetTransform.position = transform.position;

        if (planner != null)
        {
            planner.target = targetTransform;
        }
        else
        {
            Debug.LogError("[WorldMapExplorer] Bot2 requires a PathPlanner because BotMover uses it for movement steering.", this);
            enabled = false;
            return;
        }

        if (mover == null)
        {
            Debug.LogError("[WorldMapExplorer] Bot2 requires BotMover.", this);
            enabled = false;
            return;
        }

        if (mapBuilder == null)
        {
            Debug.LogError("[WorldMapExplorer] Bot2 requires WorldMapBuilder.", this);
            enabled = false;
            return;
        }

        lastProgressPosition = transform.position;
    }

    private void Start()
    {
        SelectNewFrontier();
    }

    private void Update()
    {
        if (stopped || !enabled || mapBuilder == null || mapBuilder.Map == null)
            return;

        if (Time.time >= nextTargetSelectionTime)
        {
            nextTargetSelectionTime = Time.time + chooseTargetEvery;
            EvaluateCurrentTarget();
        }

        if (!hasTarget && !explorationComplete)
            SelectNewFrontier();
    }

    private void EvaluateCurrentTarget()
    {
        if (!hasTarget)
            return;

        float distance = HorizontalDistance(transform.position, currentExplorationTarget);
        if (distance <= targetReachedDistance && mover.IsGrounded && !mover.IsBusy)
        {
            Vector3Int key = PositionKey(currentExplorationTarget);
            visitedFrontiers.Add(key);
            targetsVisited++;
            hasTarget = false;
            noProgressTimer = 0f;
            if (logExploration)
                Debug.Log($"[WorldMapExplorer] Frontier reached. Visited: {targetsVisited}; abandoned: {targetsAbandoned}.", this);
            SelectNewFrontier();
            return;
        }

        if (mover.IsBusy || !mover.IsGrounded)
            return;

        if (distance < bestDistanceToTarget - targetStuckDistance)
        {
            bestDistanceToTarget = distance;
            lastProgressPosition = transform.position;
            noProgressTimer = 0f;
        }
        else
        {
            noProgressTimer += chooseTargetEvery;
        }

        bool timedOut = Time.time - targetStartedTime >= targetTimeout;
        bool stuck = noProgressTimer >= targetTimeout * 0.5f;
        if (timedOut || stuck)
        {
            Vector3Int key = PositionKey(currentExplorationTarget);
            failedFrontiers[key] = Time.time;
            targetsAbandoned++;
            hasTarget = false;
            noProgressTimer = 0f;
            if (logExploration)
                Debug.LogWarning($"[WorldMapExplorer] Frontier abandoned at {currentExplorationTarget} (timeout/stuck).", this);
            SelectNewFrontier();
        }
    }

    /// <summary>Selects a known-free cell adjacent to at least one unknown cell.</summary>
    public bool SelectNewFrontier()
    {
        if (mapBuilder == null || mapBuilder.Map == null || mapBuilder.Map.cells == null)
            return false;

        List<WorldMapBuilder.SpaceCell> cells = mapBuilder.Map.cells;
        Dictionary<Vector3Int, WorldMapBuilder.SpaceCell> byIndex = new Dictionary<Vector3Int, WorldMapBuilder.SpaceCell>();
        for (int i = 0; i < cells.Count; i++)
        {
            WorldMapBuilder.SpaceCell cell = cells[i];
            if (cell != null)
                byIndex[cell.gridIndex] = cell;
        }

        float bestScore = float.PositiveInfinity;
        WorldMapBuilder.SpaceCell bestCell = null;
        Vector3 botPosition = transform.position;
        float cellSize = Mathf.Max(0.1f, mapBuilder.cellSize);

        for (int i = 0; i < cells.Count; i++)
        {
            WorldMapBuilder.SpaceCell cell = cells[i];
            if (cell == null || cell.state != WorldMapBuilder.SpaceState.Free || !cell.hasGround)
                continue;

            Vector3 candidate = cell.groundPoint;
            if (candidate == Vector3.zero)
                candidate = cell.center;

            // Don't target the cell Bot2 is already standing on.
            if (HorizontalDistance(botPosition, candidate) < minimumTargetSeparation)
                continue;

            Vector3Int key = PositionKey(candidate);
            if (visitedFrontiers.Contains(key))
                continue;

            if (failedFrontiers.TryGetValue(key, out float failedAt))
            {
                if (Time.time - failedAt < targetTimeout * 2f)
                    continue;
                failedFrontiers.Remove(key);
            }

            int unknownNeighbours = CountUnknownNeighbours(cell.gridIndex, byIndex);
            if (unknownNeighbours == 0)
                continue;

            float distance = HorizontalDistance(botPosition, candidate);
            float score = distance * distanceWeight - unknownNeighbours * frontierWeight;
            // Mildly discourage choosing a cell close to a recently failed frontier.
            foreach (KeyValuePair<Vector3Int, float> failed in failedFrontiers)
            {
                if (Time.time - failed.Value < targetTimeout * 2f && GridKeyDistance(cell.gridIndex, failed.Key) <= 2)
                    score += revisitPenalty;
            }

            if (score < bestScore)
            {
                bestScore = score;
                bestCell = cell;
            }
        }

        if (bestCell == null)
        {
            hasTarget = false;
            explorationComplete = true;
            if (saveMapWhenExplorationStops)
                mapBuilder.SaveMap();
            if (logExploration)
                Debug.Log("[WorldMapExplorer] No unvisited frontiers remain in the observed map. Exploration stopped; this does not prove the entire level was mapped.", this);
            return false;
        }

        Vector3 targetPosition = bestCell.hasGround ? bestCell.groundPoint : bestCell.center;
        targetPosition.y = bestCell.hasGround ? bestCell.groundPoint.y : bestCell.center.y;
        currentExplorationTarget = targetPosition;
        targetTransform.position = targetPosition;
        hasTarget = true;
        explorationComplete = false;
        targetStartedTime = Time.time;
        bestDistanceToTarget = HorizontalDistance(transform.position, targetPosition);
        lastProgressPosition = transform.position;
        noProgressTimer = 0f;
        nextTargetSelectionTime = Time.time + chooseTargetEvery;

        if (logExploration)
            Debug.Log($"[WorldMapExplorer] New frontier target: {targetPosition}; unknown neighbours: {CountUnknownNeighbours(bestCell.gridIndex, byIndex)}.", this);
        return true;
    }

    private int CountUnknownNeighbours(Vector3Int index, Dictionary<Vector3Int, WorldMapBuilder.SpaceCell> cells)
    {
        int count = 0;
        for (int x = -1; x <= 1; x++)
        {
            for (int z = -1; z <= 1; z++)
            {
                if (x == 0 && z == 0) continue;
                Vector3Int neighbour = new Vector3Int(index.x + x, index.y, index.z + z);
                if (!cells.TryGetValue(neighbour, out WorldMapBuilder.SpaceCell adjacent) || adjacent == null || adjacent.state == WorldMapBuilder.SpaceState.Unknown)
                    count++;
            }
        }
        return count;
    }

    private float HorizontalDistance(Vector3 a, Vector3 b)
    {
        a.y = 0f;
        b.y = 0f;
        return Vector3.Distance(a, b);
    }

    private Vector3Int PositionKey(Vector3 position)
    {
        float size = mapBuilder != null ? Mathf.Max(0.1f, mapBuilder.cellSize) : 0.5f;
        return new Vector3Int(
            Mathf.RoundToInt(position.x / size),
            Mathf.RoundToInt(position.y / size),
            Mathf.RoundToInt(position.z / size));
    }

    private int GridKeyDistance(Vector3Int a, Vector3Int b)
    {
        return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.z - b.z);
    }

    public void StopExploration()
    {
        stopped = true;
        hasTarget = false;
        if (saveMapWhenExplorationStops && mapBuilder != null)
            mapBuilder.SaveMap();
        if (planner != null && planner.target == targetTransform)
            planner.target = null;
    }

    public void ResumeExploration()
    {
        if (planner == null || mapBuilder == null)
            return;
        stopped = false;
        explorationComplete = false;
        planner.target = targetTransform;
        SelectNewFrontier();
    }

    private void OnDisable()
    {
        if (saveMapWhenExplorationStops && mapBuilder != null)
            mapBuilder.SaveMap();
    }

    private void OnDestroy()
    {
        if (targetTransform != null)
            Destroy(targetTransform.gameObject);
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawDebugGizmos || !hasTarget) return;
        Gizmos.color = Color.magenta;
        Gizmos.DrawSphere(currentExplorationTarget + Vector3.up * 0.1f, 0.2f);
        Gizmos.DrawLine(transform.position, currentExplorationTarget);
    }
}
