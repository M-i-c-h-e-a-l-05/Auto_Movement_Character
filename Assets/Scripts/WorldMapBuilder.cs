using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Builds a persistent, goal-independent map from the surroundings observed by Bot2.
/// Attach this component to Bot2. It records nearby colliders and samples local space
/// as Bot2 moves. It does not choose exploration destinations; that belongs in
/// WorldMapExplorer.cs.
///
/// Important: movement assessments are conservative observations, not proof that a
/// jump/slide/crouch is safe. A separate traversal test should verify those actions.
/// </summary>
public class WorldMapBuilder : MonoBehaviour
{
    public enum SpaceState { Unknown, Free, Occupied }
    public enum MovementAssessment { Unknown, Untested, Possible, Blocked }

    [Serializable]
    public class WorldMapData
    {
        public string mapVersion = "1.0";
        public string sceneName;
        public string savedAtUtc;
        public Vector3 mapOrigin;
        public float cellSize;
        public float scanRadius;
        public List<ObjectRecord> objects = new List<ObjectRecord>();
        public List<SpaceCell> cells = new List<SpaceCell>();
    }

    [Serializable]
    public class ObjectRecord
    {
        public string objectId;
        public string objectName;
        public string hierarchyPath;
        public string tagName;
        public int layer;
        public string layerName;
        public bool isTrigger;
        public string colliderType;
        public Vector3 position;
        public Vector3 rotationEuler;
        public Vector3 lossyScale;
        public Vector3 boundsCenter;
        public Vector3 boundsMin;
        public Vector3 boundsMax;
        public Vector3 boundsSize;
        public bool isStatic;
        public bool isMapBoundaryCandidate;
        public bool isJumpType;
        public bool isSlideType;
        public bool hasGroundBelow;
        public MovementAssessment walk = MovementAssessment.Unknown;
        public MovementAssessment run = MovementAssessment.Unknown;
        public MovementAssessment jump = MovementAssessment.Untested;
        public MovementAssessment crouch = MovementAssessment.Untested;
        public MovementAssessment slide = MovementAssessment.Untested;
        public string observationStatus = "detected";
        public int observations;
        public float firstSeenTime;
        public float lastSeenTime;
    }

    [Serializable]
    public class SpaceCell
    {
        public Vector3 center;
        public Vector3Int gridIndex;
        public SpaceState state;
        public bool hasGround;
        public Vector3 groundPoint;
        public Vector3 groundNormal;
        public float lastObservedTime;
        public int observations;
    }

    [Header("Scanning")]
    [Min(0.5f)] public float scanRadius = 8f;
    [Min(0.05f)] public float scanInterval = 0.25f;
    [Min(0.1f)] public float cellSize = 0.5f;
    [Min(0.05f)] public float occupancyProbeRadius = 0.12f;
    public LayerMask scanMask = ~0;
    public LayerMask groundMask = ~0;
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Collide;
    public bool scanOnlyWhileMoving = true;
    [Min(0f)] public float movementRequiredToRescan = 0.1f;

    [Header("Bot body sampling")]
    [Tooltip("Approximate height of Bot1/Bot2 used for local clearance sampling.")]
    [Min(0.2f)] public float botHeight = 2f;
    [Tooltip("Approximate radius of the bot used for space sampling.")]
    [Min(0.05f)] public float botRadius = 0.35f;
    [Min(0.1f)] public float groundRayHeight = 2f;
    [Min(0.1f)] public float groundRayDistance = 5f;

    [Header("Classification")]
    public string jumpTag = "Jump_type";
    public string slideTag = "Slide_type";
    [Tooltip("Optional tags treated as likely map boundaries, e.g. Wall, Boundary, MapEdge.")]
    public List<string> boundaryTags = new List<string> { "Boundary", "MapBoundary", "Wall" };
    [Tooltip("Optional layers treated as likely map boundaries. Object layer is always recorded regardless.")]
    public LayerMask boundaryLayerMask;
    public List<string> ignoredTags = new List<string> { "Player", "Bot1", "Bot2", "MainCamera" };

    [Header("Saving")]
    public string saveFileName = "world_navigation_map.json";
    public bool saveAfterEveryScan = false;
    public bool saveOnDisable = true;
    public bool logScanSummary = true;

    public WorldMapData Map { get; private set; }
    public string SavePath => Path.Combine(Application.persistentDataPath, saveFileName);

    private readonly Dictionary<string, ObjectRecord> objectRecords = new Dictionary<string, ObjectRecord>();
    private readonly Dictionary<Vector3Int, SpaceCell> cellsByIndex = new Dictionary<Vector3Int, SpaceCell>();
    private Collider[] overlapBuffer = new Collider[256];
    private float nextScanTime;
    private Vector3 lastScanPosition;

    private void Awake()
    {
        Map = new WorldMapData
        {
            sceneName = SceneManager.GetActiveScene().name,
            mapOrigin = transform.position,
            cellSize = cellSize,
            scanRadius = scanRadius
        };
        lastScanPosition = transform.position;
    }

    private void Start()
    {
        ScanNow();
    }

    private void Update()
    {
        if (Time.time < nextScanTime)
            return;

        nextScanTime = Time.time + scanInterval;
        if (scanOnlyWhileMoving && Vector3.Distance(transform.position, lastScanPosition) < movementRequiredToRescan)
            return;

        ScanNow();
    }

    /// <summary>Immediately records objects and space around Bot2.</summary>
    public void ScanNow()
    {
        if (Map == null)
            return;

        lastScanPosition = transform.position;
        Map.sceneName = SceneManager.GetActiveScene().name;
        Map.cellSize = cellSize;
        Map.scanRadius = scanRadius;

        ScanNearbyObjects();
        SampleNearbySpace();
        RebuildSerializableLists();

        if (saveAfterEveryScan)
            SaveMap();

        if (logScanSummary)
            Debug.Log($"[WorldMapBuilder] Objects: {Map.objects.Count}, observed cells: {Map.cells.Count}, file: {SavePath}", this);
    }

    private void ScanNearbyObjects()
    {
        int count = Physics.OverlapSphereNonAlloc(transform.position, scanRadius, overlapBuffer, scanMask, triggerInteraction);
        if (count == overlapBuffer.Length)
        {
            Array.Resize(ref overlapBuffer, overlapBuffer.Length * 2);
            count = Physics.OverlapSphereNonAlloc(transform.position, scanRadius, overlapBuffer, scanMask, triggerInteraction);
        }

        for (int i = 0; i < count; i++)
        {
            Collider hit = overlapBuffer[i];
            if (hit == null || IsPartOfThisBot(hit.transform) || IsIgnored(hit.gameObject))
                continue;

            string key = GetStableHierarchyPath(hit.transform);
            if (!objectRecords.TryGetValue(key, out ObjectRecord record))
            {
                record = new ObjectRecord
                {
                    objectId = key,
                    hierarchyPath = key,
                    objectName = hit.gameObject.name,
                    firstSeenTime = Time.time,
                    observations = 0
                };
                objectRecords.Add(key, record);
            }

            UpdateObjectRecord(record, hit);
        }
    }

    private void UpdateObjectRecord(ObjectRecord record, Collider hit)
    {
        Bounds b = hit.bounds;
        record.objectName = hit.gameObject.name;
        record.tagName = hit.tag;
        record.layer = hit.gameObject.layer;
        record.layerName = LayerMask.LayerToName(hit.gameObject.layer);
        record.isTrigger = hit.isTrigger;
        record.colliderType = hit.GetType().Name;
        record.position = hit.transform.position;
        record.rotationEuler = hit.transform.eulerAngles;
        record.lossyScale = hit.transform.lossyScale;
        record.boundsCenter = b.center;
        record.boundsMin = b.min;
        record.boundsMax = b.max;
        record.boundsSize = b.size;
        record.isStatic = hit.gameObject.isStatic;
        record.isMapBoundaryCandidate = IsBoundaryCandidate(hit.gameObject);
        record.isJumpType = SafeCompareTag(hit.gameObject, jumpTag);
        record.isSlideType = SafeCompareTag(hit.gameObject, slideTag);
        record.hasGroundBelow = HasGroundBelow(b.center);
        record.observations++;
        record.lastSeenTime = Time.time;
        record.observationStatus = "detected";

        // Tags are hints only. Physical traversal tests must update these fields later.
        if (record.isJumpType)
            record.jump = MovementAssessment.Untested;
        if (record.isSlideType)
        {
            record.crouch = MovementAssessment.Untested;
            record.slide = MovementAssessment.Untested;
        }
    }

    private void SampleNearbySpace()
    {
        float step = Mathf.Max(0.1f, cellSize);
        int cellsAround = Mathf.CeilToInt(scanRadius / step);
        Vector3 origin = transform.position;
        Vector3Int botCell = WorldToGrid(origin);

        for (int x = -cellsAround; x <= cellsAround; x++)
        {
            for (int z = -cellsAround; z <= cellsAround; z++)
            {
                Vector3 flat = new Vector3(origin.x + x * step, origin.y, origin.z + z * step);
                if (Vector2.Distance(new Vector2(flat.x, flat.z), new Vector2(origin.x, origin.z)) > scanRadius)
                    continue;

                // Sample the bot's lower body, torso and head clearance at this location.
                bool bodyOccupied = false;
                float[] heights = { botRadius, botHeight * 0.5f, Mathf.Max(botRadius, botHeight - botRadius) };
                for (int h = 0; h < heights.Length; h++)
                {
                    Vector3 sample = new Vector3(flat.x, origin.y + heights[h], flat.z);
                    if (Physics.CheckSphere(sample, occupancyProbeRadius, scanMask, triggerInteraction))
                    {
                        // Ignore the body colliders attached to this bot.
                        Collider[] localHits = Physics.OverlapSphere(sample, occupancyProbeRadius, scanMask, triggerInteraction);
                        bool externalHit = false;
                        for (int j = 0; j < localHits.Length; j++)
                        {
                            if (localHits[j] != null && !IsPartOfThisBot(localHits[j].transform) && !IsIgnored(localHits[j].gameObject))
                            {
                                externalHit = true;
                                break;
                            }
                        }
                        if (externalHit)
                        {
                            bodyOccupied = true;
                            break;
                        }
                    }
                }

                Vector3Int index = new Vector3Int(botCell.x + x, botCell.y, botCell.z + z);
                if (!cellsByIndex.TryGetValue(index, out SpaceCell cell))
                {
                    cell = new SpaceCell { gridIndex = index, center = GridToWorld(index), state = SpaceState.Unknown };
                    cellsByIndex.Add(index, cell);
                }

                cell.state = bodyOccupied ? SpaceState.Occupied : SpaceState.Free;
                cell.lastObservedTime = Time.time;
                cell.observations++;

                Vector3 rayStart = new Vector3(flat.x, origin.y + groundRayHeight, flat.z);
                if (Physics.Raycast(rayStart, Vector3.down, out RaycastHit groundHit, groundRayDistance, groundMask, QueryTriggerInteraction.Ignore)
                    && !IsPartOfThisBot(groundHit.transform))
                {
                    cell.hasGround = true;
                    cell.groundPoint = groundHit.point;
                    cell.groundNormal = groundHit.normal;
                }
                else
                {
                    cell.hasGround = false;
                    cell.groundPoint = Vector3.zero;
                    cell.groundNormal = Vector3.up;
                }
            }
        }
    }

    private bool HasGroundBelow(Vector3 point)
    {
        Vector3 start = point + Vector3.up * groundRayHeight;
        return Physics.Raycast(start, Vector3.down, out RaycastHit hit, groundRayDistance, groundMask, QueryTriggerInteraction.Ignore)
               && !IsPartOfThisBot(hit.transform);
    }

    private bool IsBoundaryCandidate(GameObject obj)
    {
        if (((1 << obj.layer) & boundaryLayerMask.value) != 0)
            return true;

        for (int i = 0; i < boundaryTags.Count; i++)
        {
            if (SafeCompareTag(obj, boundaryTags[i]))
                return true;
        }

        string n = obj.name.ToLowerInvariant();
        return n.Contains("boundary") || n.Contains("map_edge") || n.Contains("mapedge");
    }

    private bool IsIgnored(GameObject obj)
    {
        for (int i = 0; i < ignoredTags.Count; i++)
        {
            if (SafeCompareTag(obj, ignoredTags[i]))
                return true;
        }
        return false;
    }

    private bool SafeCompareTag(GameObject obj, string tagName)
    {
        if (obj == null || string.IsNullOrWhiteSpace(tagName))
            return false;
        try { return obj.CompareTag(tagName); }
        catch (UnityException) { return false; }
    }

    private bool IsPartOfThisBot(Transform candidate)
    {
        return candidate == transform || candidate.IsChildOf(transform);
    }

    private string GetStableHierarchyPath(Transform t)
    {
        string path = t.name;
        Transform parent = t.parent;
        while (parent != null)
        {
            path = parent.name + "/" + path;
            parent = parent.parent;
        }
        return SceneManager.GetActiveScene().name + ":" + path;
    }

    private Vector3Int WorldToGrid(Vector3 position)
    {
        float size = Mathf.Max(0.1f, cellSize);
        return new Vector3Int(
            Mathf.RoundToInt(position.x / size),
            Mathf.RoundToInt(position.y / size),
            Mathf.RoundToInt(position.z / size));
    }

    private Vector3 GridToWorld(Vector3Int index)
    {
        float size = Mathf.Max(0.1f, cellSize);
        return new Vector3(index.x * size, index.y * size, index.z * size);
    }

    private void RebuildSerializableLists()
    {
        Map.objects.Clear();
        Map.objects.AddRange(objectRecords.Values);
        Map.cells.Clear();
        Map.cells.AddRange(cellsByIndex.Values);
    }

    [ContextMenu("Save World Map")]
    public void SaveMap()
    {
        if (Map == null)
            return;

        RebuildSerializableLists();
        Map.savedAtUtc = DateTime.UtcNow.ToString("o");
        try
        {
            string directory = Path.GetDirectoryName(SavePath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                Directory.CreateDirectory(directory);

            string json = JsonUtility.ToJson(Map, true);
            File.WriteAllText(SavePath, json);
            Debug.Log($"[WorldMapBuilder] Saved map to: {SavePath}", this);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[WorldMapBuilder] Failed to save map: {ex.Message}", this);
        }
    }

    [ContextMenu("Load Existing World Map")]
    public void LoadMap()
    {
        if (!File.Exists(SavePath))
        {
            Debug.LogWarning($"[WorldMapBuilder] No saved map found at: {SavePath}", this);
            return;
        }

        try
        {
            WorldMapData loaded = JsonUtility.FromJson<WorldMapData>(File.ReadAllText(SavePath));
            if (loaded == null)
                throw new InvalidDataException("Map JSON could not be parsed.");

            Map = loaded;
            objectRecords.Clear();
            cellsByIndex.Clear();
            if (Map.objects != null)
            {
                foreach (ObjectRecord record in Map.objects)
                {
                    if (record != null && !string.IsNullOrEmpty(record.objectId))
                        objectRecords[record.objectId] = record;
                }
            }
            if (Map.cells != null)
            {
                foreach (SpaceCell cell in Map.cells)
                {
                    if (cell != null)
                        cellsByIndex[cell.gridIndex] = cell;
                }
            }
            Debug.Log($"[WorldMapBuilder] Loaded map: {Map.objects.Count} objects, {Map.cells.Count} cells.", this);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[WorldMapBuilder] Failed to load map: {ex.Message}", this);
        }
    }

    private void OnDisable()
    {
        if (saveOnDisable)
            SaveMap();
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, scanRadius);
    }
}
