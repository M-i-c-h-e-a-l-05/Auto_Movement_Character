using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public class CursorTarget : MonoBehaviour
{
    [Tooltip("Layers the cursor can land on. Set this to the same layer as the bot's Ground Layer.")]
    public LayerMask groundMask = ~0;

    [Tooltip("Lifts the target above the ground so it doesn't sink into it.")]
    public float heightOffset = 0.5f;

    private Camera cam;

    private void Start()
    {
        cam = Camera.main;

        if (cam == null)
        {
            Debug.LogWarning("[CursorTarget] No camera tagged MainCamera found.");
            enabled = false;
        }
    }

    private void Update()
    {
#if ENABLE_INPUT_SYSTEM
        Vector2 mouse = Mouse.current != null
            ? Mouse.current.position.ReadValue()
            : Vector2.zero;
#else
        Vector2 mouse = Input.mousePosition;
#endif

        Ray ray = cam.ScreenPointToRay(mouse);

        if (Physics.Raycast(
            ray,
            out RaycastHit hit,
            1000f,
            groundMask,
            QueryTriggerInteraction.Ignore))
        {
            transform.position = hit.point + Vector3.up * heightOffset;
        }
    }
}