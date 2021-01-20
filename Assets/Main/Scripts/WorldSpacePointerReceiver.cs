using Unity.Assertions;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.EventSystems;

public sealed class WorldSpacePointerReceiver : MonoBehaviour,IPointerDownHandler , IDragHandler,IPointerExitHandler,IPointerUpHandler
{
    public static WorldSpacePointerReceiver Instance { get; private set; }

    internal NativeHashSet<int> newPointers;
    internal NativeHashMap<int, float3> pointerPositions;
    internal NativeHashSet<int> deletedPointers;

    void Start()
    {
        newPointers = new NativeHashSet<int>(16, Allocator.Persistent);
        pointerPositions = new NativeHashMap<int, float3>(16, Allocator.Persistent);
        deletedPointers = new NativeHashSet<int>(16, Allocator.Persistent);
        Instance = this;
    }

    void OnDestroy()
    {
        Instance = null;
        newPointers.Dispose();
        pointerPositions.Dispose();
        deletedPointers.Dispose();
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        AddPointer(eventData);
    }

    private void AddPointer(PointerEventData eventData)
    {
        deletedPointers.Remove(eventData.pointerId);
        newPointers.Add(eventData.pointerId);
        RefleshPointerPosition(eventData);
    }

    public void OnDrag(PointerEventData eventData)
    {
        RefleshPointerPosition(eventData);
    }

    private void RefleshPointerPosition(PointerEventData eventData)
    {
        pointerPositions[eventData.pointerId] = eventData.pointerCurrentRaycast.worldPosition;
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        RemovePointer(eventData);
    }

    private void RemovePointer(PointerEventData eventData)
    {
        newPointers.Remove(eventData.pointerId);
        pointerPositions.Remove(eventData.pointerId);
        deletedPointers.Add(eventData.pointerId);
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        RemovePointer(eventData);
    }
}
