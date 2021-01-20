using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Plane = Unity.Physics.Plane;
sealed class WallAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        var normal = - transform.forward;
        var distance = math.dot(transform.position, -normal);
        var plane = new Plane(normal, distance);
        dstManager.AddComponentData(entity, new Wall() { Plane = plane });
    }
}