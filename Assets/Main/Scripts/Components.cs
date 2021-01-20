using System;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;

struct Boid : IComponentData
{
    [InternalBufferCapacity(256)]
    internal struct Neighbor : IBufferElementData
    {
        public Entity Value;
        public static implicit operator Neighbor(Entity entity) => new Neighbor() { Value = entity};
    }

}
[Serializable]
struct Boids : IComponentData
{
    public float WallWeight;
    public float SeparationWeight;
    public float CohesionWeight;
    public float AlignmentWeight;
    public float ObstacleWeight;
    public struct Initialization : IComponentData
    {
        public Entity BoidPrefab;
        public int SpawnCount;
        public float3 SpawnRadius;
    }
    public struct ObstaclePrefab : IComponentData
    {
        public Entity Value;
    }
}
[Serializable]
struct Wall : IComponentData
{
    public Plane Plane;
}

[Serializable]
struct Obstacle : IComponentData
{
    public float Radius;
}

struct PointerHandle : IComponentData
{
    public int PointerId;
}