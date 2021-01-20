using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

sealed class BoidsAuthoring : MonoBehaviour, IConvertGameObjectToEntity , IDeclareReferencedPrefabs
{
    public GameObject BoidPrefab;

    public int SpawnCount = 100;
    public float3 SpawnRadius = 0.5f;

    public float WallWeight = 1;
    public float SeparationWeight = 5;
    public float CohesionWeight = 3;
    public float AlignmentWeight = 2;
    public float ObstacleWeight = 2;
    public float ObstacleRadius = 0.5f;
    public GameObject ObstaclePrefab;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, Boids);
        var boidPrefab = conversionSystem.GetPrimaryEntity(BoidPrefab);
        dstManager.AddComponents(boidPrefab, new ComponentTypes(ComponentType.ReadWrite<Boid>(), ComponentType.ReadWrite<Boid.Neighbor>()));
        dstManager.AddComponentData(entity, new Boids.Initialization()
        {
            BoidPrefab = boidPrefab,
            SpawnCount = SpawnCount,
            SpawnRadius = SpawnRadius,
        });
        if (ObstaclePrefab)
        {
            var obstaclePrefab = conversionSystem.GetPrimaryEntity(ObstaclePrefab);
            dstManager.AddComponents(obstaclePrefab, new ComponentTypes(ComponentType.ReadWrite<Obstacle>(),ComponentType.ReadWrite<PointerHandle>()));
            dstManager.SetComponentData(obstaclePrefab, new Obstacle() { Radius = ObstacleRadius });
            dstManager.AddComponentData(entity, new Boids.ObstaclePrefab() { Value = obstaclePrefab });
        }
    }

    internal Boids Boids => new Boids()
    {
        WallWeight = WallWeight,
        SeparationWeight = SeparationWeight,
        CohesionWeight = CohesionWeight,
        AlignmentWeight = AlignmentWeight,
        ObstacleWeight = ObstacleWeight,
    };

    public void DeclareReferencedPrefabs(List<GameObject> referencedPrefabs)
    {
        referencedPrefabs.Add(BoidPrefab);
        referencedPrefabs.Add(ObstaclePrefab);
    }
}
