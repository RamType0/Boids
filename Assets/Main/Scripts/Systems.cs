using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEditor;

[UpdateInGroup(typeof(InitializationSystemGroup))]
[UpdateAfter(typeof(ConvertToEntitySystem))]
public sealed class BoidsInitializationSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireSingletonForUpdate<Boids.Initialization>();
    }

    protected override void OnUpdate()
    {
        var entityManager = EntityManager;
        var initializationEntity = GetSingletonEntity<Boids.Initialization>();
        var initialization = GetComponent<Boids.Initialization>(initializationEntity);
        entityManager.Instantiate(initialization.BoidPrefab, initialization.SpawnCount, Allocator.Temp).Dispose();
        entityManager.RemoveComponent<Boids.Initialization>(initializationEntity);
        var spawnRadius = initialization.SpawnRadius;
        Entities
            .WithAll<Boid>()
            .ForEach((int entityInQueryIndex, ref Translation translation, ref PhysicsVelocity physicsVelocity) =>
            {
                var random = Random.CreateFromIndex((uint)entityInQueryIndex);
                float3 pos;
                do
                {
                    pos = random.NextFloat3(-1, 1);
                } while (math.length(pos) > 1);
                pos *= spawnRadius;
                translation.Value = pos;
                physicsVelocity.Linear = pos;
            }).ScheduleParallel();
    }
}

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
public sealed class BoidsSimulationSystemGroup : ComponentSystemGroup
{

}
[UpdateInGroup(typeof(BoidsSimulationSystemGroup), OrderFirst = true)]
public sealed class BoidsHybridSynchronizationSystemGroup : ComponentSystemGroup
{

}

[UpdateInGroup(typeof(BoidsHybridSynchronizationSystemGroup))]
public sealed class CopyFromBoidsAuthoringSystem : SystemBase
{
    protected override void OnUpdate()
    {
        Entities
            .WithoutBurst()
            .ForEach((BoidsAuthoring boidsAuthoring, ref Boids boids) =>
            {
                boids = boidsAuthoring.Boids;
            }).Run();
    }
}
[UpdateInGroup(typeof(BoidsHybridSynchronizationSystemGroup))]
public sealed class RefleshPointerHandleTranslationSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireSingletonForUpdate<Boids.ObstaclePrefab>();
    }
    protected override void OnUpdate()
    {
        var pointerReceiver = WorldSpacePointerReceiver.Instance;
        if (pointerReceiver is object)
        {
            var prefab = GetSingleton<Boids.ObstaclePrefab>().Value;
            var deletedPointers = pointerReceiver.deletedPointers;
            var entityManager = EntityManager;
            Entities
                .WithReadOnly(deletedPointers)
                .WithStructuralChanges()
                .ForEach((Entity entity, in PointerHandle boid) =>
                {
                    if (deletedPointers.Remove(boid.PointerId))
                    {
                        entityManager.DestroyEntity(entity);
                    }

                }).Run();
            var newPointers = pointerReceiver.newPointers;
            foreach (var pointerId in newPointers)
            {
                var boid = EntityManager.Instantiate(prefab);
                SetComponent(boid, new PointerHandle() { PointerId = pointerId });
            }
            var pointerPositions = pointerReceiver.pointerPositions;
            var inverseDeltaTime = math.rcp(Time.DeltaTime);
            Entities
                .WithReadOnly(pointerPositions)
                .ForEach((ref Translation translation, in PointerHandle pointerHandle) =>
                {
                    var pointerId = pointerHandle.PointerId;
                    var previousPosition = translation.Value;
                    translation.Value = pointerPositions[pointerId];
                    newPointers.Remove(pointerId);
                    //if (!newPointers.Remove(pointerId))
                    //{
                    //    physicsVelocity.Linear = (translation.Value - previousPosition) * inverseDeltaTime;
                    //}

                }).Schedule();


        }
    }
}

[UpdateInGroup(typeof(BoidsSimulationSystemGroup))]
public sealed class SearchNeighborsSystem : SystemBase
{
    BuildPhysicsWorld buildPhysicsWorld;
    StepPhysicsWorld stepPhysicsWorld;
    protected override void OnCreate()
    {
        buildPhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>();
        stepPhysicsWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
    }
    protected override void OnUpdate()
    {
        Entities.ForEach((ref DynamicBuffer<Boid.Neighbor> neighbors) => neighbors.Clear()).ScheduleParallel();
        Dependency = new TriggerJob()
        {
            NeighborFromEntity = GetBufferFromEntity<Boid.Neighbor>(),
            //NearbyWallFromEntity = GetBufferFromEntity<Boid.NearbyWall>(),
            //WallFromEntity = GetComponentDataFromEntity<Wall>(true),
        }.Schedule(stepPhysicsWorld.Simulation, ref buildPhysicsWorld.PhysicsWorld, Dependency);
    }
    [BurstCompile]
    struct TriggerJob : ITriggerEventsJob
    {
        public BufferFromEntity<Boid.Neighbor> NeighborFromEntity;
        //public BufferFromEntity<Boid.NearbyWall> NearbyWallFromEntity;
        //[ReadOnly] public ComponentDataFromEntity<Wall> WallFromEntity;
        public void Execute(TriggerEvent triggerEvent)
        {
            var entityA = triggerEvent.EntityA;
            var entityB = triggerEvent.EntityB;
            if (NeighborFromEntity.HasComponent(entityA) && NeighborFromEntity.HasComponent(entityB))
            {
                TryAdd(NeighborFromEntity[entityA], entityB);
                TryAdd(NeighborFromEntity[entityB], entityA);
            }
            else
            {
                UnityEngine.Debug.LogError("Unexpected entity in trigger event!");
            }
        }

        static void TryAdd(DynamicBuffer<Boid.Neighbor> neighbors, Entity entity)
        {
            if (neighbors.Length + 1 <= neighbors.Capacity)
            {
                neighbors.Add(entity);
            }
        }

        //void Process(Entity entityA,Entity entityB)
        //{
        //    if (NeighborFromEntity.HasComponent(entityA)) // Is Boid
        //    {
        //        if (NeighborFromEntity.HasComponent(entityB))
        //        {
        //            NeighborFromEntity[entityA].Add(entityB);
        //        }
        //        else if (WallFromEntity.HasComponent(entityB))
        //        {
        //            NearbyWallFromEntity[entityA].Add(entityB);
        //        }
        //    }
        //}


    }
}
[UpdateInGroup(typeof(BoidsSimulationSystemGroup))]
public sealed class WallSystem : SystemBase
{
    EntityQuery wallQuery;
    protected override void OnCreate()
    {
        wallQuery = GetEntityQuery(ComponentType.ReadOnly<Wall>());
        RequireSingletonForUpdate<Boids>();
    }
    protected override void OnUpdate()
    {
        var walls = wallQuery.ToComponentDataArrayAsync<Wall>(Allocator.TempJob, out var collectWallsJobHandle);
        Dependency = JobHandle.CombineDependencies(Dependency, collectWallsJobHandle);

        var deltaTime = Time.DeltaTime;
        var boids = GetSingleton<Boids>();
        var wallWeight = boids.WallWeight;
        Entities
            .WithReadOnly(walls)
            .WithDisposeOnCompletion(walls)
            .WithAll<Boid>()
        .ForEach((ref PhysicsVelocity physicsVelocity, in Translation translation) =>
        {
            var force = new float3();
            for (int i = 0; i < walls.Length; i++)
            {
                var wall = walls[i].Plane;
                var distance = wall.SignedDistanceToPoint(translation.Value);
                force += wall.Normal / (distance * distance);
            }
            force *= wallWeight;
            physicsVelocity.Linear += force * deltaTime;

        }).ScheduleParallel();

    }
}
[UpdateInGroup(typeof(BoidsSimulationSystemGroup))]
public sealed class ObstacleSystem : SystemBase
{
    EntityQuery obstacleQuery;
    protected override void OnCreate()
    {
        obstacleQuery = GetEntityQuery(ComponentType.ReadOnly<Obstacle>(),ComponentType.ReadOnly<Translation>());
        RequireSingletonForUpdate<Boids>();
    }
    protected override void OnUpdate()
    {
        var obstacles = obstacleQuery.ToComponentDataArrayAsync<Obstacle>(Allocator.TempJob, out var collectObstaclesJobHandle);
        var obstaclePositions = obstacleQuery.ToComponentDataArrayAsync<Translation>(Allocator.TempJob, out var collectObstaclePositionsJobHandle);
        Dependency = JobHandle.CombineDependencies(Dependency, collectObstaclesJobHandle,collectObstaclePositionsJobHandle);

        var deltaTime = Time.DeltaTime;
        var boids = GetSingleton<Boids>();
        var obstacleWeight = boids.ObstacleWeight;
        Entities
            .WithReadOnly(obstacles)
            .WithDisposeOnCompletion(obstacles)
            .WithReadOnly(obstaclePositions)
            .WithDisposeOnCompletion(obstaclePositions)
            .WithAll<Boid>()
        .ForEach((ref PhysicsVelocity physicsVelocity, in Translation translation) =>
        {
            var force = new float3();
            for (int i = 0; i < obstacles.Length; i++)
            {
                var obstacle = obstacles[i];
                var obstaclePosition = obstaclePositions[i].Value;
                var relativePosFromObstacle = translation.Value - obstaclePosition;
                var distance = math.length(relativePosFromObstacle);

                force += math.select(relativePosFromObstacle / (distance * distance), 0, distance > obstacle.Radius);
            }
            force *= obstacleWeight;
            physicsVelocity.Linear += force * deltaTime;

        }).ScheduleParallel();

    }
}
[UpdateInGroup(typeof(BoidsSimulationSystemGroup))]
[UpdateAfter(typeof(SearchNeighborsSystem))]
public sealed class SeparationSystem : SystemBase
{
    EntityQuery query;
    protected override void OnCreate()
    {
        query = GetEntityQuery(new[]
        {
            ComponentType.ReadWrite<PhysicsVelocity>(),
            ComponentType.ReadOnly<Boid.Neighbor>(),
            ComponentType.ReadOnly<Translation>()
        });
        RequireSingletonForUpdate<Boids>();
    }
    protected override void OnUpdate()
    {
        var boids = GetSingleton<Boids>();
        Dependency = new SeparationJob()
        {
            PhysicsVelocityTypeHandle = GetComponentTypeHandle<PhysicsVelocity>(),
            NeighborTypeHandle = GetBufferTypeHandle<Boid.Neighbor>(true),
            TranslationTypeHandle = GetComponentTypeHandle<Translation>(true),
            TranslationFromEntity = GetComponentDataFromEntity<Translation>(true),
            DeltaTime = Time.DeltaTime,
            Weight = boids.SeparationWeight,
        }.ScheduleParallel(query, dependsOn: Dependency);
    }
    [BurstCompile]
    struct SeparationJob : IJobEntityBatch
    {
        [NoAlias] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityTypeHandle;
        [NoAlias, ReadOnly] public BufferTypeHandle<Boid.Neighbor> NeighborTypeHandle;
        [ReadOnly] public ComponentTypeHandle<Translation> TranslationTypeHandle;
        [ReadOnly] public ComponentDataFromEntity<Translation> TranslationFromEntity;
        public float DeltaTime;
        public float Weight;
        public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
        {
            var physicsVelocities = batchInChunk.GetNativeArray(PhysicsVelocityTypeHandle);
            var neighborsAccessor = batchInChunk.GetBufferAccessor(NeighborTypeHandle);
            var translations = batchInChunk.GetNativeArray(TranslationTypeHandle);
            for (int entityInBatchIndex = 0; entityInBatchIndex < batchInChunk.Count; entityInBatchIndex++)
            {
                var force = new float3();
                var neighbors = neighborsAccessor[entityInBatchIndex];
                for (int i = 0; i < neighbors.Length; i++)
                {
                    var neighbor = neighbors[i].Value;
                    var neighborPos = TranslationFromEntity[neighbor].Value;
                    var position = translations[entityInBatchIndex].Value;
                    force += math.normalizesafe(position - neighborPos);
                }
                ;
                force = math.select(force / neighbors.Length, 0, neighbors.Length == 0);
                force *= Weight;
                unsafe
                {
                    UnsafeUtility.ArrayElementAsRef<PhysicsVelocity>(physicsVelocities.GetUnsafePtr(), entityInBatchIndex).Linear += force * DeltaTime;
                }
            }
        }
    }

}
[UpdateInGroup(typeof(BoidsSimulationSystemGroup))]
[UpdateAfter(typeof(SearchNeighborsSystem))]
public sealed class CohesionSystem : SystemBase
{
    EntityQuery query;
    protected override void OnCreate()
    {
        query = GetEntityQuery(new[]
        {
            ComponentType.ReadWrite<PhysicsVelocity>(),
            ComponentType.ReadOnly<Boid.Neighbor>(),
            ComponentType.ReadOnly<Translation>(),
        });
        RequireSingletonForUpdate<Boids>();
    }
    protected override void OnUpdate()
    {
        var boids = GetSingleton<Boids>();
        Dependency = new CohesionJob()
        {
            PhysicsVelocityTypeHandle = GetComponentTypeHandle<PhysicsVelocity>(),
            NeighborTypeHandle = GetBufferTypeHandle<Boid.Neighbor>(true),
            TranslationTypeHandle = GetComponentTypeHandle<Translation>(true),
            TranslationFromEntity = GetComponentDataFromEntity<Translation>(true),
            DeltaTime = Time.DeltaTime,
            Weight = boids.CohesionWeight,
        }.ScheduleParallel(query, dependsOn: Dependency);
    }
    [BurstCompile]
    struct CohesionJob : IJobEntityBatch
    {
        [NoAlias] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityTypeHandle;
        [NoAlias, ReadOnly] public BufferTypeHandle<Boid.Neighbor> NeighborTypeHandle;
        [ReadOnly] public ComponentTypeHandle<Translation> TranslationTypeHandle;
        [ReadOnly] public ComponentDataFromEntity<Translation> TranslationFromEntity;
        public float DeltaTime;
        public float Weight;
        public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
        {
            var physicsVelocities = batchInChunk.GetNativeArray(PhysicsVelocityTypeHandle);
            var neighborsAccessor = batchInChunk.GetBufferAccessor(NeighborTypeHandle);
            var translations = batchInChunk.GetNativeArray(TranslationTypeHandle);
            for (int entityInBatchIndex = 0; entityInBatchIndex < batchInChunk.Count; entityInBatchIndex++)
            {
                var averagePos = new float3();
                var neighbors = neighborsAccessor[entityInBatchIndex];
                for (int i = 0; i < neighbors.Length; i++)
                {
                    var neighbor = neighbors[i].Value;
                    var neighborPos = TranslationFromEntity[neighbor].Value;
                    averagePos += neighborPos;
                }
                averagePos /= neighbors.Length;
                var position = translations[entityInBatchIndex].Value;
                var force = math.select(averagePos - position, 0, neighbors.Length == 0) * Weight;
                unsafe
                {
                    UnsafeUtility.ArrayElementAsRef<PhysicsVelocity>(physicsVelocities.GetUnsafePtr(), entityInBatchIndex).Linear += force * DeltaTime;
                }
            }
        }
    }

}
[UpdateInGroup(typeof(BoidsSimulationSystemGroup))]
[UpdateAfter(typeof(SearchNeighborsSystem))]
public sealed class AlignmentSystem : SystemBase
{
    EntityQuery query;
    protected override void OnCreate()
    {
        query = GetEntityQuery(new[]
        {
            ComponentType.ReadWrite<PhysicsVelocity>(),
            ComponentType.ReadOnly<Boid.Neighbor>(),
            ComponentType.ReadOnly<Translation>(),
        });
        RequireSingletonForUpdate<Boids>();
    }
    protected override void OnUpdate()
    {
        var boids = GetSingleton<Boids>();
        var count = query.CalculateEntityCount();
        var velocities = new NativeArray<float3>(count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        Dependency = new AlignmentJob()
        {
            PhysicsVelocityTypeHandle = GetComponentTypeHandle<PhysicsVelocity>(true),
            NeighborTypeHandle = GetBufferTypeHandle<Boid.Neighbor>(true),
            PhysicsVelocityFromEntity = GetComponentDataFromEntity<PhysicsVelocity>(true),
            DeltaTime = Time.DeltaTime,
            Weight = boids.AlignmentWeight,
            Velocities = velocities,
        }.ScheduleParallel(query, dependsOn: Dependency);
        Dependency = new WriteVelocityJob()
        {
            PhysicsVelocityTypeHandle = GetComponentTypeHandle<PhysicsVelocity>(),
            Velocities = velocities,
        }.ScheduleParallel(query, dependsOn: Dependency);
    }
    [BurstCompile]
    struct AlignmentJob : IJobEntityBatchWithIndex
    {
        [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityTypeHandle;
        [NoAlias, ReadOnly] public BufferTypeHandle<Boid.Neighbor> NeighborTypeHandle;
        [ReadOnly] public ComponentDataFromEntity<PhysicsVelocity> PhysicsVelocityFromEntity;
        public float DeltaTime;
        public float Weight;
        [WriteOnly] public NativeArray<float3> Velocities;
        public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int indexOfFirstEntityInQuery)
        {
            var physicsVelocities = batchInChunk.GetNativeArray(PhysicsVelocityTypeHandle);
            var neighborsAccessor = batchInChunk.GetBufferAccessor(NeighborTypeHandle);
            for (int entityInBatchIndex = 0; entityInBatchIndex < batchInChunk.Count; entityInBatchIndex++)
            {
                var averageVelocity = new float3();
                var neighbors = neighborsAccessor[entityInBatchIndex];
                for (int i = 0; i < neighbors.Length; i++)
                {
                    var neighbor = neighbors[i].Value;
                    var neighborVelocity = PhysicsVelocityFromEntity[neighbor].Linear;
                    averageVelocity += neighborVelocity;
                }
                averageVelocity /= neighbors.Length;
                unsafe
                {
                    var velocity = UnsafeUtility.ArrayElementAsRef<PhysicsVelocity>(physicsVelocities.GetUnsafeReadOnlyPtr(), entityInBatchIndex).Linear;
                    var force = math.select(averageVelocity - velocity, 0, neighbors.Length == 0) * Weight;
                    Velocities[indexOfFirstEntityInQuery + entityInBatchIndex] = velocity + force * DeltaTime;
                }
            }
        }
    }
    [BurstCompile]
    struct WriteVelocityJob : IJobEntityBatchWithIndex
    {
        public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityTypeHandle;
        [ReadOnly, DeallocateOnJobCompletion] public NativeArray<float3> Velocities;
        public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int indexOfFirstEntityInQuery)
        {
            var physicsVelocities = batchInChunk.GetNativeArray(PhysicsVelocityTypeHandle);

            var velocities = Velocities.GetSubArray(indexOfFirstEntityInQuery, physicsVelocities.Length);
            unsafe
            {
                UnsafeUtility.MemCpyStride(
                    physicsVelocities.GetUnsafePtr(), sizeof(PhysicsVelocity),
                    velocities.GetUnsafeReadOnlyPtr(), sizeof(float3),
                    sizeof(float3),
                    physicsVelocities.Length);
            }

            //for (int entityInBatchIndex = 0; entityInBatchIndex < batchInChunk.Count; entityInBatchIndex++)
            //{
            //    unsafe
            //    {
            //        UnsafeUtility.ArrayElementAsRef<PhysicsVelocity>(physicsVelocities.GetUnsafePtr(), entityInBatchIndex).Linear = Velocities[indexOfFirstEntityInQuery + entityInBatchIndex];
            //    }
            //}
        }
    }

}
[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(FixedStepSimulationSystemGroup))]
public sealed class AlignWithVelocitySystem : SystemBase
{
    protected override void OnUpdate()
    {
        Entities
            .WithAll<Boid>()
            .ForEach((ref Rotation rotation,in PhysicsVelocity physicsVelocity) =>
            {
                rotation.Value = quaternion.LookRotationSafe(physicsVelocity.Linear, math.up());
            }).ScheduleParallel();

    }
}