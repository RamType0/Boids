using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;

public class FixedStepSimulationTimeStepConfiguration : MonoBehaviour
{
    public float TimeStep = 0.02f;
    private void Start()
    {
        World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<FixedStepSimulationSystemGroup>().Timestep = TimeStep;
    }
}
