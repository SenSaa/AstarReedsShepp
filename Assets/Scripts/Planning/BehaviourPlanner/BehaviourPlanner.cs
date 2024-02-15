using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices.ComTypes;
using UnityEngine;
using UtilityFunctions;

// Behaviour Planner using as Finite State Machine.

public class BehaviourPlanner
{

    private FSM.State BehaviourState;
    private FSM fsm;
    private double TargetSpeed_ms;
    private float TargetSpeed_kmh = 10; // km/h
    Path path;
    BroadcastData BroadcastData;
    VehicleControlCommandFlags vehicleControlCommandFlags;

    public void InitState(Path path)
    {
        fsm = new FSM();
        fsm.ChangeState(FSM.State.Idle);

        TargetSpeed_kmh = 10;
        TargetSpeed_ms = TargetSpeed_kmh / 3.6f; // [km/h] -> [m/s]

        this.path = path;

        this.BroadcastData = new BroadcastData();
        vehicleControlCommandFlags = new VehicleControlCommandFlags();
    }

    public double StateUpdate(PurePursuit.State state, int targetIndex, int pathDir)
    {
        StateTransition(state);

        BehaviourState = fsm.GetState();

        Update(pathDir, targetIndex);



        return TargetSpeed_ms;
    }

    private void StateTransition(PurePursuit.State state)
    {
        if (Driving() && fsm.GetState() == FSM.State.Idle)
        {
            fsm.ChangeState(FSM.State.Cruise);
        }
        if (NearGoal(state, path, 5) && fsm.GetState() == FSM.State.Cruise)
        {
            fsm.ChangeState(FSM.State.Stopping);
        }
        if (AtGoal(state, path, 0.5f) && fsm.GetState() == FSM.State.Stopping)
        {
            fsm.ChangeState(FSM.State.Idle);
        }
    }

    private void Update(int pathDir, int targetIndex)
    {
        switch (BehaviourState)
        {
            case FSM.State.Idle:
                Idle();
                break;

            case FSM.State.Cruise:
                Cruise(pathDir, targetIndex);
                break;

            case FSM.State.Stopping:
                Stopping(pathDir);
                break;
        }
    }

    private void Idle()
    {
        TargetSpeed_kmh = 0;
        TargetSpeed_ms = 0;

        vehicleControlCommandFlags.Reset();
        vehicleControlCommandFlags.ApplyBrake = true;
        vehicleControlCommandFlags.ReleaseThrottle = true;
        vehicleControlCommandFlags.ReleaseSteering = true;
        this.BroadcastData.BroadcastBehaviourPlanUpdate(vehicleControlCommandFlags);
    }

    private void Cruise(int pathDir, int targetIndex)
    {
        TargetSpeed_kmh = pathDir == 1 ? 10 : -5;
        TargetSpeed_ms = TargetSpeed_kmh / 3.6f;

        vehicleControlCommandFlags.Reset();
        vehicleControlCommandFlags.ReleaseBrake = true;

        // *
        // Slow down when a direction change si approaching!
        if (Utils.DirectionChangeAhead(path, targetIndex) && fsm.GetState() != FSM.State.Idle)
        {
            TargetSpeed_kmh = pathDir == 1 ? 3 : -3;
            TargetSpeed_ms = TargetSpeed_kmh / 3.6f;
        }
    }

    private void Stopping(int pathDir)
    {
        TargetSpeed_kmh = pathDir == 1 ? 4 : -4;
        TargetSpeed_ms = TargetSpeed_kmh / 3.6f;
    }

    private bool Driving()
    {
        return Math.Abs(TargetSpeed_kmh) > 0.1;
    }

    private bool NearGoal(PurePursuit.State state, Path path, float nearDist)
    {
        return Utils.CloseToDestination(state.x, state.y, path, nearDist);
    }

    private bool AtGoal(PurePursuit.State state, Path path, float nearDist)
    {
        return Utils.CloseToDestination(state.x, state.y, path, nearDist);
    }

    public FSM.State GetBehaviourState()
    {
        return BehaviourState;
    }

}
