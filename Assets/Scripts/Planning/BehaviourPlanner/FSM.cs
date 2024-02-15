using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Finite State Machine.

public class FSM
{

    public enum State
    {
        Idle,
        Cruise,
        Stopping
    }

    private State CurrentState;

    public FSM()
    {
        CurrentState = State.Idle;
    }

    // Change current state to new state.
    public void ChangeState(State newState)
    {
        CurrentState = newState;
        Debug.Log($"Transitioned to state: {CurrentState}");
    }

    // Retrieve current state.
    public State GetState()
    {
        return CurrentState;
    }

}
