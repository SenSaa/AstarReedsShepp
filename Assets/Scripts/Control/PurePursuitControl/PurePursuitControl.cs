using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UtilityFunctions;

// Path Tracking using Pure Pursuit steering control and PID speed controller.

namespace PurePursuit
{

    public class PurePursuitControl
    {

        private Transform Agent;
        private Vector3 AgentCenterPos;
        private State state;
        TargetCourse target_course;
        private PursuitTarget pursuitTarget;
        private Path ParameteisedPath = new Path(); // Target course
        private Vector3 TargetPos;
        private int TargetDirRelativeToAgent;

        public void PathTrackingInit(Transform agent, Path parameteisedPath)
        {
            this.Agent = agent;
            this.ParameteisedPath = parameteisedPath;
            state = new State(Utils.CalculateFrontWheelCenterPosition(Agent).x, Utils.CalculateFrontWheelCenterPosition(Agent).z, Agent.eulerAngles.y, 0);
            target_course = new TargetCourse(ParameteisedPath);
            pursuitTarget = target_course.search_target_index(state);
        }

        public (State state,int targetInd) PathTrackingUpdate(int pathDir)
        {
            TargetPos.Set((float)ParameteisedPath.x[pursuitTarget.TargetIndex], 0, (float)ParameteisedPath.y[pursuitTarget.TargetIndex]);
            
            // Update vehicle pose
            state.x = Utils.CalculateCenterPosition(Agent).x;
            state.y = Utils.CalculateCenterPosition(Agent).z;
            state.yaw = Utils.GetYaw(Agent);
            state.rear_x = state.x - ((Params.WB/2) * Math.Cos(state.yaw));
            state.rear_y = state.y - ((Params.WB/2) * Math.Sin(state.yaw));
            state.front_x = state.x + ((Params.WB/2) * Math.Cos(state.yaw));
            state.front_y = state.y + ((Params.WB/2) * Math.Sin(state.yaw));

            AgentCenterPos.Set((float)state.x, 0, (float)state.y);
            TargetDirRelativeToAgent = Utils.CheckDirection(TargetPos, Agent, AgentCenterPos);

            // Calc steering control
            PurePursuit purePursuit = new PurePursuit();
            pursuitTarget = target_course.search_target_index(state, pathDir, TargetDirRelativeToAgent);
            Tuple<double, int> di_target_ind = purePursuit.pure_pursuit_steer_control(state, target_course, pursuitTarget.TargetIndex, pathDir);
            // Update the value of steering angle (in degrees)
            state.delta = di_target_ind.Item1 * (double)Mathf.Rad2Deg;

            return (state, pursuitTarget.TargetIndex);
        }

    }

}
