using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

/// <summary>
/// Pure Pursuit Steer Controller.
/// </summary>

namespace PurePursuit
{

    public class PurePursuit
    {

        double tx, ty;

        public Tuple<double, int> pure_pursuit_steer_control(State state, TargetCourse trajectory, int pind, int dir)
        {
            bool isReversing = dir == -1 ? true : false;

            PursuitTarget pursuitTarget = trajectory.search_target_index(state);
            
            if (pind >= pursuitTarget.TargetIndex)
            {
                pursuitTarget.TargetIndex = pind;
            }

            if (pursuitTarget.TargetIndex < trajectory.Path.x.Count)
            {
                tx = trajectory.Path.x[pursuitTarget.TargetIndex];
                ty = trajectory.Path.y[pursuitTarget.TargetIndex];
            }
            else  // toward goal
            {
                tx = trajectory.Path.x[trajectory.Path.x.Count - 1];
                ty = trajectory.Path.y[trajectory.Path.y.Count - 1];
                pursuitTarget.TargetIndex = trajectory.Path.x.Count - 1;
            }

            double alpha = Math.Atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw;

            // * Account for reversing scenarios -> adjust lookahead!
            double delta = 0;
            if (isReversing)
            {
                delta = -Math.Atan2(2.0 * Params.WB * Math.Sin(alpha) / pursuitTarget.Lookahead, 1.0f);
            }
            else
            {
                delta = Math.Atan2(2.0 * Params.WB * Math.Sin(alpha) / pursuitTarget.Lookahead, 1.0f);
            }

            return new Tuple<double, int>(delta, pursuitTarget.TargetIndex);
        }

    }

}
