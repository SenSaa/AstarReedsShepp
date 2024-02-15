using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using static UnityEditor.PlayerSettings;

namespace PurePursuit
{

    public class TargetCourse
    {

        public Path Path;
        int old_nearest_point_index;
        PursuitTarget pursuitTarget;
        public bool DirChanged;
        public int test_index1;
        public int test_index2;

        public TargetCourse(Path path)
        {
            this.Path = path;
            this.old_nearest_point_index = 0;
            pursuitTarget = new PursuitTarget();
        }

        public PursuitTarget search_target_index(State state, int pathDir = 1, int travelDir = 1)
        {
            var L = pathDir != -1 ? Params.Lfc : Params.Lbc;

            var target_idx = this.old_nearest_point_index;
            (double x, double z) target_point = (Path.x[target_idx], Path.y[target_idx]);
            double curr_dist;
            curr_dist = pathDir != -1 ? state.calc_distance(target_point.x, target_point.z) : state.calc_distance_R(target_point.x, target_point.z);

            if (pathDir != -1)
            {
                while (curr_dist < L && target_idx < this.Path.x.Count - 1)
                {
                    target_idx += 1;
                    target_point = (Path.x[target_idx], Path.y[target_idx]);
                    curr_dist = state.calc_distance(target_point.x, target_point.z);
                }
            }
            // *** Handle case when path is behind vehicle while travelling forward.
            // Skip waypoints!
            else if (pathDir == -1 && travelDir != -1) 
            {
                target_idx += 1;
            }

            this.old_nearest_point_index = target_idx;
            pursuitTarget.TargetIndex = target_idx;
            pursuitTarget.Lookahead = L;
            return pursuitTarget;
        }

    }

}
