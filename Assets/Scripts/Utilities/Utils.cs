using MathNet.Numerics.RootFinding;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using UnityEngine;

namespace UtilityFunctions
{

    public static class Utils
    {

        public static T DeepClone<T>(this T obj)
        {
            using (var ms = new MemoryStream())
            {
                var formatter = new BinaryFormatter();
                formatter.Serialize(ms, obj);
                ms.Position = 0;

                return (T)formatter.Deserialize(ms);
            }
        }

        public static string ListToString<T>(List<T> list)
        {
            string result = "";
            for (int i = 0; i < list.Count; i++)
            {
                result += list[i] + "   ";
            }
            return result;
        }

        // Zip three collections/lists.
        public static IEnumerable<TResult> ZipThree<T1, T2, T3, TResult>(
            this IEnumerable<T1> source,
            IEnumerable<T2> second,
            IEnumerable<T3> third,
            Func<T1, T2, T3, TResult> func)
        {
            using (var e1 = source.GetEnumerator())
            using (var e2 = second.GetEnumerator())
            using (var e3 = third.GetEnumerator())
            {
                while (e1.MoveNext() && e2.MoveNext() && e3.MoveNext())
                    yield return func(e1.Current, e2.Current, e3.Current);
            }
        }

        public static bool EqualPathTypes(List<string> path1_ctypes, List<string> path2_ctypes)
        {
            if (path1_ctypes[0].Equals(path2_ctypes[0]) && path1_ctypes[1].Equals(path2_ctypes[1]) && path1_ctypes[2].Equals(path2_ctypes[2]))
            {
                return true;
            }
            return false;
        }

        // Return a float with the magnitude (absolute value) of x but the sign of y.
        public static double CopySign(double x, double y)
        {
            if (y == 0) { return Math.Abs(x); }
            return Math.Abs(x) * Math.Sign(y);
        }

        // Constrain angle to range of 0->2pi.
        public static double NormaliseAngle(double angle)
        {
            return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
        }

        public static double Mod2pi(double x)
        {
            double v = x % Utils.CopySign(2.0 * Math.PI, x);
            if (v < -Math.PI)
            {
                v += 2.0 * Math.PI;
            }
            else
            {
                if (v > Math.PI)
                {
                    v -= 2.0 * Math.PI;

                }
            }
            return v;
        }

        public static Path FindMinLengthPath(List<Path> list)
        {
            Path min = list[0];
            for (int i = 1; i < list.Count; i++)
            {
                if (list[i].L < min.L)
                {
                    min = list[i];
                }
            }
            return min;
        }

        public static Path ListsToPath(List<double> xs, List<double> ys, List<double> yaws, List<string> modes, List<double> lengths, List<int> dirs)
        {
            Path path = new Path();
            for (int i = 0; i < xs.Count; i++)
            {
                path.x.Add(xs[i]); // x positions
                path.y.Add(ys[i]); // y positions
                path.yaw.Add(yaws[i]); // orientations [rad]

                path.directions.Add(dirs[i]); // directions (1:forward, -1:backward)
            }

            //  Total lengths of the path
            // course segment length  (negative value is backward segment)
            // * Less set of values than above path params, cannot be in the same loop!
            for (int i = 0; i < lengths.Count; i++)
            {
                path.lengths.Add(lengths[i]);
            }

            // course segment type char ("S": straight, "L": left, "R": right)
            // * List of three letters, cannot be in the same loop as above path params!
            for (int i = 0; i < modes.Count; i++)
            {
                path.ctypes.Add(modes[i]);
            }

            return path;
        }

        // Tranform RHS coordinate system (commonly used) to LHS (Unity ver).
        // By negating the yaw!
        // Then add 90. <- Probably due to the z-up in Unity in contrast to y-up used commonly.
        public static float TransformYaw(double yaw)
        {
            float transformedYaw;
            transformedYaw = (float)-yaw + 90;
            if (transformedYaw >= 180) { transformedYaw -= 360; }
            else if (transformedYaw <= -180) { transformedYaw += 360; }
            return transformedYaw;
        }

        // * Note: Agent position reference is top right of vehicle 3d design model.
        // * Manipulate accordingly to return center of front wheel pos.
        public static Vector3 CalculateFrontWheelCenterPosition(Transform agent)
        {
            var localPos = agent.InverseTransformDirection(agent.position); // Global to local pos.
            float localZPos = localPos.z; // Longitudinal pos.
            float localXPos = localPos.x; // Longitudinal pos.
            float zOffset = -(Params.CarLength / 16);
            float xOffset = -(Params.CarWidth / 3);
            float newLocalZPos = localZPos + zOffset; // Shift longitudinal pos backwards.
            float newLocalXPos = localXPos + xOffset; // Shift longitudinal pos backwards.
            var newLocalPos = new Vector3(newLocalXPos, localPos.y, newLocalZPos);
            var frontWheelCenterPos = agent.TransformDirection(newLocalPos);
            return frontWheelCenterPos;
        }
        public static Vector3 CalculateRearWheelCenterPosition(Transform agent)
        {
            var localPos = agent.InverseTransformDirection(agent.position); // Global to local pos.
            float localZPos = localPos.z; // Longitudinal pos.
            float localXPos = localPos.x; // Longitudinal pos.
            float zOffset = -(Params.CarLength / 2);
            float xOffset = -(Params.CarWidth / 3);
            float newLocalZPos = localZPos + zOffset; // Shift longitudinal pos backwards.
            float newLocalXPos = localXPos + xOffset; // Shift longitudinal pos backwards.
            var newLocalPos = new Vector3(newLocalXPos, localPos.y, newLocalZPos);
            var frontWheelCenterPos = agent.TransformDirection(newLocalPos);
            return frontWheelCenterPos;
        }
        public static Vector3 CalculateCenterPosition(Transform agent)
        {
            var localPos = agent.InverseTransformDirection(agent.position); // Global to local pos.
            float localZPos = localPos.z; // Longitudinal pos.
            float localXPos = localPos.x; // Longitudinal pos.
            float zOffset = -(Params.CarLength / 3.5f);
            float xOffset = -(Params.CarWidth / 2.5f);
            float newLocalZPos = localZPos + zOffset; // Shift longitudinal pos backwards.
            float newLocalXPos = localXPos + xOffset; // Shift longitudinal pos backwards.
            var newLocalPos = new Vector3(newLocalXPos, localPos.y, newLocalZPos);
            var frontWheelCenterPos = agent.TransformDirection(newLocalPos);
            return frontWheelCenterPos;
        }

        public static float GetYaw(Transform Agent)
        {
            float rawYaw = Agent.rotation.eulerAngles.y;
            float transformedYaw = -(rawYaw) + 90; // LHS -> RHS
            transformedYaw = transformedYaw >= 180 ? transformedYaw -= 360 : transformedYaw += 360;
            float yawInRads = transformedYaw * Mathf.Deg2Rad; // Degrees -> Radians
            return yawInRads;
        }

        public static float GetCurrentSpeed(Transform agent, Rigidbody rigidbody)
        {
            // Calculate current speed in relation to the forward direction of the car
            // this returns a negative number when traveling backwards
            float forwardSpeed = Vector3.Dot(agent.forward, rigidbody.velocity);
            float speed = forwardSpeed; // m/s
            return speed;
        }

        public static float GetCurrentSteeringAngle(WheelCollider frontWC)
        {
            float steeringAngle = frontWC.steerAngle;
            if (steeringAngle >= 360) { steeringAngle -= 360; }
            if (steeringAngle <= -360) { steeringAngle += 360; }
            return -steeringAngle;
        }


        public static int CheckDirection(Vector3 target, Transform agent, Vector3 agentCenterPos)
        {
            Vector3 forward = agent.TransformDirection(Vector3.forward);
            Vector3 dist = target - agentCenterPos;
            var dotProduct = Vector3.Dot(forward, dist);
            int direction = dotProduct >= 0 ? 1 : -1;
            return direction;
        }

        public static bool CloseToDestination(double current_x, double current_y, Path path, float nearGoalDist)
        {
            float distToGoal = Vector3.Distance(new Vector3((float)path.x[path.x.Count - 1], 0, (float)path.y[path.y.Count - 1]), new Vector3((float)current_x, 0, (float)current_y));
            if (distToGoal < nearGoalDist)
            {
                return true;
            }
            return false;
        }

        public static bool DirectionChangeAhead(Path path, int target_idx, int howManyWPsToLookAhead = 5)
        {
            var lookAheadIndex = target_idx + howManyWPsToLookAhead;
            if (lookAheadIndex <= path.x.Count - 1)
            {
                if (path.directions[lookAheadIndex] != path.directions[target_idx])
                {
                    return true;
                }
            }
            return false;
        }

    }

}
