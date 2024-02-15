using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using UnityEngine;
using UtilityFunctions;
using Accord.Math;
using MathNet.Numerics;
using System.Reflection;

namespace ReedsShepp
{

    // Reeds Shepp path planner
    // Based on:
    // https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py
    // author Atsushi Sakai(@Atsushi_twi)

    public class ReedsSheppPath
    {

        private Tuple<bool, double, double, double> straight_left_straight(double x, double y, double phi)
        {
            try
            {
                phi = Utils.Mod2pi(phi);

                if (y > 0.0 && 0.0 < phi && phi < Math.PI * 0.99)
                {
                    double tanPhi = phi;  // Approximation for small angles
                    double xd = x - tanPhi * y;
                    double t = xd - tanPhi / 2.0;
                    double u = phi;
                    double v = Math.Sqrt(Math.Pow((x - xd), 2) + Math.Pow(y, 2)) - tanPhi / 2.0;

                    return new Tuple<bool, double, double, double>(true, t, u, v);
                }
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }

            return new Tuple<bool, double, double, double>(false, 0.0, 0.0, 0.0);
        }

        private List<Path> set_path(List<Path> paths, List<double> lengths, List<string> ctypes, double step_size, string pathType)
        {

            Path path = new Path();
            path.ctypes = ctypes;
            path.lengths = lengths;
            path.L = lengths.Select(Math.Abs).Sum();

            // check same path exist
            foreach (Path i_path in paths)
            {
                bool type_is_same = Utils.EqualPathTypes(i_path.ctypes, path.ctypes);
                string i_path_ctypes = i_path.ctypes[0] + "," + i_path.ctypes[1] + "," + i_path.ctypes[2];
                string path_ctypes = path.ctypes[0] + "," + path.ctypes[1] + "," + path.ctypes[2];
                List<double> absOf_i_Lengths = i_path.lengths.Select(l => Math.Abs(l)).ToList();
                double sumOf_i_Lengths = absOf_i_Lengths.Sum(l => l);
                bool length_is_close = (sumOf_i_Lengths - path.L) <= step_size;
                if (type_is_same && length_is_close)
                {
                    return paths;  // same path found, so do not insert path
                }
            }

            if (path.L <= step_size)
            {
                return paths; // too short, so do not insert path
            }
            paths.Add(path);
            return paths;
        }

        public List<Path> straight_curve_straight(double x, double y, double phi, List<Path> paths, double step_size)
        {
            var _tup_1 = straight_left_straight(x, y, phi);
            var flag = _tup_1.Item1;
            var t = _tup_1.Item2;
            var u = _tup_1.Item3;
            var v = _tup_1.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    t,
                    u,
                    v
                }, new List<string> {
                    "S",
                    "L",
                    "S"
                }, step_size, "SCS");
            }
            var _tup_2 = straight_left_straight(x, -y, -phi);
            flag = _tup_2.Item1;
            t = _tup_2.Item2;
            u = _tup_2.Item3;
            v = _tup_2.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    t,
                    u,
                    v
                }, new List<string> {
                    "S",
                    "R",
                    "S"
                }, step_size, "SCS");
            }
            return paths;
        }

        private Tuple<double, double> polar(double x, double y)
        {
            double r = Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2));
            double theta = Math.Atan2(y, x);
            return new Tuple<double, double>(r, theta);
        }

        private Tuple<bool, double, double, double> left_straight_left(double x, double y, double phi)
        {
            try
            {
                Tuple<double, double> u_t = polar(x - Math.Sin(phi), y - 1.0 + Math.Cos(phi));
                if (u_t.Item2 >= 0.0)
                {
                    double v = Utils.Mod2pi(phi - u_t.Item2);
                    if (v >= 0.0)
                    {
                        return new Tuple<bool, double, double, double>(true, u_t.Item2, u_t.Item1, v);
                    }
                }
            }
            catch (Exception e) { Debug.LogError(e); }
            return new Tuple<bool, double, double, double>(false, 0.0, 0.0, 0.0);
        }

        private Tuple<bool, double, double, double> left_right_left(double x, double y, double phi)
        {
            try
            {
                Tuple<double, double> u1_t1 = polar(x - Math.Sin(phi), y - 1.0 + Math.Cos(phi));

                if (u1_t1.Item1 <= 4.0)
                {
                    double u = -2.0 * Math.Asin(0.25 * u1_t1.Item1);
                    double t = Utils.Mod2pi(u1_t1.Item2 + 0.5 * u + Math.PI);
                    double v = Utils.Mod2pi(phi - t + u);

                    if (t >= 0.0 && 0.0 >= u)
                    {
                        return new Tuple<bool, double, double, double>(true, t, u, v);
                    }
                }
            }
            catch (Exception e) { Debug.LogError(e); }
            return new Tuple<bool, double, double, double>(false, 0.0, 0.0, 0.0);
        }

        public List<Path> curve_curve_curve(double x, double y, double phi, List<Path> paths, double step_size)
        {
            var _tup_1 = left_right_left(x, y, phi);
            var flag = _tup_1.Item1;
            var t = _tup_1.Item2;
            var u = _tup_1.Item3;
            var v = _tup_1.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    t,
                    u,
                    v
                }, new List<string> {
                    "L",
                    "R",
                    "L"
                }, step_size, "CCC");
            }
            var _tup_2 = left_right_left(-x, y, -phi);
            flag = _tup_2.Item1;
            t = _tup_2.Item2;
            u = _tup_2.Item3;
            v = _tup_2.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    -t,
                    -u,
                    -v
                }, new List<string> {
                    "L",
                    "R",
                    "L"
                }, step_size, "CCC");
            }
            var _tup_3 = left_right_left(x, -y, -phi);
            flag = _tup_3.Item1;
            t = _tup_3.Item2;
            u = _tup_3.Item3;
            v = _tup_3.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    t,
                    u,
                    v
                }, new List<string> {
                    "R",
                    "L",
                    "R"
                }, step_size, "CCC");
            }
            var _tup_4 = left_right_left(-x, -y, phi);
            flag = _tup_4.Item1;
            t = _tup_4.Item2;
            u = _tup_4.Item3;
            v = _tup_4.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    -t,
                    -u,
                    -v
                }, new List<string> {
                    "R",
                    "L",
                    "R"
                }, step_size, "CCC");
            }
            // backwards
            var xb = x * Math.Cos(phi) + y * Math.Sin(phi);
            var yb = x * Math.Sin(phi) - y * Math.Cos(phi);
            var _tup_5 = left_right_left(xb, yb, phi);
            flag = _tup_5.Item1;
            t = _tup_5.Item2;
            u = _tup_5.Item3;
            v = _tup_5.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    v,
                    u,
                    t
                }, new List<string> {
                    "L",
                    "R",
                    "L"
                }, step_size, "CCC");
            }
            var _tup_6 = left_right_left(-xb, yb, -phi);
            flag = _tup_6.Item1;
            t = _tup_6.Item2;
            u = _tup_6.Item3;
            v = _tup_6.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    -v,
                    -u,
                    -t
                }, new List<string> {
                    "L",
                    "R",
                    "L"
                }, step_size, "CCC");
            }
            var _tup_7 = left_right_left(xb, -yb, -phi);
            flag = _tup_7.Item1;
            t = _tup_7.Item2;
            u = _tup_7.Item3;
            v = _tup_7.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    v,
                    u,
                    t
                }, new List<string> {
                    "R",
                    "L",
                    "R"
                }, step_size, "CCC");
            }
            var _tup_8 = left_right_left(-xb, -yb, phi);
            flag = _tup_8.Item1;
            t = _tup_8.Item2;
            u = _tup_8.Item3;
            v = _tup_8.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    -v,
                    -u,
                    -t
                }, new List<string> {
                    "R",
                    "L",
                    "R"
                }, step_size, "CCC");
            }
            return paths;
        }

        public List<Path> curve_straight_curve(
            double x, double y, double phi, List<Path> paths, double step_size)
        {
            var _tup_1 = left_straight_left(x, y, phi);
            var flag = _tup_1.Item1;
            var t = _tup_1.Item2;
            var u = _tup_1.Item3;
            var v = _tup_1.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    t,
                    u,
                    v
                }, new List<string> {
                    "L",
                    "S",
                    "L"
                }, step_size, "CSC");
            }
            var _tup_2 = left_straight_left(-x, y, -phi);
            flag = _tup_2.Item1;
            t = _tup_2.Item2;
            u = _tup_2.Item3;
            v = _tup_2.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    -t,
                    -u,
                    -v
                }, new List<string> {
                    "L",
                    "S",
                    "L"
                }, step_size, "CSC");
            }
            var _tup_3 = left_straight_left(x, -y, -phi);
            flag = _tup_3.Item1;
            t = _tup_3.Item2;
            u = _tup_3.Item3;
            v = _tup_3.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    t,
                    u,
                    v
                }, new List<string> {
                    "R",
                    "S",
                    "R"
                }, step_size, "CSC");
            }
            var _tup_4 = left_straight_left(-x, -y, phi);
            flag = _tup_4.Item1;
            t = _tup_4.Item2;
            u = _tup_4.Item3;
            v = _tup_4.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    -t,
                    -u,
                    -v
                }, new List<string> {
                    "R",
                    "S",
                    "R"
                }, step_size, "CSC");
            }
            var _tup_5 = left_straight_right(x, y, phi);
            flag = _tup_5.Item1;
            t = _tup_5.Item2;
            u = _tup_5.Item3;
            v = _tup_5.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    t,
                    u,
                    v
                }, new List<string> {
                    "L",
                    "S",
                    "R"
                }, step_size, "CSC");
            }
            var _tup_6 = left_straight_right(-x, y, -phi);
            flag = _tup_6.Item1;
            t = _tup_6.Item2;
            u = _tup_6.Item3;
            v = _tup_6.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    -t,
                    -u,
                    -v
                }, new List<string> {
                    "L",
                    "S",
                    "R"
                }, step_size, "CSC");
            }
            var _tup_7 = left_straight_right(x, -y, -phi);
            flag = _tup_7.Item1;
            t = _tup_7.Item2;
            u = _tup_7.Item3;
            v = _tup_7.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    t,
                    u,
                    v
                }, new List<string> {
                    "R",
                    "S",
                    "L"
                }, step_size, "CSC");
            }
            var _tup_8 = left_straight_right(-x, -y, phi);
            flag = _tup_8.Item1;
            t = _tup_8.Item2;
            u = _tup_8.Item3;
            v = _tup_8.Item4;
            if (flag)
            {
                paths = set_path(paths, new List<double> {
                    -t,
                    -u,
                    -v
                }, new List<string> {
                    "R",
                    "S",
                    "L"
                }, step_size, "CSC");
            }
            return paths;
        }

        private Tuple<bool, double, double, double> left_straight_right(double x, double y, double phi)
        {
            Tuple<double, double> u1_t1 = polar(x + Math.Sin(phi), y - 1.0 - Math.Cos(phi));
            double u1 = u1_t1.Item1;
            double t1 = u1_t1.Item2;
            u1 = Math.Pow(u1, 2);
            if (u1 >= 4.0)
            {
                double u = Math.Sqrt(u1 - 4.0);
                double theta = Math.Atan2(2.0, u);
                double t = Utils.Mod2pi(t1 + theta);
                double v =  Utils.Mod2pi(t - phi);

                if (t >= 0.0 && v >= 0.0)
                {
                    return new Tuple<bool, double, double, double>(true, t, u, v);
                }
            }
            return new Tuple<bool, double, double, double>(false, 0.0, 0.0, 0.0);
        }

        public List<Path> generate_path(List<double> q0, List<double> q1, double max_curvature, double step_size)
        {
            var dx = q1[0] - q0[0];
            var dy = q1[1] - q0[1];
            var dth = q1[2] - q0[2];
            var c = Math.Cos(q0[2]);
            var s = Math.Sin(q0[2]);
            var x = (c * dx + s * dy) * max_curvature;
            var y = (-s * dx + c * dy) * max_curvature;
            var paths_empty = new List<Path>();
            var paths_SCS = straight_curve_straight(x, y, dth, paths_empty, step_size);
            var paths_CSC = curve_straight_curve(x, y, dth, paths_SCS, step_size);
            var paths = curve_curve_curve(x, y, dth, paths_CSC, step_size);
            return paths;
        }

        private List<List<double>> calc_interpolate_dists_list(List<double> lengths, double step_size)
        {
            List<List<double>> interpolate_dists_list = new List<List<double>>();
            List<double> interp_dists_debug = new List<double>();
            foreach (double length in lengths)
            {
                try
                {
                    double d_dist = length >= 0.0 ? step_size : -step_size;
                    var interp_dists = Generate.LinearRange(0.0, d_dist, length); 
                    List<double> temp_interp_dists = new List<double>(interp_dists);
                    temp_interp_dists.Add(length);
                    interpolate_dists_list.Add(temp_interp_dists);
                }
                catch(Exception e) 
                { 
                    Debug.LogError(e); 
                }
            }
            return interpolate_dists_list;
        }

        private Tuple<List<double>, List<double>, List<double>, List<int>> generate_local_course(List<double> lengths, List<string> modes, double max_curvature, double step_size)
        {
            List<List<double>> interpolate_dists_list = calc_interpolate_dists_list(lengths, step_size);

            double origin_x = 0.0;
            double origin_y = 0.0;
            double origin_yaw = 0.0;
            List<double> xs = new List<double>();
            List<double> ys = new List<double>();
            List<double> yaws = new List<double>();
            List<int> directions = new List<int>();

            var interpolate_dists_list_modes_lengths = interpolate_dists_list.Zip(modes, (a, b) => new
            {
                interp_dists = a,
                mode = b,
            })
            .Zip(lengths, (a, b) => new
            {
            // use the properties from a & b to construct your desired result
            interp_dists = a.interp_dists,
                mode = a.mode,
                length = b,
            });

            foreach (var interp_dists_mode_length in interpolate_dists_list_modes_lengths)
            {
                var interp_dists = interp_dists_mode_length.interp_dists;
                var mode = interp_dists_mode_length.mode;
                var length = interp_dists_mode_length.length;
                string dist_Str = "";
                foreach (var dist in interp_dists)
                {
                    dist_Str += dist + ", ";
                    
                    Tuple<double, double, double, int> x_y_yaw_direction = interpolate(dist, length, mode, max_curvature, origin_x, origin_y, origin_yaw);
                    xs.Add(x_y_yaw_direction.Item1);
                    ys.Add(x_y_yaw_direction.Item2);
                    yaws.Add(x_y_yaw_direction.Item3);
                    directions.Add(x_y_yaw_direction.Item4);
                }
                origin_x = xs[xs.Count - 1];
                origin_y = ys[ys.Count - 1];
                origin_yaw = yaws[yaws.Count - 1];
            }

            return new Tuple<List<double>, List<double>, List<double>, List<int>>(xs, ys, yaws, directions);
        }

        public Tuple<double, double, double, int> interpolate(
            double dist, 
            double length, 
            string mode, 
            double max_curvature, 
            double origin_x, 
            double origin_y, 
            double origin_yaw)
        {
            double yaw;
            double y;
            double x;
            if (mode == "S")
            {
                x = origin_x + dist / max_curvature * Math.Cos(origin_yaw);
                y = origin_y + dist / max_curvature * Math.Sin(origin_yaw);
                yaw = origin_yaw;
            }
            else
            {
                // curve
                var ldx = Math.Sin(dist) / max_curvature;
                var ldy = 0.0;
                yaw = 0;
                if (mode == "L")
                {
                    // left turn
                    ldy = (1.0 - Math.Cos(dist)) / max_curvature;
                    yaw = origin_yaw + dist;
                }
                else if (mode == "R")
                {
                    // right turn
                    ldy = (1.0 - Math.Cos(dist)) / -max_curvature;
                    yaw = origin_yaw - dist;
                }
                var gdx = Math.Cos(-origin_yaw) * ldx + Math.Sin(-origin_yaw) * ldy;
                var gdy = -Math.Sin(-origin_yaw) * ldx + Math.Cos(-origin_yaw) * ldy;
                x = origin_x + gdx;
                y = origin_y + gdy;
            }
            return Tuple.Create(x, y, yaw, length > 0.0 ? 1 : -1);
        }

        public List<Path> calc_paths(double sx, double sy, double syaw, double gx, double gy, double gyaw, double maxc, double step_size)
        {
            List<double> q0 = new List<double> { sx, sy, syaw };
            List<double> q1 = new List<double> { gx, gy, gyaw };

            List<Path> paths = generate_path(q0, q1, maxc, step_size);
            foreach (Path path in paths)
            {
                Tuple<List<double>, List<double>, List<double>, List<int>> xs_ys_yaws_directions = generate_local_course(path.lengths, path.ctypes, maxc, step_size * maxc);
                List<double> xs = xs_ys_yaws_directions.Item1;
                List<double> ys = xs_ys_yaws_directions.Item2;
                List<double> yaws = xs_ys_yaws_directions.Item3;
                List<int> directions = xs_ys_yaws_directions.Item4;

                // convert global coordinate
                var xs_ys = xs.Zip(ys, (a, b) => new { ix = a, iy = b });
                foreach (var ix_iy in xs_ys)
                {
                    path.x.Add(Math.Cos(-q0[2]) * ix_iy.ix + Math.Sin(-q0[2]) * ix_iy.iy + q0[0]);
                }
                foreach (var ix_iy in xs_ys)
                {
                    path.y.Add(-Math.Sin(-q0[2]) * ix_iy.ix + Math.Cos(-q0[2]) * ix_iy.iy + q0[1]);
                }
                foreach (var yaw in yaws)
                {
                    path.yaw.Add(Utils.NormaliseAngle(yaw + q0[2]));
                }
                path.directions = directions;
                foreach (var length in path.lengths.ToList())
                {
                    path.lengths.Add(length / maxc);
                }

                path.L /= maxc;
            }

            return paths;
        }

        public Tuple<List<double>, List<double>, List<double>, List<string>, List<double>, List<int>> reeds_shepp_path_planning(double sx, double sy, double syaw, double gx, double gy, double gyaw, double maxc, double step_size = 0.5)
        {
            List<Path> paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size);
            if (paths == null || paths.Count < 1)
            {
                return new Tuple<List<double>, List<double>, List<double>, List<string>, List<double>, List<int>>(null, null, null, null, null, null); // could not generate any path
            }
            // search minimum cost path
            var minLengthPath = Utils.FindMinLengthPath(paths);
            int best_path_index = paths.IndexOf(minLengthPath);
            Path b_path = paths[best_path_index];

            return new Tuple<List<double>, List<double>, List<double>, List<string>, List<double>, List<int>>(b_path.x, b_path.y, b_path.yaw, b_path.ctypes, b_path.lengths, b_path.directions);
        }

    }
}
