using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace PurePursuit
{

    public class Helpers
    {

        public static List<double> Hypotenuse(List<double> side1, List<double> side2)
        {
            List<double> hypot = new List<double>();
            for (int i = 0; i < side1.Count; i++)
            {
                hypot.Add(Math.Sqrt(Math.Pow(side1[i], 2) + Math.Pow(side2[i], 2)));
            }
            return hypot;
        }
        public static double Hypotenuse(double side1, double side2)
        {
            double hypot = 0;
            {
                hypot = Math.Sqrt(Math.Pow(side1, 2) + Math.Pow(side2, 2));
            }
            return hypot;
        }

    }

}
