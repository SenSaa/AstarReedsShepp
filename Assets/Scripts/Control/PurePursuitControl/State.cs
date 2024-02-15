using System.Collections;
using System.Collections.Generic;
using System;

namespace PurePursuit
{

    public class State
    {

        public double x, y, yaw, v, delta, rear_x, rear_y, front_x, front_y;

        public State()
        {
            this.x = 0;
            this.y = 0;
            this.yaw = 0;
            this.delta = 0;
            this.v = 0;
            rear_x = 0;
            rear_y = 0;
            front_x = 0;
            front_y = 0;
        }

        public State(double x, double y, double yaw, double v)
        {
            this.x = x;
            this.y = y;
            this.yaw = yaw;
            this.v = v;
            rear_x = this.x - ((Params.WB / 2) * Math.Cos(this.yaw));
            rear_y = this.y - ((Params.WB / 2) * Math.Sin(this.yaw));
            front_x = this.x + ((Params.WB / 2) * Math.Cos(this.yaw));
            front_y = this.y + ((Params.WB / 2) * Math.Sin(this.yaw));
        }

        public void update(double a, double delta)
        {
            this.x += this.v * Math.Cos(this.yaw) * Params.dt;
            this.y += this.v * Math.Sin(this.yaw) * Params.dt;
            this.yaw += this.v / Params.WB * Math.Tan(delta) * Params.dt;
            this.v += a * Params.dt;
            this.rear_x = this.x - ((Params.WB / 2) * Math.Cos(this.yaw));
            this.rear_y = this.y - ((Params.WB / 2) * Math.Sin(this.yaw));
            front_x = this.x + ((Params.WB / 2) * Math.Cos(this.yaw));
            front_y = this.y + ((Params.WB / 2) * Math.Sin(this.yaw));
        }

        public double calc_distance(double point_x, double point_y)
        {
            double dx = this.rear_x - point_x;
            double dy = this.rear_y - point_y;
            return Helpers.Hypotenuse(dx, dy);
        }
        public double calc_distance_R(double point_x, double point_y)
        {
            double dx = this.front_x - point_x;
            double dy = this.front_y - point_y;
            return Helpers.Hypotenuse(dx, dy);
        }

    }

}
