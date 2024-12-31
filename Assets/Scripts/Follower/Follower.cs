using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BezierNavigator {
    namespace PathFollower {
        public class Follower
        {
            private Pose[] splinePts;
            private double look;
            private double t_Res;

            public Follower(Pose[] splinePts, float look, double t)
            {
                this.splinePts = splinePts;
                this.look = look;
                this.t_Res = t;
            }

            public Pose getTarget(Pose obj)
            {
                Pose vector = PursuitMath.getMovementVector1(obj, splinePts, look, t_Res);
                return new Pose(vector.x + obj.x, vector.y + obj.y);
            }
        }
    }
}