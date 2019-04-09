using System;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
    public class StaticForce : ForceModel
    {
        public StaticForce (Vector<double> F, Vector<double> M)
        {
            this.Force = F;
            this.Torque = M;
        }

        public StaticForce (Vector<double> F)
        {
            this.Force = F;
        }

        public override void Update(double dt)
        {
        }
    }
}

