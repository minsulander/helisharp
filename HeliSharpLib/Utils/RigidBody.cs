using System;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
    public class RigidBody
    {

        public double Mass {
            get { return M; }
            set { M = value; }
        }
        public Matrix<double> Inertia {
            get { return Iinv.Inverse(); }
            set { Iinv = value.Inverse(); }
        }
        public Vector<double> Position {
            get { return x; }
            set { x = value; }
        }
        public Vector<double> Quaternion {
            get { return q; }
            set { q = value; R = q.ToRotationMatrix(); }
        }
        public Matrix<double> Rotation {
            get { return R; }
            set { R = value; q = R.ToQuaternion(); }
        }
        public Vector<double> Velocity {
            get { return v; }
            set { v = value; }
        }
        public Vector<double> AngularVelocity {
            get { return w; }
            set { w = value; }
        }
        public Vector<double> Force {
            get { return F; }
            set { F = value; }
        }
        public Vector<double> Torque {
            get { return T; }
            set { T = value; }
        }


        public ForceModel ForceModel { get; set; }
        public ODESolver Solver { get; set; }

        // Following the nomenclature of http://www.cs.unc.edu/~lin/COMP768-F07/LEC/rbd1.pdf

        private double M = 1;
        private Matrix<double> Iinv = Matrix<double>.Build.DenseIdentity(3);

        private Vector<double> x = Vector<double>.Build.Dense(3); // position
        private Vector<double> q = Vector<double>.Build.Quaternion(); // rotation
        private Matrix<double> R = Matrix<double>.Build.DenseIdentity(3); // rotation matrix
        private Vector<double> P = Vector<double>.Build.Dense(3); // linear momentum
        private Vector<double> L = Vector<double>.Build.Dense(3); // angular momentum

        private Vector<double> v = Vector<double>.Build.Dense(3); // linear velocity
        private Vector<double> w = Vector<double>.Build.Dense(3); // angular velocity
        private Vector<double> F = Vector<double>.Build.Dense(3); // force
        private Vector<double> T = Vector<double>.Build.Dense(3); // torque

        public Vector<double> State()
        {
            // State vector = [ x q P L ]
            return Vector<double>.Build.DenseOfArray(new double[] { 
                x.x(), x.y(), x.z(),
                q.w(), q.x(), q.y(), q.z(),
                P.x(), P.y(), P.z(),
                L.x(), L.y(), L.z()
            });
        }

        public Vector<double> TimeDerivatives (double t, Vector<double> y)
        {
            // Time derivative vector = [ xdot qdot Pdot Ldot ]
            var xdot = v;

            //var w4 = Vector<double>.Build.DenseOfArray(new double[] { 0, w.x(), w.y(), w.z() });
            //var qdot = 0.5 * w4 * q;
            // http://www.euclideanspace.com/physics/kinematics/angularvelocity/QuaternionDifferentiation2.pdf
            var qdot = Vector<double>.Build.DenseOfArray(new double[] {
                -0.5*(w.x()*q.x()+w.y()*q.y()+w.z()*q.z()),
                0.5*(w.x()*q.w()+w.y()*q.z()-w.z()*q.y()),
                0.5*(w.y()*q.w()+w.z()*q.x()-w.x()*q.z()),
                0.5*(w.z()*q.w()+w.x()*q.y()-w.y()*q.x())
            });

            var Pdot = F;
            var Ldot = T;

            return Vector<double>.Build.DenseOfArray(new double[] {
                xdot.x(), xdot.y(), xdot.z(),
                qdot.w(), qdot.x(), qdot.y(), qdot.z(),
                Pdot.x(), Pdot.y(), Pdot.z(),
                Ldot.x(), Ldot.y(), Ldot.z()
            });
        }

        public void ApplyState(Vector<double> y)
        {
            x[0] = y[0]; x[1] = y[1]; x[2] = y[2];
            q[0] = y[3]; q[1] = y[4]; q[2] = y[5]; q[3] = y[6];
            P[0] = y[7]; P[1] = y[8]; P[2] = y[9];
            L[0] = y[10]; L[1] = y[11]; L[2] = y[12];
        }

        public void Update(double dt)
        {
            ForceModel.Update(dt);

            v = P / M;
            w = Iinv * L;
            F = R * ForceModel.Force;
            T = R * ForceModel.Torque;

            if (Solver == null)
                Solver = new RK4Solver(TimeDerivatives, 0, State());

            Solver.Step(dt);
            ApplyState(Solver.State);

            R = q.ToRotationMatrix();
            var Rinv = R.Inverse();

            ForceModel.Translation = x;
            ForceModel.Rotation = R;
            ForceModel.Velocity = Rinv*v;
            ForceModel.AngularVelocity = Rinv*w;
        }
    }
}

