using System;
using MathNet.Numerics.LinearAlgebra;
using Newtonsoft.Json;

namespace HeliSharp
{
    [Serializable]
    public class Fuselage : ForceModel
    {
        /// Fuselage / blunt body aerodynamic model.
        /// Based on standard drag model with equivalent flat-plate surface areas and curve-fit coefficients.
        /// A centre of pressure shift factor scaled with airspeed is also available (based on Heffley model).

        // Inputs
        [JsonIgnore]
        public double Density { get; set; }
        [JsonIgnore]
        public Vector<double> mrdistance;

        // Parameters
        public double // Curve-fit coefficients
            CXuu, CXvu, CXwu,
            CYuv, CYvv, CYwv,
            CZuw, CZvw, CZww,
            CLuu, CLww, CLup,
            CMuu, CMuw, CMuq,
            CNuv, CNur;
        public double 
            cpshift_xu, // centre of pressure shift scaling factor (Backlund model)
            cpshift_yv,
            cpshift_yu;

        public Fuselage ()
        {
            Density = 1.225;
        }

        public Fuselage LoadDefault()
        {
            CXuu = -1.00;
            CYvv = -15.5;
            CZww = -7.9;
            CLuu = 0.5;
            CLww = -10.0;
            cpshift_xu = 2.5;
            cpshift_yu = -1.0;
            return this;
        }

        public override void Update(double dt)
        {
            Vector<double> V = Velocity;
            Vector<double> W = AngularVelocity;
            // Sum up forces using curve-fit coefficients, Dreier eq (8.11)
            Vector<double> F = 0.5 * Density * Vector<double>.Build.DenseOfArray(new double[] {
                CXuu*V.x()*Math.Abs(V.x()) + CXvu*V.y()*V.x() + CXwu*V.z()*V.x(),
                CYuv*V.x()*V.y() + CYvv*V.y()*Math.Abs(V.y()) + CYwv*V.z()*V.y(),
                CZuw*V.x()*V.z() + CZvw*V.y()*V.z() + CZww*V.z()*Math.Abs(V.z())
            });
            Vector<double> M = 0.5 * Density * Vector<double>.Build.DenseOfArray(new double[] {
                CLuu*V.x()*Math.Abs(V.x()) + CLww*V.z()*Math.Abs(V.z()) + CLup*V.x()*W.x(),
                CMuu*V.x()*Math.Abs(V.x()) + CMuw*V.x()*V.z() + CMuq*V.x()*W.y(),
                CNuv*V.x()*V.y() + CNur*V.x()*W.z()
            });
            // Add shifting of center of pressure (Backlund thesis or Heffley et al (NASA CR177476))
            if (mrdistance != null && mrdistance.Norm(2) > 0 && Math.Abs(V.z()) > 0.1) { 
                double dxac = cpshift_xu * (-V.x() / V.z() * mrdistance.z() + mrdistance.x());
                double dyac = cpshift_yv * (-V.y() / V.z() * mrdistance.z() + mrdistance.y())
                    + cpshift_yu * (-V.x() / V.z() * mrdistance.z() + mrdistance.y());
                M += Vector<double>.Build.DenseOfArray( new double[] {F.z() * dyac, -F.z() * dxac, 0});
            }
            Force = F;
            Torque = M;
        }
    }
}

