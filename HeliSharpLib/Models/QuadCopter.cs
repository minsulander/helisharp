using System;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
    [Serializable]
    public class QuadCopter : Helicopter
    {
        public override Rotor[] Rotors { get; } = new Rotor[4];

        public QuadCopter()
        {
            for (var i = 0; i < 4; i++) Rotors[i] = new Rotor();
            SetModel("Rotor0", Rotors[0]);
            SetModel("Rotor1", Rotors[1]);
            SetModel("Rotor2", Rotors[2]);
            SetModel("Rotor3", Rotors[3]);
        }

        public override Helicopter LoadDefault()
        {
            Mass = 750;
            Inertia = Matrix<double>.Build.DenseOfDiagonalArray(3, 3, new double[] {1000, 3000, 3000});

            foreach (var rotor in Rotors)
            {
                rotor.LoadDefaultTailRotor();
            }
            Rotors[0].Translation = Vector<double>.Build.DenseOfArray(new double[] { 2, 2, -1 });
            Rotors[1].Translation = Vector<double>.Build.DenseOfArray(new double[] { -2, 2, -1 });
            Rotors[2].Translation = Vector<double>.Build.DenseOfArray(new double[] { -2, -2, -1 });
            Rotors[3].Translation = Vector<double>.Build.DenseOfArray(new double[] { 2, -2, -1 });
            Rotors[0].rotdir = Rotors[2].rotdir = 1;
            Rotors[1].rotdir = Rotors[3].rotdir = -1;

            FCS = new FlightControlSystem().LoadDefault();
            return this;
        }

        public override void PreUpdate(double dt)
        {
            // Update sub-model atmospheric properties
            foreach(var rotor in Rotors)
            {
                rotor.Density = Atmosphere.Density;
                rotor.HeightAboveGround = Height - rotor.Translation[2];
            }

            // Update sub-model controls
            foreach (var rotor in Rotors)
            {
                rotor.Collective = FCS.Collective;
                if (rotor.Translation.x() > 0) rotor.Collective -= FCS.LongCyclic;
                if (rotor.Translation.x() < 0) rotor.Collective += FCS.LongCyclic;
                if (rotor.Translation.y() > 0) rotor.Collective -= FCS.LatCyclic;
                if (rotor.Translation.y() < 0) rotor.Collective += FCS.LatCyclic;
                if (rotor.rotdir > 0) rotor.Collective += FCS.Pedal;
                if (rotor.rotdir < 0) rotor.Collective -= FCS.Pedal;

                if (rotor.Collective > 1.0) rotor.Collective = 1.0;
                if (rotor.Collective < -1.0) rotor.Collective = -1.0;
            }
        }

        public override void TrimInit()
        {
            foreach (var rotor in Rotors)
            {
                rotor.beta_0 = 3.0 * Math.PI / 180.0;
                rotor.beta_cos = 0.0;
                rotor.beta_sin = 0.0;
                rotor.RotSpeed = rotor.designOmega;

            }
            Collective = 0.5;
            LongCyclic = 0.0;
            LatCyclic = 0.0;
            Pedal = 0.0;
        }

        public override void Trim()
        {
            bool fcs = FCS.enabled;
            FCS.enabled = false;
            bool useDynamicInflow = Rotors[0].useDynamicInflow;
            foreach (var rotor in Rotors) rotor.useDynamicInflow = false;
            bool gravity = Gravity.Enabled;
            Gravity.Enabled = true;

            Vector<double> initialGuess = Vector<double>.Build.DenseOfArray(new double[] {
                Collective, LatCyclic, LongCyclic, Pedal, Attitude[0], Attitude[1]
            });

            new JacobianTrimmer().Trim(
                initialGuess,
                Vector<double>.Build.DenseOfArray (new double[] { 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3 }),
                delegate (Vector<double> x) {
                    // Set input variables from input vector, x=(t0 ts tc tp phi theta)
                    Collective = x[0];
                    LongCyclic = x[1];
                    LatCyclic = x[2];
                    Pedal = x[3];
                    Attitude = Vector<double>.Build.DenseOfArray(new double[] { x[4], x[5], Attitude[2] });
                    // Update
                    for (int i = 0; i < 10; i++) {
                        Update(0.01);
                    }
                    // Set output vector
                    return Vector<double>.Build.DenseOfArray (new double[] {
                        Force[0], Force[1], Force[2],
                        Torque[0], Torque[1], Torque[2]
                    });
                }
            );

            FCS.TrimCollective = Collective;
            FCS.TrimLongCyclic = LongCyclic;
            FCS.TrimLatCyclic = LatCyclic;
            FCS.TrimPedal = Pedal;
            FCS.TrimAttitude = Attitude.Clone();
            if (FCS.trimControl) {
                FCS.CollectiveCommand = 0;
                FCS.LongCommand = 0;
                FCS.LatCommand = 0;
                FCS.PedalCommand = 0;
            }
            FCS.enabled = fcs;
            foreach (var rotor in Rotors) rotor.useDynamicInflow = useDynamicInflow;
            Gravity.Enabled = gravity;
        }

    }
}