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

            foreach (var rotor in Rotors) {
                rotor.LoadDefaultTailRotor();
            }
            Rotors[0].Translation = Vector<double>.Build.DenseOfArray(new double[] { 2, 2, -1 });
            Rotors[1].Translation = Vector<double>.Build.DenseOfArray(new double[] { -2, 2, -1 });
            Rotors[2].Translation = Vector<double>.Build.DenseOfArray(new double[] { -2, -2, -1 });
            Rotors[3].Translation = Vector<double>.Build.DenseOfArray(new double[] { 2, -2, -1 });
            Rotors[0].direction = Rotors[2].direction = Rotor.Direction.CounterClockwise;
            Rotors[1].direction = Rotors[3].direction = Rotor.Direction.Clockwise;

            FCS = new FlightControlSystem().LoadDefault();
            Engine = new Engine().LoadDefault();
            GearBox = new GearBox().LoadDefault();
            Fuselage = new Fuselage();
            return this;
        }

        public override void InitEngine(bool running) {
            UseEngineModel = true;
            GearBox.MainRotorRatio = Engine.designRPM / Rotors[0].designRPM;
            if (running) {
                foreach (var rotor in Rotors) {
                    rotor.RotSpeed = rotor.designOmega;
                }
                Engine.Init(600);
            } else
                Engine.InitStopped();
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
                if (rotor.direction == Rotor.Direction.CounterClockwise) rotor.Collective += FCS.Pedal;
                if (rotor.direction == Rotor.Direction.Clockwise) rotor.Collective -= FCS.Pedal;

                if (rotor.Collective > 1.0) rotor.Collective = 1.0;
                if (rotor.Collective < -1.0) rotor.Collective = -1.0;
            }

            // Update engine and drivetrain
            if (UseEngineModel) {
                if (Engine.phase == Engine.Phase.START && GearBox.autoBrakeRPM > 1e-5) GearBox.BrakeEnabled = false;
                GearBox.MainRotorLoad = 0;
                GearBox.MainRotorInertia = 0;
                foreach (var rotor in Rotors) {
                    if (!rotor.Enabled) continue;
                    GearBox.MainRotorLoad += Math.Abs(rotor.ShaftTorque);
                    GearBox.MainRotorInertia += rotor.Inertia;
                }
                GearBox.TailRotorLoad = 0;
                GearBox.TailRotorInertia = 0;
                GearBox.Update(dt);
                Engine.load = GearBox.Load;
                Engine.inertia = GearBox.Inertia;
                Engine.Update(dt);
                GearBox.RotspeedDrive = Engine.RotSpeed;
                foreach (var rotor in Rotors) {
                    rotor.RotSpeed = rotor.Enabled ? GearBox.MainRotorSpeed : 0;
                }
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
            bool engineModel = UseEngineModel;
            UseEngineModel = false;
            bool gravity = Gravity.Enabled;
            Gravity.Enabled = true;

            Vector<double> initialGuess = Vector<double>.Build.DenseOfArray(new double[] {
                Collective, LatCyclic, LongCyclic, Pedal, Attitude[0], Attitude[1]
            });

            try {
                Trimmer.Trim(
                    initialGuess,
                    Vector<double>.Build.DenseOfArray(new double[] { 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3 }),
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
                        return Vector<double>.Build.DenseOfArray(new double[] {
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
                    FCS.CollectiveInput = 0;
                    FCS.LongInput = 0;
                    FCS.LatInput = 0;
                    FCS.PedalInput = 0;
                }
            } catch (TrimmerException e) {
                Collective = initialGuess[0];
                LatCyclic = initialGuess[1];
                LongCyclic = initialGuess[2];
                Attitude = Vector<double>.Build.DenseOfArray(new double[] { initialGuess[3], initialGuess[4], Attitude[2] });
                throw e;
            } finally {
                FCS.enabled = fcs;
                foreach (var rotor in Rotors) rotor.useDynamicInflow = useDynamicInflow;
                UseEngineModel = engineModel;
                Gravity.Enabled = gravity;
            }
        }

    }
}