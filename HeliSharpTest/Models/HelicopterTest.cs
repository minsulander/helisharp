using System;
using MathNet.Numerics.LinearAlgebra;
using NUnit.Framework;

namespace HeliSharp
{
    public abstract class HelicopterTest
    {

        public const double DT = 0.01;

        public void TrimSweep(Helicopter model, double ustart = -20, double uend = 60, double ustep = 1, double v = 0, double w = 0, double h = 1000)
        {
            model.Height = h;
            for (var u = ustart; u <= uend+ustep/2; u += ustep) {
                Console.WriteLine("Trimming u " + u.ToStr() + " m/s");
                model.AbsoluteVelocity = Vector<double>.Build.DenseOfArray(new double[] { u, v, w });
                model.AngularVelocity = Vector<double>.Build.Zero3();
                try {
                    model.TrimInit();
                    model.Trim();
                    LogTrimState(model);
                    AssertTrimmed(model);
                } catch (TrimmerException e) {
                    Console.WriteLine("  Failed: " + e.Message);
                }

            }
        }

        public void LogTrimState(Helicopter model)
        {
            Console.WriteLine("  col " + Math.Round(model.Collective*100)
                              + " lon " + Math.Round(model.LongCyclic*100)
                              + " lat " + Math.Round(model.LatCyclic*100)
                              + " ped " + Math.Round(model.Pedal*100)
                              + " roll " + (model.RollAngle * 180 / Math.PI).ToStr()
                              + " pitch " + (model.PitchAngle * 180 / Math.PI).ToStr());
        }

        public void AssertTrimmed(Helicopter model)
        {
            // Forces and moments should be near-zero
            Assert.IsTrue(model.Force.Norm (2) < 0.1);
            Assert.IsTrue(model.Torque.Norm (2) < 0.1);

            // Controls should be reasonable
            Assert.IsTrue(Math.Abs(model.Collective) < 0.5);
            Assert.IsTrue(Math.Abs(model.LongCyclic) < 0.5);
            Assert.IsTrue(Math.Abs(model.LatCyclic) < 0.5);
            Assert.IsTrue(Math.Abs(model.Pedal) < 0.5);

            // Attitude should be reasonable
            Assert.IsTrue(Math.Abs(model.RollAngle) < 10*Math.PI/180);
            Assert.IsTrue(Math.Abs(model.PitchAngle) < 10*Math.PI/180);
        }

        public abstract Helicopter SetupModelForSimulation();

        public RigidBody SetupRigidBody()
        {
            Helicopter model = SetupModelForSimulation();
            RigidBody body = new RigidBody
            {
                ForceModel = model,
                Mass = model.Mass,
                Inertia = model.Inertia
            };
            return body;

        }

        public double Simulate(RigidBody body, double duration) {
            for (double t = 0.0; t <= duration; t += DT) {
                body.Update(DT);
            }
            return duration;
        }

        public void LogSimulationState(RigidBody body)
        {
            Console.WriteLine("  F " + body.Force.ToStr());
            Console.WriteLine("  T " + body.Torque.ToStr());
            Console.WriteLine("  v " + body.Velocity.ToStr() + " p " + body.Position.ToStr());
            Console.WriteLine("  w " + body.AngularVelocity.ToStr() + " R " + (body.Rotation.EulerAngles() * 180.0 / Math.PI).ToStr());
        }

        [Test]
        public void ForwardCyclic_Response()
        {
            RigidBody body = SetupRigidBody();
            Helicopter model = (Helicopter) body.ForceModel;

            var trimPosition = model.Translation;
            var trimPitchAngle = model.PitchAngle;

            model.LongCyclic = 1.0;
            Simulate(body, 1.0);
            LogSimulationState(body);

            Assert.IsTrue(model.PitchAngle < trimPitchAngle);
            Assert.IsTrue(model.Translation.x() > trimPosition.x());
        }

        [Test]
        public void AftCyclic_Response()
        {
            RigidBody body = SetupRigidBody();
            Helicopter model = (Helicopter) body.ForceModel;

            var trimPosition = model.Translation;
            var trimPitchAngle = model.PitchAngle;

            model.LongCyclic = -1.0;
            Simulate(body, 1.0);
            LogSimulationState(body);

            Assert.IsTrue(model.PitchAngle > trimPitchAngle);
            Assert.IsTrue(model.Translation.x() < trimPosition.x());
        }

        [Test]
        public void RightCyclic_Response()
        {
            RigidBody body = SetupRigidBody();
            Helicopter model = (Helicopter) body.ForceModel;

            var trimPosition = model.Translation;
            var trimRollAngle = model.RollAngle;

            model.LatCyclic = 1.0;
            Simulate(body, 0.1);
            LogSimulationState(body);

            Assert.IsTrue(model.RollAngle > trimRollAngle);
            Assert.IsTrue(model.Translation.y() > trimPosition.y());
        }

        [Test]
        public void LeftCyclic_Response()
        {
            RigidBody body = SetupRigidBody();
            Helicopter model = (Helicopter) body.ForceModel;

            var trimPosition = model.Translation;
            var trimRollAngle = model.RollAngle;

            model.LatCyclic = -1.0;
            Simulate(body, 0.1);
            LogSimulationState(body);

            Assert.IsTrue(model.RollAngle < trimRollAngle);
            Assert.IsTrue(model.Translation.y() < trimPosition.y());
        }

        [Test]
        public void RightPedal_Response()
        {
            RigidBody body = SetupRigidBody();
            Helicopter model = (Helicopter) body.ForceModel;

            var trimPosition = model.Translation;
            var trimHeading = model.Heading;

            model.Pedal = 1.0;
            Simulate(body, 1.0);
            LogSimulationState(body);

            Assert.IsTrue(model.Heading > trimHeading);
            double distanceMoved = (model.Translation - trimPosition).Norm(2);
            Assert.IsTrue(distanceMoved < 3.0);
        }

        [Test]
        public void LeftPedal_Response()
        {
            RigidBody body = SetupRigidBody();
            Helicopter model = (Helicopter) body.ForceModel;

            var trimPosition = model.Translation;
            var trimHeading = model.Heading;

            model.Pedal = -1.0;
            Simulate(body, 1.0);
            LogSimulationState(body);

            Assert.IsTrue(model.Heading < trimHeading);
            double distanceMoved = (model.Translation - trimPosition).Norm(2);
            Assert.IsTrue(distanceMoved < 3.0);
        }

        [Test]
        public void UpCollective_Response()
        {
            RigidBody body = SetupRigidBody();
            Helicopter model = (Helicopter) body.ForceModel;

            var trimPosition = model.Translation;

            model.Collective = 1.0;
            Simulate(body, 1.0);
            LogSimulationState(body);

            Assert.IsTrue(model.Translation.z() < trimPosition.z());
            double horizontalDistanceMoved = (model.Translation.SubVector(0,2) - trimPosition.SubVector(0,2)).Norm(2);
            Assert.IsTrue(horizontalDistanceMoved < 1.0);
        }

        [Test]
        public void DownCollective_Response()
        {
            RigidBody body = SetupRigidBody();
            Helicopter model = (Helicopter) body.ForceModel;

            var trimPosition = model.Translation;

            model.Collective = -1.0;
            Simulate(body, 1.0);
            LogSimulationState(body);

            Assert.IsTrue(model.Translation.z() > trimPosition.z());
            double horizontalDistanceMoved = (model.Translation.SubVector(0,2) - trimPosition.SubVector(0,2)).Norm(2);
            Assert.IsTrue(horizontalDistanceMoved < 1.0);
        }

    }
}