using NUnit.Framework;
using System;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;
using Newtonsoft.Json;

namespace HeliSharp
{
    [TestFixture]
    public class SingleMainRotorHelicopterTest : HelicopterTest
    {

        [Test]
        public void TrimTheModel ()
        {
            // Trimming means to find the inputs (collective, cyclic, pedal, attitude) that
            // results in equilibrium, i.e. outputs (forces, moments) are near zero
            SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) new SingleMainRotorHelicopter().LoadDefault();
            model.FCS.trimControl = false;
            model.TrimInit();
            model.Trim();

            LogTrimState(model);
            AssertTrimmed(model);

            // Sanity-check: the main-rotor should contribute to most of the upward force,
            // i.e. be somewhat equal to mass * gravity
            Assert.IsTrue(-model.MainRotor.Force[2] / (model.Mass * 9.81) > 0.9);

            Console.WriteLine("Wash velocity " + model.MainRotor.WashVelocity.ToStr());
            Console.WriteLine("Fuselage velocity " + model.Fuselage.Velocity.ToStr());
            Console.WriteLine("Horizontal stabilizer velocity " + model.HorizontalStabilizer.Velocity.ToStr());
            Console.WriteLine("Vertical stabilizer velocity " + model.VerticalStabilizer.Velocity.ToStr());
            Console.WriteLine("Velocity " + model.Velocity.ToStr());
        }

        [Test]
        public void TestTrimSweep()
        {
            SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) new SingleMainRotorHelicopter().LoadDefault();
            model.MainRotor.useDynamicInflow = false;
            model.TailRotor.useDynamicInflow = false;
            model.FCS.trimControl = false;
            TrimSweep(model);
        }

        [Test]
        public void TestStabilizersForceDirections()
        {
            SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) new SingleMainRotorHelicopter().LoadDefault();
            model.MainRotor.useDynamicInflow = false;
            model.TailRotor.useDynamicInflow = false;
            // Trim going downward and right
            model.AbsoluteVelocity = Vector<double>.Build.DenseOfArray(new double[] { 0, 5, 5 });
            model.AngularVelocity = Vector<double>.Build.Zero3();
            model.TrimInit();
            model.Trim();
            Console.WriteLine("H/S F " + model.HorizontalStabilizer.ForceContribution.ToStr() + " M " + model.HorizontalStabilizer.TorqueContribution.ToStr());
            Console.WriteLine("V/S F " + model.VerticalStabilizer.ForceContribution.ToStr() + " M " + model.VerticalStabilizer.TorqueContribution.ToStr());

            // Horizontal stabilizer should give nose-down pitching moment
            Assert.IsTrue(model.HorizontalStabilizer.TorqueContribution.y() < -100);
            // Vertical stabilizer should give nose-right yaw moment
            Assert.IsTrue(model.VerticalStabilizer.TorqueContribution.z() > 100);

            // Nose-right angular velocity should give nose-left yaw moment
            model.AbsoluteVelocity = Vector<double>.Build.Zero3();
            model.AngularVelocity = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 5 });
            model.Update(0.1);
            Console.WriteLine("A/V nose right V/S F " + model.VerticalStabilizer.ForceContribution.ToStr() + " M " + model.VerticalStabilizer.TorqueContribution.ToStr());
            Assert.IsTrue(model.VerticalStabilizer.TorqueContribution.z() < -100);

            // Nose-down angular velocity should give nose-up pitching moment
            model.AbsoluteVelocity = Vector<double>.Build.Zero3();
            model.AngularVelocity = Vector<double>.Build.DenseOfArray(new double[] { 0, -5, 0 });
            model.Update(0.1);
            Console.WriteLine("A/V nose down H/S F " + model.HorizontalStabilizer.ForceContribution.ToStr() + " M " + model.HorizontalStabilizer.TorqueContribution.ToStr());
            Assert.IsTrue(model.HorizontalStabilizer.TorqueContribution.y() > 100);
        }

        [Test]
        public void TestFuselageForceDirections()
        {
            SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) new SingleMainRotorHelicopter().LoadDefault();
            model.MainRotor.useDynamicInflow = false;
            model.TailRotor.useDynamicInflow = false;
            model.AbsoluteVelocity = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 0 });
            model.AngularVelocity = Vector<double>.Build.Zero3();
            model.TrimInit();
            model.Trim();

            // In a hover, the fuselage should mainly contribute with a down-force (due to main rotor downwash)
            Console.WriteLine("Fuselage hover F " + model.Fuselage.ForceContribution.ToStr() + " M " + model.Fuselage.TorqueContribution.ToStr());
            Assert.IsTrue(model.Fuselage.ForceContribution.z() > 100);
            Assert.IsTrue(Math.Abs(model.Fuselage.ForceContribution.z())/model.Fuselage.ForceContribution.Norm(2) > 0.9);

            // In forward flight, the fuselage should mainly contribute with drag
            model.AbsoluteVelocity = Vector<double>.Build.DenseOfArray(new double[] { 30, 0, 0 });
            model.Update(0.1);
            Console.WriteLine("Fuselage forward F " + model.Fuselage.ForceContribution.ToStr() + " M " + model.Fuselage.TorqueContribution.ToStr());
            Assert.IsTrue(model.Fuselage.ForceContribution.x() < -100);
            Assert.IsTrue(Math.Abs(model.Fuselage.ForceContribution.x())/model.Fuselage.ForceContribution.Norm(2) > 0.9);

        }

        [Test]
        public void TestSerialize()
        {
            SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) new SingleMainRotorHelicopter().LoadDefault();
            Console.WriteLine(model.Inertia[0,0]);

            // One way to skin a cat...
            /*
            var serializer = new JavaScriptSerializer();
            serializer.RegisterConverters(new JavaScriptConverter[] {
                // TODO custom serializer handling vectors and matrices.. ugh.
            });
            var json = serializer.Serialize(model);
            Console.WriteLine(json);
            model = serializer.Deserialize<SingleMainRotorHelicopter>(json);
            var json2 = serializer.Serialize(model);
            Console.WriteLine(json);
            Assert.AreEqual(json, json2);
            */

            // And another way...
            var json = Newtonsoft.Json.JsonConvert.SerializeObject(model, Formatting.Indented);
            Console.WriteLine(json);
            model = JsonConvert.DeserializeObject<SingleMainRotorHelicopter>(json);
            Assert.IsNotNull(model.Inertia[0, 0]);
            Console.WriteLine(model.Inertia[0,0]);
        }

        public override Helicopter SetupModelForSimulation()
        {
            SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) new SingleMainRotorHelicopter().LoadDefault();
            model.MainRotor.useDynamicInflow = false;
            model.TailRotor.useDynamicInflow = false;
            model.FCS.trimControl = false;
            model.TrimInit();
            model.Trim();
            return model;
        }
    }
}

