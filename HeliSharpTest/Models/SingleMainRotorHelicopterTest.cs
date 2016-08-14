using NUnit.Framework;
using System;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;
using System.Web.Script.Serialization;
using Newtonsoft.Json;

namespace HeliSharp
{
	[TestFixture]
	public class SingleMainRotorHelicopterTest
	{

		[Test]
		public void TrimTheModel ()
		{
			// Trimming means to find the inputs (collective, cyclic, pedal, attitude) that
			// results in equilibrium, i.e. outputs (forces, moments) are near zero
			SingleMainRotorHelicopter model = new SingleMainRotorHelicopter().LoadDefault();
			model.MainRotor.useDynamicInflow = false;
			model.TailRotor.useDynamicInflow = false;
			model.TrimInit();
			model.Trim();

			Console.WriteLine("Force after trim " + model.Force.ToStr());
			Console.WriteLine("Torque after trim " + model.Torque.ToStr());

			Console.WriteLine("Collective " + Math.Round(model.Collective*100) + "%");
			Console.WriteLine("Longitudinal cyclic " + Math.Round(model.LongCyclic*100) + "%");
			Console.WriteLine("Lateral cyclic " + Math.Round(model.LatCyclic*100) + "%");
			Console.WriteLine("Pedal " + Math.Round(model.Pedal*100) + "%");
			Console.WriteLine("Roll angle " + (model.RollAngle * 180 / Math.PI).ToStr() + " deg");
			Console.WriteLine("Pitch angle " + (model.PitchAngle * 180 / Math.PI).ToStr() + " deg");

			// Forces and moments should be near-zero
			Assert.IsTrue(model.Force.Norm (2) < 0.1);
			Assert.IsTrue(model.Torque.Norm (2) < 0.1);

			// Controls should be reasonable
			Assert.IsTrue(model.Collective <= 1);
			Assert.IsTrue(model.Collective >= -1);
			Assert.IsTrue(model.LongCyclic <= 1);
			Assert.IsTrue(model.LongCyclic >= -1);
			Assert.IsTrue(model.LatCyclic <= 1);
			Assert.IsTrue(model.LatCyclic >= -1);
			Assert.IsTrue(model.Pedal <= 1);
			Assert.IsTrue(model.Pedal >= -1);

			// Attitude should be reasonable

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
			SingleMainRotorHelicopter model = new SingleMainRotorHelicopter().LoadDefault();
			model.MainRotor.useDynamicInflow = false;
			model.TailRotor.useDynamicInflow = false;
			model.FCS.trimControl = false;
			TrimSweep(model);
		}

		public void TrimSweep(SingleMainRotorHelicopter model, double ustart = -20, double uend = 60, double ustep = 1, double v = 0, double w = 0, double h = 1000)
		{
			model.Height = h;
			for (var u = ustart; u <= uend+ustep/2; u += ustep) {
				Console.WriteLine("Trimming u " + u.ToStr() + " m/s");
				model.AbsoluteVelocity = Vector<double>.Build.DenseOfArray(new double[] { u, v, w });
				model.AngularVelocity = Vector<double>.Build.Zero3();
				try {
					model.TrimInit();
					model.Trim();

					Console.WriteLine("  col " + Math.Round(model.Collective*100)
						+ " lon " + Math.Round(model.LongCyclic*100)
						+ " lat " + Math.Round(model.LatCyclic*100)
						+ " ped " + Math.Round(model.Pedal*100)
						+ " roll " + (model.RollAngle * 180 / Math.PI).ToStr()
						+ " pitch " + (model.PitchAngle * 180 / Math.PI).ToStr());
				} catch (TrimmerException e) {
					Console.WriteLine("  Failed: " + e.Message);
				}

			}
		}

		[Test]
		public void TestStabilizersForceDirections()
		{
			SingleMainRotorHelicopter model = new SingleMainRotorHelicopter().LoadDefault();
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
			SingleMainRotorHelicopter model = new SingleMainRotorHelicopter().LoadDefault();
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
			SingleMainRotorHelicopter model = new SingleMainRotorHelicopter().LoadDefault();

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
		}
	}
}

