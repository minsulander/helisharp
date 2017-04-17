using NUnit.Framework;
using System;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;
using System.Web.Script.Serialization;
using Newtonsoft.Json;

namespace HeliSharp
{
	[TestFixture]
	public class RotorTest
	{
		[Test]
		public void TrimStep ()
		{
			Rotor rotor = new Rotor ();
			rotor.LoadDefault ();
			double mass = 2450;
			double CTguess = mass*9.81/rotor.RhoAOR2;
			rotor.CT = CTguess;
			Console.WriteLine (rotor.CT);
			rotor.beta_0 = 3.0 * Math.PI / 180.0;
			rotor.beta_cos = 0.0;
			rotor.beta_sin = 0.0;
			rotor.Collective = 0.5;
			rotor.LongCyclic = 0.0;
			rotor.LatCyclic = 0.0;

			Simulate(rotor, 0.1, 0.01);
		}

		[Test]
		public void TailRotorTrimStep ()
		{
			Rotor rotor = new Rotor ().LoadDefaultTailRotor();
			rotor.CT = 0.005;
			rotor.beta_0 = 3.0 * Math.PI / 180.0;
			rotor.Collective = 0.5;
			rotor.LongCyclic = 0.0;
			rotor.LatCyclic = 0.0;
			Simulate(rotor, 0.1, 0.01);
		}

		[Test]
		public void RotorCollectiveSanityChecks ()
		{
			Rotor rotor = new Rotor ().LoadDefault ();
			rotor.useDynamicInflow = false;
			rotor.Collective = 0;
			Simulate (rotor, 1.0, 0.1);
			double thrust0 = rotor.Force [2];
			double torque0 = rotor.Torque [2];
			double coning0 = rotor.beta_0;

			// Increase collective

			rotor.Collective = 1.0;
			Simulate (rotor, 1.0, 0.1);
			double thrust1 = rotor.Force [2];
			double torque1 = rotor.Torque [2];
			double coning1 = rotor.beta_0;

			// Thrust, moment and coning should increase

			Assert.IsTrue (Math.Abs (thrust1) > Math.Abs (thrust0));
			Assert.IsTrue (Math.Abs (torque1) > Math.Abs (torque0));
			Assert.IsTrue (Math.Abs (coning1) > Math.Abs (coning0));

			// Move upward

			rotor.Velocity [2] = -10;
			Simulate (rotor, 1.0, 0.1);
			double thrust2 = rotor.Force [2];
			double torque2 = rotor.Torque [2];
			double coning2 = rotor.beta_0;

			// Thrust and coning should decrease, moment pretty much the same

			Assert.IsTrue (Math.Abs (thrust2) < Math.Abs (thrust1));
			Assert.IsTrue (Math.Abs (coning2) < Math.Abs (coning1));
			double torquePercentChange = Math.Abs ((1 - torque2 / torque1) * 100);
			Assert.IsTrue (torquePercentChange < 10);
		}

		[Test]
		public void RotorFlappingSanityChecks ()
		{
			// Sanity checks as specified by Dreier 11.2 (pg170)

			// 1) At hover, a one-to-one correspondence exists between longitudinal flapping and longitudinal cyclic.
			//    This is the so-called equivalence of flapping and feathering.

			Rotor rotor = new Rotor ().LoadDefault ();
			rotor.useDynamicInflow = false;
			Simulate (rotor, 1.0, 0.1);
			double beta_cos_hover = rotor.beta_cos;
			Vector<double> F_hover = rotor.Force.Clone();
			Vector<double> M_hover = rotor.Torque.Clone();

			rotor.LongCyclic = -1.0;
			Simulate (rotor, 1.0, 0.1);
			double beta_cos_hover_cyclic = rotor.beta_cos;
			Console.WriteLine ("Hover Long Before " + (beta_cos_hover*180.0/Math.PI).ToStr() + " after " + (beta_cos_hover_cyclic*180.0/Math.PI).ToStr());
			Assert.IsTrue (Math.Abs(beta_cos_hover_cyclic) > Math.Abs(beta_cos_hover));
			Console.WriteLine("Hover Long F Before " + F_hover.ToStr() + " after " + rotor.Force.ToStr());
			Console.WriteLine("Hover Long M Before " + M_hover.ToStr() + " after " + rotor.ShaftTorque.ToStr());

			// 2) As airspeed increases, the longitudinal flapping-to-feathering ratio is amplified.

			rotor = new Rotor ().LoadDefault ();
			rotor.useDynamicInflow = false;
			rotor.Velocity [0] = 30;
			Simulate (rotor, 1.0, 0.1);
			double beta_cos_forward = rotor.beta_cos;

			rotor.LongCyclic = -1.0;
			Simulate (rotor, 1.0, 0.1);
			double beta_cos_forward_cyclic = rotor.beta_cos;
			Console.WriteLine ("Forward Long Before " + (beta_cos_forward*180.0/Math.PI).ToStr() + " after " + (beta_cos_forward_cyclic*180.0/Math.PI).ToStr());
			Assert.IsTrue (Math.Abs(beta_cos_forward_cyclic) > Math.Abs(beta_cos_forward));
			Assert.IsTrue (Math.Abs (beta_cos_forward_cyclic) > Math.Abs (beta_cos_hover_cyclic));

			// 3) Forward flight causes aft flapping

			Assert.IsTrue (beta_cos_forward < 0);
			Assert.IsTrue (beta_cos_forward < beta_cos_hover);

			// 4) Lateral flapping and lateral cyclic are in one-to-one correspondence over the entire velocity range

			rotor = new Rotor ().LoadDefault ();
			rotor.useDynamicInflow = false;
			Simulate (rotor, 1.0, 0.1);
			double beta_sin_hover = rotor.beta_sin;
			F_hover = rotor.Force.Clone();
			M_hover = rotor.Torque.Clone();

			rotor.LatCyclic = 1.0;
			Simulate (rotor, 1.0, 0.1);
			double beta_sin_hover_cyclic = rotor.beta_sin;

			Console.WriteLine ("Hover Lat Before " + (beta_sin_hover*180.0/Math.PI).ToStr() + " after " + (beta_sin_hover_cyclic*180.0/Math.PI).ToStr());
			Assert.IsTrue (Math.Abs (beta_sin_hover_cyclic) > Math.Abs (beta_sin_hover));
			Console.WriteLine("Hover Lat F Before " + F_hover.y().ToStr() + " after " + rotor.Force.y().ToStr());
			Console.WriteLine("Hover Lat M Before " + M_hover.x().ToStr() + " after " + rotor.Torque.x().ToStr());

			rotor = new Rotor ().LoadDefault ();
			rotor.useDynamicInflow = false;
			rotor.Velocity [0] = 30;
			Simulate (rotor, 1.0, 0.1);
			double beta_sin_forward = rotor.beta_sin;

			rotor.LatCyclic = 1.0;
			Simulate (rotor, 1.0, 0.1);
			double beta_sin_forward_cyclic = rotor.beta_sin;

			Console.WriteLine ("Forward Lat Before " + (beta_sin_forward*180.0/Math.PI).ToStr() + " after " + (beta_sin_forward_cyclic*180.0/Math.PI).ToStr());
			Assert.IsTrue (Math.Abs (beta_sin_forward_cyclic) > Math.Abs (beta_sin_forward));
		}

		[Test]
		public void TestDynamicInflow ()
		{
			Rotor rotor = new Rotor ().LoadDefault ();
			rotor.useDynamicInflow = true;
			rotor.trimflow = true;

			rotor.Collective = 0.25;
			rotor.Update (0.1);
			rotor.Collective = 1.0;

			// Simulate a bit
			Simulate (rotor, 0.05, 0.02);

			double thrust0 = rotor.Force [2];
			double torque0 = rotor.Torque [2];
			double coning0 = rotor.beta_0;

			Console.WriteLine ("---");

			// Simulate a bit further
			Simulate (rotor, 0.26, 0.02);

			double thrust1 = rotor.Force [2];
			double torque1 = rotor.Torque [2];
			double coning1 = rotor.beta_0;

			// Thrust and coning should initially be larger and then decrease due to inflow buildup
			// The opposite goes for moment

			Assert.IsTrue (Math.Abs (thrust1) < Math.Abs (thrust0));
			Assert.IsTrue (Math.Abs (torque1) > Math.Abs (torque0));
			Assert.IsTrue (Math.Abs (coning1) < Math.Abs (coning0));

		}


		private void Simulate(Rotor rotor, double time, double step) {
			for (double t = step; t < time; t += step) {
				rotor.Update (step);
				double theta_0, theta_sin, theta_cos;
				rotor.GetControlAngles (out theta_0, out theta_sin, out theta_cos);
				Console.WriteLine (t + "\tF " + rotor.Force.ToStr() + "\tM " + rotor.Torque.ToStr() +
				                   "\tb0 " + (rotor.beta_0*180.0/Math.PI).ToStr() + "\tbs " + (rotor.beta_sin*180.0/Math.PI).ToStr() + "\tbc " + (rotor.beta_cos*180.0/Math.PI).ToStr() +
				                   "\tt0 " + (theta_0*180.0/Math.PI).ToStr() + "\tts " + (theta_sin*180.0/Math.PI).ToStr() + "\ttc " + (theta_cos*180.0/Math.PI).ToStr());
			}
		}

		[Test]
		public void TestControlAngles ()
		{
			Rotor rotor = new Rotor ();
			rotor.minCollective = 2;
			rotor.maxCollective = 8;
			rotor.minLongCyclic = -10;
			rotor.maxLongCyclic = 12;
			rotor.minLatCyclic = -8;
			rotor.maxLatCyclic = 8;

			double t0, ts, tc;
			rotor.GetControlAnglesFromNormalized (0, 0, 0, out t0, out ts, out tc);
			Assert.AreEqual (5 * Math.PI / 180, t0, 0.001);
			Assert.AreEqual (1 * Math.PI / 180, ts, 0.001);
			Assert.AreEqual (0, tc);

		}

		[Test]
		public void TestSerialize()
		{
			Rotor rotor = new Rotor().LoadDefault();

		    /*
			// One way to skin a cat...
			var serializer = new JavaScriptSerializer();
            var json = serializer.Serialize(rotor);
            Console.WriteLine(json);
            rotor = serializer.Deserialize<Rotor>(json);
            */

		    // And another way...
            var json = Newtonsoft.Json.JsonConvert.SerializeObject(rotor, Formatting.Indented);
            Console.WriteLine(json);
		}

	}
}

