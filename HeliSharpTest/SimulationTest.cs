using System;
using NUnit.Framework;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
	[TestFixture]
	public class SimulationTest
	{

		public const double DT = 0.01;

		[Test]
		public void Simulate()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			double time = 0;
			time += Simulate(body, 1.0);
			Console.WriteLine("t = " + time + " trim");
			WriteState(body);

			double trimCyclic = model.LongCyclic;
			model.LongCyclic = 0.1;
			time += Simulate(body, 1.0);
			Console.WriteLine("t = " + time + " forward cyclic");
			WriteState(body);

			model.LongCyclic = trimCyclic;
			time += Simulate(body, 1.0);
			Console.WriteLine("t = " + time + " trim");
			WriteState(body);

		}

		[Test]
		public void ForwardCyclic_Response()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			var trimPosition = model.Translation;
			var trimPitchAngle = model.PitchAngle;

			model.LongCyclic = 1.0;
			Simulate(body, 1.0);
			WriteState(body);

			Assert.IsTrue(model.PitchAngle < trimPitchAngle);
			Assert.IsTrue(model.Translation.x() > trimPosition.x());
		}

		[Test]
		public void AftCyclic_Response()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			var trimPosition = model.Translation;
			var trimPitchAngle = model.PitchAngle;

			model.LongCyclic = -1.0;
			Simulate(body, 1.0);
			WriteState(body);

			Assert.IsTrue(model.PitchAngle > trimPitchAngle);
			Assert.IsTrue(model.Translation.x() < trimPosition.x());
		}

		[Test]
		public void RightCyclic_Response()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			var trimPosition = model.Translation;
			var trimRollAngle = model.RollAngle;

			model.LatCyclic = 1.0;
			Simulate(body, 1.0);
			WriteState(body);

			Assert.IsTrue(model.RollAngle > trimRollAngle);
			Assert.IsTrue(model.Translation.y() > trimPosition.y());
		}

		[Test]
		public void LeftCyclic_Response()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			var trimPosition = model.Translation;
			var trimRollAngle = model.RollAngle;

			model.LatCyclic = -1.0;
			Simulate(body, 1.0);
			WriteState(body);

			Assert.IsTrue(model.RollAngle < trimRollAngle);
			Assert.IsTrue(model.Translation.y() < trimPosition.y());
		}

		[Test]
		public void RightPedal_Response()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			var trimPosition = model.Translation;
			var trimHeading = model.Heading;

			model.Pedal = 1.0;
			Simulate(body, 1.0);
			WriteState(body);

			Assert.IsTrue(model.Heading > trimHeading);
			double distanceMoved = (model.Translation - trimPosition).Norm(2);
			Assert.IsTrue(distanceMoved < 1.0);
		}

		[Test]
		public void LeftPedal_Response()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			var trimPosition = model.Translation;
			var trimHeading = model.Heading;

			model.Pedal = -1.0;
			Simulate(body, 1.0);
			WriteState(body);

			Assert.IsTrue(model.Heading < trimHeading);
			double distanceMoved = (model.Translation - trimPosition).Norm(2);
			Assert.IsTrue(distanceMoved < 1.0);
		}

		[Test]
		public void UpCollective_Response()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			var trimPosition = model.Translation;

			model.Collective = 1.0;
			Simulate(body, 1.0);
			WriteState(body);

			Assert.IsTrue(model.Translation.z() < trimPosition.z());
			double horizontalDistanceMoved = (model.Translation.SubVector(0,2) - trimPosition.SubVector(0,2)).Norm(2);
			Assert.IsTrue(horizontalDistanceMoved < 1.0);
		}

		[Test]
		public void DownCollective_Response()
		{
			RigidBody body = SetupModels();
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) body.ForceModel;

			var trimPosition = model.Translation;

			model.Collective = -1.0;
			Simulate(body, 1.0);
			WriteState(body);

			Assert.IsTrue(model.Translation.z() > trimPosition.z());
			double horizontalDistanceMoved = (model.Translation.SubVector(0,2) - trimPosition.SubVector(0,2)).Norm(2);
			Assert.IsTrue(horizontalDistanceMoved < 1.0);
		}

		private RigidBody SetupModels()
		{
			SingleMainRotorHelicopter model = new SingleMainRotorHelicopter().LoadDefault();
			model.MainRotor.useDynamicInflow = false;
			model.TailRotor.useDynamicInflow = false;
			model.FCS.trimControl = false;
			model.TrimInit();
			model.Trim();

			RigidBody body = new RigidBody();
			body.ForceModel = model;
			body.Mass = model.Mass;
			body.Inertia = model.Inertia;

			return body;
		}

		private double Simulate(RigidBody body, double duration) {
			for (double t = 0.0; t <= duration; t += DT) {
				body.Update(DT);
			}
			return duration;
		}



		private void WriteState(RigidBody body)
		{
			Console.WriteLine("  F " + body.Force.ToStr());
			Console.WriteLine("  T " + body.Torque.ToStr());
			Console.WriteLine("  v " + body.Velocity.ToStr() + " p " + body.Position.ToStr());
			Console.WriteLine("  w " + body.AngularVelocity.ToStr() + " R " + (body.Rotation.EulerAngles() * 180.0 / Math.PI).ToStr());
		}
	}
}

