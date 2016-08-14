using NUnit.Framework;
using System;
using HeliSharp;

namespace HeliSharp
{

	[TestFixture]
	public class EngineTest
	{

		Engine model = new Engine().LoadDefault();

		[Test]
		public void ModerateLoad_MaintainsRPM() {
			model.Init(100);
			model.load = 800;
			model.throttle = 1.0;

			for (double t = 0.0; t < 10.0; t += 0.1) {
				model.Update(0.1);
			}
			Console.WriteLine("Omega " + model.Omega.ToStr() + " omega0 " + model.Omega0.ToStr());
			Assert.IsTrue(model.Omega > 0.99*model.Omega0);
			Console.WriteLine("P " + (model.Qeng * model.Omega).ToStr() + " max " + model.Pmax);
			Assert.IsTrue(model.Qeng * model.Omega < 0.99*model.Pmax);
		}

		[Test]
		public void HighLoad_DroopsRPM() {
			model.Init(1000);
			model.load = 5000;
			model.throttle = 1.0;

			for (double t = 0.0; t < 10.0; t += 0.1) {
				model.Update(0.1);
			}
			Console.WriteLine("Omega " + model.Omega.ToStr() + " omega0 " + model.Omega0.ToStr());
			Assert.IsTrue(model.Omega < 0.99*model.Omega0);
			Console.WriteLine("P " + (model.Qeng * model.Omega).ToStr() + " max " + model.Pmax);
			Assert.IsTrue(model.Qeng * model.Omega >= 0.99*model.Pmax);
		}

		[Test]
		public void SuddenLowLoad_IncreasesRPM() {
			model.Init(1000);
			model.throttle = 1.0;

			for (double t = 0.0; t < 10.0; t += 0.1) {
				model.Update(0.1);
			}

			model.load = 0;
			for (double t = 0.0; t < 0.5; t += 0.1) {
				model.Update(0.1);
			}

			Console.WriteLine("Omega " + model.Omega.ToStr() + " omega0 " + model.Omega0.ToStr());
			Assert.IsTrue(model.Omega > model.Omega0);
		}


		[Test]
		public void StartStop() {
			model.InitStopped();
			model.throttle = 0.0;
			model.phase = Engine.Phase.START;

			for (double t = 0.0; t < 30.0; t += 0.1) {
				model.Update(0.1);
				Console.WriteLine("Phase " + model.phase + " Omega " + model.Omega.ToStr() + " omegaStart " + model.OmegaStart.ToStr() + " Qeng " + model.Qeng.ToStr());
			}

			Assert.AreEqual(Engine.Phase.RUN, model.phase);
			Assert.IsTrue(model.Omega > 0.9 * model.OmegaStart);
			Assert.IsTrue(model.Omega > 0.9 * model.Omega0 * model.idleRatio);

			model.phase = Engine.Phase.CUTOFF;
			for (double t = 0.0; t < 30.0; t += 0.1) {
				model.Update(0.1);
				Console.WriteLine("Phase " + model.phase + " Omega " + model.Omega.ToStr() + " omegaStart " + model.OmegaStart.ToStr() + " Qeng " + model.Qeng.ToStr());
			}

			Assert.IsTrue(model.Omega < model.OmegaStart);
		}
	}
}

