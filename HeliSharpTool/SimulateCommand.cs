using System;
using ManyConsole;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;

namespace CsHeli
{
	public class SimulateCommand : ConsoleCommand
	{

		public double u { get; set; }
		public double v { get; set; }
		public double w { get; set; }

		public string control { get; set; }
		public double deflection { get; set; }
		public double deflectionTime { get; set; }
		public double deflectionDuration { get; set; }
		public double time { get; set; }
		public double timeStep { get; set; }

		public SimulateCommand()
		{
			time = 6.0;
			timeStep = 0.01;
			deflectionTime = 1.0;
			deflectionDuration = 0.55;

			IsCommand("sim", "Run a short simulation of a model");
			HasOption("u|speed=", "Forward speed (u) at simulation start, default 0 m/s", p => u = Double.Parse(p));
			HasOption("v|latspeed=", "Lateral speed (v), positive right, default 0 m/s", p => v = Double.Parse(p));
			HasOption("w|vspeed=", "Vertical speed (w), positive down, default 0 m/s", p => w = Double.Parse(p));

			HasOption("c|control=", "Which control to delfect - collective, longcyclic, latcyclic, pedal or none (default)", p => control = p);
			HasOption("d|deflection=", "Control deflection, default 0 degrees", p => deflection = Double.Parse(p) * Math.PI / 180.0);
			HasOption("T|deftime=", "Control deflection starts at time, default at t=1 second", p => deflectionTime = Double.Parse(p));
			HasOption("D|defduration=", "Control deflection duration, default 0.55 seconds", p => deflectionDuration = Double.Parse(p));
			HasOption("t|time=", "Simulation time, default 6 seconds", p => time = Double.Parse(p));
			HasOption("s|step=", "Simulation time step, default 0.01 seconds", p => timeStep = Double.Parse(p));
			SkipsCommandSummaryBeforeRunning();
		}

		public override int Run(string[] remainingArguments)
		{
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) new SingleMainRotorHelicopter().LoadDefault();
			model.MainRotor.useDynamicInflow = false; // TODO optional
			model.TailRotor.useDynamicInflow = false;
			model.FCS.trimControl = false;
			model.FCS.enabled = false; // TODO optional

			model.AbsoluteVelocity = Vector<double>.Build.DenseOfArray(new double[] { u, v, w });
			model.TrimInit();
			model.Trim();

			RigidBody body = new RigidBody();
			body.ForceModel = model;
			body.Mass = model.Mass;
			body.Inertia = model.Inertia;
			body.Rotation = model.Rotation;

			Console.WriteLine("%Simulation at u=" + u + " v=" + v + " w=" + w + " control=" + control + " deflection=" + deflection*180.0/Math.PI + " deftime=" + deflectionTime);
			Console.WriteLine("%t"
				+ "\tHelicopter.u\tHelicopter.v\tHelicopter.w"
				+ "\tHelicopter.p\tHelicopter.q\tHelicopter.r"
				+ "\tHelicopter.phi\tHelicopter.theta\tHelicopter.psi"
				+ "\tHelicopter.theta_0\tHelicopter.theta_sin\tHelicopter.theta_cos\tHelicopter.theta_p"
				+ "\tMainRotor.beta_0\tMainRotor.beta_sin\tMainRotor.beta_cos\tTailRotor.beta_0"
				+ "\tEngine.Omega\tEngine.Qeng\tEngine.Qload"
			);

			// Store neutral control angles
			double t0n, tsn, tcn, tpn;
			model.GetControlAngles(out t0n, out tsn, out tcn, out tpn);

			for (double t = 0.0; t <= time; t += timeStep) {

				if (Math.Abs(deflection) > 1e-5 && deflectionTime > 1e-5 && deflectionDuration > 1e-5 && !String.IsNullOrWhiteSpace(control)) {
					if (t >= deflectionTime && t <= deflectionTime + deflectionDuration) {
						if (control.StartsWith("col"))
							model.SetControlAngles(t0n + deflection, tsn, tcn, tpn);
						else if (control.StartsWith("lon"))
							model.SetControlAngles(t0n, tsn + deflection, tcn, tpn);
						else if (control.StartsWith("lat"))
							model.SetControlAngles(t0n, tsn, tcn + deflection, tpn);
						else if (control.StartsWith("ped"))
							model.SetControlAngles(t0n, tsn, tcn, tpn + deflection);
					} else if (t > deflectionTime + deflectionDuration)
						model.SetControlAngles(t0n, tsn, tcn, tpn);
				}

				body.Update(timeStep);

				double t0, ts, tc, tp;
				model.GetControlAngles(out t0, out ts, out tc, out tp);

				Console.WriteLine(t
					+ "\t" + model.Velocity.x() + "\t" + model.Velocity.y() + "\t" + model.Velocity.z()
					+ "\t" + model.AngularVelocity.x() + "\t" + model.AngularVelocity.y() + "\t" + model.AngularVelocity.z()
					+ "\t" + model.Attitude.x() + "\t" + model.Attitude.y() + "\t" + model.Attitude.z()
					+ "\t" + t0 + "\t" + ts + "\t" + tc + "\t" + tp
					+ "\t" + model.MainRotor.beta_0 + "\t" + model.MainRotor.beta_sin + "\t" + model.MainRotor.beta_cos + "\t" + model.TailRotor.beta_0
					+ "\t" + model.Engine.Omega + "\t" + model.Engine.Qeng + "\t" + model.Engine.load
				);
			}


			return 0;
		}
	}
}

