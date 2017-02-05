using System;
using ManyConsole;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;

namespace CsHeli
{
	public class TrimCommand : ConsoleCommand
	{

		public double ustart { get; set; }
		public double uend { get; set; }
		public double ustep { get; set; }
		public double v { get; set; }
		public double w { get; set; }
		public double h { get; set; }

		public TrimCommand()
		{
			ustart = -20;
			uend = 60;
			ustep = 1;
			v = 0;
			w = 0;
			h = 1000;
			IsCommand("trim", "Run a trim sweep on a model");
			HasOption("s|ustart=", "Initial forward speed (u), default -20 m/s", p => ustart = Double.Parse(p));
			HasOption("e|uend=", "Final forward speed (u), default 60 m/s", p => uend = Double.Parse(p));
			HasOption("d|ustep=", "Forward speed (u) step, default 1 m/s", p => ustep = Double.Parse(p));
			HasOption("v=", "Lateral speed (v), positive right, default 0 m/s", p => v = Double.Parse(p));
			HasOption("w=", "Vertical speed (w), positive down, default 0 m/s", p => w = Double.Parse(p));
			HasOption("h=", "Height above ground, default 1000 m", p => h = Double.Parse(p));
			SkipsCommandSummaryBeforeRunning();
		}

		public override int Run(string[] remainingArguments)
		{
			SingleMainRotorHelicopter model = (SingleMainRotorHelicopter) new SingleMainRotorHelicopter().LoadDefault();
			model.MainRotor.useDynamicInflow = false;
			model.TailRotor.useDynamicInflow = false;
			model.FCS.trimControl = false;

			bool ok = true;
			Console.WriteLine("%Trim sweep u=" + ustart + " => " + uend + " v=" + v + " w=" + w + " h=" + h);
			Console.WriteLine("%u\tHelicopter.powerreq" 
				+ "\tHelicopter.theta_0\tHelicopter.theta_p\tHelicopter.theta_cos\tHelicopter.theta_sin"
				+ "\tMainRotor.beta_0\tMainRotor.beta_cos\tMainRotor.beta_sin"
				+ "\tHelicopter.theta\tHelicopter.phi");
			model.Height = h;
			for (var u = ustart; u <= uend+ustep/2; u += ustep) {
				model.AbsoluteVelocity = Vector<double>.Build.DenseOfArray(new double[] { u, v, w });
				model.AngularVelocity = Vector<double>.Build.Zero3();
				try {
					model.TrimInit();
					model.Trim();

					Console.Error.WriteLine(u + " m/s col " + Math.Round(model.Collective*100)
						+ " lon " + Math.Round(model.LongCyclic*100)
						+ " lat " + Math.Round(model.LatCyclic*100)
						+ " ped " + Math.Round(model.Pedal*100)
						+ " roll " + (model.RollAngle * 180 / Math.PI).ToStr()
						+ " pitch " + (model.PitchAngle * 180 / Math.PI).ToStr());
					double theta_0, theta_sin, theta_cos;
					model.MainRotor.GetControlAngles(out theta_0, out theta_sin, out theta_cos);
					double theta_p, temp;
					model.TailRotor.GetControlAngles(out theta_p, out temp, out temp);
					Console.WriteLine(u + "\t" + model.PowerRequired 
						+ "\t" + theta_0 + "\t" + theta_p + "\t" + theta_cos + "\t" + theta_sin
						+ "\t" + model.MainRotor.beta_0 + "\t" + model.MainRotor.beta_cos + "\t" + model.MainRotor.beta_sin
						+ "\t" + model.Attitude.y() + "\t" + model.Attitude.x());
				} catch (TrimmerException e) {
					Console.Error.WriteLine("  Failed: " + e.Message);
					ok = false;
				}
			}

			return ok ? 0 : 1;
		}

	}
}

