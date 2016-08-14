using NUnit.Framework;
using System;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{

	[TestFixture]
	public class AtmosphereTest
	{

		Atmosphere model = new Atmosphere();

		[Test]
		public void HigherAltitude_LessPressure() {
			model.Update(0.1);
			Console.WriteLine("Pressure @ SL " + model.Pressure.ToStr());
			Console.WriteLine("Density @ SL " + model.Density.ToStr());
			Console.WriteLine("Temperature @ SL " + model.Temperature.ToStr());
			Console.WriteLine("Geopotential altitude @ SL " + model.GeopotentialAltitude.ToStr());

			double sealevelPressure = model.Pressure;
			double sealevelDensity = model.Density;

			model.Position = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, -1000 });
			model.Update(0.1);
			Console.WriteLine("Pressure @ 1000m " + model.Pressure.ToStr());
			Console.WriteLine("Density @ 1000m " + model.Density.ToStr());
			Console.WriteLine("Temperature @ 1000m " + model.Temperature.ToStr());
			Console.WriteLine("Geopotential altitude @ 1000m " + model.GeopotentialAltitude.ToStr());

			Assert.IsTrue(model.Pressure < sealevelPressure);
			Assert.IsTrue(model.Density < sealevelDensity);
		}

	}
}

