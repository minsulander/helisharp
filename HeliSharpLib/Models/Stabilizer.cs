using System;
using MathNet.Numerics.LinearAlgebra;
using System.Web.Script.Serialization;
using Newtonsoft.Json;

namespace HeliSharp
{
	[Serializable]
	public class Stabilizer : ForceModel
	{

		///  Simple aerodynamic model of a stabilizer, using table-lookups of lift, drag and moment versus angle of attack.

		// Inputs
		[JsonIgnore]
		public double Density { get; set; }
		// Parameters
		public double span;
		public double chord;

		[JsonIgnore]
		public Airfoil airfoil;
		public string airfoilName {
			get { return airfoil != null ? airfoil.name : null; }
			set { airfoil = Airfoil.Get(value); }
		}

		public Stabilizer () {
			Density = 1.225;
		}

		public Stabilizer LoadDefaultHorizontal() {
			airfoil = Airfoil.Get("NACA0012");
			span = 3.2;
			chord = 0.4;
			return this;
		}

		public Stabilizer LoadDefaultVertical() {
			airfoil = Airfoil.Get("NACA0012");
			span = 2.0;
			chord = 0.7;
			return this;
		}

		public override void Update(double dt) {
			var normalizedVelocity = Velocity.Normalize(2);
			var alpha = Math.Atan2(normalizedVelocity.z(), normalizedVelocity.x());

			var CL = airfoil.CL(alpha * 180.0 / Math.PI);
			var CD = airfoil.CD(alpha * 180.0 / Math.PI);
			var CM = airfoil.CM(alpha * 180.0 / Math.PI);

			var V2 = Velocity.Norm(2);
			var L = 0.5 * Density * V2 * span * CL;
			var D = 0.5 * Density * V2 * span * CD;
			var M = 0.5 * Density * V2 * span * chord * CM;

			Force = Vector<double>.Build.DenseOfArray(new double[] { 
				-D * Math.Cos(alpha) + L * Math.Sin(alpha),
				0,
				-L * Math.Cos(alpha) - D * Math.Sin(alpha)
			});
			Torque = Vector<double>.Build.DenseOfArray(new double[] { 0, M, 0 });
		}
	}
}

