using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
	[Serializable]
	public class Airfoil
	{

		/// Encapsulation of tabulated airfoil data.

		public string name;
		public Matrix<double> cl_alpha;
		public Matrix<double> cd_alpha;
		public Matrix<double> cm_alpha;

		private Airfoil LoadDefault() {
			name = "NACA0012";
			cl_alpha = Matrix<double>.Build.DenseOfArray(new double[,] {
				{ -180,    0 },
				{ -170,    0.85 },
				{ -160,    0.66 },
				{ -135,    0.945 },
				{ -90,     0 },
				{ -50,     -1.085 },
				{ -26,     -0.95 },
				{ -16,     -0.4696 },
				{ -11,     -1.075 },
				{ 11,      1.075 },
				{ 16,      0.4696 },
				{ 26,      0.95 },
				{ 50,      1.085 },
				{ 90,      0 },
				{ 135,     -0.945 },
				{ 160,     -0.66 },
				{ 170,     -0.85 },
				{ 180,     0 }
			});
			cd_alpha = Matrix<double>.Build.DenseOfArray(new double[,] {
				{ -180,    0.01 },
				{ -155,    0.42 },
				{ -115,    1.55 },
				{ -90,     1.8 },
				{ -65,     1.55 },
				{ -16,     0.21 },
				{ -14,     0.02 },
				{ 0,       0.01 },
				{ 14,      0.02 },
				{ 16,      0.21 },
				{ 65,      1.55 },
				{ 90,      1.8 },
				{ 115,     1.55 },
				{ 155,     0.42 },
				{ 180,     0.01 }
			});
			cm_alpha = Matrix<double>.Build.DenseOfArray(new double[,] {
				{ -180,    0 },
				{ -175,    0.3 },
				{ -100,    0.5 },
				{ 100,     -0.5 },
				{ 175,     -0.3 },
				{ 180,     0 }
			});
			return this;
		}

		public double CL(double alpha) {
			if (cl_alpha == null) LoadDefault();
			return Numerics.InterpolateLinear(cl_alpha, alpha);
		}

		public double CD(double alpha) {
			if (cd_alpha == null) LoadDefault();
			return Numerics.InterpolateLinear(cd_alpha, alpha);
		}

		public double CM(double alpha) {
			if (cm_alpha == null) LoadDefault();
			return Numerics.InterpolateLinear(cm_alpha, alpha);
		}

		private static Dictionary<string, Airfoil> airfoils = new Dictionary<string, Airfoil>() { {"NACA0012", new Airfoil().LoadDefault()} };

		public static Airfoil Get(string name) {
			return airfoils[name];
		}

		public static void Add(Airfoil airfoil) {
			airfoils[airfoil.name] = airfoil;
		}
	}
}

