using NUnit.Framework;
using System;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
	[TestFixture]
	public class TrimmerTest
	{

		[Test]
		public void TestCase ()
		{
			JacobianTrimmer trimmer = new JacobianTrimmer ();
			trimmer.Trim (
				Vector<double>.Build.DenseOfArray (new double[] { 1, 2 }),
				Vector<double>.Build.DenseOfArray (new double[] { 0.1, 0.1 }),
				delegate (Vector<double> x) {
					return Matrix<double>.Build.DenseOfArray(new double[,] {
						{1, 2},
						{3, 4}
					}) * x;
				}
			);
		}
	}
}

