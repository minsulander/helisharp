using NUnit.Framework;
using System;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
    [TestFixture()]
    public class ODESolverTest
    {

        public Vector<double> ODEFunction (double t, Vector<double> y)
        {
            Vector<double> ydot = Vector<double>.Build.DenseOfArray (new double[] {
                5*y[0]+3
            });
            return ydot;
        }

        [Test()]
        public void TestCase ()
        {
            RK4Solver solver = new RK4Solver (ODEFunction, 0, Vector<double>.Build.DenseOfArray(new double[] {0}));
            //solver.Init(0, Vector<double>.Build.DenseOfArray(new double[] { 0.0 }));
            solver.Step (1);
            Console.WriteLine (solver.State [0]);
            solver.Step (1);
            Console.WriteLine (solver.State [0]);
            solver.Step (1);
            Console.WriteLine (solver.State [0]);
        }
    }
}

