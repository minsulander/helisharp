using NUnit.Framework;
using System;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
    [TestFixture()]
    public class ForceAssemblyTest
    {
        [Test]
        public void TestAssembleForces ()
        {
            // F=10 in x direction, displaced by 5 on the y-axis = moment of -50 around z-axis in right hand coordinate system
            ForceAssembly assembly = new ForceAssembly ();
            StaticForce force = new StaticForce(Vector<double>.Build.DenseOfArray(new double[] { 10, 0, 0 }));
            force.Translation = Vector<double>.Build.DenseOfArray(new double[] { 0, 5, 0 });
            assembly.SetModel("Test", force);
            assembly.Update (1);
            Assert.AreEqual (-50, assembly.Torque [2]);
        }
    }
}

