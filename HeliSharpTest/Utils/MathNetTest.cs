using NUnit.Framework;
using System;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using HeliSharp;

namespace HeliSharp
{
    [TestFixture()]
    public class MathNetTest
    {
        [Test()]
        public void TestMatrix ()
        {
            Matrix<double> M = Matrix<double>.Build.DenseOfArray (new double[,] {
                {1, 0, 0},
                {0, 2, 0},
                {0, 0, 1}
            });
            Vector<double> v = Vector<double>.Build.DenseOfArray (new double[] { 1, 2, 3 });

            var a = M * v;
            Console.WriteLine (a);
        }

        [Test]
        public void TestRotationMatrix ()
        {
            double a = Math.PI/6;
            Matrix<double> R = Matrix<double>.Build.DenseOfArray(new double[,] {
                { Math.Cos(a), Math.Sin(a), 0},
                { -Math.Sin(a), Math.Cos(a), 0},
                { 0, 0, 1}
            });
            Vector<double> v = Vector<double>.Build.DenseOfArray(new double[] { 1,1,1 });
            Console.WriteLine (R * v);
        }

        [Test]
        public void TestSolve()
        {
            // Ax=b

            var A = Matrix<double>.Build.DenseOfArray(new double[,] {
                { 3, 2, -1 },
                { 2, -2, 4 },
                { -1, 0.5, -1 }
            });
            var b = Vector<double>.Build.Dense(new double[] { 1, -2, 0 });
            var x = A.Solve (b);
            Console.WriteLine (x);
        }

        [Test]
        public void TestVectors() {
            var v = Vector<double>.Build.Dense (3);
            Console.WriteLine (v);

            v [0] = 1;
            v [1] = 1;
            Console.WriteLine (v.Norm (2));
        }

        [Test]
        public void LolImDumb() {
            double[,] M = { 
                { 1, 2, 3 }, 
                { 4, 5, 6 } 
            };
            Console.WriteLine (M [0,2]);
            var v1 = Vector<double>.Build.DenseOfArray(new double[] { 3, 1, 2 });
            Console.WriteLine(v1.Norm(1));
            Console.WriteLine(v1.Norm(2));
        }

        [Test]
        public void Extensions() {
            var v = Vector<double>.Build.Zero3();
            Console.WriteLine (v);
            Assert.AreEqual (0, v.Norm (2));
        }

        [Test]
        public void CrossProduct() {
            var a = Vector<double>.Build.DenseOfArray(new double[] {1, 2, 3});
            var b = Vector<double>.Build.DenseOfArray(new double[] {4, 5, 6});
            Console.WriteLine(a.Cross(b).ToStr());
        }

        [Test]
        public void Normalize() {
            var v = Vector<double>.Build.DenseOfArray(new double[] { 3, 1, 2 });
            var n = v.Normalize(2);
            Console.WriteLine(v.ToStr());
            Console.WriteLine(n.ToStr());
        }

        [Test]
        public void EulerAngles() {
            Matrix<double> R = Matrix<double>.Build.EulerRotation(Vector<double>.Build.Dense3(0.1, 0.2, 0.3));
            Vector<double> euler = R.EulerAngles();
            Assert.AreEqual(0.1, euler.x(), 1e-5);
            Assert.AreEqual(0.2, euler.y(), 1e-5);
            Assert.AreEqual(0.3, euler.z(), 1e-5);
        }

        [Test]
        public void QuaternionConversions() {
            Matrix<double> R = Matrix<double>.Build.EulerRotation(Vector<double>.Build.Dense3(0.1, 0.2, 0.3));
            Vector<double> q = R.ToQuaternion();
            Matrix<double> R2 = q.ToRotationMatrix();

            Assert.True(R.AlmostEqual(R2, 1e-5));
            Assert.AreEqual(1, q.Norm(2), 1e-5);
        }

}

}

