using System;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{

    public delegate Vector<double> TrimFunction(Vector<double> input);

    public class TrimmerException : Exception
    {
        public TrimmerException(string message) : base(message)
        {
        }

    }

    public class JacobianTrimmer
    {

        /// General-purpose class for trimming a force system (adjusting a number of inputs so that the resultant
        /// of forces ends up near zero). Uses a Jacobian / Newton-Raphson equations solving scheme.

        public double MaxNorm { get; set; }
        public double Tolerance { get; set; }
        public int MaxIterations { get; set; }

        private double norm;
        private int iterations;

        public JacobianTrimmer ()
        {
            MaxNorm = 1e5;
            Tolerance = 1e-5;
            MaxIterations = 100;
        }

        public Vector<double> Trim(Vector<double> initialGuess, Vector<double> steps, TrimFunction trimFunction)
        {
            var x = initialGuess.Clone();
            var dx = steps.Clone();
            var y = trimFunction(x);
            if (x.Count != dx.Count)
                throw new TrimmerException ("TrimInit and TrimSteps vectors must have same size");
            if (x.Count != y.Count)
                throw new TrimmerException ("TrimInit and Trim vectors must have same size");
            norm = y.Norm (2);
            iterations = 0;
            if (norm > MaxNorm)
                throw new TrimmerException ("Bad initial guess");

            Matrix<double> J = Matrix<double>.Build.DenseIdentity (x.Count);
            while (iterations < MaxIterations) {
                iterations++;
                for (int c = 0; c < x.Count; c++) {
                    // Central difference
                    var xp = x.Clone();
                    xp [c] = x [c] + dx [c] / 2.0;
                    var yp = trimFunction(xp);
                    var xm = x.Clone();
                    xm [c] = x [c] - dx [c] / 2.0;
                    var ym = trimFunction(xm);
                    for (int r = 0; r < y.Count; r++) {
                        J[r,c] = (yp[r]-ym[r])/dx[c];
                    }
                }
                //Console.WriteLine ("x: " + x);
                //Console.WriteLine ("J: " + J);
                //Console.WriteLine ("y: " + y);

                // Solve x(n+1)=J^-1*(-y(n))+x(n)
                var d = J.Solve (-y);
                //Console.WriteLine ("d: " + d);
                for (int i = 0; i < d.Count; i++) {
                    if (double.IsNaN (d [i]))
                        throw new TrimmerException ("No solution");
                }
                x += d;
                // Evaluate result
                y = trimFunction(x);
                norm = y.Norm (2);
                //Console.WriteLine ("  #" + iterations + "\tnorm " + norm.ToStr() + "\tx " + x.ToStr() + "\ty " + y.ToStr()); 
                if (norm < Tolerance)
                    return x;
                if (norm > MaxNorm)
                    throw new TrimmerException ("Diverged");
            }
            if (iterations >= MaxIterations)
                throw new TrimmerException ("Exceeded max iterations");
            return x;
        }
    }
}

