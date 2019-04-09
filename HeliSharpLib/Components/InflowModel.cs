using System;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
     public class InflowModel
    {

        /// Simple linear model for uniform inflow over a helicopter rotor.
         /// Following the method outlined in Erik Backlund's thesis.
         /// Table data from Castles&New 'A Blade-Element Analysis For Lifting Rotors...'
  
        RK4Solver solver;

        // lambda = inflow ratio
        // xi = wake skew angle

        double mu, 	
            lambdabar, 	// average inflow ratio
            vbar, 		// average velocity
            dCt, 		// thrust deviation from reference
            Ct0, 		// reference thrust
            Omega;		// rotational velocity

        public InflowModel()
        {
            solver = new RK4Solver (ODEFunction, 0, Vector<double>.Build.DenseOfArray (new double[] { 0 }));
            mu = lambdabar = vbar = dCt = Ct0 = Omega = 0;
        }

        public Vector<double> ODEFunction (double t, Vector<double> y)
        {
            Vector<double> ydot = Vector<double>.Build.DenseOfArray (new double[] {
                ( dCt - 2.0*( (square(mu)+lambdabar*(lambdabar+vbar))/Math.Sqrt(square(mu)+square(lambdabar)) )*y[0] ) / (8.0/(3.0*Math.PI*Omega))
            });
            return ydot;
        }

        public void EvaluateDynamicInflow(double dt, double mu_x, double mu_z, double beta_cosw, double CT, 
                                          double R, double Omega,
                                          out double lambda_i0, out double xi, out double E_x)
        {
            lambdabar = -mu_z + vbar + solver.State[0];
            //if (lambdabar > 0) lambdabar = 0;
            mu = mu_x;
            dCt = CT - Ct0;
            this.Omega = Omega;
            solver.Step(dt);
            lambda_i0 = -(vbar+solver.State[0]);
            if (lambda_i0 > 0) {
                lambda_i0 = 0;
                TrimDynamicInflow(mu_x, mu_z, beta_cosw, CT, R);
            }
            xi = calculateWakeSkew (mu_x, mu_z, beta_cosw, lambda_i0);
            E_x=(15.0*Math.PI/32.0)*Math.Tan(xi/2.0); // Pitt&Peters choice of coefficients
            // Ground effect reduction is done on CT in Rotor instead..
        }

        public void TrimDynamicInflow(double mu_x, double mu_z, double beta_cosw, double CT, double R)
        {
            double dummy;
            EvaluateInflow(mu_x, mu_z, beta_cosw, CT, R, 1000.0, out vbar, out dummy, out dummy);
            Ct0 = CT;
            mu = mu_x;
            vbar = -vbar;
            solver.Init(0, Vector<double>.Build.DenseOfArray (new double[] { 0 }));
        }

        static internal void EvaluateInflow(double mu_x, double mu_z, double beta_cosw, double CT,
                                   double R, double height,
                                   out double lambda_i0, out double xi, out double E_x)
        {
            if (CT == 0)
                lambda_i0=0;
            else {
                double lambdaxh=mu_x*Math.Sqrt((2.0-3.0*mu_x*mu_x)/Math.Abs(CT));
                double lambdazh=mu_z*Math.Sqrt((2.0-3.0*mu_x*mu_x)/Math.Abs(CT));

                // Bilinear interpolation
                double lambdaih = Numerics.InterpolateBilinear(lambdaX,lambdaZ,lambdaTable,32,19,lambdaxh,lambdazh);

                lambda_i0=-lambdaih*Math.Sqrt(Math.Abs(CT)/(2.0-3.0*mu_x*mu_x));
            }
            xi = calculateWakeSkew (mu_x, mu_z, beta_cosw, lambda_i0);

            E_x=(15.0*Math.PI/32.0)*Math.Tan(xi/2.0); // Pitt&Peters choice of coefficients

            //if (CT < 0) lambda_i0 = -lambda_i0;

            // Reduce inflow due to ground effect
            if (height/R <= 0.1) {
                // this shouldn't happen but avoid singularity
                lambda_i0 = 0.0;
            } else if (height < 3.0*R && lambda_i0 > 1e-5) {
                /*
                //double kG = 1/(0.9926+0.0379 * (2*R/height)*(2*R/height) ); // Hayden model (Leishmann pg 260)
                double kG = (8*height*height/(R*R))/(1+8*height*height/(R*R)); // Mark Dreier pg 236
                // Reduce ground effect at forward speed (using wake skew angle) - my own invention
                kG *= exp(sqrt(xi)/10.0);
                // Reduced it a bit in general, since this model seems to overestimate it a bit...
                kG *= 1.07;
                if (kG > 1.0) kG=1.0;
                */

                // Increase thrust due to ground effect (Padfield eq 3.218 pg 141 (2nd edition))
                // modified for adjusting inflow instead, so that torque is corrected and stuff..
                double kG = 1.0-1.0/16.0*Square(R/height)/(1.0+Square(mu_x/lambda_i0));

                lambda_i0 = kG*lambda_i0;
            }

        }

        private static double calculateWakeSkew(double mu_x, double mu_z, double beta_cosw, double lambda_i0)
        {
            double xi;
            if (Math.Abs(mu_z+lambda_i0-beta_cosw*mu_x) < clamp_xi_denom)
                xi=Math.Atan(mu_x/clamp_xi_denom);
            else
                xi=Math.Atan(mu_x/Math.Abs(mu_z+lambda_i0-beta_cosw*mu_x));
            return xi;
        }




        private static double Square (double x) { return x*x; }

        const double clamp_xi_denom = 0.08; // TODO make parameter?

        static readonly double[,] lambdaTable = {
            {0.960,0.740,0.580,0.481,0.457,0.433,0.410,0.390,0.371,0.349,0.315,0.285,0.261,0.239,0.210,0.184,0.156,0.120,0.097},
            {1.140,0.880,0.680,0.543,0.509,0.476,0.444,0.418,0.392,0.369,0.329,0.295,0.267,0.245,0.224,0.186,0.158,0.121,0.098},
            {1.360,1.070,0.820,0.630,0.574,0.526,0.484,0.450,0.418,0.389,0.344,0.305,0.275,0.250,0.228,0.188,0.159,0.122,0.099},
            {1.650,1.340,1.030,0.767,0.659,0.585,0.529,0.483,0.445,0.410,0.357,0.315,0.282,0.256,0.233,0.191,0.151,0.122,0.099},
            {2.260,1.810,1.420,1.000,0.769,0.654,0.577,0.518,0.472,0.432,0.370,0.325,0.289,0.260,0.237,0.192,0.162,0.123,0.099},
            {2.440,2.050,1.770,1.220,0.896,0.727,0.627,0.550,0.496,0.452,0.384,0.333,0.295,0.265,0.240,0.195,0.153,0.124,0.099},
            {2.240,1.880,1.650,1.250,0.976,0.789,0.668,0.582,0.520,0.470,0.395,0.341,0.301,0.269,0.243,0.196,0.164,0.124,0.099},
            {2.010,1.720,1.520,1.210,1.000,0.824,0.698,0.613,0.539,0.485,0.404,0.347,0.306,0.272,0.246,0.197,0.165,0.124,0.100},
            {1.800,1.560,1.390,1.150,0.984,0.833,0.713,0.621,0.552,0.494,0.413,0.352,0.309,0.276,0.248,0.198,0.166,0.125,0.100},
            {1.600,1.410,1.270,1.070,0.947,0.820,0.712,0.625,0.556,0.500,0.415,0.356,0.311,0.277,0.249,0.199,0.166,0.125,0.100},
            {1.420,1.280,1.160,1.000,0.897,0.792,0.698,0.619,0.554,0.500,0.416,0.357,0.312,0.278,0.250,0.200,0.167,0.125,0.100},
            {1.250,1.150,1.060,0.924,0.842,0.756,0.677,0.606,0.547,0.494,0.414,0.357,0.312,0.278,0.250,0.200,0.167,0.125,0.100},
            {1.000,0.961,0.914,0.854,0.786,0.715,0.648,0.586,0.533,0.486,0.410,0.354,0.310,0.278,0.250,0.200,0.167,0.125,0.100}, 
            // ^^^	
            //1.10  1.02  0.96 0.854 0.786 0.715 0.648 0.586 0.533 0.486 0.410 0.354 0.310 0.278 0.250 0.200 0.167 0.125  % Experiment data (first 3 columns)  
            //1.05  0.99  0.937 0.854 0.786 0.715 0.648 0.586 0.533 0.486 0.410 0.354 0.310 0.278 0.250 0.200 0.167 0.125  % Weighted data 50% experiment-50% analysis (first 3 columns)  
            //1.0  0.961  0.914 0.854 0.786 0.715 0.648 0.586 0.533 0.486 0.410 0.354 0.310 0.278 0.250 0.200 0.167 0.125 % Analysed data (first 3 columns)   

            {0.905,0.874,0.833,0.787,0.731,0.673,0.613,0.564,0.516,0.474,0.404,0.350,0.309,0.275,0.248,0.199,0.166,0.125,0.100},
            {0.820,0.796,0.765,0.724,0.680,0.632,0.584,0.539,0.497,0.461,0.395,0.345,0.305,0.273,0.247,0.198,0.166,0.125,0.100},
            {0.744,0.725,0.699,0.668,0.630,0.592,0.551,0.513,0.477,0.443,0.386,0.339,0.301,0.270,0.245,0.197,0.165,0.125,0.100},
            {0.677,0.658,0.640,0.615,0.586,0.553,0.520,0.487,0.453,0.426,0.374,0.331,0.296,0.267,0.242,0.196,0.165,0.125,0.100},
            {0.618,0.605,0.588,0.569,0.544,0.517,0.489,0.462,0.435,0.409,0.362,0.323,0.290,0.262,0.239,0.194,0.164,0.124,0.099},
            {0.566,0.556,0.543,0.526,0.506,0.484,0.460,0.433,0.413,0.392,0.349,0.314,0.284,0.258,0.235,0.192,0.163,0.124,0.099},
            {0.521,0.512,0.501,0.488,0.472,0.453,0.433,0.413,0.394,0.374,0.337,0.305,0.277,0.252,0.231,0.190,0.161,0.123,0.099},
            {0.481,0.473,0.464,0.454,0.440,0.426,0.408,0.391,0.374,0.358,0.325,0.296,0.270,0.247,0.227,0.188,0.160,0.122,0.099},
            {0.445,0.439,0.432,0.424,0.411,0.399,0.385,0.371,0.356,0.341,0.312,0.286,0.263,0.242,0.223,0.186,0.158,0.121,0.098},
            {0.414,0.409,0.403,0.395,0.386,0.376,0.364,0.352,0.339,0.326,0.300,0.277,0.255,0.236,0.219,0.183,0.157,0.121,0.098},
            {0.362,0.358,0.355,0.350,0.342,0.334,0.327,0.318,0.308,0.298,0.278,0.259,0.241,0.225,0.209,0.178,0.153,0.119,0.097},
            {0.320,0.318,0.316,0.311,0.306,0.301,0.294,0.287,0.280,0.273,0.258,0.242,0.227,0.213,0.200,0.172,0.149,0.117,0.096},
            {0.287,0.284,0.282,0.280,0.276,0.272,0.267,0.262,0.256,0.250,0.239,0.226,0.214,0.202,0.191,0.166,0.145,0.115,0.095},
            {0.259,0.257,0.256,0.254,0.251,0.248,0.244,0.240,0.236,0.231,0.222,0.216,0.201,0.191,0.182,0.160,0.141,0.113,0.094},
            {0.236,0.235,0.234,0.233,0.230,0.227,0.225,0.223,0.221,0.214,0.207,0.198,0.189,0.181,0.173,0.154,0.137,0.111,0.093},
            {0.193,0.192,0.192,0.191,0.189,0.187,0.186,0.184,0.182,0.180,0.175,0.170,0.164,0.159,0.155,0.139,0.127,0.105,0.089},
            {0.162,0.162,0.162,0.161,0.160,0.159,0.158,0.157,0.156,0.155,0.152,0.148,0.144,0.140,0.137,0.126,0.117,0.094,0.085},
            {0.123,0.123,0.123,0.122,0.122,0.122,0.121,0.121,0.120,0.120,0.118,0.120,0.115,0.112,0.111,0.105,0.093,0.088,0.078},
            {0.100,0.100,0.100,0.100,0.099,0.099,0.099,0.098,0.098,0.097,0.096,0.095,0.094,0.093,0.092,0.089,0.085,0.078,0.070}
        };

        static readonly double[] lambdaZ = {2.4000, 2.2000, 2.0000, 1.8000, 1.6000, 1.4000, 1.2000, 1.0000,
            0.8000, 0.6000, 0.4000, 0.2000, 0, -0.2000,	-0.4000, -0.6000, -0.8000, -1.0000, -1.2000, -1.4000, -1.6000,
            -1.8000, -2.0000, -2.4000, -2.8000, -3.2000, -3.6000, -4.0000, -5.0000, -6.0000, -8.0000, -10.0000};

        static readonly double[] lambdaX = {0,0.4,0.6,0.8,1,1.2,1.4,1.6,1.8,2,2.4,2.8,3.2,3.6,4,5,6,8,10};

        private double square(double x)
        {
            return x*x;
        }



    }
}

