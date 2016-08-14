using System;
using MathNet.Numerics.LinearAlgebra;
using Newtonsoft.Json;

namespace HeliSharp
{
	public class Atmosphere
	{

		/// Simple model of atmospheric properties (pressure, density, temperature, wind etc)
		/// based on position of an object

		// Inputs
		[JsonIgnore]
		public Vector<double> Position { get; set; }
		[JsonIgnore]
		public Vector<double> GlobalWind { get; set; }

		// Outputs
		[JsonIgnore]
		public double Pressure { get; set; }				// p	[Pa]
		[JsonIgnore]
		public double Density { get; set; }					// rho	[kg/m^3]
		[JsonIgnore]
		public double Temperature { get; set; }				// T	[K]
		[JsonIgnore]
		public double SpeedOfSound { get; set; }			// c	[m/s]
		[JsonIgnore]
		public double Viscosity { get; set; }				// mu	[kg/(m*s)]
		[JsonIgnore]
		public double GeopotentialAltitude { get; set; }	// h	[m]
		[JsonIgnore]
		public Vector<double> Wind { get; set; }

		// Parameters which we probably don't want to change
		// Global atmospheric properties
		double
		p0,		///< Sea level pressure			Pa
		rho0,	///< Sea level density			kg/m^3
		T0,		///< Sea level temperature		K
		R,		///< ?							J/(kg*K)
		g0,		///< Gravitational acceleration	m/s^2
		a,		///< Temperature lapse rate		K/m
		r,		///< Planet radius				m
		mu0,	///< Sea level viscosity		kg/(m*s)
		S,		///< ?							K
		gamma;	///< ?							cp/cv

		public Atmosphere()
		{
			p0=101.325e3;   // Pa
			rho0=1.225;     // kg/m^3
			T0=288.16;      // K
			R=287.04;       // J/(kg*K)
			g0=9.80665;     // m/s^2
			a=-0.0065;      // K/m
			r=6.356766e6;   // m
			mu0=1.780e-5;   // kg/(m*s)
			S=110.6;        // K
			gamma=1.4;      // cp/cv

			// Default inputs
			Position = Vector<double>.Build.Zero3();
			GlobalWind = Vector<double>.Build.Zero3();
		}

		public void Update(double dt) {
			double z = -Position.z();
			double p, rho, T;
			double h=r/(r+z)*z;
			double T11=T0+a*11000.0; //temperature at h=11000m
			if (h>11000.0) {
				// Treat temperature differently in stratosphere
				T=T11;
				p=p0*0.22336*Math.Exp((-g0/R/T11)*(h-11000.0));
				rho=rho0*0.29707*Math.Exp(-(g0/R/T11)*(h-11000.0));
			} else {
				// Troposphere temperature from standard lapse rate
				T=T0+a*h;
				p=p0*Math.Pow(T/T0,-g0/(a*R));
				rho=rho0*Math.Pow(T/T0,-g0/(a*R)+1.0);
			}
			double c = Math.Sqrt(gamma * R * T);
			double mu = Math.Pow(mu0*(T/T0),1.5*(T0+S)/(T+S));

			Pressure = p;
			Density = rho;
			Temperature = T;
			SpeedOfSound = c;
			Viscosity = mu;
			GeopotentialAltitude = h;

			// Future improvement: This could be replaced with a wind map and some dynamics
			Wind = GlobalWind;
		}

	}
}

