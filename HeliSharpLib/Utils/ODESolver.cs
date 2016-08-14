using System;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{

	// The ODEFunction should return the time derivative of the state vector at a given time
	public delegate Vector<double> ODEFunction (double time, Vector<double> state);

	public abstract class ODESolver
	{
		protected ODEFunction ode;
		public Vector<double> State { get; protected set; }
		public double Time { get; protected set; }

		public ODESolver(ODEFunction ode, double initialTime, Vector<double> initialState)
		{
			this.ode = ode;
			Init (initialTime, initialState);
		}

		public void Init(double t0, Vector<double> y0)
		{
			Time = t0;
			State = y0.Clone ();
		}

		public abstract void Step(double dt);

	}

	public class RK4Solver : ODESolver
	{
		public RK4Solver(ODEFunction ode, double initialTime, Vector<double> initialState) : base(ode, initialTime, initialState)
		{
		}

		public override void Step(double dt)
		{
			var k1=dt*ode(Time,State);
			var yt=State+0.5*k1;
			var k2=dt*ode(Time+dt*0.5,yt);
			yt=State+0.5*k2;
			var k3=dt*ode(Time+dt*0.5,yt);
			yt=State+k3;
			var k4=dt*ode(Time+dt,yt);

			State=State+(k1+2.0*k2+2.0*k3+k4)/6.0;
			Time += dt;
		}
	}
}

