using System;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;
using System.Web.Script.Serialization;
using Newtonsoft.Json;

namespace HeliSharp
{
	[Serializable]
	public class Engine
	{

		/// Simple turboshaft engine simulation model.
		/// Based on Mark Dreier's description.

		public enum Phase { FAIL, CUTOFF, START, RUN };

		// Inputs
		[JsonIgnore]
		public double throttle { get; set; }
		[JsonIgnore]
		public Phase phase { get; set; }

		public double load { get; set; }
		public double inertia { get; set; }

		// Outputs
		[JsonIgnore]
		public double RotSpeed { get; set; }

		// Parameters
		public double gain, timeConstant, internalInertia, proportionalGain, integralGain, derivativeGain, maxPower, integratorLimit, idleRatio; // parameters
		public double startTorque, accelerationTorque, accelerationRPM, startDelay; // starting torques and when to switch (parameters)
		public double friction; // internal friction (for stopping the engine)
		public double designRPM; //rotational speed

	    // Calculation names of parameters above
	    private double K_eng => gain;
	    private double tao_eng => timeConstant;
	    private double J0 => internalInertia;
	    private double Kp => proportionalGain;
	    private double Ki => integralGain;
	    private double Kd => derivativeGain;
	    private double Pmax => maxPower;
	    private double intmax => integratorLimit;
	    private double Omega0 => designRPM / 9.5492966;

	    // states - output torque and change in rotational speed
		[JsonIgnore]
		public double Qeng { get; set; }
		[JsonIgnore]
		public double Omega { get; set; }

	    [JsonIgnore]
	    public double RPM => Omega * 9.5492966;

	    private double starttime;

		public ODESolver Solver { get; set; }

		private double dOmega;
		private bool initialized;

		public Engine() {
			phase = Phase.CUTOFF;
		}

		public Engine LoadDefault() {
			gain = 300;
			timeConstant = 1000;
			proportionalGain = 200;
			integralGain = 2;
			derivativeGain = 10;
			integratorLimit = 1;
			internalInertia = 3;
		    designRPM = 6000;
			maxPower = 840000;
			startTorque = 30;
			accelerationTorque = 250;
			accelerationRPM = 250;
			startDelay = 2.5;
		    friction = 0.1;
		    idleRatio = 0.4;

		    inertia = 10;
			return this;
		}

		public void Init(double Q0) {
			Qeng = load = Q0;
			Omega = Omega0;
			dOmega = 0;
			RotSpeed = Omega;
			phase = Phase.RUN;
			throttle = 1.0;
			Solver = new RK4Solver (ODEFunction, 0, Vector<double>.Build.DenseOfArray (new double[] { Q0, 0, 0 }));
			initialized = true;
		}

		public void InitStopped() {
			Qeng = Omega = dOmega = 0;
			load = 0;
			RotSpeed = 0;
			phase = Phase.CUTOFF;
			throttle = 0;
			Solver = new RK4Solver (ODEFunction, 0, Vector<double>.Build.DenseOfArray (new double[] { 0, 0, 0 }));
			initialized = true;
		}

		public void Update(double dt) {
			if (!initialized) Init(0);
			// Transition to new phases
			if (phase == Phase.START) {
				if (starttime < -0.1) // first start update
					starttime = 0;
				if (Omega > idleRatio*Omega0)
					phase = Phase.RUN;
				starttime += dt;
			} else
				starttime = -1;
			// Update
			double Omega_setpoint;
			if (phase == Phase.RUN)
				Omega_setpoint = Omega0*(idleRatio+(1.0-idleRatio) * throttle);
			else
				Omega_setpoint = 0;
			Solver.State[1] = Omega - Omega_setpoint;
			Solver.Step(dt);
			Qeng = Solver.State[0];
			dOmega = Solver.State[1];
			Omega = Omega_setpoint+dOmega;
			// Limit power
			if (Qeng*Omega > Pmax)
				Qeng = Pmax/Omega;
			if (Qeng < 0)
				Qeng = 0;
			if (phase == Phase.CUTOFF || phase == Phase.FAIL)
				Qeng = 0;
			Solver.State[0] = Qeng;
			// Limit integrator term
			if (Math.Abs(Solver.State[2]) > intmax) {
				if (Solver.State[2] > 0)
					Solver.State[2] = intmax;
				else
					Solver.State[2] = -intmax;
			}
			// Set output ports
		    if (double.IsNaN(Omega) || double.IsInfinity(Omega)) throw new ModelException("Engine rotation speed would be " + Omega);
			RotSpeed = Omega;
		}

		public Vector<double> ODEFunction (double t, Vector<double> state)
		{
			// Get load and inertia from input ports
			var Qload = load;
			var J = J0 + inertia;
			Vector<double> ydot = Vector<double>.Build.Dense(state.Count);
			// state=[Q Omega pidintegrator]
			if (phase == Phase.RUN) {
				ydot[0] = -K_eng*Kd/(tao_eng*J)*state[0] - K_eng*Kp/tao_eng*state[1] - K_eng*Ki/tao_eng*state[2] + K_eng*Kd/(tao_eng*J)*Qload;
				ydot[1] = 1.0/J*state[0] - 1.0/J*Qload;
				ydot[2] = state[1];
			} else if (phase == Phase.CUTOFF || phase == Phase.FAIL) {
				ydot[0] = 0;
				ydot[1] = -Qload/J - friction*state[1];
				ydot[2] = 0;
			} else if (phase == Phase.START) {
				ydot[0] = 0;
				if (starttime < startDelay)
					Solver.State[0] = 0;
				else if (Omega < accelerationRPM / 9.5492966)
					Solver.State[0] = startTorque;
				else
					Solver.State[0] = accelerationTorque;
				ydot[1] = state[0]/J - Qload/J;
				ydot[2] = 0;
			}
			return ydot;
		}
	}
}

