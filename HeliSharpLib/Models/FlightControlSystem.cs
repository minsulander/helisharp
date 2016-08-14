using System;
using MathNet.Numerics.LinearAlgebra;
using HeliSharp;
using Newtonsoft.Json;

namespace HeliSharp
{
	[Serializable]
	public class FlightControlSystem
	{
		// Constants
		public const double MAX_ROLL_FOR_YAW_CONTROL = 85.0 * Math.PI / 180.0;

		// Parameters
		public bool enabled = true;
		public bool trimControl = true;
		public double longExpo = 1, latExpo = 1, pedalExpo = 1;
		public double yawDamperVelocity1, yawDamperVelocity2;

		public PIDController pitchRatePID, rollRatePID, yawRatePID, pitchPID, rollPID;

		// Inputs
		[JsonIgnore]
		public double CollectiveCommand { get; set; } // positive up
		[JsonIgnore]
		public double LongCommand { get; set; } // positive forward
		[JsonIgnore]
		public double LatCommand { get; set; } // positive right
		[JsonIgnore]
		public double PedalCommand { get; set; } // positive right
		[JsonIgnore]
		public double TrimCollective { get; set; }
		[JsonIgnore]
		public double TrimLongCyclic { get; set; }
		[JsonIgnore]
		public double TrimLatCyclic { get; set; }
		[JsonIgnore]
		public double TrimPedal { get; set; }
		[JsonIgnore]
		public Vector<double> TrimAttitude { get; set; }
		[JsonIgnore]
		public Vector<double> Velocity { get; set; }
		[JsonIgnore]
		public Vector<double> AngularVelocity { get; set; }
		[JsonIgnore]
		public Vector<double> Attitude { get; set; }

		// Outputs
		[JsonIgnore]
		public double Collective { get; private set; }
		[JsonIgnore]
		public double LongCyclic { get; private set; }
		[JsonIgnore]
		public double LatCyclic { get; private set; }
		[JsonIgnore]
		public double Pedal { get; private set; }

		public FlightControlSystem() {
			TrimAttitude = Vector<double>.Build.Zero3();
		}

		public FlightControlSystem LoadDefault() {
			pitchRatePID = new PIDController() {
				kp = 0.5,
				ki = 0.1,
				kd = 0.05,
				integratorLimit = 0.5,
				desiredScale = 0,
				outputLimit = 0.3
			};
			rollRatePID = new PIDController() {
				kp = 0.5,
				ki = 0.1,
				kd = 0.05,
				integratorLimit = 0.5,
				desiredScale = 0,
				outputLimit = 0.3,
			};
			yawRatePID = new PIDController() {
				kp = 2.0,
				ki = 1.5,
				kd = 0,
				integratorLimit = 0.5,
				outputLimit = 1,
			};
			pitchPID = new PIDController() {
				kp = 1.0,
				ki = 0.3,
				kd = 0.01,
				integratorLimit = 0.5,
				desiredScale = 0.5236, // 30 deg
				outputLimit = 0.25
			};
			rollPID = new PIDController() {
				kp = 1.0,
				ki = 0.3,
				kd = 0.01,
				integratorLimit = 1,
				desiredScale = 0.5236, // 30 deg
				outputLimit = 0.25
			};
			yawDamperVelocity1 = 10.0;
			yawDamperVelocity2 = 20.0;
			return this;
		}

		public void Update(double dt) {
			// Start with 1:1 control
			Collective = CollectiveCommand;
			LongCyclic = LongCommand;
			LatCyclic = LatCommand;
			Pedal = PedalCommand;

			// Apply exponential controls
			LongCyclic = Math.Sign(LongCyclic) * Math.Pow(Math.Abs(LongCyclic), longExpo);
			LatCyclic = Math.Sign(LatCyclic) * Math.Pow(Math.Abs(LatCyclic), latExpo);
			Pedal = Math.Sign(Pedal) * Math.Pow(Math.Abs(Pedal), pedalExpo);

			if (!enabled) return;

			if (trimControl) {
				Collective = TrimCollective + Collective * (1 - Math.Sign(Collective) * Math.Sign(TrimCollective) * Math.Abs(TrimCollective));
				LongCyclic = TrimLongCyclic + LongCyclic * (1 - Math.Sign(LongCyclic) * Math.Sign(TrimLongCyclic) * Math.Abs(TrimLongCyclic));
				LatCyclic = TrimLatCyclic + LatCyclic * (1 - Math.Sign(LatCyclic) * Math.Sign(TrimLatCyclic) * Math.Abs(TrimLatCyclic));
				Pedal = TrimPedal + Pedal * (1 - Math.Sign(Pedal) * Math.Sign(TrimPedal) * Math.Abs(TrimPedal));
			}

			if (pitchRatePID != null) {
				LongCyclic -= pitchRatePID.Calculate(dt, -LongCommand, AngularVelocity.y());
			}
			if (rollRatePID != null) {
				LatCyclic += rollRatePID.Calculate(dt, LatCommand, AngularVelocity.x());
			}
			if (pitchPID != null) {
				LongCyclic -= pitchPID.Calculate(dt, TrimAttitude.y() - LongCommand, Attitude.y());
			}
			if (rollPID != null) {
				LatCyclic += rollPID.Calculate(dt, TrimAttitude.x() + LatCommand, Attitude.x());
			}
			if (Attitude != null && Math.Abs(Attitude.x()) < MAX_ROLL_FOR_YAW_CONTROL) {
				double yawscale = 1.0;
				if (yawRatePID != null) {
					// Gradually decrease yaw damper output with airspeed
					if (yawDamperVelocity1 > 1e-5 && yawDamperVelocity2 > 1e-5) {
						double v = Velocity.x();
						if (v > yawDamperVelocity2)
							yawscale = 0.0;
						else if (v > yawDamperVelocity1)
							yawscale = 1.0 - (v - yawDamperVelocity1) / (yawDamperVelocity2 - yawDamperVelocity1);
					}
					Pedal += yawscale * yawRatePID.Calculate(dt, PedalCommand, AngularVelocity.z());
				}
			}
			Collective = limit(Collective, 1);
			LongCyclic = limit(LongCyclic, 1);
			LatCyclic = limit(LatCyclic, 1);
			Pedal = limit(Pedal, 1);
		}

		private double limit(double val, double limit) {
			if (val > limit) val = limit;
			if (val < -limit) val = -limit;
			return val;
		}
}
}

