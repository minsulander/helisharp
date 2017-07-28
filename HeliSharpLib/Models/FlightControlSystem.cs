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
        public bool verticalRateControl = true;
        public bool attitudeControl = true;
        public bool positionControl = true;
        public double maxPitchAngle = 30; // degrees
        public double maxRollAngle = 30; // degrees
	    public double collectiveDirect = 1, longDirect = 1, latDirect = 1, pedalDirect = 1;
		public double yawDamperVelocity1, yawDamperVelocity2;
	    public double collectiveNullZone, longNullZone, latNullZone, pedalNullZone;
	    public double collectiveExpo = 1, longExpo = 1, latExpo = 1, pedalExpo = 1;
        public double positionResetVelocity;

		public PIDController pitchRatePID, rollRatePID, yawRatePID, verticalRatePID, pitchPID, rollPID, longitudinalPID, lateralPID;

		// Inputs
		[JsonIgnore]
		public double CollectiveInput { get; set; } // positive up
		[JsonIgnore]
		public double LongInput { get; set; } // positive forward
		[JsonIgnore]
		public double LatInput { get; set; } // positive right
		[JsonIgnore]
		public double PedalInput { get; set; } // positive right
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
	    public Vector<double> HorizonVelocity { get; set; }
		[JsonIgnore]
		public Vector<double> Velocity { get; set; }
		[JsonIgnore]
		public Vector<double> AngularVelocity { get; set; }
		[JsonIgnore]
		public Vector<double> Attitude { get; set; }
        [JsonIgnore]
        public bool IsOnGround { get; set; }
        [JsonIgnore]
        public double LongitudinalPositionBypass { get; set; }

		// Outputs
		[JsonIgnore]
		public double Collective { get; private set; }
		[JsonIgnore]
		public double LongCyclic { get; private set; }
		[JsonIgnore]
		public double LatCyclic { get; private set; }
		[JsonIgnore]
		public double Pedal { get; private set; }

	    [JsonIgnore]
	    public double CollectiveCommand { get; private set; }
	    [JsonIgnore]
	    public double LongCommand { get; private set; }
	    [JsonIgnore]
	    public double LatCommand { get; private set; }
	    [JsonIgnore]
	    public double PedalCommand { get; private set; }
        [JsonIgnore]
        public double LongPositionOutput { get; private set; }

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
            verticalRatePID = new PIDController() {
                kp = 1.0,
                ki = 0.1,
                kd = 0,
                integratorLimit = 0.5,
                desiredScale = 15,
                outputLimit = 0.5,
            };
            pitchPID = new PIDController() {
                kp = 2.0,
                ki = 0.5,
                kd = 0.01,
                integratorLimit = 0.5,
                desiredScale = 1,
                outputLimit = 0.25
            };
            rollPID = new PIDController() {
                kp = 2.0,
                ki = 0.5,
                kd = 0.01,
                integratorLimit = 1,
                desiredScale = 1,
                outputLimit = 0.25
            };
            longitudinalPID = new PIDController() {
                kp = 0.05,
                ki = 0.01,
                kd = 0.005,
                outputLimit = 0.5,
                integratorLimit = 1,
                desiredScale = 10,
            };
            lateralPID = new PIDController() {
                kp = 0.1,
                ki = 0.01,
                outputLimit = 0.5,
                integratorLimit = 1,
                desiredScale = 10,
            };
			yawDamperVelocity1 = 10.0;
			yawDamperVelocity2 = 20.0;
			return this;
		}

        public void Update(double dt) {
            CollectiveCommand = CollectiveInput;
            LongCommand = LongInput;
            LatCommand = LatInput;
            PedalCommand = PedalInput;

            if (enabled) {
                // Apply nullzones
                if (trimControl && collectiveNullZone > 1e-5) CollectiveCommand = Math.Abs(CollectiveCommand) > collectiveNullZone
                        ? (CollectiveCommand - Math.Sign(CollectiveCommand) * collectiveNullZone) / (1.0 - collectiveNullZone)
                        : 0.0;
                if (longNullZone > 1e-5) LongCommand = Math.Abs(LongCommand) > longNullZone
                        ? (LongCommand - Math.Sign(LongCommand) * longNullZone) / (1.0 - longNullZone)
                        : 0.0;
                if (latNullZone > 1e-5) LatCommand = Math.Abs(LatCommand) > latNullZone
                        ? (LatCommand - Math.Sign(LatCommand) * latNullZone) / (1.0 - latNullZone)
                        : 0.0;
                if (pedalNullZone > 1e-5) PedalCommand = Math.Abs(PedalCommand) > pedalNullZone
                        ? (PedalCommand - Math.Sign(PedalCommand) * pedalNullZone) / (1.0 - pedalNullZone)
                        : 0.0;

                // Apply exponential controls
                if (trimControl && collectiveExpo > 1e-5) CollectiveCommand = Math.Sign(CollectiveCommand) * Math.Pow(Math.Abs(CollectiveCommand), collectiveExpo);
                if (longExpo > 1e-5) LongCyclic = Math.Sign(LongCommand) * Math.Pow(Math.Abs(LongCommand), longExpo);
                if (latExpo > 1e-5) LatCyclic = Math.Sign(LatCommand) * Math.Pow(Math.Abs(LatCommand), latExpo);
                if (pedalExpo > 1e-5) PedalCommand = Math.Sign(PedalCommand) * Math.Pow(Math.Abs(PedalCommand), pedalExpo);
            }

            // Start with 1:1 control
            Collective = CollectiveCommand * collectiveDirect;
            LongCyclic = LongCommand * longDirect;
            LatCyclic = LatCommand * latDirect;
            Pedal = PedalCommand * pedalDirect;

            if (!enabled) return;

            // Apply PID controllers
            if (pitchRatePID != null) {
                LongCyclic -= pitchRatePID.Calculate(dt, -LongCommand, AngularVelocity.y());
            }
            if (rollRatePID != null) {
                LatCyclic += rollRatePID.Calculate(dt, LatCommand, AngularVelocity.x());
            }
            if (verticalRateControl && verticalRatePID != null) {
                if (IsOnGround) {
                    verticalRatePID.Reset();
                    if (trimControl) CollectiveCommand -= 0.1;
                }
                Collective += verticalRatePID.Calculate(dt, CollectiveCommand, -HorizonVelocity.z());
            }

            var desiredPitchAngle = TrimAttitude.y();
            var desiredRollAngle = TrimAttitude.x();
            if (attitudeControl) {
                desiredPitchAngle -= LongCommand * maxPitchAngle * Math.PI / 180.0;
                desiredRollAngle += LatCommand * maxRollAngle * Math.PI / 180.0;
            }
            if (positionControl) {
                if (longitudinalPID != null) {
                    if (IsOnGround || LongCommand > 1e-5 || Math.Abs(HorizonVelocity.x()) > positionResetVelocity) longitudinalPID.Reset();
                    LongPositionOutput = longitudinalPID.Calculate(dt, LongCommand, HorizonVelocity.x());
                    // Longitudinal pitch bypass used for tiltrotor control
                    desiredPitchAngle -= (1.0 - LongitudinalPositionBypass) * LongPositionOutput * maxPitchAngle * Math.PI / 180.0;

                }
                if (lateralPID != null) {
                    if (IsOnGround || LatCommand > 1e-5 || Math.Abs(HorizonVelocity.y()) > positionResetVelocity) lateralPID.Reset();
                    desiredRollAngle += lateralPID.Calculate(dt, LatCommand, HorizonVelocity.y()) * maxRollAngle * Math.PI / 180.0;
                }
            }
            if (attitudeControl || positionControl) {
                if (pitchPID != null) {
                    LongCyclic -= pitchPID.Calculate(dt, desiredPitchAngle, Attitude.y());
                }
                if (rollPID != null) {
                    LatCyclic += rollPID.Calculate(dt, desiredRollAngle, Attitude.x());
                }
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

		    if (trimControl) {
		        Collective = TrimCollective + Collective * (1 - Math.Sign(Collective) * Math.Sign(TrimCollective) * Math.Abs(TrimCollective));
		        LongCyclic = TrimLongCyclic + LongCyclic * (1 - Math.Sign(LongCyclic) * Math.Sign(TrimLongCyclic) * Math.Abs(TrimLongCyclic));
		        LatCyclic = TrimLatCyclic + LatCyclic * (1 - Math.Sign(LatCyclic) * Math.Sign(TrimLatCyclic) * Math.Abs(TrimLatCyclic));
		        Pedal = TrimPedal + Pedal * (1 - Math.Sign(Pedal) * Math.Sign(TrimPedal) * Math.Abs(TrimPedal));
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

