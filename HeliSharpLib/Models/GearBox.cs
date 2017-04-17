using System;
using Newtonsoft.Json;

namespace HeliSharp
{
	[Serializable]
	public class GearBox
	{
		/// Simple clutch and gear box model for single main rotor helicopters.
		/// Somewhat improvised but based on Dreier's description.

		// Inputs
		[JsonIgnore] public bool BrakeEnabled { get; set; }
		[JsonIgnore] public double MainRotorLoad { get; internal set; }
		[JsonIgnore] public double TailRotorLoad { get; internal set; }
		[JsonIgnore] public double RotspeedDrive { get; internal set; }
		[JsonIgnore] public double MainRotorInertia { get; internal set; }
		[JsonIgnore] public double TailRotorInertia { get; internal set; }
		[JsonIgnore] public double MainRotorRatio { get; internal set; }
		[JsonIgnore] public double TailRotorRatio { get; internal set; }

		// Outputs
		[JsonIgnore] public double MainRotorSpeed { get; private set; }
		[JsonIgnore] public double TailRotorSpeed { get; private set; }
		[JsonIgnore] public double Load { get; private set; }
		[JsonIgnore] public double Inertia { get; private set; }

		// States
		internal double Omega;
		internal bool engaged;

		// Parameters
		public double friction;
        public double brakeTorque;
	    public double limitRPM;
        public double autoBrakeRPM;

		public GearBox() {
			MainRotorRatio = TailRotorRatio = 1.0;
			friction = 0;
			Omega = 0;
			engaged = true;
			MainRotorSpeed = 0;
			TailRotorSpeed = 0;
			Load = 0;
			limitRPM = 0;
		    autoBrakeRPM = 0;
			MainRotorInertia = 1;
			TailRotorInertia = 1;
			MainRotorLoad = 0;
			TailRotorLoad = 0;
		}

		public GearBox LoadDefault() {
			friction = 0.1697; // powerloss / engine.Omega0^2
            brakeTorque = 75;
		    limitRPM = 7500;
            autoBrakeRPM = 1500;
			return this;
		}

		public void Update(double dt) {
            if (autoBrakeRPM > 1e-5 && Omega < autoBrakeRPM / 9.5492966 && RotspeedDrive < Omega) BrakeEnabled = true;
		    var Qload = MainRotorLoad/MainRotorRatio + TailRotorLoad/TailRotorRatio + friction*Omega + (BrakeEnabled && Omega > 0 ? brakeTorque : 0);
		    var J = MainRotorInertia/(MainRotorRatio*MainRotorRatio) + TailRotorInertia/(TailRotorRatio*TailRotorRatio);
            // Clutch condition - if engine RPM >= rotor RPM, clutch engages load
            if (RotspeedDrive > Omega) {
				// Engine driving
				engaged = true;
				Omega = RotspeedDrive;
			} else {
				Omega -= Qload/J*dt;
				// Reduce the load when there is a bit of margin between rotation speeds
				// (otherwise the load just oscillates between updates)
				if (RotspeedDrive / Omega < 0.99) {
					// Clutch disengaged - rotors spin freely
					engaged = false;
					Qload = 0;
					J = 0;
				}
			}
			if (limitRPM > 1e-5 && Omega > limitRPM / 9.5492966) {
				Omega = limitRPM / 9.5492966;
			}
		    var OmegaMR = Omega / MainRotorRatio;
		    var OmegaTR = Omega / TailRotorRatio;
		    // Validate output
		    if (double.IsNaN(Qload) || double.IsInfinity(Qload)) throw new ModelException("Gearbox load would be " + Qload);
		    if (double.IsNaN(J) || double.IsInfinity(J)) throw new ModelException("Gearbox inertia would be " + J);
		    if (double.IsNaN(OmegaMR) || double.IsInfinity(OmegaMR)) throw new ModelException("Gearbox main rotor speed would be " + OmegaMR);
		    if (double.IsNaN(OmegaTR) || double.IsInfinity(OmegaTR)) throw new ModelException("Gearbox tail rotor speed would be " + OmegaTR);
		    // Set output ports
		    Load = Qload;
		    Inertia = J;
			MainRotorSpeed = OmegaMR;
			TailRotorSpeed = OmegaTR;
		}
	}
}

