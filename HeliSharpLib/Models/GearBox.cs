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
		public double friction, limit;
        public double brakeTorque;
        public double autoBrakeOmega;

		public GearBox() {
			MainRotorRatio = TailRotorRatio = 1.0;
			friction = 0;
			Omega = 0;
			engaged = true;
			MainRotorSpeed = 0;
			TailRotorSpeed = 0;
			Load = 0;
			limit = 0;
			MainRotorInertia = 1;
			TailRotorInertia = 1;
			MainRotorLoad = 0;
			TailRotorLoad = 0;
		}

		public GearBox LoadDefault() {
			limit = 785; // 7500 rpm
			friction = 0.1697; // powerloss / engine.Omega0^2
            brakeTorque = 75;
            autoBrakeOmega = 150.0;
			return this;
		}

		public void Update(double dt) {
            if (autoBrakeOmega > 1e-5 && Omega < autoBrakeOmega && RotspeedDrive < Omega) BrakeEnabled = true;
            // Clutch condition - if engine RPM >= rotor RPM, clutch engages load
            if (RotspeedDrive > Omega) {
				// Engine driving
				engaged = true;
				Omega = RotspeedDrive;
				Load = MainRotorLoad/MainRotorRatio + TailRotorLoad/TailRotorRatio + friction*Omega + (BrakeEnabled && Omega > 0 ? brakeTorque : 0);
				Inertia = MainRotorInertia/(MainRotorRatio*MainRotorRatio) + TailRotorInertia/(TailRotorRatio*TailRotorRatio);
			} else {
				double J = MainRotorInertia/(MainRotorRatio*MainRotorRatio) + TailRotorInertia/(TailRotorRatio*TailRotorRatio);
				double Q = MainRotorLoad/MainRotorRatio + TailRotorLoad/TailRotorRatio + friction*Omega + (BrakeEnabled && Omega > 0 ? brakeTorque : 0);
				Omega -= Q/J*dt;
				// Reduce the load when there is a bit of margin between rotation speeds
				// (otherwise the load just oscillates between updates)
				if (RotspeedDrive / Omega < 0.99) {
					// Clutch disengaged - rotors spin freely
					engaged = false;
					Load = 0;
					Inertia = 0;
				}
			}
			if (limit > 1e-5 && Omega > limit) {
				Omega = limit;
				//warnflag = true;
				//warnstr = "Limiting rotation speed";
			}
			MainRotorSpeed = Omega / MainRotorRatio;
			TailRotorSpeed = Omega / TailRotorRatio;
		}
	}
}

