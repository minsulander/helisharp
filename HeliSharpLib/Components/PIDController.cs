using System;

namespace HeliSharp
{
	[Serializable]
	public class PIDController
	{
		// Parameters
		public double kp, ki, kd;
		public double outputLimit = Double.PositiveInfinity, integratorLimit = Double.PositiveInfinity;
		public double inputScale = 1, desiredScale = 1;

		// State
		private double intsum, prev;

		public double Calculate(double dt, double desired, double input) {
			double error = desiredScale*desired - inputScale*input;
			double der = (error-prev)/dt;
			intsum += error*dt;
			intsum = limit(intsum, integratorLimit);
			double output = kp*error + ki*intsum + kd*der;
			prev = error;
			output = limit(output, outputLimit);
			return output;
		}

        public void Reset()
        {
            intsum = prev = 0.0;
        }

		private double limit(double val, double limit) {
			if (val > limit) val = limit;
			if (val < -limit) val = -limit;
			return val;
		}
	}
}

