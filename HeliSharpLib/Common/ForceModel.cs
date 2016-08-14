using System;
using MathNet.Numerics.LinearAlgebra;
using HeliSharp;
using System.Web.Script.Serialization;
using System.Collections.Generic;
using Newtonsoft.Json;

namespace HeliSharp
{
	[Serializable]
	public abstract class ForceModel
	{

		[JsonIgnore]
		public bool Enabled { get; set; }

		[JsonIgnore]
		public Vector<double> Velocity, AngularVelocity;
		[JsonIgnore]
		public Vector<double> Force, Torque;

		[JsonIgnore]
		public virtual Vector<double> Translation { get; set; }

		// For serialization
		[JsonProperty("translation")]
		public double[] TranslationD {
			get { return Translation.ToArray(); }
			set { Translation = Vector<double>.Build.DenseOfArray(value); }
		}

		[JsonIgnore]
		public virtual Matrix<double> Rotation { 
			get { return this.rotation; }
			set { 
				this.rotation = value;
				this.invRotation = value.Inverse (); 
			}
		}

		// For serialization
		[JsonProperty("rotation")]
		public double[] RotationD {
			get { return Rotation.ToColumnWiseArray(); }
			set { Rotation = Matrix<double>.Build.DenseOfColumnMajor(3, 3, value); }
		}

		[JsonIgnore]
		public Vector<double> ForceContribution { get { return Rotation * Force; } }
		[JsonIgnore]
		public Vector<double> TorqueContribution { get { return Rotation * Torque + Translation.Cross(ForceContribution); } }

		[JsonIgnore]
		public Matrix<double> InvRotation { get { return invRotation; }}
		private Matrix<double> rotation, invRotation;

		[JsonIgnore]
		public bool Error { get; protected set; }
		[JsonIgnore]
		public string ErrorMessage { get; protected set; }
		[JsonIgnore]
		public bool Warning { get; protected set; }
		[JsonIgnore]
		public string WarningMessage { get; protected set; }

		public ForceModel ()
		{
			Velocity = Vector<double>.Build.Zero3 ();
			AngularVelocity = Vector<double>.Build.Zero3 ();
			Force = Vector<double>.Build.Zero3 ();
			Torque = Vector<double>.Build.Zero3 ();
			Translation = Vector<double>.Build.Zero3();
			rotation = invRotation = Matrix<double>.Build.DenseIdentity(3);
			Enabled = true;
		}

		public abstract void Update(double dt);
	}
}

