using System;
using MathNet.Numerics.LinearAlgebra;
using Newtonsoft.Json;

namespace HeliSharp
{
    public abstract class Helicopter : ForceAssembly, IHelicopterControls
    {

        /// Abstract base class for all helicopter types

        // Control

        public FlightControlSystem FCS { get; set; }

        [JsonIgnore]
        public double Collective
        {
            // positive up
            get { return FCS.CollectiveCommand; }
            set { FCS.CollectiveCommand = value; }
        }

        [JsonIgnore]
        public double LongCyclic
        {
            // positive forward
            get { return FCS.LongCommand; }
            set { FCS.LongCommand = value; }
        }

        [JsonIgnore]
        public double LatCyclic
        {
            // positive right
            get { return FCS.LatCommand; }
            set { FCS.LatCommand = value; }
        }

        [JsonIgnore]
        public double Pedal
        {
            // positive right
            get { return FCS.PedalCommand; }
            set { FCS.PedalCommand = value; }
        }

        // Orientation

        public override Matrix<double> Rotation
        {
            get { return base.Rotation; }
            set
            {
                base.Rotation = value;
                Attitude = value.EulerAngles();
            }
        }

        // Attitude needs to be set from rigid body simulation
        // phi (roll +right), theta (pitch +down), psi (heading +right)

        private Vector<double> attitude = Vector<double>.Build.Zero3();

        [JsonIgnore]
        public Vector<double> Attitude
        {
            get { return attitude; }
            set
            {
                attitude = value;
                if (attitude[0] > Math.PI) attitude[0] -= 2 * Math.PI;
                if (attitude[0] < -Math.PI) attitude[0] += 2 * Math.PI;
                if (attitude[1] > Math.PI) attitude[1] -= 2 * Math.PI;
                if (attitude[1] < -Math.PI) attitude[1] += 2 * Math.PI;
                base.Rotation = Matrix<double>.Build.EulerRotation(attitude);
                Gravity.Rotation = Matrix<double>.Build.EulerRotation(-attitude[0], -attitude[1], -attitude[2]);
            }
        }

        [JsonIgnore]
        public double RollAngle
        {
            // positive right
            get { return attitude[0]; }
            set { attitude[0] = value; }
        }

        [JsonIgnore]
        public double PitchAngle
        {
            // positive up
            get { return attitude[1]; }
            set { attitude[1] = value; }
        }

        [JsonIgnore]
        public double Heading
        {
            // positive right
            get { return attitude[2]; }
            set { attitude[2] = value; }
        }

        // Inertia

        [JsonIgnore]
        public StaticForce Gravity { get; private set; }

        private double mass;

        public double Mass
        {
            get { return mass; }
            set
            {
                mass = value;
                Gravity.Force[2] = mass * 9.81;
            }
        }

        [JsonIgnore]
        public Matrix<double> Inertia { get; set; }

        // For serialization
        [JsonProperty("inertia")]
        public double[] InertiaD
        {
            get { return Inertia.ToColumnWiseArray(); }
            set { Inertia = Matrix<double>.Build.DenseOfColumnMajor(3, 3, value); }
        }

        // Atmospheric stuff

        [JsonIgnore]
        public Atmosphere Atmosphere { get; private set; }

        [JsonIgnore]
        public Vector<double> Wind { get; set; }

        public Vector<double> GroundVelocity
        {
            set
            {
                Matrix<double> R = Matrix<double>.Build.EulerRotation(attitude);
                Velocity = value - R * Wind;
            }
        }

        public Vector<double> AbsoluteVelocity
        {
            set { GroundVelocity = Gravity.Rotation * value; }
        }

        [JsonIgnore]
        public abstract double Height { get; set; }

        protected Helicopter()
        {
            Wind = Vector<double>.Build.Zero3();
            Gravity = new StaticForce(Vector<double>.Build.Dense3(0, 0, Mass * 9.81));
            SetModel("Gravity", Gravity);
            Atmosphere = new Atmosphere();

        }

        public override void Update(double dt)
        {
            // Update atmospheric properties
            Atmosphere.Position = Translation;
            Atmosphere.Update(dt);

            // Update FCS
            FCS.Velocity = Velocity;
            FCS.AngularVelocity = AngularVelocity;
            FCS.Attitude = Attitude;
            FCS.Update(dt);

            PreUpdate(dt);
            base.Update(dt);
            PostUpdate(dt);
        }

        public virtual void PreUpdate(double dt)
        {

        }

        public virtual void PostUpdate(double dt)
        {

        }
    }
}