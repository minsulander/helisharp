using System;
using MathNet.Numerics.LinearAlgebra;
using Newtonsoft.Json;

namespace HeliSharp
{

    [Serializable]
    public class Rotor : ForceModel
    {

        /// Force model of a helicopter rotor.
        /// Based on Erik Backlund's model. Assumptions include quasi-steady dynamics and uniform inflow.
        /// No lead-lag dynamics simulated. Lots of other room for improvement...

        // Inputs
        [JsonIgnore]
        public double Density { get; set; }							// Air density input port
        [JsonIgnore]
        public double Collective { get; set; }						// Collective control input port, positive up
        [JsonIgnore]
        public double LongCyclic { get; set; }						// Longitudinal cyclic control input port, positive forward
        [JsonIgnore]
        public double LatCyclic { get; set; }						// Lateral cyclic control input port, positive right
        [JsonIgnore]
        public double RotSpeed { get; set; }						// Rotational speed input port
        [JsonIgnore]
        public double HeightAboveGround { get; set; }				// Height above ground - for ground effect

        // Outputs
        [JsonIgnore]
        public double ShaftTorque { get; private set; }
        [JsonIgnore]
        public double Inertia { get; private set; }
        [JsonIgnore]
        public Vector<double> WashVelocity { get; private set; }
        // the rest are forces in ForceModel

        // States
        [JsonIgnore]
        public double beta_0 { get; set; }
        [JsonIgnore]
        public double beta_sin { get; set; }
        [JsonIgnore]
        public double beta_cos { get; set; }
        private double lambda_i0, xi;
        [JsonIgnore]
        public double CT { get; set; }
        private double CQ,CH,CY;
        [JsonIgnore]
        public bool trimflow { get; set; }		// Set once to trim dynamic inflow model

        public enum Direction
        {
            CounterClockwise,
            Clockwise,
        }

        // Static parameters
        public int bladeCount;
        public double radius;
        public double effectiveRadius;
        public double effectiveRootCutout;
        public double chord;
        public double twist;
        public double liftSlope;
        public double liftOffset;
        public double drag;
        public double effectiveHingeOffset;
        public double pitchFlapCoupling;
        public double flapInertia;
        public double flapStiffness;
        public double designRPM;
        public double rotationalInertia;

        public Direction direction = Direction.CounterClockwise; // Direction of rotation, when viewed from above
        public double maxCollective;
        public double minCollective;
        public double maxLongCyclic;
        public double minLongCyclic;
        public double maxLatCyclic;
        public double minLatCyclic;


        public bool cyclicFlapping;					// Wether to allow cyclic flapping or not

        public bool useDynamicInflow;			// Use the dynamic inflow model?
        public bool useGroundEffectInflow = true;
        public bool applyTorque;				// Apply rotor torque?

        // Calculation names of parameters above
        private int Nb => bladeCount;			// Number of blades
        private int rotdir => direction == Direction.CounterClockwise ? 1 : -1;
        private double theta_0_max => maxCollective * Math.PI / 180.0;		// Maximum collective control deflection
        private double theta_0_min => minCollective * Math.PI / 180.0;		// Minimum collective control deflection
        private double theta_sin_max => maxLongCyclic * Math.PI / 180.0;	// Maximum longitudinal cyclic control deflection
        private double theta_sin_min => minLongCyclic * Math.PI / 180.0;	// Minimum longitudinal cyclic control deflection
        private double theta_cos_max => maxLatCyclic * Math.PI / 180.0;		// Maximum lateral cyclic control deflection
        private double theta_cos_min => minLatCyclic * Math.PI / 180.0;		// Minimum lateral cyclic control deflection
        private double R => radius;									        // Blade radius
        private double B => effectiveRadius;								// Effective blade radius (normalized)
        private double R0 => effectiveRootCutout;							// Effective blade root cutout (normalized)
        private double c => chord;									        // Blade chord
        private double k => twist;									        // Blade linear twist
        private double a => liftSlope;									    // Lift coefficient slope
        private double cl0 => liftOffset;								    // Cl offset at 0 angle of attack
        private double cd0 => drag;								            // Constant drag coefficient
        private double e => effectiveHingeOffset;							// Effective hinge offset
        private double theta_beta => pitchFlapCoupling;						// Pitch-flap coupling
        private double I_beta => flapInertia;								// Blade flap inertia
        private double k_beta => flapStiffness;								// Blade flap stiffness
        public double designOmega => designRPM / 9.5492966;				// Design rotation speed
        private double J0 => rotationalInertia;									    // Rotational inertia


        [NonSerialized]
        private InflowModel inflow;
        private double flapIterationTolerance = 1e-5;
        private int maxFlapIterations = 10;

        [JsonIgnore]
        public double RhoAOR2 { get { 
                double A=Math.PI*square(R);
                return Density * A * square(RotSpeed*R); 
        } }

        [JsonIgnore]
        public double RPM => RotSpeed * 9.5492966;

        public Rotor() : base()
        {
            inflow = new InflowModel ();

            WashVelocity = Vector<double>.Build.DenseOfArray(new double[] {0,0,0});
            cyclicFlapping = true;
            useDynamicInflow = false;
            trimflow = true;
            applyTorque = true;

            // set defaults
            Collective = LatCyclic = LongCyclic = 0;
            Density = 1.225;
            HeightAboveGround = 1000.0; // default to no ground effect
        }

        public Rotor LoadDefault ()
        {
            bladeCount = 4;
            radius = 5.486;
            effectiveRadius = 0.98;
            effectiveRootCutout = 0.2;
            chord = 0.3353;
            twist = 0.105;
            liftSlope = 5.2;
            liftOffset = 0;
            drag = 0.011;
            effectiveHingeOffset = 0.027;
            pitchFlapCoupling = 0;
            flapInertia = 287.43;
            flapStiffness = 0; // correct?
            designRPM = 385;
            rotationalInertia = 2711.6359; // kg m^2 = 2000 slug ft^2
            maxCollective = 18.1;
            minCollective = 4.4;
            maxLongCyclic = 12.5;
            minLongCyclic = -10.5;
            maxLatCyclic = 10.5;
            minLatCyclic = -7.5;
            cyclicFlapping = true;

            RotSpeed = designOmega;
            Inertia = J0;
            double col, lat, lon;
            GetNormalizedControlAngles (1, 0, 0, out col, out lat, out lon);
            Collective = col; LatCyclic = lat; LongCyclic = lon;
            useDynamicInflow = true;
            return this;
        }

        public void CopyParameters(Rotor r) {
            bladeCount = r.bladeCount;
            direction = r.direction;
            maxCollective = r.maxCollective;
            minCollective = r.minCollective;
            maxLongCyclic = r.maxLongCyclic;
            minLongCyclic = r.minLongCyclic;
            maxLatCyclic = r.maxLatCyclic;
            minLatCyclic = r.minLatCyclic;
            radius = r.radius;
            effectiveRadius = r.effectiveRadius;
            effectiveRootCutout = r.effectiveRootCutout;
            chord = r.chord;
            twist = r.twist;
            liftSlope = r.liftSlope;
            liftOffset = r.liftOffset;
            drag = r.drag;
            effectiveHingeOffset = r.effectiveHingeOffset;
            pitchFlapCoupling = r.pitchFlapCoupling;
            flapInertia = r.flapInertia;
            flapStiffness = r.flapStiffness;
            designRPM = r.designRPM;
            rotationalInertia = r.rotationalInertia;
            cyclicFlapping = r.cyclicFlapping;
            useDynamicInflow = r.useDynamicInflow;
            applyTorque = r.applyTorque;
            RotSpeed = r.RotSpeed;
            Inertia = r.Inertia;
            Collective = r.Collective;
            LatCyclic = r.LatCyclic;
            LongCyclic = r.LongCyclic;
        }

        public Rotor LoadDefaultTailRotor ()
        {
            bladeCount = 2;
            radius = 1;
            effectiveRadius = 0.98;
            effectiveRootCutout = 0.1;
            chord = 0.2;
            twist = 0;
            liftSlope = 5.0;
            liftOffset = 0;
            drag = 0.011;
            effectiveHingeOffset = 0;
            pitchFlapCoupling = 0;
            flapInertia = 0.24587;
            flapStiffness = 0;
            designRPM = 2080;
            rotationalInertia = 2.71163; // kg m^2 = 2 slug ft^2
            maxCollective = 23;
            minCollective = -7;
            cyclicFlapping = false;

            RotSpeed = designOmega;
            Inertia = J0;
            double col, lat, lon;
            GetNormalizedControlAngles (1, 0, 0, out col, out lat, out lon);
            Collective = col; LatCyclic = lat; LongCyclic = lon;
            useDynamicInflow = true;
            return this;

        }

        public override void Update(double dt)
        {
            // Validate inputs
            if (double.IsNaN(Density) || double.IsInfinity(Density)) throw new ModelException("Rotor density is " + Density);
            if (double.IsNaN(RotSpeed) || double.IsInfinity(RotSpeed)) throw new ModelException("Rotor rotational speed is " + RotSpeed);
            if (double.IsNaN(HeightAboveGround) || double.IsInfinity(HeightAboveGround)) throw new ModelException("Rotor height above ground is " + HeightAboveGround);
            if (double.IsNaN(Collective) || double.IsInfinity(Collective)) throw new ModelException("Rotor collective is " + Collective);
            if (cyclicFlapping && (double.IsNaN(LongCyclic) || double.IsInfinity(LongCyclic))) throw new ModelException("Rotor longitudinal cyclic is " + LongCyclic);
            if (cyclicFlapping && (double.IsNaN(LatCyclic) || double.IsInfinity(LatCyclic))) throw new ModelException("Rotor lateral cyclic is " + LatCyclic);

            // Constants
            double A=Math.PI*square(R);
            //sigma = Nb*R*(B-R0)*c/A;
            double sigma = Nb*c/(Math.PI*R);
            double lock_rho = a*c*powi(R,4)*(powi(B,4)-powi(R0,4))/I_beta;
            double k2 = 1+3*e/(2*(1-e));

            Warning = Error = false;
            WarningMessage = ErrorMessage = null;
            // Get values from input ports
            double rho = Density;
            Vector<double> V = Velocity;
            if (V.Norm(2) < 1e-10) V = Vector<double>.Build.Zero3(); // precision errors
            Vector<double> W = AngularVelocity;
            if (W.Norm(2) < 1e-10) W = Vector<double>.Build.Zero3(); // precision errors

            double Omega = RotSpeed;
            // If rotational speed is too low, just don't generate any forces (yes I'm lazy)
            if (Omega < 1.0) {
                Force = Vector<double>.Build.Zero3();
                Torque = Vector<double>.Build.Zero3();
                Inertia = J0;
                ShaftTorque = 0;
                WashVelocity = Vector<double>.Build.Zero3();
                return;
            }


            // Constants depending on rotation speed
            double M_betacos=-I_beta*square(Omega)*3*e/(2*(1-e))*2;
            double R_betasin=-I_beta*square(Omega)*3*e/(2*(1-e))*2;
            double lambda_beta=Math.Sqrt(k2+k_beta/(square(Omega)*I_beta));

            // Get control angles based on normalized inputs (from input ports) and min/max values (from parameters)
            double theta_0, theta_sin, theta_cos;
            GetControlAngles(out theta_0, out theta_sin, out theta_cos);
            if (double.IsNaN(theta_0) || double.IsInfinity(theta_0)) throw new ModelException("Rotor collective angle is " + theta_0);
            if (double.IsNaN(theta_sin) || double.IsInfinity(theta_sin)) throw new ModelException("Rotor longitudinal cyclic angle is " + theta_sin);
            if (double.IsNaN(theta_cos) || double.IsInfinity(theta_cos)) throw new ModelException("Rotor lateral cyclic angle is " + theta_cos);

            // Hub plane side-slip transformation (cases to avoid singularities)
            double deltaPsi=Math.Atan2(V[1],V[0]);

            // Transformation to wind-aligned system
            double u_w=Math.Sqrt(V[1]*V[1]+V[0]*V[0]);
            double w_w=V[2];
            double p_w=W[0]*Math.Cos(deltaPsi)+W[1]*Math.Sin(deltaPsi);
            double q_w=W[1]*Math.Cos(deltaPsi)-W[0]*Math.Sin(deltaPsi);
            double r_w=W[2];
            double theta_sinw=theta_cos*Math.Sin(deltaPsi)+theta_sin*Math.Cos(deltaPsi);
            double theta_cosw=theta_cos*Math.Cos(deltaPsi)-theta_sin*Math.Sin(deltaPsi);
            double beta_sinw=beta_cos*Math.Sin(deltaPsi)+beta_sin*Math.Cos(deltaPsi);
            double beta_cosw=beta_cos*Math.Cos(deltaPsi)-beta_sin*Math.Sin(deltaPsi);

            if (!cyclicFlapping) {
                theta_sinw = theta_cosw = 0.0;
            }

            // Tip speed ratios
            double mu_x=u_w/(Omega*R);
            double mu_z=w_w/(Omega*R);

            // Lock number
            double Gamma = lock_rho*rho;

            // Inflow properties
            double E_x = 0;

            // Calculate simulation state
            // Input: theta_0, theta_sin, theta_cos, velocities
            // Output: beta_0, beta_sin, beta_cos, forces and moments

            // Since inflow depends on previous CT and beta_cosw, iterate until residual is low enough
            double residual = 100.0;
            int iter=0;
            while (residual > flapIterationTolerance) {

                // Save previous state in order to evaluate residual
                double oldCT = CT;
                double oldBC = beta_cosw;

                // Lookup inflow
                bool usingDynamicInflow = useDynamicInflow && Math.Abs (CT) > 1e-8;
                if (usingDynamicInflow) {
                    if (trimflow) {
                        inflow.TrimDynamicInflow(mu_x, mu_z, beta_cosw, CT, R);
                        trimflow = false;
                    }
                    inflow.EvaluateDynamicInflow(dt, mu_x, mu_z, beta_cosw, CT, R, Omega, out lambda_i0, out xi, out E_x);
                } else {
                    InflowModel.EvaluateInflow(mu_x, mu_z, beta_cosw, CT, R, useGroundEffectInflow ? HeightAboveGround : 1000.0, out lambda_i0, out xi, out E_x);
                }

                double b0b0=2.0*square(lambda_beta)/Gamma+(1.0/4.0*powi(R0,4)-1.0/3.0*e*powi(R0,3)+square(mu_x)*(1.0/4.0*square(R0)-1.0/2.0*e*R0))*theta_beta-(1.0/4.0*powi(B,4)-1.0/3.0*e*powi(B,3)+square(mu_x)*(1.0/4.0*square(B)-1.0/2.0*e*B))*theta_beta;
                double b0bs=(1.0/3.0*powi(R0,3)-1.0/2.0*e*square(R0))*mu_x*theta_beta-(1.0/3.0*powi(B,3)-1.0/2.0*e*square(B))*mu_x*theta_beta;
                double b0bc=mu_x*(1.0/4.0*e*square(B)-1.0/2.0*B*square(e))-mu_x*(1.0/4.0*e*square(R0)-1.0/2.0*R0*square(e));

                double bsb0=2.0*(1.0/3.0*powi(R0,3)-1.0/2.0*e*square(R0))*mu_x*theta_beta-2.0*(1.0/3.0*powi(B,3)-1.0/2.0*e*square(B))*mu_x*theta_beta;
                double bsbs=2.0*(square(lambda_beta)-1.0)/Gamma+(1.0/4.0*powi(R0,4)-1.0/3.0*e*powi(R0,3)+3.0/4.0*square(mu_x)*(1.0/2.0*square(R0)-e*R0))*theta_beta-(1.0/4.0*powi(B,4)-1.0/3.0*e*powi(B,3)+3.0/4.0*square(mu_x)*(1.0/2.0*square(B)-e*B))*theta_beta;
                double bsbc=-1.0/4.0*square(mu_x)*(1.0/2.0*square(R0)-e*R0)+1.0/4.0*powi(R0,4)-2.0/3.0*e*powi(R0,3)+1.0/2.0*square(R0)*square(e)+1.0/4.0*square(mu_x)*(1.0/2.0*square(B)-e*B)-1.0/4.0*powi(B,4)+2.0/3.0*e*powi(B,3)-1.0/2.0*square(B)*square(e);

                double bcb0=(1.0/3.0*powi(B,3)-1.0/2.0*e*square(B))*mu_x-(1.0/3.0*powi(R0,3)-1.0/2.0*e*square(R0))*mu_x;
                double bcbs=1.0/4.0*powi(B,4)-2.0/3.0*e*powi(B,3)+1.0/2.0*square(B)*square(e)+1.0/4.0*square(mu_x)*(1.0/4.0*square(B)-e*B)-1.0/4.0*powi(R0,4)+2.0/3.0*e*powi(R0,3)-1.0/2.0*square(R0)*square(e)-1.0/4.0*square(mu_x)*(1.0/4.0*square(R0)-e*R0);
                double bcbc=2.0*(square(lambda_beta)-1.0)/Gamma-(1.0/4.0*powi(B,4)-1.0/3.0*e*powi(B,3)+1.0/4.0*square(mu_x)*(1.0/2.0*square(B)-e*B))*theta_beta+(1.0/4.0*powi(R0,4)-1.0/3.0*e*powi(R0,3)+1.0/4.0*square(mu_x)*(1.0/2.0*square(R0)-e*R0))*theta_beta;

                double b0t0=1.0/4.0*powi(B,4)-1.0/3.0*e*powi(B,3)+1.0/4.0*square(B)*square(mu_x)-1.0/2.0*square(mu_x)*e*B-1.0/4.0*powi(R0,4)+1.0/3.0*e*powi(R0,3)-1.0/4.0*square(R0)*square(mu_x)+1.0/2.0*square(mu_x)*e*R0;
                double b0ts=1.0/3.0*mu_x*powi(B,3)-1.0/2.0*mu_x*e*square(B)-1.0/3.0*mu_x*powi(R0,3)+1.0/2.0*mu_x*e*square(R0);
                double b0tc=0;
                double b0li=1.0/3.0*powi(B,3)-1.0/2.0*e*square(B)-1.0/3.0*powi(R0,3)+1.0/2.0*e*square(R0);
                double b0mz=1.0/3.0*powi(B,3)-1.0/2.0*e*square(B)-1.0/3.0*powi(R0,3)+1.0/2.0*e*square(R0);
                double b0pO=-1.0/12.0*mu_x*(2.0*powi(R0,3)-3.0*e*square(R0)-2.0*powi(B,3)+3.0*e*square(B));
                double b0qO=0;

                double bst0=2.0/3.0*mu_x*powi(B,3)-mu_x*e*square(B)-2.0/3.0*mu_x*powi(R0,3)+mu_x*e*square(R0);
                double bsts=1.0/4.0*powi(B,4)-1.0/3.0*e*powi(B,3)+3.0/8.0*square(B)*square(mu_x)-3.0/4.0*square(mu_x)*e*B-1.0/4.0*powi(R0,4)+1.0/3.0*e*powi(R0,3)-3.0/8.0*square(R0)*square(mu_x)+3.0/4.0*square(mu_x)*e*R0;
                double bstc=0;
                double bsli=1.0/2.0*mu_x*square(B)-mu_x*e*B-1.0/2.0*mu_x*square(R0)+mu_x*e*R0;
                double bsmz=1.0/2.0*mu_x*square(B)-mu_x*e*B-1.0/2.0*mu_x*square(R0)+mu_x*e*R0;
                double bspO=1.0/4.0*powi(B,4)-1.0/3.0*e*powi(B,3)-1.0/4.0*powi(R0,4)+1.0/3.0*e*powi(R0,3);
                double bsqO=-4.0*k2/Gamma;

                double bct0=0;
                double bcts=0;
                double bctc=1.0/4.0*powi(B,4)-1.0/3.0*e*powi(B,3)+1.0/8.0*square(B)*square(mu_x)-1.0/4.0*square(mu_x)*e*B-1.0/4.0*powi(R0,4)+1.0/3.0*e*powi(R0,3)-1.0/8.0*square(R0)*square(mu_x)+1.0/4.0*square(mu_x)*e*R0;
                double bcli=1.0/4.0*E_x*powi(B,4)-1.0/3.0*E_x*e*powi(B,3)-1.0/4.0*E_x*powi(R0,4)+1.0/3.0*E_x*e*powi(R0,3);
                double bcmz=0;
                double bcpO=4.0*k2/Gamma;
                double bcqO=1.0/4.0*powi(B,4)-1.0/3.0*e*powi(B,3)-1.0/4.0*powi(R0,4)+1.0/3.0*e*powi(R0,3);

                double b0c=1.0/60.0*(-15.0*powi(R0,4)*cl0+20.0*powi(R0,3)*cl0*e-15.0*square(R0)*cl0*square(mu_x)+30.0*R0*cl0*square(mu_x)*e-12.0*powi(B,5)*k*a+15.0*powi(B,4)*k*a*e-10.0*powi(B,3)*k*a*square(mu_x)+15.0*square(B)*k*a*square(mu_x)*e+12.0*powi(R0,5)*k*a-15.0*powi(R0,4)*k*a*e+10.0*powi(R0,3)*k*a*square(mu_x)-15.0*square(R0)*k*a*square(mu_x)*e+15.0*powi(B,4)*cl0-20.0*powi(B,3)*cl0*e+15.0*square(B)*cl0*square(mu_x)-30.0*B*cl0*square(mu_x)*e)/a;
                double bsc=1.0/6.0*mu_x*(-3.0*powi(B,4)*k*a+4.0*powi(B,3)*k*a*e+3.0*powi(R0,4)*k*a-4.0*powi(R0,3)*k*a*e+4.0*powi(B,3)*cl0-6.0*square(B)*cl0*e-4.0*powi(R0,3)*cl0+6.0*square(R0)*cl0*e)/a;
                double bcc=0;

                Matrix<double> LHS = Matrix<double>.Build.DenseOfArray (new double[,] {
                    { b0b0, b0bs, b0bc },
                    { bsb0, bsbs, bsbc },
                    { bcb0, bcbs, bcbc }
                });
                Vector<double> RHS = Vector<double>.Build.DenseOfArray (new double[] {
                    b0t0*theta_0 + b0ts*theta_sinw + b0tc*theta_cosw + b0li*lambda_i0 + b0mz*mu_z + b0pO * p_w/Omega + b0qO * q_w/Omega + b0c,
                    bst0*theta_0 + bsts*theta_sinw + bstc*theta_cosw + bsli*lambda_i0 + bsmz*mu_z + bspO * p_w/Omega + bsqO * q_w/Omega + bsc,
                    bct0*theta_0 + bcts*theta_sinw + bctc*theta_cosw + bcli*lambda_i0 + bcmz*mu_z + bcpO * p_w/Omega + bcqO * q_w/Omega + bcc
                });

                var x = LHS.Solve (RHS);
                beta_0 = x [0];
                beta_sinw = x [1];
                beta_cosw = x [2];

                if (!cyclicFlapping) {
                    beta_sinw = beta_cosw = 0.0;
                }

                CT=-1.0/24.0*sigma*(-6.0*Omega*powi(B,2)*a*lambda_i0+4.0*Omega*powi(R0,3)*a*theta_0-3.0*a*k*Omega*powi(R0,4)+3.0*a*k*Omega*powi(B,4)-6.0*Omega*mu_x*R0*a*beta_cosw*e-6.0*Omega*powi(B,2)*a*theta_beta*beta_sinw*mu_x-3.0*Omega*powi(R0,2)*powi(mu_x,2)*a*k-6.0*Omega*powi(B,2)*a*theta_sinw*mu_x+6.0*Omega*powi(mu_x,2)*R0*cl0-6.0*Omega*powi(mu_x,2)*B*cl0-4.0*Omega*powi(B,3)*cl0+6.0*Omega*powi(R0,2)*a*theta_beta*beta_sinw*mu_x+6.0*Omega*powi(R0,2)*a*mu_z+6.0*Omega*mu_x*B*a*beta_cosw*e-6.0*Omega*powi(B,2)*a*mu_z+4.0*Omega*powi(R0,3)*cl0+6.0*Omega*powi(R0,2)*a*lambda_i0+6.0*Omega*powi(mu_x,2)*R0*a*theta_beta*beta_0+6.0*Omega*powi(mu_x,2)*R0*a*theta_0-6.0*Omega*powi(mu_x,2)*B*a*theta_0-6.0*Omega*powi(mu_x,2)*B*a*theta_beta*beta_0+6.0*Omega*powi(R0,2)*a*theta_sinw*mu_x+3.0*powi(R0,2)*a*p_w*mu_x+3.0*Omega*powi(B,2)*powi(mu_x,2)*a*k-3.0*powi(B,2)*a*p_w*mu_x+4.0*Omega*powi(R0,3)*a*theta_beta*beta_0-4.0*Omega*powi(B,3)*a*theta_0-4.0*Omega*powi(B,3)*a*theta_beta*beta_0)/Omega;

                if ((usingDynamicInflow || !useGroundEffectInflow) && Math.Abs(lambda_i0) > 1e-5) {
                    // Increase thrust due to ground effect (Padfield eq 3.218 pg 141 (2nd edition))
                    CT *= 1.0/(1.0-1.0/16.0*square(R/(HeightAboveGround))/(1.0+square(mu_x/lambda_i0)));
                    break;
                }

                if (double.IsNaN(CT) || double.IsInfinity(CT)) throw new ModelException("Rotor thrust would be " + CT);

                residual = Math.Abs(CT-oldCT)+Math.Abs(beta_cosw-oldBC);
                ++iter;
                if (iter > maxFlapIterations) {
                    Warning = true;
                    WarningMessage = "Max iterations exceeded";
                    break;
                }

            }

            // Transform to hub frame
            beta_sin=-beta_cosw*Math.Sin(deltaPsi)+beta_sinw*Math.Cos(deltaPsi);
            beta_cos=beta_cosw*Math.Cos(deltaPsi)+beta_sinw*Math.Sin(deltaPsi);

            // Calculate hub forces
            CQ=1/96.0*sigma*(6.0*powi(R0,4)*a*theta_cosw*powi(Omega,2)*lambda_i0*E_x-16.0*powi(B,3)*a*powi(Omega,2)*beta_sinw*e*lambda_i0*E_x-12.0*powi(R0,4)*a*k*powi(Omega,2)*mu_z-12.0*powi(B,4)*a*p_w*Omega*beta_cosw+12.0*powi(B,4)*a*powi(Omega,2)*beta_sinw*lambda_i0*E_x-8.0*powi(B,3)*cl0*Omega*mu_x*p_w+8.0*powi(B,3)*a*theta_cosw*powi(Omega,2)*mu_x*beta_0-24.0*powi(B,2)*a*powi(lambda_i0,2)*powi(Omega,2)+8.0*powi(R0,3)*cl0*Omega*mu_x*p_w+16.0*powi(R0,3)*a*theta_beta*powi(Omega,2)*beta_0*mu_z+8.0*powi(R0,3)*a*theta_0*Omega*mu_x*p_w-16.0*powi(R0,3)*a*powi(Omega,2)*powi(beta_cosw,2)*e-16.0*powi(B,3)*a*q_w*Omega*beta_sinw*e-12.0*powi(R0,4)*a*q_w*Omega*beta_sinw-6.0*powi(B,4)*a*theta_beta*Omega*beta_sinw*p_w-12.0*powi(B,4)*a*q_w*lambda_i0*Omega*E_x-16.0*powi(B,3)*a*theta_beta*powi(Omega,2)*beta_0*lambda_i0+12.0*powi(B,4)*a*k*powi(Omega,2)*mu_z-8.0*powi(R0,3)*a*theta_cosw*powi(Omega,2)*mu_x*beta_0-16.0*powi(B,3)*cl0*powi(Omega,2)*lambda_i0-24.0*powi(R0,2)*a*powi(Omega,2)*beta_sinw*e*mu_x*beta_0-6.0*powi(B,4)*a*powi(Omega,2)*powi(beta_cosw,2)+6.0*powi(R0,4)*a*theta_sinw*powi(Omega,2)*beta_cosw+3.0*powi(B,2)*a*theta_cosw*powi(Omega,2)*powi(mu_x,2)*beta_sinw+6.0*powi(R0,4)*a*powi(Omega,2)*powi(beta_sinw,2)-8.0*powi(R0,3)*a*theta_beta*powi(Omega,2)*beta_0*mu_x*beta_cosw+12.0*powi(R0,2)*a*theta_sinw*powi(Omega,2)*mu_x*lambda_i0-12.0*powi(B,2)*a*theta_sinw*powi(Omega,2)*mu_x*lambda_i0-12.0*powi(R0,4)*a*powi(Omega,2)*beta_sinw*lambda_i0*E_x+16.0*powi(R0,3)*a*powi(Omega,2)*beta_sinw*e*lambda_i0*E_x+12.0*powi(B,2)*a*theta_0*powi(Omega,2)*mu_x*beta_cosw*e-16.0*powi(R0,3)*a*q_w*mu_x*Omega*beta_0-16.0*powi(B,3)*a*theta_beta*powi(Omega,2)*beta_0*mu_z+12.0*powi(B,4)*a*q_w*Omega*beta_sinw-8.0*powi(B,3)*a*k*powi(Omega,2)*mu_x*beta_cosw*e-6.0*powi(R0,4)*a*k*Omega*mu_x*p_w+16.0*powi(B,3)*a*powi(Omega,2)*powi(beta_sinw,2)*e-6.0*powi(B,4)*a*theta_beta*Omega*beta_cosw*q_w+12.0*powi(R0,2)*a*theta_sinw*powi(Omega,2)*mu_x*mu_z-6.0*powi(B,4)*a*theta_sinw*Omega*p_w+9.0*powi(R0,2)*a*powi(mu_x,2)*powi(Omega,2)*powi(beta_cosw,2)+6.0*powi(R0,4)*a*theta_beta*Omega*beta_sinw*p_w-3.0*powi(R0,2)*a*theta_cosw*powi(Omega,2)*powi(mu_x,2)*beta_sinw-3.0*powi(B,2)*a*powi(mu_x,2)*powi(Omega,2)*powi(beta_sinw,2)-12.0*powi(R0,2)*cd0*powi(Omega,2)*powi(mu_x,2)+16.0*powi(R0,3)*cl0*powi(Omega,2)*mu_z-8.0*powi(B,3)*a*theta_cosw*powi(Omega,2)*beta_sinw*e-12.0*powi(B,2)*a*theta_beta*powi(Omega,2)*beta_sinw*mu_x*mu_z+16.0*powi(B,3)*a*powi(Omega,2)*powi(beta_cosw,2)*e-6.0*powi(R0,4)*a*theta_cosw*powi(Omega,2)*beta_sinw-12.0*powi(B,2)*a*powi(Omega,2)*powi(beta_cosw,2)*powi(e,2)-3.0*powi(R0,2)*a*theta_sinw*powi(Omega,2)*powi(mu_x,2)*beta_cosw+6.0*powi(R0,4)*a*theta_cosw*Omega*q_w-12.0*powi(R0,2)*cl0*powi(Omega,2)*mu_x*beta_cosw*e+24.0*powi(B,2)*a*mu_z*powi(Omega,2)*mu_x*beta_cosw+12.0*powi(R0,2)*a*powi(Omega,2)*powi(beta_sinw,2)*powi(e,2)+12.0*powi(R0,4)*a*p_w*Omega*beta_cosw+8.0*powi(R0,3)*a*k*powi(Omega,2)*mu_x*beta_cosw*e+6.0*powi(B,2)*a*theta_beta*powi(Omega,2)*beta_sinw*powi(mu_x,2)*beta_cosw+3.0*powi(B,2)*a*theta_sinw*powi(Omega,2)*powi(mu_x,2)*beta_cosw+12.0*powi(B,2)*cl0*powi(Omega,2)*mu_x*beta_cosw*e+6.0*powi(R0,4)*a*theta_sinw*Omega*p_w+6.0*powi(R0,4)*a*theta_beta*Omega*beta_cosw*q_w-16.0*powi(R0,3)*a*powi(Omega,2)*powi(beta_sinw,2)*e+12.0*powi(B,2)*cd0*powi(Omega,2)*powi(mu_x,2)+6.0*powi(R0,4)*a*powi(p_w,2)+6.0*powi(R0,4)*a*powi(q_w,2)-24.0*powi(B,2)*a*powi(mu_z,2)*powi(Omega,2)-12.0*powi(B,2)*a*theta_sinw*powi(Omega,2)*mu_x*mu_z-9.0*powi(B,2)*a*powi(mu_x,2)*powi(Omega,2)*powi(beta_cosw,2)+6.0*powi(R0,4)*a*powi(Omega,2)*powi(beta_cosw,2)+6.0*powi(B,4)*a*theta_cosw*powi(Omega,2)*beta_sinw+8.0*powi(R0,3)*a*theta_cosw*powi(Omega,2)*beta_sinw*e-24.0*powi(R0,2)*a*mu_z*powi(Omega,2)*mu_x*beta_cosw-16.0*powi(B,3)*cl0*powi(Omega,2)*mu_z-8.0*powi(R0,3)*a*theta_sinw*powi(Omega,2)*beta_cosw*e-6.0*powi(R0,2)*a*theta_beta*powi(Omega,2)*beta_sinw*powi(mu_x,2)*beta_cosw+16.0*powi(R0,3)*a*q_w*Omega*beta_sinw*e-6.0*powi(B,4)*a*powi(Omega,2)*powi(beta_sinw,2)+12.0*powi(R0,2)*a*powi(Omega,2)*powi(beta_cosw,2)*powi(e,2)+12.0*powi(R0,2)*a*theta_beta*powi(Omega,2)*beta_sinw*mu_x*mu_z+12.0*powi(B,4)*cd0*powi(Omega,2)+6.0*powi(B,4)*a*k*Omega*mu_x*p_w-6.0*powi(B,4)*a*powi(q_w,2)-6.0*powi(B,4)*a*powi(p_w,2)+24.0*powi(R0,2)*a*powi(mu_z,2)*powi(Omega,2)-12.0*powi(R0,4)*cd0*powi(Omega,2)-6.0*powi(B,4)*a*powi(lambda_i0,2)*powi(Omega,2)*powi(E_x,2)-6.0*powi(B,4)*a*theta_sinw*powi(Omega,2)*beta_cosw+3.0*powi(R0,2)*a*powi(mu_x,2)*powi(Omega,2)*powi(beta_sinw,2)+8.0*powi(B,3)*a*theta_sinw*powi(Omega,2)*beta_cosw*e+16.0*powi(R0,3)*a*theta_0*powi(Omega,2)*lambda_i0+16.0*powi(R0,3)*a*theta_beta*powi(Omega,2)*beta_0*lambda_i0-16.0*powi(B,3)*a*theta_0*powi(Omega,2)*mu_z-12.0*powi(R0,4)*a*k*powi(Omega,2)*lambda_i0-16.0*powi(R0,3)*a*mu_x*powi(Omega,2)*beta_0*lambda_i0*E_x+12.0*powi(R0,2)*a*powi(mu_x,2)*powi(Omega,2)*powi(beta_0,2)+16.0*powi(R0,3)*cl0*powi(Omega,2)*lambda_i0+24.0*powi(B,2)*a*powi(Omega,2)*beta_sinw*e*mu_x*beta_0+24.0*powi(R0,2)*a*powi(lambda_i0,2)*powi(Omega,2)+6.0*powi(R0,4)*a*theta_beta*powi(Omega,2)*beta_cosw*lambda_i0*E_x+8.0*powi(R0,3)*a*theta_beta*Omega*beta_0*mu_x*p_w+6.0*powi(R0,4)*a*powi(lambda_i0,2)*powi(Omega,2)*powi(E_x,2)+16.0*powi(B,3)*a*mu_x*powi(Omega,2)*beta_0*lambda_i0*E_x-8.0*powi(B,3)*a*theta_beta*Omega*beta_0*mu_x*p_w+16.0*powi(R0,3)*a*theta_0*powi(Omega,2)*mu_z+48.0*powi(R0,2)*a*lambda_i0*powi(Omega,2)*mu_z-16.0*powi(B,3)*a*theta_0*powi(Omega,2)*lambda_i0+24.0*powi(B,2)*a*lambda_i0*powi(Omega,2)*mu_x*beta_cosw+16.0*powi(B,3)*a*p_w*Omega*beta_cosw*e-16.0*powi(R0,3)*a*p_w*Omega*beta_cosw*e-6.0*powi(B,4)*a*theta_cosw*Omega*q_w-12.0*powi(B,2)*a*powi(Omega,2)*powi(beta_sinw,2)*powi(e,2)+12.0*powi(B,4)*a*k*powi(Omega,2)*lambda_i0-48.0*powi(B,2)*a*lambda_i0*powi(Omega,2)*mu_z-24.0*powi(R0,2)*a*lambda_i0*powi(Omega,2)*mu_x*beta_cosw-12.0*powi(B,2)*a*powi(mu_x,2)*powi(Omega,2)*powi(beta_0,2)-12.0*powi(B,2)*a*theta_beta*powi(Omega,2)*beta_sinw*mu_x*lambda_i0+12.0*powi(B,2)*a*theta_beta*powi(Omega,2)*beta_0*mu_x*beta_cosw*e-16.0*powi(B,3)*a*powi(Omega,2)*beta_sinw*mu_x*beta_0+12.0*powi(R0,2)*a*theta_beta*powi(Omega,2)*beta_sinw*mu_x*lambda_i0-6.0*powi(B,4)*a*theta_beta*powi(Omega,2)*beta_cosw*lambda_i0*E_x+8.0*powi(B,3)*a*theta_beta*powi(Omega,2)*beta_0*mu_x*beta_cosw-8.0*powi(B,3)*a*theta_0*Omega*mu_x*p_w-12.0*powi(R0,2)*a*theta_0*powi(Omega,2)*mu_x*beta_cosw*e+16.0*powi(B,3)*a*q_w*mu_x*Omega*beta_0+12.0*powi(R0,4)*a*q_w*lambda_i0*Omega*E_x+16.0*powi(R0,3)*a*powi(Omega,2)*beta_sinw*mu_x*beta_0-6.0*powi(B,4)*a*theta_cosw*powi(Omega,2)*lambda_i0*E_x-12.0*powi(R0,2)*a*theta_beta*powi(Omega,2)*beta_0*mu_x*beta_cosw*e)/powi(Omega,2);
            CH=-1.0/96.0*sigma*(6.0*a*theta_cosw*Omega*mu_x*beta_sinw*e*B+9.0*a*theta_beta*beta_sinw*mu_x*powi(B,2)*p_w-12.0*a*theta_sinw*Omega*powi(R0,2)*mu_z-24.0*a*theta_0*Omega*mu_x*lambda_i0*R0+12.0*a*theta_sinw*Omega*powi(B,2)*lambda_i0+24.0*a*lambda_i0*Omega*beta_cosw*powi(B,2)+3.0*a*theta_cosw*Omega*mu_x*lambda_i0*E_x*powi(B,2)+6.0*a*theta_0*Omega*powi(mu_x,2)*beta_cosw*R0-6.0*a*lambda_i0*Omega*E_x*powi(B,2)*mu_x*beta_sinw+3.0*a*k*powi(B,2)*Omega*powi(mu_x,2)*beta_cosw-6.0*a*theta_sinw*Omega*powi(R0,2)*mu_x*beta_cosw-24.0*cd0*Omega*powi(B,2)*mu_x-6.0*a*mu_x*Omega*powi(beta_cosw,2)*powi(B,2)+3.0*a*theta_beta*beta_cosw*mu_x*powi(B,2)*q_w+6.0*a*Omega*powi(beta_sinw,2)*powi(B,2)*mu_x-24.0*a*theta_beta*Omega*beta_0*mu_x*lambda_i0*R0+6.0*a*mu_x*Omega*powi(beta_cosw,2)*powi(R0,2)+18.0*a*theta_sinw*Omega*mu_x*beta_cosw*e*R0-12.0*cl0*Omega*powi(B,2)*beta_cosw*e-12.0*a*k*powi(B,2)*Omega*mu_x*mu_z+12.0*a*theta_beta*Omega*beta_sinw*powi(B,2)*mu_z+8.0*a*theta_0*powi(B,3)*p_w-6.0*a*k*powi(B,4)*Omega*beta_cosw+24.0*cl0*Omega*mu_x*mu_z*B+8.0*cl0*powi(B,3)*p_w-12.0*a*theta_beta*Omega*beta_sinw*powi(R0,2)*mu_z+12.0*a*theta_beta*Omega*beta_cosw*mu_x*beta_sinw*e*R0-6.0*a*theta_cosw*Omega*mu_x*beta_sinw*e*R0-6.0*cl0*Omega*powi(mu_x,2)*beta_cosw*B-3.0*a*k*powi(R0,2)*Omega*powi(mu_x,2)*beta_cosw+6.0*a*powi(R0,2)*p_w*mu_x*beta_cosw+24.0*a*mu_z*Omega*beta_cosw*powi(B,2)-8.0*cl0*powi(R0,3)*p_w+12.0*a*theta_beta*Omega*beta_sinw*powi(B,2)*lambda_i0-6.0*a*theta_0*Omega*powi(mu_x,2)*beta_cosw*B-3.0*a*theta_cosw*Omega*mu_x*lambda_i0*E_x*powi(R0,2)+24.0*a*theta_0*Omega*mu_x*lambda_i0*B-12.0*a*theta_beta*Omega*beta_0*powi(B,2)*beta_cosw*e+12.0*a*theta_sinw*Omega*powi(B,2)*mu_z+24.0*cd0*Omega*powi(R0,2)*mu_x-24.0*a*lambda_i0*Omega*beta_cosw*powi(R0,2)+8.0*a*theta_beta*Omega*beta_0*powi(B,3)*beta_cosw+12.0*a*k*powi(R0,2)*Omega*mu_x*lambda_i0-24.0*cl0*Omega*mu_x*lambda_i0*R0+24.0*a*theta_beta*Omega*beta_0*mu_x*mu_z*B-12.0*a*theta_beta*Omega*beta_cosw*powi(mu_x,2)*beta_0*B+24.0*a*mu_z*powi(B,2)*p_w+3.0*a*theta_cosw*mu_x*powi(B,2)*q_w+9.0*a*theta_sinw*mu_x*powi(B,2)*p_w+12.0*cl0*Omega*powi(R0,2)*beta_cosw*e-8.0*a*k*powi(R0,3)*Omega*beta_cosw*e-12.0*a*theta_beta*Omega*beta_sinw*mu_x*beta_cosw*e*B-12.0*a*Omega*powi(beta_cosw,2)*e*mu_x*R0+12.0*a*Omega*powi(beta_sinw,2)*e*mu_x*R0-3.0*a*theta_beta*Omega*beta_cosw*mu_x*lambda_i0*E_x*powi(R0,2)-12.0*a*theta_0*Omega*powi(B,2)*beta_cosw*e+24.0*cl0*Omega*mu_x*lambda_i0*B+24.0*a*theta_0*Omega*mu_x*mu_z*B-6.0*a*theta_cosw*Omega*powi(mu_x,2)*beta_0*B-8.0*a*theta_0*powi(R0,3)*p_w+6.0*a*theta_cosw*Omega*powi(mu_x,2)*beta_0*R0-24.0*a*theta_beta*Omega*beta_0*mu_x*mu_z*R0-9.0*a*theta_beta*beta_sinw*mu_x*powi(R0,2)*p_w-12.0*a*k*powi(B,2)*Omega*mu_x*lambda_i0-48.0*a*lambda_i0*Omega*beta_cosw*e*B-12.0*a*theta_sinw*Omega*powi(R0,2)*lambda_i0-24.0*a*lambda_i0*powi(R0,2)*p_w+48.0*a*mu_z*Omega*beta_cosw*e*R0+6.0*a*theta_sinw*Omega*powi(B,2)*mu_x*beta_cosw+12.0*a*k*powi(R0,2)*Omega*mu_x*mu_z-3.0*a*theta_beta*beta_cosw*mu_x*powi(R0,2)*q_w-18.0*a*theta_sinw*Omega*mu_x*beta_cosw*e*B+8.0*a*k*powi(B,3)*Omega*beta_cosw*e-6.0*a*powi(B,2)*q_w*mu_x*beta_sinw-12.0*a*Omega*powi(beta_sinw,2)*e*mu_x*B+6.0*a*powi(R0,2)*q_w*mu_x*beta_sinw-3.0*a*theta_cosw*mu_x*powi(R0,2)*q_w-6.0*a*powi(B,2)*p_w*mu_x*beta_cosw-12.0*a*powi(mu_x,2)*Omega*beta_0*beta_sinw*R0-24.0*a*theta_0*Omega*mu_x*mu_z*R0-9.0*a*theta_sinw*mu_x*powi(R0,2)*p_w-48.0*a*mu_z*Omega*beta_cosw*e*B-24.0*a*mu_z*Omega*beta_cosw*powi(R0,2)+24.0*a*lambda_i0*powi(B,2)*p_w+8.0*cl0*Omega*powi(B,3)*beta_cosw+6.0*a*theta_cosw*Omega*powi(R0,2)*mu_x*beta_sinw+6.0*a*k*powi(R0,4)*Omega*beta_cosw-8.0*a*theta_beta*Omega*beta_0*powi(R0,3)*beta_cosw+12.0*a*powi(mu_x,2)*Omega*beta_0*beta_sinw*B-6.0*a*Omega*powi(beta_sinw,2)*powi(R0,2)*mu_x+6.0*a*k*powi(R0,4)*p_w+12.0*a*Omega*powi(beta_cosw,2)*e*mu_x*B-24.0*cl0*Omega*mu_x*mu_z*R0-6.0*a*k*powi(B,4)*p_w-6.0*a*theta_cosw*Omega*powi(B,2)*mu_x*beta_sinw-24.0*a*mu_z*powi(R0,2)*p_w+24.0*a*theta_beta*Omega*beta_0*mu_x*lambda_i0*B+6.0*cl0*Omega*powi(mu_x,2)*beta_cosw*R0-8.0*cl0*Omega*powi(R0,3)*beta_cosw+48.0*a*lambda_i0*Omega*beta_cosw*e*R0+6.0*a*lambda_i0*Omega*E_x*powi(R0,2)*mu_x*beta_sinw-8.0*a*theta_beta*beta_0*powi(R0,3)*p_w+8.0*a*theta_beta*beta_0*powi(B,3)*p_w-8.0*a*theta_0*Omega*powi(R0,3)*beta_cosw-12.0*a*theta_beta*Omega*beta_sinw*powi(R0,2)*lambda_i0+12.0*a*theta_beta*Omega*beta_cosw*powi(mu_x,2)*beta_0*R0+8.0*a*theta_0*Omega*powi(B,3)*beta_cosw+12.0*a*theta_beta*Omega*beta_0*powi(R0,2)*beta_cosw*e+12.0*a*theta_0*Omega*powi(R0,2)*beta_cosw*e+3.0*a*theta_beta*Omega*beta_cosw*mu_x*lambda_i0*E_x*powi(B,2))/Omega;
            CY=1/96.0*sigma*(-48.0*a*lambda_i0*Omega*mu_x*beta_0*B+6.0*a*theta_cosw*Omega*powi(R0,2)*mu_x*beta_cosw+48.0*a*mu_z*Omega*beta_sinw*e*B-8.0*cl0*Omega*powi(B,3)*beta_sinw+48.0*a*lambda_i0*Omega*beta_sinw*e*B+48.0*a*mu_z*Omega*mu_x*beta_0*R0-6.0*a*theta_sinw*Omega*mu_x*beta_sinw*e*R0+24.0*a*mu_z*powi(B,2)*q_w+3.0*a*theta_sinw*mu_x*powi(B,2)*q_w+6.0*a*k*powi(R0,4)*q_w-24.0*a*mu_z*Omega*beta_sinw*powi(B,2)-6.0*a*theta_beta*Omega*powi(beta_sinw,2)*mu_x*e*R0-3.0*a*theta_sinw*Omega*mu_x*lambda_i0*E_x*powi(R0,2)+6.0*a*powi(R0,2)*p_w*mu_x*beta_sinw-12.0*a*mu_x*Omega*beta_cosw*beta_sinw*powi(R0,2)-12.0*a*theta_beta*Omega*beta_sinw*powi(mu_x,2)*beta_0*B+6.0*a*theta_cosw*Omega*mu_x*beta_cosw*e*R0-3.0*a*theta_beta*beta_cosw*mu_x*powi(R0,2)*p_w-6.0*a*theta_cosw*Omega*mu_x*beta_cosw*powi(B,2)+24.0*a*powi(lambda_i0,2)*Omega*E_x*powi(B,2)-18.0*a*powi(B,2)*q_w*mu_x*beta_cosw+24.0*a*mu_z*Omega*beta_sinw*powi(R0,2)+12.0*a*theta_cosw*Omega*powi(B,2)*lambda_i0-18.0*a*mu_x*Omega*beta_cosw*lambda_i0*E_x*powi(B,2)-12.0*a*theta_0*Omega*powi(B,2)*mu_x*beta_0+6.0*a*theta_beta*Omega*powi(beta_cosw,2)*mu_x*e*R0-6.0*a*theta_beta*Omega*powi(beta_sinw,2)*powi(B,2)*mu_x+6.0*a*theta_beta*Omega*powi(beta_sinw,2)*mu_x*e*B+12.0*a*theta_beta*Omega*beta_sinw*powi(mu_x,2)*beta_0*R0+24.0*a*mu_z*Omega*lambda_i0*E_x*powi(B,2)+8.0*a*theta_beta*Omega*beta_0*powi(B,3)*lambda_i0*E_x-3.0*a*k*powi(R0,2)*Omega*powi(mu_x,2)*beta_sinw+12.0*cl0*Omega*powi(B,2)*beta_sinw*e+12.0*a*theta_beta*Omega*beta_cosw*powi(B,2)*mu_z-12.0*cl0*Omega*powi(R0,2)*beta_sinw*e-8.0*a*k*powi(B,3)*Omega*beta_sinw*e+8.0*cl0*Omega*powi(R0,3)*beta_sinw+3.0*a*theta_beta*Omega*beta_sinw*mu_x*lambda_i0*E_x*powi(B,2)-12.0*a*theta_cosw*Omega*powi(R0,2)*mu_z+6.0*a*theta_beta*Omega*powi(beta_sinw,2)*powi(R0,2)*mu_x+8.0*cl0*powi(B,3)*q_w-6.0*a*k*powi(B,4)*q_w-6.0*a*k*powi(R0,4)*Omega*beta_sinw-24.0*a*Omega*beta_sinw*e*mu_x*beta_cosw*B-3.0*a*theta_sinw*mu_x*powi(R0,2)*q_w-8.0*a*theta_0*powi(R0,3)*q_w-6.0*a*powi(B,2)*p_w*mu_x*beta_sinw-3.0*a*theta_beta*beta_sinw*mu_x*powi(R0,2)*q_w+24.0*a*Omega*beta_cosw*e*mu_x*beta_sinw*R0+12.0*a*theta_cosw*Omega*powi(B,2)*mu_z+6.0*a*k*powi(R0,4)*Omega*lambda_i0*E_x-24.0*a*lambda_i0*Omega*beta_sinw*powi(B,2)-6.0*a*theta_0*Omega*powi(mu_x,2)*beta_sinw*B+8.0*a*theta_0*powi(B,3)*q_w-6.0*a*theta_cosw*Omega*mu_x*beta_cosw*e*B-6.0*cl0*Omega*powi(mu_x,2)*beta_sinw*B-3.0*a*theta_cosw*mu_x*powi(R0,2)*p_w+6.0*cl0*Omega*powi(mu_x,2)*beta_sinw*R0+6.0*a*theta_sinw*Omega*mu_x*beta_sinw*e*B-36.0*a*powi(mu_x,2)*Omega*beta_cosw*beta_0*R0-8.0*a*theta_beta*beta_0*powi(R0,3)*q_w-6.0*a*k*powi(B,4)*Omega*lambda_i0*E_x+12.0*a*theta_beta*Omega*powi(beta_0,2)*powi(R0,2)*mu_x+12.0*cl0*Omega*powi(R0,2)*mu_x*beta_0+8.0*a*theta_beta*beta_0*powi(B,3)*q_w+12.0*a*theta_0*Omega*powi(R0,2)*mu_x*beta_0-12.0*a*theta_cosw*Omega*powi(R0,2)*lambda_i0-6.0*a*theta_beta*Omega*powi(beta_cosw,2)*mu_x*e*B-48.0*a*mu_z*Omega*beta_sinw*e*R0-12.0*a*theta_beta*Omega*powi(beta_0,2)*powi(B,2)*mu_x-6.0*a*theta_sinw*Omega*powi(B,2)*mu_x*beta_sinw-8.0*cl0*Omega*powi(R0,3)*lambda_i0*E_x-24.0*a*mu_z*powi(R0,2)*q_w-6.0*a*theta_beta*Omega*powi(beta_cosw,2)*mu_x*powi(B,2)+3.0*a*theta_cosw*mu_x*powi(B,2)*p_w-8.0*cl0*powi(R0,3)*q_w+12.0*a*mu_x*Omega*beta_cosw*beta_sinw*powi(B,2)+8.0*a*k*powi(R0,3)*Omega*beta_sinw*e+18.0*a*powi(R0,2)*q_w*mu_x*beta_cosw+3.0*a*theta_beta*beta_sinw*mu_x*powi(B,2)*q_w+6.0*a*k*powi(B,4)*Omega*beta_sinw-12.0*a*theta_beta*Omega*beta_cosw*powi(R0,2)*mu_z+3.0*a*k*powi(B,2)*Omega*powi(mu_x,2)*beta_sinw+6.0*a*theta_beta*Omega*powi(beta_cosw,2)*powi(R0,2)*mu_x+3.0*a*theta_beta*beta_cosw*mu_x*powi(B,2)*p_w+6.0*a*theta_sinw*Omega*powi(R0,2)*mu_x*beta_sinw+12.0*a*theta_beta*Omega*beta_0*powi(B,2)*beta_sinw*e-48.0*a*lambda_i0*Omega*beta_sinw*e*R0+36.0*a*powi(mu_x,2)*Omega*beta_cosw*beta_0*B-24.0*a*powi(lambda_i0,2)*Omega*E_x*powi(R0,2)-8.0*a*theta_beta*Omega*beta_0*powi(B,3)*beta_sinw+8.0*a*theta_beta*Omega*beta_0*powi(R0,3)*beta_sinw+6.0*a*theta_0*Omega*powi(mu_x,2)*beta_sinw*R0-8.0*a*theta_beta*Omega*beta_0*powi(R0,3)*lambda_i0*E_x+8.0*a*theta_0*Omega*powi(R0,3)*beta_sinw-8.0*a*theta_0*Omega*powi(B,3)*beta_sinw+18.0*a*mu_x*Omega*beta_cosw*lambda_i0*E_x*powi(R0,2)-12.0*a*theta_0*Omega*powi(R0,2)*beta_sinw*e+8.0*a*theta_0*Omega*powi(B,3)*lambda_i0*E_x-24.0*a*lambda_i0*powi(R0,2)*q_w+3.0*a*theta_sinw*Omega*mu_x*lambda_i0*E_x*powi(B,2)+12.0*a*theta_0*Omega*powi(B,2)*beta_sinw*e-3.0*a*theta_beta*Omega*beta_sinw*mu_x*lambda_i0*E_x*powi(R0,2)+24.0*a*lambda_i0*powi(B,2)*q_w-12.0*a*theta_beta*Omega*beta_0*powi(R0,2)*beta_sinw*e-6.0*a*theta_sinw*Omega*powi(mu_x,2)*beta_0*B-24.0*a*mu_z*Omega*lambda_i0*E_x*powi(R0,2)+12.0*a*theta_beta*Omega*beta_cosw*powi(B,2)*lambda_i0-8.0*a*theta_0*Omega*powi(R0,3)*lambda_i0*E_x+24.0*a*lambda_i0*Omega*beta_sinw*powi(R0,2)-12.0*cl0*Omega*powi(B,2)*mu_x*beta_0-12.0*a*theta_beta*Omega*beta_cosw*powi(R0,2)*lambda_i0-8.0*a*k*powi(R0,3)*Omega*mu_x*beta_0+6.0*a*theta_sinw*Omega*powi(mu_x,2)*beta_0*R0+8.0*a*k*powi(B,3)*Omega*mu_x*beta_0+8.0*cl0*Omega*powi(B,3)*lambda_i0*E_x-48.0*a*mu_z*Omega*mu_x*beta_0*B+48.0*a*lambda_i0*Omega*mu_x*beta_0*R0)/Omega;

            // Calculate forces and transform to hub frame
            double T=CT*rho*A*powi(Omega*R,2);
            double Q=CQ*rho*A*powi(Omega*R,2)*R;
            double XHw=-CH*rho*A*powi(Omega*R,2);
            double YYw=CY*rho*A*powi(Omega*R,2);
            double XHYh=XHw*Math.Cos(deltaPsi)-YYw*Math.Sin(deltaPsi);
            double YHYh=XHw*Math.Sin(deltaPsi)+YYw*Math.Cos(deltaPsi);

            Vector<double> F = Vector<double>.Build.DenseOfArray (new double[] {
                XHYh + T*Math.Sin(beta_cos)*Math.Cos(beta_sin),
                YHYh - T*Math.Cos(beta_cos)*Math.Sin(beta_sin),
                -T*Math.Cos(beta_cos)*Math.Cos(beta_sin)
            });
            Vector<double> M = Vector<double>.Build.DenseOfArray (new double[] {
                R_betasin*beta_sin - Q*Math.Sin(beta_cos)*Math.Cos(beta_sin),
                M_betacos*beta_cos + Q*Math.Cos(beta_cos)*Math.Sin(beta_sin),
                applyTorque ? Q*Math.Cos(beta_cos)*Math.Cos(beta_sin) * rotdir : 0
            });

            // Calculate downwash velocity vector
            double Vi = Omega*R*lambda_i0;
            Matrix<double> RM = Matrix<double>.Build.RotationZ (deltaPsi);
            WashVelocity = RM * Vector<double>.Build.DenseOfArray(new double[] {-Math.Sin(xi) * Vi, 0, Math.Cos(xi)*Vi});

            var Qshaft = Q*Math.Cos(beta_cos)*Math.Cos(beta_sin) * rotdir;
            var J = J0 * square(Math.Cos(beta_0)); // adjust inertia for coning
            // Validate output
            if (double.IsNaN(F.x()) || double.IsInfinity(F.x())) throw new ModelException("Rotor force Fx would be " + F.x());
            if (double.IsNaN(F.y()) || double.IsInfinity(F.y())) throw new ModelException("Rotor force Fy would be " + F.y());
            if (double.IsNaN(F.z()) || double.IsInfinity(F.z())) throw new ModelException("Rotor force Fz would be " + F.z());
            if (double.IsNaN(M.x()) || double.IsInfinity(M.x())) throw new ModelException("Rotor torque Mx would be " + M.x());
            if (double.IsNaN(M.y()) || double.IsInfinity(M.y())) throw new ModelException("Rotor torque My would be " + M.y());
            if (double.IsNaN(M.z()) || double.IsInfinity(M.z())) throw new ModelException("Rotor torque Mz would be " + M.z());
            if (double.IsNaN(Qshaft) || double.IsInfinity(Qshaft)) throw new ModelException("Rotor shaft torque would be " + Qshaft);
            if (double.IsNaN(J) || double.IsInfinity(J)) throw new ModelException("Rotor inertia would be " + J);
            // Set output ports
            Force = F;
            Torque = M;
            ShaftTorque = Qshaft;
            Inertia = J;
        }

        public void GetControlAngles(out double theta_0, out double theta_sin, out double theta_cos)
        {
            GetControlAnglesFromNormalized(Collective, LongCyclic, LatCyclic, out theta_0, out theta_sin, out theta_cos);
        }

        public void GetControlAnglesFromNormalized(double collective, double longitudinal, double lateral, out double theta_0, out double theta_sin, out double theta_cos)
        {
            theta_0 = limit( theta_0_min + (collective + 1.0)/2.0 * (theta_0_max-theta_0_min), theta_0_min, theta_0_max);
            if (cyclicFlapping) {
                theta_sin = limit(theta_sin_min + (-longitudinal + 1.0) / 2.0 * (theta_sin_max - theta_sin_min), theta_sin_min, theta_sin_max);
                theta_cos = limit(theta_cos_min + (-lateral + 1.0) / 2.0 * (theta_cos_max - theta_cos_min), theta_cos_min, theta_cos_max);
            } else {
                theta_sin = theta_cos = 0;
            }
        }

        public void GetNormalizedControlAngles(double theta_0, double theta_sin, double theta_cos, out double collective, out double longitudinal, out double lateral)
        {
            collective=2*(theta_0-theta_0_min)/(theta_0_max-theta_0_min)-1;
            longitudinal=-(2*(theta_sin-theta_sin_min)/(theta_sin_max-theta_sin_min)-1);
            lateral=-(2*(theta_cos-theta_cos_min)/(theta_cos_max-theta_cos_min)-1);
        }

        /** Get downwash velocity at some distance from rotor hub.
            Dreier eq 12.29, pg 257 */
        public Vector<double> GetDownwashVelocity(double distance)
        {
            return WashVelocity * (1+(distance/R)/Math.Sqrt(1+square(distance/R)));
        }

        /** Get radius of downwash streamtube at some distance from rotor hub.
            Dreier eq 12.30, pg 257 */
        public double GetDownwashRadius(double distance)
        {
            return R*Math.Sqrt(Math.Sqrt(1+square(distance/R))/(distance/R+Math.Sqrt(1+square(distance/R))));
        }


        private double limit(double input, double min, double max) {
            if (input > max) return max;
            if (input < min) return min;
            return input;
        }


        private double square(double x)
        {
            return x*x;
        }

        private double powi(double x, int y)
        {
            double o = x;
            for (int i=1; i < y; i++) o *= x;
            return o;
        }

    }
}

