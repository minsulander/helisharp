using System;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using Newtonsoft.Json;

namespace HeliSharp
{
    [Serializable]
    public class ForceAssembly : ForceModel
    {
        [JsonIgnore]
        public Dictionary<string, ForceModel> SubModels = new Dictionary<string, ForceModel>();

        public void SetModel(string name, ForceModel model)
        {
            SubModels[name] = model;
        }

        public ForceModel GetModel(string name)
        {
            return SubModels[name];
        }

        public override void Update(double dt)
        {
            Vector<double> F = Vector<double>.Build.Zero3 ();
            Vector<double> M = Vector<double>.Build.Zero3 ();
            Error = Warning = false;
            ErrorMessage = WarningMessage = null;
            foreach (string name in SubModels.Keys) {
                var model = SubModels[name];
                if (model == null || !model.Enabled)
                    continue;
                // Update model local velocity and forces/moments
                updateLocalVelocity (model);
                model.Update (dt);
                // Transform force/moment from model local reference frame
                var dF = model.ForceContribution;
                var dM = model.TorqueContribution;
                for (var i = 0; i < 3; i++) {
                    if (double.IsNaN (dF [i]) || double.IsNaN (dM [i])) {
                        Error = true;
                        ErrorMessage += name + ": NaN forces\n";
                    }
                }
                // Sanity check
                if (Math.Abs (dF.Norm (2)) < 1e10 && Math.Abs (dM.Norm (2)) < 1e10) {
                    F += dF;
                    M += dM;
                } else {
                    Error = true;
                    ErrorMessage += name + ": Unreasonable forces\n";
                }
                if (model.Warning) {
                    Warning = true;
                    WarningMessage += name + (model.WarningMessage != null ? ": " + model.WarningMessage : "") + "\n";
                }
                if (model.Error) {
                    Error = true;
                    ErrorMessage += name + (model.ErrorMessage != null ? ": " + model.ErrorMessage : "") + "\n";
                }
                Force = F;
                Torque = M;
            }
        }

        virtual protected void updateLocalVelocity (ForceModel model)
        {
            model.AngularVelocity = model.InvRotation * AngularVelocity;
            model.Velocity = model.InvRotation * (Velocity - model.Translation.Cross (AngularVelocity));
        }

    }
}

