using System;
using NUnit.Framework;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
    [TestFixture]
    public class QuadCopterTest : HelicopterTest
    {
        [Test]
        public void TorqueDirections()
        {
            QuadCopter model = (QuadCopter) new QuadCopter().LoadDefault();
            //model.Gravity.Enabled = true;
            //model.Collective = 0.5;
            model.Pedal = -1.0;
            model.Update(0.01);
            Console.WriteLine("F " + model.Force.ToStr());
            Console.WriteLine("Q " + model.Torque.ToStr());
            for (var i = 0; i < 4; i++)
            {
                Console.WriteLine("  CT" + i + " " + model.Rotors[i].CT.ToStr());
                Console.WriteLine("  b0" + i + " " + model.Rotors[i].beta_0.ToStr());
                Console.WriteLine("  F" + i + " " + model.Rotors[i].Force.ToStr());
                Console.WriteLine("  Q" + i + " " + model.Rotors[i].Torque.ToStr());
            }
        }

        [Test]
        public void TrimTheModel ()
        {
            QuadCopter model = (QuadCopter) new QuadCopter().LoadDefault();
            model.FCS.trimControl = false;
            model.TrimInit();
            model.Trim();

            LogTrimState(model);
            AssertTrimmed(model);
        }

        [Test]
        public void TestTrimSweep()
        {
            QuadCopter model = (QuadCopter) new QuadCopter().LoadDefault();
            model.FCS.trimControl = false;
            TrimSweep(model);
        }

        public override Helicopter SetupModelForSimulation()
        {
            QuadCopter model = (QuadCopter) new QuadCopter().LoadDefault();
            model.FCS.trimControl = false;
            model.TrimInit();
            model.Trim();
            return model;
        }


    }
}