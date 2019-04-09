using NUnit.Framework;
using System;
using HeliSharp;

namespace HeliSharp
{

    [TestFixture]
    public class EngineTest
    {

        Engine model = new Engine().LoadDefault();

        [Test]
        public void ModerateLoad_MaintainsRPM() {
            model.Init(100);
            model.load = 800;
            model.throttle = 1.0;

            for (double t = 0.0; t < 10.0; t += 0.1) {
                model.Update(0.1);
            }
            Console.WriteLine("RPM " + model.RPM.ToStr() + " designRPM " + model.designRPM.ToStr());
            Assert.IsTrue(model.RPM > 0.99*model.designRPM);
            Console.WriteLine("P " + (model.Qeng * model.Omega).ToStr() + " max " + model.maxPower);
            Assert.IsTrue(model.Qeng * model.Omega < 0.99*model.maxPower);
        }

        [Test]
        public void HighLoad_DroopsRPM() {
            model.Init(1000);
            model.load = 5000;
            model.throttle = 1.0;

            for (double t = 0.0; t < 10.0; t += 0.1) {
                model.Update(0.1);
            }
            Console.WriteLine("RPM " + model.RPM.ToStr() + " designRPM " + model.designRPM.ToStr());
            Assert.IsTrue(model.RPM < 0.99*model.designRPM);
            Console.WriteLine("P " + (model.Qeng * model.Omega).ToStr() + " max " + model.maxPower);
            Assert.IsTrue(model.Qeng * model.Omega >= 0.99*model.maxPower);
        }

        [Test]
        public void SuddenLowLoad_IncreasesRPM() {
            model.Init(1000);
            model.throttle = 1.0;

            for (double t = 0.0; t < 10.0; t += 0.1) {
                model.Update(0.1);
            }

            model.load = 0;
            for (double t = 0.0; t < 0.5; t += 0.1) {
                model.Update(0.1);
            }

            Console.WriteLine("RPM " + model.RPM.ToStr() + " designRPM " + model.designRPM.ToStr());
            Assert.IsTrue(model.RPM > model.designRPM);
        }


        [Test]
        public void StartStop() {
            model.InitStopped();
            model.throttle = 0.0;
            model.phase = Engine.Phase.START;

            for (double t = 0.0; t < 30.0; t += 0.1) {
                model.Update(0.1);
                Console.WriteLine("Phase " + model.phase + " RPM " + model.RPM.ToStr() + " accelerationRPM " + model.accelerationRPM.ToStr() + " Qeng " + model.Qeng.ToStr());
            }

            Assert.AreEqual(Engine.Phase.RUN, model.phase);
            Assert.IsTrue(model.RPM > 0.9 * model.accelerationRPM);
            Assert.IsTrue(model.RPM > 0.9 * model.designRPM * model.idleRatio);

            model.phase = Engine.Phase.CUTOFF;
            for (double t = 0.0; t < 30.0; t += 0.1) {
                model.Update(0.1);
                Console.WriteLine("Phase " + model.phase + " RPM " + model.RPM.ToStr() + " accelerationRPM " + model.accelerationRPM.ToStr() + " Qeng " + model.Qeng.ToStr());
            }

            Assert.IsTrue(model.RPM < model.accelerationRPM);
        }
    }
}

