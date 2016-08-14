using NUnit.Framework;
using System;
using HeliSharp;
using System.Web.Script.Serialization;

namespace HeliSharp
{
	[TestFixture]
	public class FlightControlSystemTest
	{
		[Test]
		public void TestTrimControl()
		{
			FlightControlSystem fcs = new FlightControlSystem();
			fcs.trimControl = true;

			fcs.TrimCollective = 0.5;
			fcs.CollectiveCommand = 0.5;
			fcs.Update(0.01);
			Assert.AreEqual(0.75, fcs.Collective);

			fcs.TrimCollective = -0.5;
			fcs.CollectiveCommand = -0.5;
			fcs.Update(0.01);
			Assert.AreEqual(-0.75, fcs.Collective);

			fcs.TrimCollective = -0.5;
			fcs.CollectiveCommand = 0.5;
			fcs.Update(0.01);
			Assert.AreEqual(0.25, fcs.Collective);

			fcs.TrimCollective = 0.5;
			fcs.CollectiveCommand = -0.5;
			fcs.Update(0.01);
			Assert.AreEqual(-0.25, fcs.Collective);

		}

	}
}

