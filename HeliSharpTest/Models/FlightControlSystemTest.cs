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
			fcs.CollectiveInput = 0.5;
			fcs.Update(0.01);
			Assert.AreEqual(0.75, fcs.Collective, 0.001);

			fcs.TrimCollective = -0.5;
			fcs.CollectiveInput = -0.5;
			fcs.Update(0.01);
			Assert.AreEqual(-0.75, fcs.Collective, 0.001);

			fcs.TrimCollective = -0.5;
			fcs.CollectiveInput = 0.5;
			fcs.Update(0.01);
			Assert.AreEqual(0.25, fcs.Collective, 0.001);

			fcs.TrimCollective = 0.5;
			fcs.CollectiveInput = -0.5;
			fcs.Update(0.01);
			Assert.AreEqual(-0.25, fcs.Collective, 0.001);

		}

	    [Test]
	    public void NullZone_Below_Returns0() {
	        FlightControlSystem fcs = new FlightControlSystem();
	        fcs.longNullZone = 0.5;
	        fcs.LongInput = 0.3;
	        fcs.Update(0.01);
	        Assert.AreEqual(0.0, fcs.LongCyclic, 0.001);
	    }

	    [Test]
	    public void NullZone_BelowNegative_Returns0() {
	        FlightControlSystem fcs = new FlightControlSystem();
	        fcs.longNullZone = 0.5;
	        fcs.LongInput = -0.2;
	        fcs.Update(0.01);
	        Assert.AreEqual(0.0, fcs.LongCyclic, 0.001);
	    }

	    [Test]
	    public void NullZone_Above_Scales() {
	        FlightControlSystem fcs = new FlightControlSystem();
	        fcs.latNullZone = 0.5;
	        fcs.LatInput = 0.75;
	        fcs.Update(0.01);
	        Assert.AreEqual(0.5, fcs.LatCyclic, 0.001);
	    }

	    [Test]
	    public void NullZone_AboveNegative_Scales() {
	        FlightControlSystem fcs = new FlightControlSystem();
	        fcs.latNullZone = 0.2;
	        fcs.LatInput = -0.8;
	        fcs.Update(0.01);
	        Assert.AreEqual(-0.75, fcs.LatCyclic, 0.001);
	    }

	    [Test]
	    public void CollectiveNullZone_TrimControl_Applies() {
	        FlightControlSystem fcs = new FlightControlSystem();
	        fcs.trimControl = true;
	        fcs.collectiveNullZone = 0.2;
	        fcs.CollectiveInput = 0.8;
	        fcs.Update(0.01);
	        Assert.AreEqual(0.75, fcs.Collective, 0.001);
	    }

	    [Test]
	    public void CollectiveNullZone_NotTrimControl_DoesNotApply() {
	        FlightControlSystem fcs = new FlightControlSystem();
	        fcs.trimControl = false;
	        fcs.collectiveNullZone = 0.2;
	        fcs.CollectiveInput = 0.8;
	        fcs.Update(0.01);
	        Assert.AreEqual(0.8, fcs.Collective, 0.001);
	    }

	}
}

