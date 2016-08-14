using NUnit.Framework;
using System;
using HeliSharp;
using System.Web.Script.Serialization;

namespace HeliSharp
{
	[TestFixture]
	public class StabilizerTest
	{
		[Test]
		public void TestAirfoil()
		{
			Airfoil airfoil = Airfoil.Get("NACA0012");
			Assert.AreEqual(0.755, airfoil.CL(-165), 0.01);
			Assert.AreEqual(0.256, airfoil.CD(-165), 0.01);
			Assert.AreEqual(0.326, airfoil.CM(-165), 0.01);
		}

		[Test]
		public void TestSerialize()
		{
			Stabilizer model = new Stabilizer().LoadDefaultHorizontal();

			// One way to skin a cat...
			var json = new JavaScriptSerializer().Serialize(model);
			Console.WriteLine(json);

			// And another way...
			/*
			json = Newtonsoft.Json.JsonConvert.SerializeObject(rotor);
			Console.WriteLine(json);
			*/
		}
}
}

