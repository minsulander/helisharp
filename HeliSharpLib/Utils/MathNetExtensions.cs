using System;
using MathNet.Numerics.LinearAlgebra;
using System.Web.Script.Serialization;
using System.Collections.Generic;

namespace HeliSharp
{
	public static class MathNetExtensions
	{

		public static Vector<double> Zero3(this VectorBuilder<double> b)
		{
			return Vector<double>.Build.Dense(3);
		}

		public static Vector<double> Dense3(this VectorBuilder<double> b, double x, double y, double z)
		{
			return b.DenseOfArray (new double[] { x, y, z });
		}

		public static Vector<double> Quaternion(this VectorBuilder<double> b)
		{
			return Quaternion(b, 1, 0, 0, 0);
		}

		public static Vector<double> Quaternion(this VectorBuilder<double> b, double w, double x, double y, double z)
		{
			return b.DenseOfArray (new double[] { w, x, y, z });
		}

		public static Vector<double> Cross(this Vector<double> left, Vector<double> right)
		{
			if ((left.Count != 3 || right.Count != 3))
			{
				string message = "Vectors must have a length of 3.";
				throw new Exception(message);
			}
			Vector<double> result = Vector<double>.Build.Dense(3);
			result[0] = left[1] * right[2] - left[2] * right[1];
			result[1] = -left[0] * right[2] + left[2] * right[0];
			result[2] = left[0] * right[1] - left[1] * right[0];

			return result;
		}

		public static double w(this Vector<double> v) {
			switch (v.Count) {
				case 4: return v[0];
				default: throw new NotImplementedException();
			}
		}

		public static double x(this Vector<double> v) {
			switch (v.Count) {
				case 3: return v[0];
				case 4: return v[1];
				default: throw new NotImplementedException();
			}
		}

		public static double y(this Vector<double> v) {
			switch (v.Count) {
				case 3: return v[1];
				case 4: return v[2];
				default: throw new NotImplementedException();
			}
		}

		public static double z(this Vector<double> v) {
			switch (v.Count) {
				case 3: return v[2];
				case 4: return v[3];
				default: throw new NotImplementedException();
			}
		}

		public static string ToStr(this Vector<double> v, string format="{0:0.00}")
		{
			string[] s = new string[v.Count];
			for (var i = 0; i < v.Count; i++)
				s [i] = String.Format (format, v [i]);
			return "[" + String.Join (",", s) + "]";
		}

		public static string ToStr(this double d, string format="{0:0.00}")
		{
			return String.Format(format, d);
		}
	
		public static Matrix<double> RotationX(this MatrixBuilder<double> b, double angle)
		{
			return b.DenseOfArray(new double[,] {
				{ 1, 0, 0},
				{ 0, Math.Cos(angle), -Math.Sin(angle)},
				{ 0, Math.Sin(angle), Math.Cos(angle)}
			});
		}

		public static Matrix<double> RotationY(this MatrixBuilder<double> b, double angle)
		{
			return b.DenseOfArray(new double[,] {
				{ Math.Cos(angle), 0, Math.Sin(angle)},
				{ 0, 1, 0},
				{ -Math.Sin(angle), 0, Math.Cos(angle)}
			});
		}

		public static Matrix<double> RotationZ(this MatrixBuilder<double> b, double angle)
		{
			return b.DenseOfArray(new double[,] {
				{ Math.Cos(angle), -Math.Sin(angle), 0},
				{ Math.Sin(angle), Math.Cos(angle), 0},
				{ 0, 0, 1}
			});
		}

		public static Matrix<double> EulerRotation(this MatrixBuilder<double> b, double phi, double theta, double psi)
		{
			return b.DenseOfArray(new double[,] {
				{ Math.Cos(theta)*Math.Cos(psi), Math.Sin(phi)*Math.Sin(theta)*Math.Cos(psi)-Math.Cos(phi)*Math.Sin(psi), Math.Cos(phi)*Math.Sin(theta)*Math.Cos(psi)+Math.Sin(phi)*Math.Sin(psi) },
				{ Math.Cos(theta)*Math.Sin(psi), Math.Sin(phi)*Math.Sin(theta)*Math.Sin(psi)+Math.Cos(phi)*Math.Cos(psi), Math.Cos(phi)*Math.Sin(theta)*Math.Sin(psi)-Math.Sin(phi)*Math.Cos(psi) },
				{ -Math.Sin(theta), Math.Sin(phi)*Math.Cos(theta), Math.Cos(phi)*Math.Cos(theta) }
			});
		}

		public static Matrix<double> EulerRotation(this MatrixBuilder<double> b, Vector<double> v)
		{
			return EulerRotation(b, v[0], v[1], v[2]);
		}

		public static Vector<double> EulerAngles(this Matrix<double> M) {
			return Vector<double>.Build.DenseOfArray(new double[] {
				Math.Atan2(M[2,1], M[2,2]),
				Math.Atan2(-M[2,0], Math.Sqrt(Math.Pow(M[2,1],2) + Math.Pow(M[2,2], 2))),
				Math.Atan2(M[1,0], M[0,0])
			});
		}

		public static Matrix<double> AngularVelocityTensor(this MatrixBuilder<double> b, Vector<double> w)
		{
			return b.DenseOfArray(new double[,] {
				{ 0, -w.z(), w.y() },
				{ w.z(), 0, -w.x() },
				{ -w.y(), w.x(), 0 }
			});
		}

		public static Vector<double> ToQuaternion(this Matrix<double> a)
		{
			// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

			Vector<double> q = Vector<double>.Build.Dense(4);
			double trace = a[0,0] + a[1,1] + a[2,2]; // I removed + 1.0f; see discussion with Ethan
			if( trace > 0 ) {// I changed M_EPSILON to 0
				double s = 0.5f / Math.Sqrt(trace+ 1.0f);
				q[0] = 0.25f / s;
				q[1] = ( a[2,1] - a[1,2] ) * s;
				q[2] = ( a[0,2] - a[2,0] ) * s;
				q[3] = ( a[1,0] - a[0,1] ) * s;
			} else {
				if ( a[0,0] > a[1,1] && a[0,0] > a[2,2] ) {
					double s = 2.0f * Math.Sqrt( 1.0f + a[0,0] - a[1,1] - a[2,2]);
					q[0] = (a[2,1] - a[1,2] ) / s;
					q[1] = 0.25f * s;
					q[2] = (a[0,1] + a[1,0] ) / s;
					q[3] = (a[0,2] + a[2,0] ) / s;
				} else if (a[1,1] > a[2,2]) {
					double s = 2.0f * Math.Sqrt( 1.0f + a[1,1] - a[0,0] - a[2,2]);
					q[0] = (a[0,2] - a[2,0] ) / s;
					q[1] = (a[0,1] + a[1,0] ) / s;
					q[2] = 0.25f * s;
					q[3] = (a[1,2] + a[2,1] ) / s;
				} else {
					double s = 2.0f * Math.Sqrt( 1.0f + a[2,2] - a[0,0] - a[1,1] );
					q[0] = (a[1,0] - a[0,1] ) / s;
					q[1] = (a[0,2] + a[2,0] ) / s;
					q[2] = (a[1,2] + a[2,1] ) / s;
					q[3] = 0.25f * s;
				}
			}
			return q;
		}

		public static Matrix<double> ToRotationMatrix(this Vector<double> q) {

			// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/

			Matrix<double> M = Matrix<double>.Build.DenseIdentity(3);
			double sqw = q[0]*q[0];
			double sqx = q[1]*q[1];
			double sqy = q[2]*q[2];
			double sqz = q[3]*q[3];

			// invs (inverse square length) is only required if quaternion is not already normalised
			double invs = 1 / (sqx + sqy + sqz + sqw);
			M[0,0]= ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
			M[1,1] = (-sqx + sqy - sqz + sqw)*invs ;
			M[2,2] = (-sqx - sqy + sqz + sqw)*invs ;

			double tmp1 = q[1]*q[2];
			double tmp2 = q[3]*q[0];
			M[1,0] = 2.0 * (tmp1 + tmp2)*invs ;
			M[0,1] = 2.0 * (tmp1 - tmp2)*invs ;

			tmp1 = q[1]*q[3];
			tmp2 = q[2]*q[0];
			M[2,0] = 2.0 * (tmp1 - tmp2)*invs ;
			M[0,2] = 2.0 * (tmp1 + tmp2)*invs ;
			tmp1 = q[2]*q[3];
			tmp2 = q[1]*q[0];
			M[2,1] = 2.0 * (tmp1 + tmp2)*invs ;
			M[1,2] = 2.0 * (tmp1 - tmp2)*invs ;      
			return M;
		}

		public static string ToStr(this Matrix<double> m) {
			var str = "[ ";
			for (var r = 0; r < m.RowCount; r++) {
				if (r != 0)
					str += ", ";
				str += m.Row(r).ToStr();
			}
			return str + " ]";
		}

	}
}
