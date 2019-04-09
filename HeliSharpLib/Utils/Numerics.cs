using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using MathNet.Numerics.LinearAlgebra;

namespace HeliSharp
{
    public class Numerics
    {

        public static double InterpolateLinear(Matrix<double> M, double xi)
        {
            var x = M.Column(0);
            var y = M.Column(1);
            var len = x.Count;
            if (x[len-1] > x[0]) {
                int idx = -1;
                for (int i = 0; i < len; i++) {
                    if (x[i] > xi) {
                        idx = i;
                        break;
                    }
                }
                if (idx < 0)
                    return y[len-1];
                else if (idx == 0)
                    return y[0];
                else
                    return y[idx-1]+(y[idx]-y[idx-1])/(x[idx]-x[idx-1])*(xi-x[idx-1]);
            } else if (x[len-1] < x[0]) {
                int idx = -1;
                for (int i = 0; i < len; i++) {
                    if (x[i] < xi) {
                        idx = i;
                        break ;
                    }
                }
                if (idx < 0)
                    return y[len-1];
                else if (idx == 0)
                    return y[0];
                else
                    return y[idx]+(y[idx-1]-y[idx])/(x[idx-1]-x[idx])*(xi-x[idx]);
            } else
                return y[0];
        }

        public static double InterpolateBilinear(double[] x, double[] y, double[,] z, int rows, int columns, double xi, double yi)
        {
            int xidx = -1;
            int yidx = -1;
            // Find lower x index
            for (int c = 0; c < columns-1; c++) {
                if ((xi >= x[c] && xi <= x[c+1]) || (xi <= x[c] && xi >= x[c+1]))
                    xidx=c;
            }
            // If xi is out of bounds, clamp to end/start of table
            if ((xidx < 0) && (x[1] > x[0] && xi > x[columns-1]) || (x[1] < x[0] && xi < x[columns-1]))
                xidx=columns-1;
            // Find lower y index
            for (int r = 0; r < rows-1; r++) {
                if ((yi >= y[r] && yi <= y[r+1]) || (yi <= y[r] && yi >= y[r+1]))
                    yidx=r;
            }
            // If yi is out of bounds, clamp to end/start of table
            if ((yidx < 0) && (y[1] > y[0] && yi > y[rows-1]) || (y[1] < y[0] && yi < y[rows-1]))
                yidx=rows-1;
            int xidx2, yidx2;
            // Find upper x index
            if (xidx < 0)
                xidx=xidx2=0;
            else if (xidx >= columns-1)
                xidx2=columns-1;
            else
                xidx2=xidx+1;
            // Find upper y index
            if (yidx < 0)
                yidx=yidx2=0;
            else if (yidx >= rows-1)
                yidx2=rows-1;
            else
                yidx2=yidx+1;
            // Extract the four values to interpolate between
            double z1,z2,z3,z4,z5,z6;
            z1=z[yidx,xidx];
            z2=z[yidx,xidx2];
            z3=z[yidx2,xidx];
            z4=z[yidx2,xidx2];
            // Interpolate
            if (xidx==xidx2 && yidx==yidx2)
                return z1; // both x and y clamped to start/end - no interpolation
            else if (xidx==xidx2) // x clamped to start/end - interpolate in y direction
                return z1+(z3-z1)*(yi-y[yidx])/(y[yidx2]-y[yidx]);
            else if (yidx==yidx2) // y clamped to start/end - interpolate in x direction
                return z1+(z2-z1)*(xi-x[xidx])/(x[xidx2]-x[xidx]);
            else {
                // first in the x direction
                z5=z1+(z2-z1)*(xi-x[xidx])/(x[xidx2]-x[xidx]);
                z6=z3+(z4-z3)*(xi-x[xidx])/(x[xidx2]-x[xidx]);
                // then in the y direction
                return z5+(z6-z5)*(yi-y[yidx])/(y[yidx2]-y[yidx]);
            }
        }

    }
}

