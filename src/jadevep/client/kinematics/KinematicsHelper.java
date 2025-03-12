package jadevep.client.kinematics;

import org.ejml.simple.SimpleMatrix;

public class KinematicsHelper {
	
	public static SimpleMatrix skew(SimpleMatrix v) {
        if (v.numRows() != 3 || v.numCols() != 1) {
            throw new IllegalArgumentException("Input matrix must be a 3x1 vector.");
        }
        double[] data = new double[3];
        for (int i = 0; i < 3; i++) {
            data[i] = v.get(i, 0);
        }
        double[][] skewData = {
            { 0, -data[2],  data[1]},
            { data[2], 0, -data[0]},
            {-data[1], data[0], 0}
        };
        return new SimpleMatrix(skewData);
    }
	
	public static SimpleMatrix dhCalc(double a, double alpha, double d, double theta) {
		double[][] Tdata = {
            {Math.cos(theta),-Math.sin(theta) * Math.cos(alpha), Math.sin(theta) * Math.sin(alpha), a * Math.cos(theta)},
            {Math.sin(theta), Math.cos(theta) * Math.cos(alpha),-Math.cos(theta) * Math.sin(alpha), a * Math.sin(theta)},
            {0.0			, Math.sin(alpha)				   , Math.cos(alpha)				  , d				   },
            {0.0			, 0.0							   , 0.0							  , 1.0				   }
        };

        return new SimpleMatrix(Tdata);
	}
	
	public static SimpleMatrix rotationMatrixX(double a) {
        SimpleMatrix R_x = new SimpleMatrix(4, 4);
        R_x.set(0, 0, 1);
        R_x.set(1, 1, Math.cos(a));
        R_x.set(1, 2, -Math.sin(a));
        R_x.set(2, 1, Math.sin(a));
        R_x.set(2, 2, Math.cos(a));
        R_x.set(3, 3, 1);
        return R_x;
    }

    public static SimpleMatrix rotationMatrixY(double b) {
        SimpleMatrix R_y = new SimpleMatrix(4, 4);
        R_y.set(0, 0, Math.cos(b));
        R_y.set(0, 2, Math.sin(b));
        R_y.set(1, 1, 1);
        R_y.set(2, 0, -Math.sin(b));
        R_y.set(2, 2, Math.cos(b));
        R_y.set(3, 3, 1);
        return R_y;
    }

    public static SimpleMatrix rotationMatrixZ(double c) {
        SimpleMatrix R_z = new SimpleMatrix(4, 4);
        R_z.set(0, 0, Math.cos(c));
        R_z.set(0, 1, -Math.sin(c));
        R_z.set(1, 0, Math.sin(c));
        R_z.set(1, 1, Math.cos(c));
        R_z.set(2, 2, 1);
        R_z.set(3, 3, 1);
        return R_z;
    }
	
}
