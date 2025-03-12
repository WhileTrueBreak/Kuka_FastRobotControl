package jadevep.utils;

import org.ejml.simple.SimpleMatrix;

public class Utils {
	public static <T extends Comparable<T>> T clamp(T val, T min, T max) {
	    if (val.compareTo(min) < 0) return min;
	    else if (val.compareTo(max) > 0) return max;
	    else return val;
	}
	
	public static SimpleMatrix unit(SimpleMatrix v) {
		double norm = v.normF();
		if (norm < 1e-10) {
            throw new IllegalArgumentException("Zero norm vector");
        }
        return v.divide(norm);
	}
	
	public static SimpleMatrix crossProduct(SimpleMatrix a, SimpleMatrix b) {
        // Ensure both vectors are 3-dimensional
        if (a.numRows() != 3 || a.numCols() != 1 || b.numRows() != 3 || b.numCols() != 1) {
            throw new IllegalArgumentException("Both input matrices must be 3x1 vectors.");
        }

        // Extract components of vector a
        double a1 = a.get(0, 0);
        double a2 = a.get(1, 0);
        double a3 = a.get(2, 0);

        // Extract components of vector b
        double b1 = b.get(0, 0);
        double b2 = b.get(1, 0);
        double b3 = b.get(2, 0);

        // Compute cross product components
        double c1 = a2 * b3 - a3 * b2;
        double c2 = a3 * b1 - a1 * b3;
        double c3 = a1 * b2 - a2 * b1;

        // Return the result as a 3x1 SimpleMatrix
        return new SimpleMatrix(3, 1, true, new double[] {c1, c2, c3});
    }
}
