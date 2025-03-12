package jadevep.client.kinematics;

import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import org.ejml.simple.SimpleMatrix;

import jadevep.utils.Utils;

public class Kinematics {
	
	private static final double TOL = 1e-8;
	
	private static double[] armLengths = {0.36, 0.42, 0.4, 0.15194};
	private static double[][] dhParams = {
            {0, -Math.PI/2, 0.36, 0},
            {0, Math.PI/2, 0, 0},
            {0, Math.PI/2, 0.42, 0},
            {0, -Math.PI/2, 0, 0},
            {0, -Math.PI/2, 0.4, 0},
            {0, Math.PI/2, 0, 0},
            {0, 0, 0.15194, 0}
    };
	
	public static FKResult ForwardKinematics(double[] joints) {
		int nj = dhParams.length;
		Configuration robotConf = new Configuration(joints[1] >= 0, joints[3] >= 0, joints[5] >= 0);
		SimpleMatrix dhMatrix = new SimpleMatrix(nj, 4);
		SimpleMatrix[] transforms = new SimpleMatrix[nj];
		for(int i = 0;i < nj;i++) {
            dhMatrix.set(i, 0, dhParams[i][0]);
            dhMatrix.set(i, 1, dhParams[i][1]);
            dhMatrix.set(i, 2, dhParams[i][2]);
            dhMatrix.set(i, 3, dhParams[i][3] + joints[i]);
            transforms[i] = new SimpleMatrix(4, 4);
		}
		for(int i = 0;i < nj;i++) {
			double a = dhMatrix.get(i, 0);
			double alpha = dhMatrix.get(i, 1);
			double d = dhMatrix.get(i, 2);
			double theta = dhMatrix.get(i, 3);
			double[] v = {a * Math.cos(theta), a * Math.sin(theta), d};
			double[][] frameTransformData = {
					{Math.cos(theta),-Math.sin(theta) * Math.cos(alpha), Math.sin(theta) * Math.sin(alpha), v[0]},
					{Math.sin(theta), Math.cos(theta) * Math.cos(alpha),-Math.cos(theta) * Math.sin(alpha), v[1]},
					{0              , Math.sin(alpha)                  , Math.cos(alpha)                  , v[2]},
					{0              , 0                                , 0                                , 1   },
			};
			SimpleMatrix frameTransform = new SimpleMatrix(frameTransformData);
			if(i == 0) {
				transforms[i] = frameTransform;
			}else {
				transforms[i] = transforms[i - 1].mult(frameTransform);
			}
		}
		SimpleMatrix xs = new SimpleMatrix(3, 1, true, new double[]{transforms[0].get(0, 3), transforms[0].get(1, 3), transforms[0].get(2, 3)});
		SimpleMatrix xe = new SimpleMatrix(3, 1, true, new double[]{transforms[3].get(0, 3), transforms[3].get(1, 3), transforms[3].get(2, 3)});
		SimpleMatrix xw = new SimpleMatrix(3, 1, true, new double[]{transforms[5].get(0, 3), transforms[5].get(1, 3), transforms[5].get(2, 3)});
		SimpleMatrix xsw = xw.minus(xs);
		SimpleMatrix pose = transforms[nj-1];
		
		RefPlaneResult refPlaneResult = referencePlane(pose, robotConf.getElbow());
		SimpleMatrix planeVector = refPlaneResult.planeVector;
//		double[] jout = refPlaneResult.joints;
		
		SimpleMatrix v1 = Utils.unit(xe.minus(xs));
		SimpleMatrix v2 = Utils.unit(xw.minus(xs));
		SimpleMatrix vc = Utils.crossProduct(v1, v2);

		SimpleMatrix UnitPlaneVector = Utils.unit(planeVector);
		SimpleMatrix UnitVC = Utils.unit(vc);
		double cosNS = UnitPlaneVector.dot(UnitVC);
		if(Math.abs(cosNS) > 1) {
			cosNS = Math.signum(cosNS);
		}
		SimpleMatrix v3 = Utils.crossProduct(UnitPlaneVector, UnitVC);
		double nsParam = 0;
		if(v3.normF() > TOL) {
			nsParam = Math.signum(v3.dot(xsw)) * Math.acos(cosNS);
		}else if (planeVector.minus(vc).normF() >= TOL) {
			nsParam = Math.PI;
		}
		return new FKResult(pose, nsParam, robotConf);
	}
	
	public static IKResult InverseKinematics(SimpleMatrix pose, double nsParam, Configuration robotConf) {
		int nj = dhParams.length;
		double[] joints = new double[nj];

		SimpleMatrix xend = pose.extractMatrix(0, 3, 3, 4);
		SimpleMatrix xs = new SimpleMatrix(3, 1, true, new double[]{0,0,dhParams[0][2]});
		SimpleMatrix xwt = new SimpleMatrix(3, 1, true, new double[]{0,0,dhParams[nj-1][2]});
		SimpleMatrix poseRot = pose.extractMatrix(0, 3, 0, 3);
		SimpleMatrix xw = xend.minus(poseRot.mult(xwt));
		SimpleMatrix xsw = xw.minus(xs);
		SimpleMatrix usw = Utils.unit(xsw);
		
//		double lbs = armLengths[0];
		double lse = armLengths[1];
		double lew = armLengths[2];
		
		if (!(xsw.normF() < lse + lew && xsw.normF() > lse - lew)) {
			throw new IllegalArgumentException("Specified pose outside reachable workspace.");
		}
		double xswNorm = xsw.normF();
		if (!(Math.abs((xswNorm*xswNorm - lse*lse - lew*lew) - (2*lse*lew)) > TOL)) {
			throw new IllegalArgumentException("Elbow singularity. Tip at reach limit.");
		}

		joints[3] = robotConf.getElbow() * Math.acos((xswNorm*xswNorm - lse*lse - lew*lew)/(2*lse*lew));
		
		SimpleMatrix T34 = KinematicsHelper.dhCalc(dhParams[3][0], dhParams[3][1], dhParams[3][2], joints[3]);
		SimpleMatrix R34 = T34.extractMatrix(0, 3, 0, 3);
		
		RefPlaneResult refPlaneResult = referencePlane(pose, robotConf.getElbow());
		SimpleMatrix R03 = refPlaneResult.elbowRot;
		
		SimpleMatrix skewUSW = KinematicsHelper.skew(usw);
		SimpleMatrix As = skewUSW.mult(R03);
		SimpleMatrix Bs = skewUSW.mult(skewUSW).negative().mult(R03);
		SimpleMatrix outerProduct = new SimpleMatrix(usw.numRows(), usw.numRows());
		VectorVectorMult_DDRM.outerProd(usw.getMatrix(), usw.getMatrix(), outerProduct.getMatrix());
		SimpleMatrix Cs = outerProduct.mult(R03);

		double psi = nsParam;
		R03 = As.scale(Math.sin(psi)).plus(Bs.scale(Math.cos(psi))).plus(Cs);
		
		int arm = robotConf.getArm();
		joints[0] = Math.atan2(arm*R03.get(1, 1), arm*R03.get(0, 1));
		joints[1] = arm*Math.acos(R03.get(2, 1));
		joints[2] = Math.atan2(arm*-R03.get(2, 2), arm*-R03.get(2, 0));
		
		SimpleMatrix Aw = R34.transpose().mult(As.transpose()).mult(poseRot);
		SimpleMatrix Bw = R34.transpose().mult(Bs.transpose()).mult(poseRot);
		SimpleMatrix Cw = R34.transpose().mult(Cs.transpose()).mult(poseRot);
		
		SimpleMatrix R47 = Aw.scale(Math.sin(psi)).plus(Bw.scale(Math.cos(psi))).plus(Cw);

		int wrist = robotConf.getWrist();
		joints[4] = Math.atan2(wrist*R47.get(1, 2), wrist*R47.get(0, 2));
		joints[5] = wrist*Math.acos(R47.get(2, 2));
		joints[6] = Math.atan2(wrist*R47.get(2, 1), wrist*-R47.get(2, 0));

		SimpleMatrix[] sMat = {As, Bs, Cs};
		SimpleMatrix[] wMat = {Aw, Bw, Cw};
		
		return new IKResult(joints, sMat, wMat);
	}
	
	private static RefPlaneResult referencePlane(SimpleMatrix pose, int elbow) {
		int nj = dhParams.length;
		double[] joints = new double[nj];
		
		SimpleMatrix xend = new SimpleMatrix(3, 1, true, new double[]{pose.get(0, 3), pose.get(1, 3), pose.get(2, 3)});
		SimpleMatrix xs0 = new SimpleMatrix(3, 1, true, new double[]{0,0,dhParams[0][2]});
		SimpleMatrix xwt = new SimpleMatrix(3, 1, true, new double[]{0,0,dhParams[nj-1][2]});
		SimpleMatrix poseRot = pose.extractMatrix(0, 3, 0, 3);
		SimpleMatrix xw0 = xend.minus(poseRot.mult(xwt));
		SimpleMatrix xsw = xw0.minus(xs0);

//		double lbs = armLengths[0];
		double lse = armLengths[1];
		double lew = armLengths[2];
		if (!(xsw.normF() < lse + lew && xsw.normF() > lse - lew)) {
			throw new IllegalArgumentException("Specified pose outside reachable workspace.");
		}
		double xswNorm = xsw.normF();
		if (!(Math.abs((xswNorm*xswNorm - lse*lse - lew*lew) - (2*lse*lew)) > TOL)) {
			throw new IllegalArgumentException("Elbow singularity. Tip at reach limit.");
		}
		joints[3] = elbow * Math.acos((xswNorm*xswNorm - lse*lse - lew*lew)/(2*lse*lew));
		
		SimpleMatrix T34 = KinematicsHelper.dhCalc(dhParams[3][0], dhParams[3][1], dhParams[3][2], joints[3]);
//		SimpleMatrix R34 = T34.extractMatrix(0, 3, 0, 3);
//		SimpleMatrix xse = new SimpleMatrix(3, 1, true, new double[]{0, lse, 0});
//      SimpleMatrix xew = new SimpleMatrix(3, 1, true, new double[]{0, 0, lew});
//      SimpleMatrix m = xse.plus(R34.mult(xew));
        
        if(Utils.crossProduct(xsw, new SimpleMatrix(3, 1, true, new double[] {0,0,1})).normF() > TOL) {
        	joints[0] = Math.atan2(xsw.get(1), xsw.get(0));
        }else {
        	joints[0] = 0;
        }
		
        double radius = Math.hypot(xsw.get(0), xsw.get(1));
        double dsw = xsw.normF();
        double phi = Math.acos((lse*lse+dsw*dsw-lew*lew)/(2*lse*dsw));
        joints[1] = Math.atan2(radius, xsw.get(2))+elbow*phi;
		
        SimpleMatrix T01 = KinematicsHelper.dhCalc(dhParams[0][0],dhParams[0][1],dhParams[0][2],joints[0]);
        SimpleMatrix T12 = KinematicsHelper.dhCalc(dhParams[1][0],dhParams[1][1],dhParams[1][2],joints[1]);
        SimpleMatrix T23 = KinematicsHelper.dhCalc(dhParams[2][0],dhParams[2][1],dhParams[2][2],0);
        SimpleMatrix T04 = T01.mult(T12).mult(T23).mult(T34);
        SimpleMatrix rotBaseElbow = T01.extractMatrix(0, 3, 0, 3)
        		.mult(T12.extractMatrix(0, 3, 0, 3))
        		.mult(T23.extractMatrix(0, 3, 0, 3));
        SimpleMatrix x0e = T04.extractMatrix(0, 3, 3, 4);
        SimpleMatrix v1 = Utils.unit(x0e.minus(xs0));
        SimpleMatrix v2 = Utils.unit(xw0.minus(xs0));
        
        SimpleMatrix refPlaneVector = Utils.crossProduct(v1, v2);
        
        return new RefPlaneResult(refPlaneVector, rotBaseElbow, joints);
	}
	
}
