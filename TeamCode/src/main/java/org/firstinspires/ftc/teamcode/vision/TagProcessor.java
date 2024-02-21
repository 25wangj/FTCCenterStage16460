package org.firstinspires.ftc.teamcode.vision;
import static java.lang.Math.*;
import android.graphics.Canvas;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;
import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.control.KalmanEstimate;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
public class TagProcessor implements VisionProcessor {
    public static final double EPS = 1e-6;
    private long tagPtr;
    private Mat gray = new Mat();
    private SimpleMatrix invCamMat;
    private double[] camPose;
    private double z;
    private double sBeta;
    private double cBeta;
    private SimpleMatrix rBeta;
    private SimpleMatrix rGamma;
    private ConcurrentLinkedQueue<KalmanEstimate> queue = new ConcurrentLinkedQueue<>();
    public TagProcessor(SimpleMatrix camMat, double[] pos, double[] rot) {
        tagPtr = AprilTagDetectorJNI.createApriltagDetector("tag36h11", 2, 3);
        invCamMat = camMat.invert();
        camPose = new double[] {pos[0], pos[1], rot[1]};
        z = pos[2];
        sBeta = sin(rot[1]);
        cBeta = cos(rot[1]);
        rBeta = new SimpleMatrix(new double[][] {{cBeta, 0, -sBeta}, {0, 1, 0}, {sBeta, 0, cBeta}});
        rGamma = new SimpleMatrix(new double[][] {{cos(rot[2]), sin(rot[2]), 0}, {-sin(rot[2]), cos(rot[2]), 0}, {0, 0, 1}});
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}
    @Override
    public Object processFrame(Mat frame, long time) {
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);
        long detectPtr = AprilTagDetectorJNI.runApriltagDetector(tagPtr, gray.dataAddr(), frame.width(), frame.height());
        if (detectPtr != 0) {
            long[] detectPtrs = ApriltagDetectionJNI.getDetectionPointers(detectPtr);
            List<double[]> camArr = new ArrayList<>();
            List<double[]> worldArr = new ArrayList<>();
            for (long ptr : detectPtrs) {
                int id = ApriltagDetectionJNI.getId(ptr);
                if (Vision.tagPts.containsKey(id)) {
                    Collections.addAll(camArr, ApriltagDetectionJNI.getCorners(ptr));
                    Collections.addAll(worldArr, Vision.tagPts.get(id));
                }
            }
            if (!camArr.isEmpty()) {
                SimpleMatrix worldM = new SimpleMatrix(worldArr.toArray(new double[0][]));
                SimpleMatrix camM = new SimpleMatrix(camArr.toArray(new double[0][]))
                        .concatColumns(SimpleMatrix.ones(camArr.size(), 1)).mult(invCamMat).mult(rGamma);
                SimpleMatrix rotM = camM.mult(rBeta);
                /*SimpleMatrix sts = new SimpleMatrix(worldM.getNumRows(), 1);
                SimpleMatrix cts = new SimpleMatrix(worldM.getNumRows(), 1);
                double c1 = 0, c2 = 0, c3 = 0, c4 = 0, a1 = 0, a2 = 0, b1 = 0, b2 = 0;
                for (int i = 1; i <= worldM.getNumRows(); i++) {
                    double mag1 = rotM.getRow(i).normF();
                    double mag2 = sqrt(rotM.get(i, 1) * rotM.get(i, 1) + rotM.get(i, 2) * rotM.get(i, 2));
                    double sp = rotM.get(i, 3) / mag1;
                    double cp = mag2 / mag1;
                    sts.set(i, rotM.get(i, 1) / mag2);
                    cts.set(i, rotM.get(i, 2) / mag2);
                    double zi = worldM.get(i, 3) - z;
                    double i1 = sp * sp * sp / (cp * zi * zi);
                    double i2 = -2 * worldM.get(i, 1) * i1;
                    double i3 = -2 * worldM.get(i, 2) * i1;
                    double i4 = (worldM.get(i, 1) * worldM.get(i, 1) + worldM.get(i, 2) * worldM.get(i, 2)) * i1 - sp * cp;
                    c1 += 4 * i1 * i1;
                    c2 += 2 * i2 * i3;
                    c3 += 2 * i1 * i2;
                    c4 += 2 * i1 + i3;
                    a1 += 4 * i1 * i4 + 2 * i2 * i2;
                    a2 += 2 * i2 * i4;
                    b1 += 4 * i1 * i4 + 2 * i3 * i3;
                    b2 += 2 * i3 * i4;
                }
                double[] xcoeffs = {a1*a1*c1*c1*c1-2*a1*b1*c1*c1*c1-4*a1*c1*c1*c3*c3+4*a1*c1*c1*c4*c4+b1*b1*c1*c1*c1+4*b1*c1*c1*c3*c3-4*b1*c1*c1*c4*c4+4*c1*c1*c1*c2*c2-16*c1*c1*c2*c3*c4+4*c1*c3*c3*c3*c3+8*c1*c3*c3*c4*c4+4*c1*c4*c4*c4*c4,
                        5*a1*a1*c1*c1*c4-8*a1*b1*c1*c1*c4-2*b2*a1*c1*c1*c1-2*a1*c1*c1*c2*c3-16*a1*c1*c3*c3*c4+16*a1*c1*c4*c4*c4+3*b1*b1*c1*c1*c4+2*b2*b1*c1*c1*c1-2*b1*c1*c1*c2*c3+20*b1*c1*c3*c3*c4-12*b1*c1*c4*c4*c4+4*a2*c1*c1*c1*c2+16*c1*c1*c2*c2*c4+4*b2*c1*c1*c3*c3-8*a2*c1*c1*c3*c4-4*b2*c1*c1*c4*c4+4*c1*c2*c3*c3*c3-60*c1*c2*c3*c4*c4+12*c3*c3*c3*c3*c4+24*c3*c3*c4*c4*c4+12*c4*c4*c4*c4*c4,
                        a1*a1*b1*c1*c1+7*a1*a1*c1*c4*c4-2*a1*a2*c1*c1*c3-2*a1*b1*b1*c1*c1-2*a1*b1*c1*c3*c3-2*a1*b1*c1*c4*c4-8*a1*b2*c1*c1*c4-6*a1*c1*c2*c3*c4-20*a1*c3*c3*c4*c4+12*a1*c4*c4*c4*c4+a2*a2*c1*c1*c1+14*a2*c1*c1*c2*c4+4*a2*c1*c3*c3*c3-28*a2*c1*c3*c4*c4+b1*b1*b1*c1*c1+3*b1*b1*c1*c3*c3-4*b1*b1*c1*c4*c4+6*b1*b2*c1*c1*c4+4*b1*c1*c1*c2*c2-22*b1*c1*c2*c3*c4+36*b1*c3*c3*c4*c4+4*b1*c4*c4*c4*c4+b2*b2*c1*c1*c1-2*b2*c1*c1*c2*c3+20*b2*c1*c3*c3*c4-12*b2*c1*c4*c4*c4-3*c1*c2*c2*c3*c3+13*c1*c2*c2*c4*c4+24*c2*c3*c3*c3*c4-40*c2*c3*c4*c4*c4,
                        2*a1*a1*b1*c1*c4+a1*a1*b2*c1*c1+3*a1*a1*c4*c4*c4-a1*a2*c1*c1*c2-4*a1*a2*c1*c3*c4-2*a1*b1*b1*c1*c4-4*a1*b1*b2*c1*c1+a1*b1*c1*c2*c3-6*a1*b1*c3*c3*c4+4*a1*b1*c4*c4*c4-2*a1*b2*c1*c3*c3-2*a1*b2*c1*c4*c4+a1*c1*c2*c2*c4-12*a1*c2*c3*c4*c4+3*a2*a2*c1*c1*c4+3*a2*b1*c1*c1*c2-6*a2*b1*c1*c3*c4+10*a2*c1*c2*c4*c4+12*a2*c3*c3*c3*c4-20*a2*c3*c4*c4*c4+3*b1*b1*b2*c1*c1-3*b1*b1*c1*c2*c3+9*b1*b1*c3*c3*c4+6*b1*b2*c1*c3*c3-8*b1*b2*c1*c4*c4+5*b1*c1*c2*c2*c4-12*b1*c2*c3*c4*c4+3*b2*b2*c1*c1*c4+4*b2*c1*c1*c2*c2-22*b2*c1*c2*c3*c4+36*b2*c3*c3*c4*c4+4*b2*c4*c4*c4*c4-4*c1*c2*c2*c2*c3+15*c2*c2*c3*c3*c4-c2*c2*c4*c4*c4,
                        2*a1*a1*b2*c1*c4+b1*a1*a1*c4*c4-2*a1*a2*c1*c2*c4-2*a1*a2*c3*c4*c4-2*a1*b2*b2*c1*c1+a1*b2*c1*c2*c3-4*b1*a1*b2*c1*c4-6*a1*b2*c3*c3*c4+4*a1*b2*c4*c4*c4+b1*a1*c1*c2*c2-a1*c2*c2*c4*c4-3*b1*a1*c2*c3*c4+3*a2*a2*c1*c4*c4+3*a2*b2*c1*c1*c2-6*a2*b2*c1*c3*c4-3*a2*c1*c2*c2*c3+3*b1*a2*c1*c2*c4+12*a2*c2*c3*c3*c4-6*b1*a2*c3*c4*c4+3*b1*b2*b2*c1*c1+3*b2*b2*c1*c3*c3-4*b2*b2*c1*c4*c4+5*b2*c1*c2*c2*c4-6*b1*b2*c1*c2*c3-12*b2*c2*c3*c4*c4+18*b1*b2*c3*c3*c4-c1*c2*c2*c2*c2+3*c2*c2*c2*c3*c4,
                        a1*a1*b2*c4*c4-a1*a2*c2*c4*c4-2*a1*b2*b2*c1*c4+a1*b2*c1*c2*c2-3*a1*b2*c2*c3*c4+a2*a2*c4*c4*c4+3*a2*b2*c1*c2*c4-6*a2*b2*c3*c4*c4-a2*c1*c2*c2*c2+3*a2*c2*c2*c3*c4+b2*b2*b2*c1*c1-3*b2*b2*c1*c2*c3+9*b2*b2*c3*c3*c4};
                double[] ycoeffs = {xcoeffs[0],
                        3*a1*a1*c1*c1*c3-8*a1*b1*c1*c1*c3+2*a2*a1*c1*c1*c1-2*a1*c1*c1*c2*c4-12*a1*c1*c3*c3*c3+20*a1*c1*c3*c4*c4+5*b1*b1*c1*c1*c3-2*a2*b1*c1*c1*c1-2*b1*c1*c1*c2*c4+16*b1*c1*c3*c3*c3-16*b1*c1*c3*c4*c4+4*b2*c1*c1*c1*c2+16*c1*c1*c2*c2*c3-4*a2*c1*c1*c3*c3-8*b2*c1*c1*c3*c4+4*a2*c1*c1*c4*c4-60*c1*c2*c3*c3*c4+4*c1*c2*c4*c4*c4+12*c3*c3*c3*c3*c3+24*c3*c3*c3*c4*c4+12*c3*c4*c4*c4*c4,
                        a1*a1*a1*c1*c1-2*a1*a1*b1*c1*c1-4*a1*a1*c1*c3*c3+3*a1*a1*c1*c4*c4+6*a1*a2*c1*c1*c3+a1*b1*b1*c1*c1-2*a1*b1*c1*c3*c3-2*a1*b1*c1*c4*c4+4*a1*c1*c1*c2*c2-22*a1*c1*c2*c3*c4+4*a1*c3*c3*c3*c3+36*a1*c3*c3*c4*c4+a2*a2*c1*c1*c1-8*a2*b1*c1*c1*c3-2*a2*c1*c1*c2*c4-12*a2*c1*c3*c3*c3+20*a2*c1*c3*c4*c4+7*b1*b1*c1*c3*c3-2*b1*b2*c1*c1*c4-6*b1*c1*c2*c3*c4+12*b1*c3*c3*c3*c3-20*b1*c3*c3*c4*c4+b2*b2*c1*c1*c1+14*b2*c1*c1*c2*c3-28*b2*c1*c3*c3*c4+4*b2*c1*c4*c4*c4+13*c1*c2*c2*c3*c3-3*c1*c2*c2*c4*c4-40*c2*c3*c3*c3*c4+24*c2*c3*c4*c4*c4,
                        3*a1*a1*a2*c1*c1-2*a1*a1*b1*c1*c3-3*a1*a1*c1*c2*c4+9*a1*a1*c3*c4*c4-4*a1*a2*b1*c1*c1-8*a1*a2*c1*c3*c3+6*a1*a2*c1*c4*c4+2*a1*b1*b1*c1*c3+a1*b1*c1*c2*c4+4*a1*b1*c3*c3*c3-6*a1*b1*c3*c4*c4+3*a1*b2*c1*c1*c2-6*a1*b2*c1*c3*c4+5*a1*c1*c2*c2*c3-12*a1*c2*c3*c3*c4+3*a2*a2*c1*c1*c3+a2*b1*b1*c1*c1-2*a2*b1*c1*c3*c3-2*a2*b1*c1*c4*c4+4*a2*c1*c1*c2*c2-22*a2*c1*c2*c3*c4+4*a2*c3*c3*c3*c3+36*a2*c3*c3*c4*c4+3*b1*b1*c3*c3*c3-b1*b2*c1*c1*c2-4*b1*b2*c1*c3*c4+b1*c1*c2*c2*c3-12*b1*c2*c3*c3*c4+3*b2*b2*c1*c1*c3+10*b2*c1*c2*c3*c3-20*b2*c3*c3*c3*c4+12*b2*c3*c4*c4*c4-4*c1*c2*c2*c2*c4-c2*c2*c3*c3*c3+15*c2*c2*c3*c4*c4,
                        -2*a2*a2*b1*c1*c1+3*a1*a2*a2*c1*c1-4*a2*a2*c1*c3*c3+3*a2*a2*c1*c4*c4+2*a2*b1*b1*c1*c3+a2*b1*c1*c2*c4-4*a1*a2*b1*c1*c3+4*a2*b1*c3*c3*c3-6*a2*b1*c3*c4*c4+3*a2*b2*c1*c1*c2-6*a2*b2*c1*c3*c4+5*a2*c1*c2*c2*c3-6*a1*a2*c1*c2*c4-12*a2*c2*c3*c3*c4+18*a1*a2*c3*c4*c4+a1*b1*b1*c3*c3-2*b1*b2*c1*c2*c3-2*b1*b2*c3*c3*c4+a1*b1*c1*c2*c2-b1*c2*c2*c3*c3-3*a1*b1*c2*c3*c4+3*b2*b2*c1*c3*c3-3*b2*c1*c2*c2*c4+3*a1*b2*c1*c2*c3+12*b2*c2*c3*c4*c4-6*a1*b2*c3*c3*c4-c1*c2*c2*c2*c2+3*c2*c2*c2*c3*c4,
                        a2*a2*a2*c1*c1-2*a2*a2*b1*c1*c3-3*a2*a2*c1*c2*c4+9*a2*a2*c3*c4*c4+a2*b1*b1*c3*c3+a2*b1*c1*c2*c2-3*a2*b1*c2*c3*c4+3*a2*b2*c1*c2*c3-6*a2*b2*c3*c3*c4-b1*b2*c2*c3*c3+b2*b2*c3*c3*c3-b2*c1*c2*c2*c2+3*b2*c2*c2*c3*c4};
                List<Double> rx = roots(ycoeffs);
                List<Double> ry = roots(xcoeffs);
                double[] init = {0, 0, 0};
                double error = Double.POSITIVE_INFINITY;
                for (double x : rx) {
                    for (double y : ry) {
                        double n = 0, d = 0;
                        for (int j = 1; j <= worldM.getNumRows(); j++) {
                            double xi = x - worldM.get(j, 1);
                            double yi = y - worldM.get(j, 2);
                            n += xi * sts.get(j) - yi * cts.get(j);
                            d += xi * cts.get(j) + yi * sts.get(j);
                        }
                        double theta = -atan(n / d);
                        double[] p1 = {x, y, theta};
                        double e1 = new SimpleMatrix(rpError(p1, camM, worldM)).normF();
                        double[] p2 = {x, y, theta + PI};
                        double e2 = new SimpleMatrix(rpError(p2, camM, worldM)).normF();
                        if (e1 < e2) {
                            if (e1 < error) {
                                error = e1;
                                init = p1;
                            }
                        } else if (e2 < error) {
                            error = e2;
                            init = p2;
                        }
                    }
                }*/
                LeastSquaresProblem lsq = new LeastSquaresBuilder()
                        .model(p -> rpError(p, camM, worldM), p -> rpJac(p, worldM))
                        .build();
                LeastSquaresOptimizer.Optimum opt = new LevenbergMarquardtOptimizer().withCostRelativeTolerance(EPS).optimize(lsq);
                double[] pt = opt.getPoint().toArray();
                SimpleMatrix pose = new SimpleMatrix(new double[] {(pt[0] - camPose[0]) * cos(camPose[2]) + (pt[1] - camPose[1]) * sin(camPose[2]),
                        (pt[1] - camPose[1]) * cos(camPose[2]) - (pt[0] - camPose[0]) * sin(camPose[2]), pt[2] - camPose[2]});
                queue.offer(new KalmanEstimate(pose, SimpleMatrix.identity(3), new SimpleMatrix(3, 3), time));
            }
        }
        return null;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
    public ConcurrentLinkedQueue<KalmanEstimate> queue() {
        return queue;
    }
    private List<Double> roots(double[] coeffs) {
        SimpleMatrix mat = new SimpleMatrix(coeffs.length - 1, coeffs.length - 1);
        for (int i = 1; i < coeffs.length; i++) {
            mat.set(coeffs.length - 1, i, -coeffs[coeffs.length - i] / coeffs[0]);
            if (i > 1) {
                mat.set(i - 1, i, 1);
            }
        }
        List<Complex_F64> eigs = mat.eig().getEigenvalues();
        List<Double> roots = new ArrayList<>();
        for (Complex_F64 eig : eigs) {
            if (eig.isReal()) {
                roots.add(eig.getReal());
            }
        }
        return roots;
    }
    private double[] rpError(double[] pose, SimpleMatrix camM, SimpleMatrix worldM) {
        double ct = cos(pose[2]);
        double st = sin(pose[2]);
        double[] err = new double[2 * worldM.getNumRows()];
        for (int i = 1; i <= worldM.getNumRows(); i++) {
            double xi = pose[0] - worldM.get(i, 1);
            double yi = pose[1] - worldM.get(i, 2);
            double zi = worldM.get(i, 3) - z;
            double d = (xi * ct + yi * st) * sBeta - zi * cBeta;
            err[2 * i - 2] = camM.get(i, 1) - ((xi * ct + yi * st) * cBeta + zi * sBeta) / d;
            err[2 * i - 1] = camM.get(i, 2) - (yi * ct - xi * st) / d;
        }
        return err;
    }
    private double[][] rpJac(double[] pose, SimpleMatrix worldM) {
        double ct = cos(pose[2]);
        double st = sin(pose[2]);
        double[][] jac = new double[2 * worldM.getNumRows()][3];
        for (int i = 1; i <= worldM.getNumRows(); i++) {
            double xi = pose[0] - worldM.get(i, 1);
            double yi = pose[1] - worldM.get(i, 2);
            double zi = worldM.get(i, 3) - z;
            double d = (xi * ct + yi * st) * sBeta - worldM.get(i, 3) * cBeta;
            d *= d;
            jac[2 * i - 2][0] = zi * ct / d;
            jac[2 * i - 2][1] = zi * st / d;
            jac[2 * i - 2][2] = zi * (yi * ct - xi * st) / d;
            jac[2 * i - 1][0] = (yi * sBeta - zi * cBeta * ct) / d;
            jac[2 * i - 1][1] = (zi * cBeta * ct - xi * sBeta) / d;
            jac[2 * i - 1][2] = sBeta * (xi * xi + yi * yi) / d;
        }
        return jac;
    }
}
