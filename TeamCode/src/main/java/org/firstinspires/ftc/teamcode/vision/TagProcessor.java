package org.firstinspires.ftc.teamcode.vision;
import static java.lang.Math.*;
import android.graphics.Canvas;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.util.Pair;
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
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.DoubleSupplier;
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
    private DoubleSupplier heading;
    private LinkedBlockingQueue<KalmanEstimate> queue = new LinkedBlockingQueue<>();
    public TagProcessor(SimpleMatrix camMat, double[] pos, double[] rot, DoubleSupplier heading) {
        tagPtr = AprilTagDetectorJNI.createApriltagDetector("tag36h11", 2, 3);
        invCamMat = camMat.invert().transpose();
        camPose = new double[] {pos[0], pos[1], rot[1]};
        z = pos[2];
        sBeta = sin(rot[1]);
        cBeta = cos(rot[1]);
        rBeta = new SimpleMatrix(new double[][] {{cBeta, 0, -sBeta}, {0, 1, 0}, {sBeta, 0, cBeta}});
        rGamma = new SimpleMatrix(new double[][] {{cos(rot[2]), sin(rot[2]), 0}, {-sin(rot[2]), cos(rot[2]), 0}, {0, 0, 1}});
        this.heading = heading;
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
                double h = heading.getAsDouble();
                double sh = sin(h);
                double ch = cos(h);
                double a1 = 0, a2 = 0, b1 = 0, b2 = 0, c1 = 0, c2 = 0;
                for (int i = 1; i <= worldM.getNumRows(); i++) {
                    double mag = sqrt(rotM.get(i, 1) * rotM.get(i, 1) + rotM.get(i, 2) * rotM.get(i, 2));
                    double si = (sh * rotM.get(i, 1) + ch * rotM.get(i, 2)) / mag;
                    double ci = (ch * rotM.get(i, 1) - sh * rotM.get(i, 2)) / mag;
                    double xi = worldM.get(i, 1);
                    double yi = worldM.get(i, 2);
                    a1 += si * si;
                    b1 -= si * ci;
                    c1 += xi * si * si - yi * si * ci;
                    a2 -= si * ci;
                    b2 += ci * ci;
                    c2 += yi * ci * ci - xi * si * ci;
                }
                LeastSquaresProblem lsq = new LeastSquaresBuilder()
                        .model(p -> rpError(p, camM, worldM))
                        .start(new double[] {(c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1), (a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1), h})
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
    private Pair<RealVector, RealMatrix> rpError(RealVector pose, SimpleMatrix camM, SimpleMatrix worldM) {
        double ct = cos(pose.getEntry(2));
        double st = sin(pose.getEntry(2));
        RealVector err = new ArrayRealVector(2 * worldM.getNumRows());
        RealMatrix jac = new Array2DRowRealMatrix(2 * worldM.getNumRows(), 3);
        for (int i = 1; i <= worldM.getNumRows(); i++) {
            double xi = pose.getEntry(0) - worldM.get(i, 1);
            double yi = pose.getEntry(1) - worldM.get(i, 2);
            double zi = worldM.get(i, 3) - z;
            double d = (xi * ct + yi * st) * sBeta - zi * cBeta;
            err.setEntry(2 * i - 2, camM.get(i, 1) - ((xi * ct + yi * st) * cBeta + zi * sBeta) / d);
            err.setEntry(2 * i - 1, camM.get(i, 2) - (yi * ct - xi * st) / d);
            d *= d;
            jac.setEntry(2 * i - 2, 0, zi * ct / d);
            jac.setEntry(2 * i - 2,1, zi * st / d);
            jac.setEntry(2 * i - 2, 2, zi * (yi * ct - xi * st) / d);
            jac.setEntry(2 * i - 1, 0, (yi * sBeta - zi * cBeta * ct) / d);
            jac.setEntry(2 * i - 1, 1, (zi * cBeta * ct - xi * sBeta) / d);
            jac.setEntry(2 * i - 1, 2, sBeta * (xi * xi + yi * yi) / d);
        }
        return new Pair<>(err, jac);
    }
    public LinkedBlockingQueue<KalmanEstimate> queue() {
        return queue;
    }
}