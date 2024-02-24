package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;
@TeleOp(name = "CameraPose")
public class CameraPose extends CommandOpMode {
    private double squareSize = 1.142;
    private double h1 = 7.918;
    private Size size = new Size(6, 5);
    private double[] dists = {18, 24, 30, 36, 42, 48};
    private List<SimpleMatrix> imgPts = new ArrayList<>();
    private List<Double> ds = new ArrayList<>();
    private List<Double> hs = new ArrayList<>();
    private Mat gray = new Mat();
    private MatOfPoint2f corners = new MatOfPoint2f();
    private RisingEdgeDetector a;
    private SimpleMatrix invCamMat;
    private int ind = 0;
    @Override
    public void initOpMode() {
        a = new RisingEdgeDetector(() -> gamepad1.a);
        String cam = "";
        while (cam.isEmpty()) {
            telemetry.addLine("Press a for front camera, b for back camera");
            telemetry.update();
            if (gamepad1.a) {
                cam = "camera1";
                invCamMat = Vision.camMat1.invert();
            } else if (gamepad1.b){
                cam = "camera2";
                invCamMat = Vision.camMat2.invert();
            }
        }
        a = new RisingEdgeDetector(() -> gamepad1.a);
        VisionProcessor proc = new VisionProcessor() {
            @Override
            public void init(int width, int height, CameraCalibration calibration) {}
            @Override
            public Object processFrame(Mat frame, long captureTimeNanos) {
                Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);
                if (Calib3d.findChessboardCorners(frame, size, corners, Calib3d.CALIB_CB_ADAPTIVE_THRESH)) {
                    Imgproc.cornerSubPix(gray, corners, new Size(11, 11), new Size(-1, -1),
                            new TermCriteria(TermCriteria.EPS, 30, 0.001));
                    Calib3d.drawChessboardCorners(frame, size, corners, true);
                    Imgproc.circle(frame, new Point(corners.get(0, 0)), 0, new Scalar(255, 0, 0), 3);
                    if (a.getAsBoolean()) {
                        for (int i = 0; i < size.height; i++) {
                            for (int j = 0; j < size.width; j++) {
                                double[] corner = corners.get(i * (int)size.width + j, 0);
                                imgPts.add(invCamMat.mult(new SimpleMatrix(new double[]{corner[0], corner[1], 1})));
                                hs.add(h1 - i * squareSize);
                                double x = (j - (size.width - 1) / 2) * squareSize;
                                ds.add(sqrt(x * x + dists[ind] * dists[ind]));
                            }
                        }
                        ind++;
                    }
                }
                return null;
            }
            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
        };
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cam))
                .setCameraResolution(new android.util.Size(640, 480))
                .addProcessor(proc)
                .build();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {};
        portal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        portal.getCameraControl(ExposureControl.class).setExposure(50, TimeUnit.MILLISECONDS);
        while (ind < dists.length) {
            telemetry.addLine("Place the camera " + dists[ind] + "in away from the board");
            telemetry.addLine("Press a to capture an image");
            telemetry.update();
        }
        LeastSquaresProblem lsq = new LeastSquaresBuilder()
                .model(this::error, this::jac)
                .start(new double[] {4, PI / 2, -PI / 2})
                .target(new double[imgPts.size()])
                .maxEvaluations(100)
                .maxIterations(100)
                .build();
        LeastSquaresOptimizer.Optimum opt = new LevenbergMarquardtOptimizer().optimize(lsq);
        while (!isStopRequested()) {
            telemetry.addData("Estimated height", opt.getPoint().getEntry(0));
            telemetry.addData("Estimated pitch", opt.getPoint().getEntry(1));
            telemetry.addData("Estimated roll", opt.getPoint().getEntry(2));
            telemetry.update();
        }
    }
    public double[] error(double[] vars) {
        double[] error = new double[imgPts.size()];
        SimpleMatrix rBeta = new SimpleMatrix(new double[][] {{cos(vars[1]), 0, sin(vars[1])}, {0, 1, 0}, {-sin(vars[1]), 0, cos(vars[1])}});
        SimpleMatrix rGamma = new SimpleMatrix(new double[][] {{cos(vars[2]), -sin(vars[2]), 0}, {sin(vars[2]), cos(vars[2]), 0}, {0, 0, 1}});
        for (int i = 0; i < imgPts.size(); i++) {
            SimpleMatrix rotImgPt = rBeta.mult(rGamma.mult(imgPts.get(i)));
            error[i] = atan(rotImgPt.get(2) / sqrt(rotImgPt.get(0) * rotImgPt.get(0) + rotImgPt.get(1) * rotImgPt.get(1)))
                     + atan((vars[0] - hs.get(i)) / ds.get(i));
        }
        return error;
    }
    public double[][] jac(double[] vars) {
        double[] er = error(vars);
        double[] er1 = error(new double[] {vars[0] + 1e-6, vars[1], vars[2]});
        double[] er2 = error(new double[] {vars[0], vars[1] + 1e-6, vars[2]});
        double[] er3 = error(new double[] {vars[0], vars[1], vars[2] + 1e-6});
        double[][] jac = new double[imgPts.size()][3];
        for (int i = 0; i < imgPts.size(); i++) {
            jac[i][0] = (er1[i] - er[i]) / 1e-6;
            jac[i][1] = (er2[i] - er[i]) / 1e-6;
            jac[i][2] = (er3[i] - er[i]) / 1e-6;
        }
        return jac;
    }
}
