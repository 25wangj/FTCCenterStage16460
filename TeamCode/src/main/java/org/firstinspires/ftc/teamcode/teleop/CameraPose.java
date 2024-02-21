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
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
@TeleOp(name = "CameraPose")
public class CameraPose extends CommandOpMode {
    private double EPS = 1e-6;
    private double squareSize = 1.142;
    private double h1 = 2.731;
    private Size size = new Size(7, 5);
    private double[] dists = {12, 18, 24, 30, 36, 42, 48};
    private boolean flip;
    private List<SimpleMatrix> imgPts = new ArrayList<>();
    private List<Double> ds = new ArrayList<>();
    private List<Double> hs = new ArrayList<>();
    private Mat gray = new Mat();
    private MatOfPoint2f corners = new MatOfPoint2f();
    private RisingEdgeDetector a;
    private SimpleMatrix invCamMat;
    private int i = 0;
    @Override
    public void initOpMode() {
        a = new RisingEdgeDetector(() -> gamepad1.a);
        String cam = "";
        while (cam.isEmpty()) {
            telemetry.addLine("Press a for front camera, b for back camera");
            telemetry.update();
            if (gamepad1.a) {
                cam = "camera1";
                flip = true;
                invCamMat = Vision.camMat1.invert();
            } else {
                cam = "camera2";
                flip = false;
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
                    if (a.getAsBoolean()) {
                        for (int i = 0; i < size.height; i++) {
                            for (int j = 0; j < size.width; j++) {
                                SimpleMatrix pt;
                                if (flip) {
                                    pt = new SimpleMatrix(new double[]{-corners.get(j, i)[0], -corners.get(j, i)[1], 1});
                                } else {
                                    pt = new SimpleMatrix(new double[]{corners.get(j, i)[0], corners.get(j, i)[1], 1});
                                }
                                imgPts.add(invCamMat.mult(pt));
                                hs.add(h1 + i * squareSize);
                                double x = (j - (size.width - 1) / 2) * squareSize;
                                ds.add(sqrt(x * x + dists[i] * dists[i]));
                            }
                        }
                        i++;
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
        while (i < dists.length) {
            telemetry.addLine("Place the camera " + dists[i] + "in away from the board");
            telemetry.addLine("Press a to capture an image");
        }
        LeastSquaresProblem lsq = new LeastSquaresBuilder()
                .model(this::error, this::jac)
                .start(new double[] {0, PI / 2})
                .build();
        LeastSquaresOptimizer.Optimum opt = new LevenbergMarquardtOptimizer().withCostRelativeTolerance(1e-6).optimize(lsq);
        while (!isStopRequested()) {
            telemetry.addData("Estimated height", opt.getPoint().getEntry(0));
            telemetry.addData("Estimated pitch", opt.getPoint().getEntry(1));
            telemetry.update();
        }
    }
    public double[] error(double[] vars) {
        double[] error = new double[imgPts.size()];
        SimpleMatrix rBeta = new SimpleMatrix(new double[][] {{cos(vars[1]), 0, sin(vars[1])}, {0, 1, 0}, {-sin(vars[1]), 0, cos(vars[1])}});
        for (int i = 0; i < imgPts.size(); i++) {
            SimpleMatrix rotImgPt = rBeta.mult(imgPts.get(i));
            error[i] = atan(rotImgPt.get(3) / sqrt(rotImgPt.get(1) * rotImgPt.get(1) + rotImgPt.get(2) * rotImgPt.get(2)))
                     + atan((vars[0] - hs.get(i)) / ds.get(i));
        }
        return error;
    }
    public double[][] jac(double[] vars) {
        double[] er = error(vars);
        double[] er1 = error(new double[] {vars[0] + EPS, vars[1]});
        double[] er2 = error(new double[] {vars[0], vars[1] + EPS});
        double[][] jac = new double[imgPts.size()][2];
        for (int i = 0; i < imgPts.size(); i++) {
            jac[i][1] = (er1[i] - er[i]) / EPS;
            jac[i][2] = (er2[i] - er[i]) / EPS;
        }
        return jac;
    }
}
