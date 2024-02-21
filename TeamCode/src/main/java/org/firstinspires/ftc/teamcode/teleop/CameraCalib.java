package org.firstinspires.ftc.teamcode.teleop;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
@TeleOp(name = "CameraCalib")
public class CameraCalib extends CommandOpMode {
    public static final int num = 20;
    public static final Size size = new Size(7, 5);
    private Mat gray = new Mat();
    private MatOfPoint2f corners = new MatOfPoint2f();
    private List<Mat> data = new ArrayList<>();
    private RisingEdgeDetector a;
    private int i = 0;
    @Override
    public void initOpMode() {
        String cam = "";
        while (cam.isEmpty()) {
            telemetry.addLine("Press a for front camera, b for back camera");
            telemetry.update();
            if (gamepad1.a) {
                cam = "camera1";
            } else {
                cam = "camera2";
            }
        }
        a = new RisingEdgeDetector(() -> gamepad1.a);
        BoardProcessor proc = new BoardProcessor();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cam))
                .setCameraResolution(new android.util.Size(640, 480))
                .addProcessor(proc)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {};
        portal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        portal.getCameraControl(ExposureControl.class).setExposure(50, TimeUnit.MILLISECONDS);
        FtcDashboard.getInstance().startCameraStream(proc, 0);
        while (i < num) {
            telemetry.addLine("Press a to capture an image");
            telemetry.addLine("Image " + (i + 1) + " / " + num);
            telemetry.update();
        }
        portal.close();
        telemetry.addLine("Done");
        telemetry.update();
        List<Point3> pts = new ArrayList<>();
        for (int i = 0; i < size.height; i++) {
            for (int j = 0; j < size.width; j++) {
                pts.add(new Point3(j, i, 0));
            }
        }
        MatOfPoint3f objPts = new MatOfPoint3f();
        objPts.fromList(pts);
        List<Mat> objList = new ArrayList<>(Collections.nCopies(num, objPts)), rvecs = new ArrayList<>(), tvecs = new ArrayList<>();
        MatOfDouble camMat = new MatOfDouble();
        MatOfDouble distCoeffs = new MatOfDouble();
        Calib3d.calibrateCamera(objList, data, new Size(640, 480), camMat, distCoeffs, rvecs, tvecs);
        double error = 0;
        for (int i = 0; i < num; i++) {
            MatOfPoint2f imgPts = new MatOfPoint2f();
            Calib3d.projectPoints(objPts, rvecs.get(i), tvecs.get(i), camMat, distCoeffs, imgPts);
            error += Core.norm(data.get(i), imgPts, Core.NORM_L2) / (imgPts.total() * num);
        }
        while(!isStopRequested()) {
            telemetry.addData("fx", camMat.get(0, 0)[0]);
            telemetry.addData("fy", camMat.get(1, 1)[0]);
            telemetry.addData("cx", camMat.get(0, 2)[0]);
            telemetry.addData("cy", camMat.get(1, 2)[0]);
            telemetry.addData("Reprojection Error", error);
            telemetry.update();
        }
    }
    public class BoardProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);
            if (Calib3d.findChessboardCorners(frame, size, corners, Calib3d.CALIB_CB_ADAPTIVE_THRESH)) {
                Imgproc.cornerSubPix(gray, corners, new Size(11, 11), new Size(-1, -1),
                        new TermCriteria(TermCriteria.EPS, 30, 0.001));
                Calib3d.drawChessboardCorners(frame, size, corners, true);
                if (a.getAsBoolean()) {
                    data.add(corners);
                    i++;
                }
            }
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }
        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}