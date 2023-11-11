package org.firstinspires.ftc.teamcode.vision;
import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
public class ColorPropProcessor implements VisionProcessor {
    public static final Scalar redLower = new Scalar(0, 150, 0);
    public static final Scalar redUpper = new Scalar(255, 255, 255);
    public static final Scalar blueLower = new Scalar(0, 0, 150);
    public static final Scalar blueUpper = new Scalar(255, 255, 255);
    public static final double heightMin = 0.5;
    public static final double heightDiff = 0.5;
    public static final double threshold = 5;
    private Mat grayscale = new Mat();
    private int width;
    private int height;
    private Scalar upper;
    private Scalar lower;
    private boolean right;
    private Case detectCase = Case.CENTER;
    public ColorPropProcessor(boolean right, Side side) {
        this.right = right;
        if (side == Side.RED) {
            lower = redLower;
            upper = redUpper;
        } else {
            lower = blueLower;
            upper = blueUpper;
        }
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, grayscale, Imgproc.COLOR_RGB2YCrCb);
        grayscale = grayscale.submat(new Rect(0, (int)(height * heightMin), width, (int)(height * heightDiff)));
        Core.inRange(grayscale, lower, upper, grayscale);
        double leftAvg = Core.sumElems(grayscale.submat(new Rect(0, 0, width / 2, grayscale.height())))
                .val[0] / (width * height);
        double rightAvg = Core.sumElems(grayscale.submat(new Rect(width / 2, 0, width / 2, grayscale.height())))
                .val[0] / (width * height);
        if (rightAvg > threshold) {
            detectCase = right ? Case.RIGHT : Case.CENTER;
        } else if (leftAvg > threshold) {
            detectCase = right ? Case.CENTER : Case.LEFT;
        } else {
            detectCase = right ? Case.LEFT : Case.RIGHT;
        }
        return null;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
    public Case getCase() {
        return detectCase;
    }
}
