package org.firstinspires.ftc.teamcode.vision;
import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
public class ColorPropProcessor implements VisionProcessor {
    public static final Scalar blueLower = new Scalar(0, 0, 150);
    public static final Scalar blueUpper = new Scalar(255, 255, 255);
    public static final Scalar redLower = new Scalar(0, 150, 0);
    public static final Scalar redUpper = new Scalar(255, 255, 255);
    public static final double hMin = 0.5;
    public static final double hDiff = 0.3;
    public static final double threshold = 10;
    private Mat grayscale = new Mat();
    private int w;
    private int h;
    private Scalar upper;
    private Scalar lower;
    private boolean right;
    private Case detectCase = Case.CENTER;
    public ColorPropProcessor(boolean right, Side side) {
        this.right = right;
        if (side == Side.BLUE) {
            lower = blueLower;
            upper = blueUpper;
        } else {
            lower = redLower;
            upper = redUpper;
        }
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.w = width;
        this.h = height;
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Core.rotate(frame, frame, Core.ROTATE_180);
        Imgproc.cvtColor(frame, grayscale, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.rectangle(frame, new Point(0, 0), new Point(w, h * hMin), new Scalar(0, 0, 0), -1);
        Imgproc.rectangle(frame, new Point(0, h * (hMin + hDiff)), new Point(w, h), new Scalar(0, 0, 0), -1);
        grayscale = grayscale.submat(new Rect(0, (int)(h * hMin), w, (int)(h * hDiff)));
        Core.inRange(grayscale, lower, upper, grayscale);
        double leftAvg = Core.sumElems(grayscale.submat(new Rect(0, 0, w / 2, grayscale.height())))
                .val[0] * 2 / (w * h * hDiff);
        double rightAvg = Core.sumElems(grayscale.submat(new Rect(w / 2, 0, w / 2, grayscale.height())))
                .val[0] * 2 / (w * h * hDiff);
        System.out.println("Left: " + leftAvg + " Right: " + rightAvg);
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
