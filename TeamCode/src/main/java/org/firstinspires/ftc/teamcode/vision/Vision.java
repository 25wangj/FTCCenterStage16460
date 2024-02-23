package org.firstinspires.ftc.teamcode.vision;
import static java.lang.Math.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;
public class Vision {
    public static final long exposure = 20;
    public static final SimpleMatrix camMat1 = new SimpleMatrix(new double[][] {{687, 0, 332}, {0, 687, 245}, {0, 0, 1}});
    public static final SimpleMatrix camMat2 = new SimpleMatrix(new double[][] {{813, 0, 311}, {0, 813, 231}, {0, 0, 1}});
    public static final double[] camPos1 = {6.0, 0.7, 3.4};
    public static final double[] camPos2 = {0, 0, 0};
    public static final double[] camRot1 = {0, PI / 2, PI};
    public static final double[] camRot2 = {0, PI / 2, 0};
    public static final HashMap<Integer, double[][]> tagPts = new HashMap<>(Map.of(
            1, new double[][] {{62, 42.5, 3.134}, {63, 42.5, 4.866}, {63, 40.5, 4.866}, {62, 40.5, 3.134}},
            2, new double[][] {{62, 36.5, 3.134}, {63, 36.5, 4.866}, {63, 34.5, 4.866}, {62, 34.5, 3.134}},
            3, new double[][] {{62, 30.5, 3.134}, {63, 30.5, 4.866}, {63, 28.5, 4.866}, {62, 28.5, 3.134}},
            4, new double[][] {{62, -28.5, 3.134}, {63, -28.5, 4.866}, {63, -30.5, 4.866}, {62, -30.5, 3.134}},
            5, new double[][] {{62, -34.5, 3.134}, {63, -34.5, 4.866}, {63, -36.5, 4.866}, {62, -36.5, 3.134}},
            6, new double[][] {{62, -40.5, 3.134}, {63, -40.5, 4.866}, {63, -42.5, 4.866}, {62, -42.5, 3.134}}));
    private boolean front;
    private VisionPortal portal1;
    private VisionPortal portal2;
    private PropProcessor prop1;
    private PropProcessor prop2;
    private TagProcessor tag1;
    private TagProcessor tag2;
    public Vision(LinearOpMode opMode, boolean right, boolean front, Side side, DoubleSupplier heading) {
        this.front = front;
        prop1 = new PropProcessor(right, true, 0.5, 0.3, side);
        //prop2 = new PropProcessor(right, false, 0.3, 0.5, side);
        tag1 = new TagProcessor(camMat1, camPos1, camRot1, heading);
        //tag2 = new TagProcessor(camMat2, camPos2, camRot2, heading);
        portal1 = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "camera1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(prop1)
                .addProcessor(tag1)
                .enableLiveView(front)
                .build();
        /*portal2 = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "camera2"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(prop1)
                .addProcessor(tag1)
                .enableLiveView(!front)
                .build();*/
        portal1.setProcessorEnabled(prop1, true);
        portal1.setProcessorEnabled(tag1, false);
        //portal2.setProcessorEnabled(prop2, true);
        //portal2.setProcessorEnabled(tag2, false);
        while (portal1.getCameraState() != VisionPortal.CameraState.STREAMING /*|| portal2.getCameraState() != VisionPortal.CameraState.STREAMING*/) {}
        portal1.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Auto);
        //portal2.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Auto);
    }
    public void start() {
        portal1.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        portal1.getCameraControl(ExposureControl.class).setExposure(exposure, TimeUnit.MILLISECONDS);
        //portal2.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        //portal2.getCameraControl(ExposureControl.class).setExposure(exposure, TimeUnit.MILLISECONDS);
        portal1.setProcessorEnabled(prop1, false);
        portal1.setProcessorEnabled(tag1, true);
        //portal2.setProcessorEnabled(prop2, false);
        //portal2.setProcessorEnabled(tag2, true);
        portal1.stopLiveView();
        //portal2.stopLiveView();
    }
    public void close() {
        portal1.close();
        //portal2.close();
    }
    public Case getCase() {
        return front ? prop1.getCase() : prop2.getCase();
    }
}