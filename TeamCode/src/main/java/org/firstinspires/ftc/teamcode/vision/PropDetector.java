package org.firstinspires.ftc.teamcode.vision;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;
public class PropDetector {
    private VisionPortal portal;
    private ColorPropProcessor proc;
    public PropDetector(LinearOpMode opMode, boolean right, Side side) {
        proc = new ColorPropProcessor(right, side);
        portal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "camera1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(proc)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }
    public void close() {
        portal.close();
    }
    public Case getCase() {
        return proc.getCase();
    }
}
