package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
public abstract class AbstractAutonomous extends CommandOpMode {
    public enum Case {
        LEFT, CENTER, RIGHT
    }
    public static final int minDetected = 10;
    protected Robot robot;
    protected PropDetector detector;
    protected Side side;
    protected Case runCase = Case.CENTER;
    private Case detectCase = Case.CENTER;
    private int detected = 0;
    public abstract void initAutonomous();
    @Override
    public void initOpMode() {
        robot = new Robot(this, true);
        initAutonomous();
    }
    @Override
    public void waitOpMode() {
        if (detectCase == detector.getCase()) {
            detected++;
        } else {
            detectCase = detector.getCase();
            detected = 0;
        }
        if (detected >= minDetected) {
            runCase = detectCase;
        }
        telemetry.addData("Case Detected", detectCase);
        telemetry.addData("Case to Run", runCase);
    }
    @Override
    public void startOpMode() {
        detector.close();
    }
    @Override
    public void endOpMode() {
        lastSide = side;
        lastPose = robot.drive.pose();
    }
}
