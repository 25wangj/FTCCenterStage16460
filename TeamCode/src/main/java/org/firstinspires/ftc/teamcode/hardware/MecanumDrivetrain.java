package org.firstinspires.ftc.teamcode.hardware;
import androidx.annotation.GuardedBy;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.movement.AbstractMecanumDrivetrain;
public class MecanumDrivetrain extends AbstractMecanumDrivetrain {
    private static final RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot
            (RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    private final Object gyroLock = new Object();
    @GuardedBy("gyroLock")
    private IMU gyro;
    private double heading;
    private double offset;
    public MecanumDrivetrain(LinearOpMode opMode, boolean auto) {
        super("fr", "fl", "br", "bl", opMode, null, auto);
        heading = 0;
        offset = 0;
        if (!auto) {
            gyro = opMode.hardwareMap.get(IMU.class, "gyro");
            synchronized (gyroLock) {
                IMU.Parameters parameters = new IMU.Parameters(orientation);
                gyro.initialize(parameters);
                gyro.resetYaw();
            }
            Thread gyroThread = new Thread(() -> {
                opMode.waitForStart();
                while (opMode.opModeIsActive()) {
                    synchronized (gyroLock) {
                        heading = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                    }
                }
            });
            gyroThread.start();
        }
    }
    public void setHeading(double h) {
        offset = h - heading;
    }
    public double getHeading() {
        return offset + heading;
    }
}
