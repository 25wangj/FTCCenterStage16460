package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import androidx.annotation.GuardedBy;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.movement.AbstractMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.movement.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.sensors.Encoder;
public class MecanumDrivetrain extends AbstractMecanumDrivetrain {
    private static final double wheelRad = 0.689;
    private static final int ticks = 8192;
    private static final double parDist = 11.01;
    private static final double perpDist = 4.85;
    private static final RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot
            (RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    private final Object gyroLock = new Object();
    @GuardedBy("gyroLock")
    private IMU gyro;
    private double heading = 0;
    private double offset = 0;
    public MecanumDrivetrain(LinearOpMode opMode, boolean auto) {
        super("fr", "fl", "br", "bl", opMode, auto);
        if (auto) {
            Encoder par1 = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "fr"));
            Encoder par2 = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "bl"));
            Encoder perp = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "fl"));
            par2.setDirection(Encoder.Direction.REVERSE);
            localizer = new ThreeWheelLocalizer(par1, par2, perp, parDist, perpDist, ticks / (2 * PI * wheelRad));
        } else {
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