package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import androidx.annotation.GuardedBy;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.PidCoefficients;
import org.firstinspires.ftc.teamcode.movement.AbstractMecanumDrive;
import org.firstinspires.ftc.teamcode.movement.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.sensors.Encoder;
public class MecanumDrive extends AbstractMecanumDrive {
    public static final double wheelRad = 0.689;
    public static final int ticks = 8192;
    public static final double parDist = 11.01;
    public static final double perpDist = 4.85;
    public static final double trackWidth = 12;
    public static final double driveKv = 0;
    public static final double driveKa = 0;
    public static final double driveKs = 0;
    public static final PidCoefficients moveCoeffs = new PidCoefficients(0, 0, 0);
    public static final PidCoefficients turnCoeffs = new PidCoefficients(0, 0, 0);
    public static final AsymConstraints moveConstraints = new AsymConstraints(0, 0, 0);
    public static final AsymConstraints turnConstraints = new AsymConstraints(0, 0, 0);
    public static final RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot
            (RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    private final Object gyroLock = new Object();
    @GuardedBy("gyroLock")
    private IMU gyro;
    private double heading = 0;
    private double offset = 0;
    public MecanumDrive(LinearOpMode opMode, boolean auto) {
        super(trackWidth, driveKs, driveKa, driveKv, moveCoeffs, turnCoeffs, moveConstraints, turnConstraints, auto);
        fr = opMode.hardwareMap.get(DcMotorEx.class, "fr");
        fl = opMode.hardwareMap.get(DcMotorEx.class, "fl");
        br = opMode.hardwareMap.get(DcMotorEx.class, "br");
        bl = opMode.hardwareMap.get(DcMotorEx.class, "bl");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (auto) {
            Encoder par1 = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "br"));
            Encoder par2 = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "fl"));
            Encoder perp = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "bl"));
            par2.setDirection(Encoder.Direction.REVERSE);
            perp.setDirection(Encoder.Direction.REVERSE);
            localizer = new ThreeWheelLocalizer(par1, par2, perp, parDist, perpDist, 2 * PI * wheelRad / ticks);
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