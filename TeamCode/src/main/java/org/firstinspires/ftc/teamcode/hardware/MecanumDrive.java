package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import androidx.annotation.GuardedBy;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.PidCoefficients;
import org.firstinspires.ftc.teamcode.movement.AbstractMecanumDrive;
import org.firstinspires.ftc.teamcode.movement.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.movement.Encoder;
public class MecanumDrive extends AbstractMecanumDrive {
    public static final double wheelRad = 0.689;
    public static final int ticks = 8192;
    public static final double parDist = 10.93;
    public static final double perpDist = 4.85;
    public static final double trackWidth = 12;
    public static final double driveKv = 0.0135;
    public static final double driveKa = 0.003;
    public static final double driveKs = 0;
    public static final double strafeMult = 1.35;
    public static final PidCoefficients moveCoeffs = new PidCoefficients(0.2, 0, 0);
    public static final PidCoefficients turnCoeffs = new PidCoefficients(2, 0, 0);
    public static final AsymConstraints moveConstraints = new AsymConstraints(60, 70, 70);
    public static final AsymConstraints turnConstraints = new AsymConstraints(6, 12, 12);
    private final Object gyroLock = new Object();
    @GuardedBy("gyroLock")
    private BNO055IMU gyro;
    private double heading = 0;
    private double offset = 0;
    public MecanumDrive(LinearOpMode opMode, boolean auto) {
        super(trackWidth, driveKs, driveKv, driveKa, strafeMult, moveCoeffs, turnCoeffs, moveConstraints, turnConstraints, auto);
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
            Encoder par1 = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "fl"));
            Encoder par2 = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "br"));
            Encoder perp = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "bl"));
            par1.setDirection(Encoder.Direction.REVERSE);
            localizer = new ThreeWheelLocalizer(par1, par2, perp, parDist, perpDist, 2 * PI * wheelRad / ticks);
        }
        gyro = opMode.hardwareMap.get(BNO055IMU.class, "gyro");
        synchronized (gyroLock) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            gyro.initialize(parameters);
        }
        Thread gyroThread = new Thread(() -> {
            while (!opMode.isStarted() && !opMode.isStopRequested()) {}
            while (opMode.opModeIsActive()) {
                synchronized (gyroLock) {
                    heading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                }
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {}
            }
        });
        gyroThread.start();
    }
    public void setHeading(double h) {
        offset = h - heading;
    }
    public double getHeading() {
        return offset + heading;
    }
}