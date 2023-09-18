package org.firstinspires.ftc.teamcode.hardware;
import androidx.annotation.GuardedBy;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
public class Robot {
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;
    private double heading = 0;
    private final Object gyroLock = new Object();
    @GuardedBy("gyroLock")
    public IMU gyro;
    public void init(HardwareMap hwMap, boolean teleop) {
        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");
        gyro = hwMap.get(IMU.class, "gyro");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        if (teleop) {
            synchronized (gyroLock) {
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
                gyro.initialize(parameters);
                gyro.resetYaw();
            }
        }
    }
    public double getHeading() {
        return heading;
    }
    public void update(double time) {
    }
    public void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
    public void startGyro(LinearOpMode opMode) {
        Thread gyroThread = new Thread(() -> {
            opMode.waitForStart();
            while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
                synchronized (gyroLock) {
                    heading = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                }
            }
        });
        gyroThread.start();
    }
}
